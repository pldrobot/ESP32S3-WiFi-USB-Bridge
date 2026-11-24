// TX role: USB host. Owns the real peripheral and mirrors it to the RX over the link.
//
// Same mechanism as EspUsbIpServer (usbipdcpp-style): async transfers with completion callbacks
// on the usb_host task, every interface (alt 0) claimed, bulk/interrupt endpoints forwarded.
// IN endpoints are read ahead one packet per transfer, so packet boundaries (short packets, ZLPs)
// reach the RX exactly as the device sent them.
#include "config.h"
#ifdef ROLE_HOST
#include "bridge.h"
#include "link.h"
#include "proto.h"
#include "usb/usb_host.h"
#include <atomic>

#define MAX_EPS   8
#define MAX_IFACE 8
#define IN_XFERS  3
#define CTRL_MAX  1024
#define NXFER     (OUT_CREDITS > IN_XFERS ? OUT_CREDITS : IN_XFERS)

struct Ep;
struct Xfer {
    usb_transfer_t* t = nullptr;
    Ep* ep = nullptr;
    volatile bool busy = false;
    bool parked = false, hasData = false;
    uint8_t errs = 0;
};
struct Ep {
    uint8_t addr = 0;
    bool in = false;
    uint16_t mps = 0;
    int inflight = 0;                 // IN: cost of records sent but not yet acknowledged by the RX
    std::atomic<int> owed{0};         // OUT: completed packets whose credit has not been sent yet
    int nx = 0;
    Xfer x[NXFER];
};

static Ep g_eps[MAX_EPS];
static int g_nEps;
static usb_host_client_handle_t g_client;
static usb_device_handle_t g_dev;
static volatile bool g_ready, g_gone, g_closeReq;
static uint32_t g_closeT;
static std::atomic<int> g_pending;     // submitted transfers whose callback has not run yet
static int g_parked;
static SemaphoreHandle_t g_mtx;
static uint8_t g_ifaces[MAX_IFACE];
static int g_nIfaces;
static uint8_t g_attach[18 + 1024];
static size_t g_attachLen;
static usb_transfer_t* g_ctrl;
static volatile bool g_ctrlBusy;
static uint16_t g_ctrlId;
static volatile bool g_enumSeen;

static bool enumFilter(const usb_device_desc_t*, uint8_t* cfg) {
    *cfg = 1;
    g_enumSeen = true;
    return true;
}

static Ep* findEp(uint8_t addr) {
    for (int i = 0; i < g_nEps; i++) if (g_eps[i].addr == addr) return &g_eps[i];
    return nullptr;
}

static void submit(Xfer* x) {
    x->busy = true;
    g_pending++;
    if (usb_host_transfer_submit(x->t) != ESP_OK) { g_pending--; x->busy = false; }
}

// Hand one completed IN packet to the link (mutex held). False = keep it parked and retry later.
static bool deliver(Xfer* x) {
    Ep* e = x->ep;
    if (x->hasData) {
        int n = x->t->actual_num_bytes;
        if (link_up()) {
            if (e->inflight >= IN_WINDOW || !link_send(M_IN, &e->addr, 1, x->t->data_buffer, n)) return false;
            e->inflight += in_cost(n);
        }                                       // link down: nobody to send to, drop
        x->hasData = false;
    }
    submit(x);
    return true;
}

static void flushCredits() {                     // mutex held
    for (int i = 0; i < g_nEps; i++) {
        Ep* e = &g_eps[i];
        int n = e->owed;
        if (e->in || n <= 0) continue;
        uint8_t b[3] = {e->addr, (uint8_t)n, (uint8_t)(n >> 8)};
        if (link_send(M_CREDIT, b, 3)) e->owed -= n;
    }
}

static void resume() {                           // link drain / credit arrived
    if (!g_ready) return;
    xSemaphoreTake(g_mtx, portMAX_DELAY);
    flushCredits();
    if (g_parked)
        for (int i = 0; i < g_nEps; i++)
            for (int j = 0; j < g_eps[i].nx; j++) {
                Xfer* x = &g_eps[i].x[j];
                if (x->parked && deliver(x)) { x->parked = false; g_parked--; }
            }
    xSemaphoreGive(g_mtx);
}

static void inCb(usb_transfer_t* t) {
    Xfer* x = (Xfer*)t->context;
    g_pending--;
    x->busy = false;
    if (g_gone || t->status == USB_TRANSFER_STATUS_NO_DEVICE || t->status == USB_TRANSFER_STATUS_CANCELED) return;
    xSemaphoreTake(g_mtx, portMAX_DELAY);
    if (t->status == USB_TRANSFER_STATUS_COMPLETED) {
        x->hasData = true;
        x->errs = 0;
    } else {
        if (t->status == USB_TRANSFER_STATUS_STALL) usb_host_endpoint_clear(g_dev, x->ep->addr);
        if (++x->errs > 20) { xSemaphoreGive(g_mtx); LOGI("IN ep 0x%02X gave up (status %d)", x->ep->addr, t->status); return; }
    }
    if (!deliver(x)) { x->parked = true; g_parked++; }
    xSemaphoreGive(g_mtx);
}

static void outCb(usb_transfer_t* t) {
    Xfer* x = (Xfer*)t->context;
    g_pending--;
    x->busy = false;
    if (t->status != USB_TRANSFER_STATUS_COMPLETED && t->status != USB_TRANSFER_STATUS_NO_DEVICE)
        LOGI("OUT ep 0x%02X status %d", x->ep->addr, t->status);
    x->ep->owed++;
    if (g_gone) return;
    xSemaphoreTake(g_mtx, portMAX_DELAY);
    flushCredits();
    xSemaphoreGive(g_mtx);
}

static void ctrlRespond(uint16_t id, bool ok, const uint8_t* data, int n) {
    uint8_t h[3] = {(uint8_t)id, (uint8_t)(id >> 8), (uint8_t)ok};
    link_send(M_CTRL_RSP, h, 3, data, n);
}

static void ctrlCb(usb_transfer_t* t) {
    g_pending--;
    bool ok = t->status == USB_TRANSFER_STATUS_COMPLETED;
    bool in = t->data_buffer[0] & 0x80;
    int n = t->actual_num_bytes - 8;
    ctrlRespond(g_ctrlId, ok, (ok && in && n > 0) ? t->data_buffer + 8 : nullptr, (ok && in && n > 0) ? n : 0);
    g_ctrlBusy = false;
}

// ── Device open / close (usb_host task) ──────────────────────────────────────
static bool openDevice(uint8_t addr) {
    usb_device_handle_t h;
    if (usb_host_device_open(g_client, addr, &h) != ESP_OK) return false;
    const usb_device_desc_t* dd;
    const usb_config_desc_t* cd;
    if (usb_host_get_device_descriptor(h, &dd) != ESP_OK || usb_host_get_active_config_descriptor(h, &cd) != ESP_OK ||
        cd->wTotalLength > 1024) {
        LOGI("USB device unusable (descriptors missing or config > 1024 B)");
        usb_host_device_close(g_client, h);
        return false;
    }
    g_dev = h;
    g_gone = false;
    g_nEps = g_nIfaces = 0;
    g_parked = 0;

    const uint8_t* p = (const uint8_t*)cd;
    for (int o = 0; o + 2 <= cd->wTotalLength && p[o]; o += p[o]) {
        if (p[o + 1] != USB_B_DESCRIPTOR_TYPE_INTERFACE) continue;
        const usb_intf_desc_t* itf = (const usb_intf_desc_t*)(p + o);
        if (itf->bAlternateSetting != 0 || g_nIfaces >= MAX_IFACE) continue;
        if (usb_host_interface_claim(g_client, h, itf->bInterfaceNumber, 0) != ESP_OK) {
            LOGI("interface %u claim failed", itf->bInterfaceNumber);
            continue;
        }
        g_ifaces[g_nIfaces++] = itf->bInterfaceNumber;
        for (int q = o + p[o]; q + 2 <= cd->wTotalLength && p[q] && p[q + 1] != USB_B_DESCRIPTOR_TYPE_INTERFACE; q += p[q]) {
            if (p[q + 1] != USB_B_DESCRIPTOR_TYPE_ENDPOINT) continue;
            const usb_ep_desc_t* ed = (const usb_ep_desc_t*)(p + q);
            usb_transfer_type_t ty = USB_EP_DESC_GET_XFERTYPE(ed);
            if (ty == USB_TRANSFER_TYPE_ISOCHRONOUS) { LOGI("iso ep 0x%02X not supported", ed->bEndpointAddress); continue; }
            if (g_nEps >= MAX_EPS) break;
            Ep* e = &g_eps[g_nEps];
            e->addr = ed->bEndpointAddress;
            e->in = (e->addr & 0x80) != 0;
            e->mps = ed->wMaxPacketSize & 0x7FF;
            e->inflight = 0;
            e->owed = 0;
            e->nx = e->in ? IN_XFERS : OUT_CREDITS;
            for (int j = 0; j < e->nx; j++) {
                Xfer* x = &e->x[j];
                *x = Xfer();
                x->ep = e;
                if (usb_host_transfer_alloc(e->mps, 0, &x->t) != ESP_OK) { e->nx = j; break; }
                x->t->device_handle = h;
                x->t->bEndpointAddress = e->addr;
                x->t->callback = e->in ? inCb : outCb;
                x->t->context = x;
                x->t->num_bytes = e->mps;
            }
            g_nEps++;
        }
    }
    if (!g_nEps) { LOGI("no usable endpoints"); g_gone = true; g_closeReq = true; g_closeT = millis(); return false; }

    memcpy(g_attach, dd, 18);
    memcpy(g_attach + 18, cd, cd->wTotalLength);
    g_attachLen = 18 + cd->wTotalLength;
    LOGI("USB device %04X:%04X ready: %d interface(s), %d endpoint(s)", dd->idVendor, dd->idProduct, g_nIfaces, g_nEps);
    g_ready = true;

    xSemaphoreTake(g_mtx, portMAX_DELAY);
    for (int i = 0; i < g_nEps; i++)
        if (g_eps[i].in)
            for (int j = 0; j < g_eps[i].nx; j++) submit(&g_eps[i].x[j]);
    xSemaphoreGive(g_mtx);
    link_send(M_ATTACH, g_attach, g_attachLen);
    return true;
}

static void tryClose() {
    if (g_pending > 0 && millis() - g_closeT < 2000) return;   // callbacks must drain first
    xSemaphoreTake(g_mtx, portMAX_DELAY);
    for (int i = 0; i < g_nEps; i++)
        for (int j = 0; j < g_eps[i].nx; j++)
            if (g_eps[i].x[j].t) { usb_host_transfer_free(g_eps[i].x[j].t); g_eps[i].x[j].t = nullptr; }
    g_nEps = 0;
    g_parked = 0;
    xSemaphoreGive(g_mtx);
    for (int i = 0; i < g_nIfaces; i++) usb_host_interface_release(g_client, g_dev, g_ifaces[i]);
    g_nIfaces = 0;
    usb_host_device_close(g_client, g_dev);
    g_dev = nullptr;
    g_closeReq = false;
    g_pending = 0;
    LOGI("USB device closed");
}

static void clientEvent(const usb_host_client_event_msg_t* m, void*) {
    if (m->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        if (!g_dev && !g_closeReq) openDevice(m->new_dev.address);
    } else if (m->event == USB_HOST_CLIENT_EVENT_DEV_GONE && g_dev && m->dev_gone.dev_hdl == g_dev) {
        LOGI("USB device unplugged");
        g_gone = true;
        g_ready = false;
        g_closeReq = true;
        g_closeT = millis();
        link_send(M_DETACH);
    }
}

static void hostTask(void*) {
    uint32_t lastAdopt = 0;
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(0, &flags);
        usb_host_client_handle_events(g_client, pdMS_TO_TICKS(2));
        if (g_closeReq) tryClose();
        // A device enumerated before this client registered never raises NEW_DEV: look for it.
        if (!g_dev && !g_closeReq && millis() - lastAdopt >= 500) {
            lastAdopt = millis();
            uint8_t addrs[2];
            int n = 0;
            if (usb_host_device_addr_list_fill(sizeof addrs, addrs, &n) == ESP_OK && n > 0) openDevice(addrs[0]);
        }
    }
}

// ── Link messages (link rx task) ─────────────────────────────────────────────
static void onCtrlReq(const uint8_t* p, uint16_t len) {
    if (len < 10) return;
    uint16_t id = p[0] | (p[1] << 8);
    uint16_t wl = p[8] | (p[9] << 8);          // setup.wLength
    bool in = p[2] & 0x80;
    if (!g_ready || g_ctrlBusy || wl > CTRL_MAX || (!in && len < 10 + wl)) { ctrlRespond(id, false, nullptr, 0); return; }
    g_ctrlBusy = true;
    g_ctrlId = id;
    memcpy(g_ctrl->data_buffer, p + 2, 8);
    if (!in && wl) memcpy(g_ctrl->data_buffer + 8, p + 10, wl);
    g_ctrl->device_handle = g_dev;
    g_ctrl->bEndpointAddress = 0;
    g_ctrl->num_bytes = 8 + wl;
    g_ctrl->callback = ctrlCb;
    g_pending++;
    if (usb_host_transfer_submit_control(g_client, g_ctrl) != ESP_OK) {
        g_pending--;
        g_ctrlBusy = false;
        ctrlRespond(id, false, nullptr, 0);
    }
}

void usb_on_msg(uint8_t type, const uint8_t* p, uint16_t len) {
    switch (type) {
    case M_LINK_UP:
    case M_LINK_DOWN:
        xSemaphoreTake(g_mtx, portMAX_DELAY);
        for (int i = 0; i < g_nEps; i++) g_eps[i].inflight = 0;
        xSemaphoreGive(g_mtx);
        resume();
        break;
    case M_REQ_ATTACH:
        if (g_ready) link_send(M_ATTACH, g_attach, g_attachLen);
        break;
    case M_OUT: {
        if (len < 1 || !g_ready) break;
        xSemaphoreTake(g_mtx, portMAX_DELAY);
        Ep* e = findEp(p[0]);
        bool sent = false;
        if (e && !e->in && len - 1 <= e->mps)
            for (int j = 0; j < e->nx && !sent; j++) {
                Xfer* x = &e->x[j];
                if (x->busy) continue;
                memcpy(x->t->data_buffer, p + 1, len - 1);
                x->t->num_bytes = len - 1;
                submit(x);
                sent = true;
            }
        xSemaphoreGive(g_mtx);
        if (!sent) LOGI("OUT ep 0x%02X dropped (no credit / bad ep)", p[0]);
        break;
    }
    case M_CREDIT: {
        if (len < 3) break;
        xSemaphoreTake(g_mtx, portMAX_DELAY);
        Ep* e = findEp(p[0]);
        if (e && e->in) e->inflight = max(0, e->inflight - (p[1] | (p[2] << 8)));
        xSemaphoreGive(g_mtx);
        resume();
        break;
    }
    case M_CTRL_REQ:
        onCtrlReq(p, len);
        break;
    }
}

void usb_poll() {}

bool usb_present() { return g_ready; }

void usb_begin() {
    g_mtx = xSemaphoreCreateMutex();
    usb_host_config_t hc = {};
    hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
    hc.enum_filter_cb = enumFilter;
    if (usb_host_install(&hc) != ESP_OK) { LOGI("usb_host_install failed"); return; }
    usb_host_client_config_t cc = {};
    cc.max_num_event_msg = 8;
    cc.async.client_event_callback = clientEvent;
    if (usb_host_client_register(&cc, &g_client) != ESP_OK) { LOGI("usb client register failed"); return; }
    usb_host_transfer_alloc(8 + CTRL_MAX, 0, &g_ctrl);
    link_on_drain(resume);
    xTaskCreatePinnedToCore(hostTask, "usb_host", 8192, nullptr, 5, nullptr, 1);
    LOGI("USB host started, waiting for a device");
}
#endif
