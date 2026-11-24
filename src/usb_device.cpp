// RX role: USB device. Plugged into the PC; presents the descriptors of the peripheral that is
// attached to the TX and forwards its traffic.
//
// A single custom TinyUSB class driver owns every interface of the mirrored device. It opens the
// endpoints named in the TX's configuration descriptor and relays:
//   control (interface/endpoint/vendor) -> TX, answered by the real device (blocking, ~1 RTT)
//   IN  data  : TX streams packets ahead, buffered here; every PC poll takes the next packet
//   OUT data  : one packet at a time to the TX, paced by credits
// Standard device requests (SET_ADDRESS, SET_CONFIGURATION, GET_DESCRIPTOR config) are TinyUSB's own.
#include "config.h"
#ifdef ROLE_DEVICE
#include "bridge.h"
#include "link.h"
#include "proto.h"
#include "esp32-hal-tinyusb.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include <atomic>
#include <freertos/message_buffer.h>

#define CTRL_MAX 1024

struct REp {
    uint8_t addr = 0;
    bool in = false, open = false, haveOut = false;
    uint16_t mps = 0, outLen = 0, owed = 0, cost = 0;
    int credits = 0;
    MessageBufferHandle_t mb = nullptr;                 // IN: [0][packet] records from the TX
    uint8_t rb[72];
    uint8_t ibuf[64] __attribute__((aligned(4)));
    uint8_t obuf[64] __attribute__((aligned(4)));
};
static REp g_rep[32];
static inline REp* repOf(uint8_t a) { return &g_rep[(a & 0x0F) | ((a & 0x80) ? 0x10 : 0)]; }

// Mirrored descriptors. g_pend* is written by the link task, applied by usb_poll().
static uint8_t g_dev[18], g_cfg[1024];
static uint16_t g_cfgLen;
static uint8_t g_pendDev[18], g_pendCfg[1024];
static uint16_t g_pendLen;
static volatile bool g_attachPending, g_attached, g_started, g_connected;
static volatile uint32_t g_linkDownT;
static std::atomic<bool> g_pumpQueued{false};

// Control relay
static SemaphoreHandle_t g_ctrlSem;
static uint16_t g_ctrlId;
static volatile bool g_rspOk;
static int g_rspLen;
static uint8_t g_rsp[CTRL_MAX] __attribute__((aligned(4)));
static uint8_t g_cbuf[CTRL_MAX] __attribute__((aligned(4)));

// ── Pump: runs on the usbd task ──────────────────────────────────────────────
static void flushCredit(REp* e) {
    if (!e->owed) return;
    uint8_t b[3] = {e->addr, (uint8_t)e->owed, (uint8_t)(e->owed >> 8)};
    if (link_send(M_CREDIT, b, 3)) e->owed = 0;
}

static void pumpEp(REp* e) {
    if (!e->open) return;
    if (e->in) {
        flushCredit(e);
        if (usbd_edpt_busy(0, e->addr)) return;
        size_t n = xMessageBufferReceive(e->mb, e->rb, sizeof e->rb, 0);
        if (!n) return;
        memcpy(e->ibuf, e->rb + 1, n - 1);
        e->cost = in_cost(n - 1);
        if (usbd_edpt_claim(0, e->addr) && !usbd_edpt_xfer(0, e->addr, e->ibuf, n - 1, false)) usbd_edpt_release(0, e->addr);
    } else {
        if (e->haveOut) {
            if (!link_send(M_OUT, &e->addr, 1, e->obuf, e->outLen)) return;   // queue full: retried on the next pump
            e->haveOut = false;
            e->credits--;
        }
        if (e->credits > 0 && !usbd_edpt_busy(0, e->addr) && usbd_edpt_claim(0, e->addr) &&
            !usbd_edpt_xfer(0, e->addr, e->obuf, e->mps, false))
            usbd_edpt_release(0, e->addr);
    }
}

static void pumpAll(void*) {
    g_pumpQueued = false;
    for (auto& e : g_rep) pumpEp(&e);
}

static void requestPump() {
    if (!g_started) return;                              // usbd queue does not exist yet
    if (!g_pumpQueued.exchange(true)) usbd_defer_func(pumpAll, nullptr, false);
}

// ── Control relay ────────────────────────────────────────────────────────────
// Sends [setup 8][out data] to the TX and waits for the real device's answer in g_rsp.
static bool forward(const uint8_t setup[8], const uint8_t* out, uint16_t outLen) {
    if (!link_up()) return false;
    uint16_t id = ++g_ctrlId;
    uint8_t h[10] = {(uint8_t)id, (uint8_t)(id >> 8)};
    memcpy(h + 2, setup, 8);
    xSemaphoreTake(g_ctrlSem, 0);
    if (!link_send(M_CTRL_REQ, h, 10, out, outLen)) return false;
    if (xSemaphoreTake(g_ctrlSem, pdMS_TO_TICKS(1500)) != pdTRUE) { LOGI("control request timed out"); return false; }
    return g_rspOk;
}

static bool handleControl(uint8_t rhport, uint8_t stage, tusb_control_request_t const* r) {
    bool in = r->bmRequestType_bit.direction == TUSB_DIR_IN;
    uint16_t wl = r->wLength;
    if (wl > CTRL_MAX) return false;
    if (stage == CONTROL_STAGE_SETUP) {
        if (r->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD && r->bRequest == TUSB_REQ_SET_INTERFACE)
            return r->wValue == 0 && tud_control_status(rhport, r);         // alternate settings > 0 are not mirrored
        if (in) return forward((const uint8_t*)r, nullptr, 0) && tud_control_xfer(rhport, r, g_rsp, min<int>(g_rspLen, wl));
        if (!wl) return forward((const uint8_t*)r, nullptr, 0) && tud_control_status(rhport, r);
        return tud_control_xfer(rhport, r, g_cbuf, wl);                     // receive the OUT data first
    }
    if (stage == CONTROL_STAGE_DATA && !in && wl) return forward((const uint8_t*)r, g_cbuf, wl);
    return true;
}

extern "C" bool tinyusb_vendor_control_request_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* r) {
    return handleControl(rhport, stage, r);
}

extern "C" uint8_t const* tud_descriptor_device_cb(void) { return g_dev; }
extern "C" uint8_t const* tud_descriptor_configuration_cb(uint8_t) { return g_cfg; }

// String descriptors are fetched from the real device on demand (the reply is already UTF-16 with its header).
extern "C" uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t buf[128];
    const uint8_t setup[8] = {0x80, 0x06, index, 0x03, (uint8_t)langid, (uint8_t)(langid >> 8), 254, 0};
    if (!forward(setup, nullptr, 0) || g_rspLen < 2) return nullptr;
    memcpy(buf, g_rsp, min<int>(g_rspLen, 254));
    return buf;
}

// ── Class driver ─────────────────────────────────────────────────────────────
static void drvInit(void) {}
static bool drvDeinit(void) { return true; }
static void drvReset(uint8_t) {
    for (auto& e : g_rep) { e.open = false; e.haveOut = false; }
}

// Called by TinyUSB per interface (or per IAD group) while applying SET_CONFIGURATION. Consumes every
// interface of the function, all alternate settings, and opens the endpoints of alternate setting 0.
static uint16_t drvOpen(uint8_t rhport, tusb_desc_interface_t const* itf, uint16_t) {
    const uint8_t* p = (const uint8_t*)itf;
    if (p < g_cfg || p >= g_cfg + g_cfgLen) return 0;
    int start = p - g_cfg, count = 1;
    if (start >= 8 && p[-8] == 8 && p[-7] == TUSB_DESC_INTERFACE_ASSOCIATION) count = p[-5];

    int seen = 0, o = start;
    bool alt0 = true;
    while (o + 2 <= g_cfgLen && g_cfg[o]) {
        const uint8_t* d = g_cfg + o;
        if (d[1] == TUSB_DESC_INTERFACE) {
            alt0 = d[3] == 0;
            if (alt0 && ++seen > count) break;
        } else if (d[1] == TUSB_DESC_ENDPOINT && alt0) {
            auto* ed = (tusb_desc_endpoint_t const*)d;
            if (ed->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS) {
                REp* e = repOf(ed->bEndpointAddress);
                if (!usbd_edpt_open(rhport, ed)) LOGI("cannot open ep 0x%02X", ed->bEndpointAddress);
                else {
                    e->addr = ed->bEndpointAddress;
                    e->in = (e->addr & 0x80) != 0;
                    e->mps = tu_edpt_packet_size(ed);
                    e->open = true;
                    e->haveOut = false;
                    e->credits = OUT_CREDITS;
                }
            }
        }
        o += d[0];
    }
    requestPump();
    return o - start;
}

static bool drvControl(uint8_t rhport, uint8_t stage, tusb_control_request_t const* r) {
    bool ok = handleControl(rhport, stage, r);
    // The real device cleared its halt; do the same on our side of the endpoint.
    if (ok && stage == CONTROL_STAGE_SETUP && r->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT &&
        r->bRequest == TUSB_REQ_CLEAR_FEATURE)
        usbd_edpt_clear_stall(rhport, (uint8_t)r->wIndex);
    return ok;
}

static bool drvXfer(uint8_t, uint8_t ep, xfer_result_t res, uint32_t n) {
    REp* e = repOf(ep);
    if (e->in) e->owed += e->cost;
    else if (res == XFER_RESULT_SUCCESS) { e->outLen = n; e->haveOut = true; }
    pumpEp(e);
    return true;
}

static const usbd_class_driver_t k_drv = {
    .name = "bridge", .init = drvInit, .deinit = drvDeinit, .reset = drvReset, .open = drvOpen,
    .control_xfer_cb = drvControl, .xfer_cb = drvXfer, .xfer_isr = nullptr, .sof = nullptr};

extern "C" usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* count) {
    *count = 1;
    return &k_drv;
}

// ── Link messages (link rx task) ─────────────────────────────────────────────
void usb_on_msg(uint8_t type, const uint8_t* p, uint16_t len) {
    switch (type) {
    case M_LINK_UP:
        for (auto& e : g_rep) if (!e.in) e.credits = OUT_CREDITS;
        link_send(M_REQ_ATTACH);
        requestPump();
        break;
    case M_LINK_DOWN:
        g_linkDownT = millis();
        break;
    case M_ATTACH:
        if (len >= 18 + 9 && len - 18 <= sizeof g_pendCfg && !g_attachPending) {
            memcpy(g_pendDev, p, 18);
            memcpy(g_pendCfg, p + 18, len - 18);
            g_pendLen = len - 18;
            g_attachPending = true;
        }
        break;
    case M_DETACH:
        g_attached = false;
        break;
    case M_IN: {
        if (len < 1) break;
        REp* e = repOf(p[0]);
        uint8_t tmp[72];
        bool ok = e->mb && len - 1 <= 64;
        if (ok) {
            tmp[0] = 0;
            memcpy(tmp + 1, p + 1, len - 1);
            ok = xMessageBufferSend(e->mb, tmp, len, 0) == len;
        }
        if (ok) requestPump();
        else {                                                           // no room / unknown ep: hand the credit back
            uint16_t c = in_cost(len - 1);
            uint8_t b[3] = {p[0], (uint8_t)c, (uint8_t)(c >> 8)};
            link_send(M_CREDIT, b, 3);
        }
        break;
    }
    case M_CREDIT:
        if (len >= 3) {
            REp* e = repOf(p[0]);
            if (!e->in) e->credits = min<int>(OUT_CREDITS, e->credits + (p[1] | (p[2] << 8)));
            requestPump();
        }
        break;
    case M_CTRL_RSP:
        if (len >= 3 && (p[0] | (p[1] << 8)) == g_ctrlId) {
            g_rspOk = p[2];
            g_rspLen = min<int>(len - 3, CTRL_MAX);
            memcpy(g_rsp, p + 3, g_rspLen);
            xSemaphoreGive(g_ctrlSem);
        }
        break;
    }
}

// ── Connection management (Arduino loop task) ────────────────────────────────
// Creates the IN buffers for the endpoints of the new configuration.
static void prepareEndpoints() {
    for (int o = 0; o + 2 <= g_cfgLen && g_cfg[o]; o += g_cfg[o]) {
        if (g_cfg[o + 1] != TUSB_DESC_ENDPOINT) continue;
        auto* ed = (tusb_desc_endpoint_t const*)(g_cfg + o);
        REp* e = repOf(ed->bEndpointAddress);
        if ((ed->bEndpointAddress & 0x80) && !e->mb) e->mb = xMessageBufferCreate(2 * IN_WINDOW + 64);
        if (e->mb) xMessageBufferReset(e->mb);
        e->owed = 0;
    }
}

void usb_poll() {
    if (g_attachPending) {
        bool same = g_attached && g_pendLen == g_cfgLen && !memcmp(g_pendDev, g_dev, 18) && !memcmp(g_pendCfg, g_cfg, g_cfgLen);
        if (!same) {
            if (g_connected) { tud_disconnect(); g_connected = false; delay(500); }   // let the PC see the old device leave
            memcpy(g_dev, g_pendDev, 18);
            memcpy(g_cfg, g_pendCfg, g_pendLen);
            g_cfgLen = g_pendLen;
            g_dev[7] = 64;                                  // our ep0 is 64 bytes
            if (g_dev[3] > 0x02 || (g_dev[3] == 0x02 && g_dev[2] > 0)) { g_dev[2] = 0; g_dev[3] = 0x02; }   // no BOS: cap at USB 2.0
            prepareEndpoints();
        }
        g_attached = true;
        g_attachPending = false;
    }
    bool want = g_attached && g_cfgLen && (link_up() || millis() - g_linkDownT < 10000);
    if (want && !g_connected) {
        if (!g_started) {
            // Strings/VID/PID come from the mirrored descriptors; this only feeds the HAL's unused defaults.
            tinyusb_device_config_t cfg = {};
            cfg.vid = g_dev[9] << 8 | g_dev[8];
            cfg.pid = g_dev[11] << 8 | g_dev[10];
            cfg.usb_version = 0x0200;
            cfg.usb_attributes = TUSB_DESC_CONFIG_ATT_SELF_POWERED;
            cfg.usb_power_ma = 500;
            g_started = tinyusb_init(&cfg) == ESP_OK;
            LOGI("USB device %s: %04X:%04X", g_started ? "started" : "start FAILED", g_dev[9] << 8 | g_dev[8], g_dev[11] << 8 | g_dev[10]);
        } else tud_connect();
        g_connected = g_started;
    } else if (!want && g_connected) {
        tud_disconnect();
        g_connected = false;
        LOGI("USB device disconnected from the PC");
    }
}

bool usb_present() { return g_attached; }

void usb_begin() {
    g_ctrlSem = xSemaphoreCreateBinary();
    link_on_drain(requestPump);
    LOGI("USB device side waiting for the TX to report a peripheral");
}
#endif
