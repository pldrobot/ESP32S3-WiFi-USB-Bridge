#include "link.h"
#include "config.h"
#include "proto.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/message_buffer.h>
#include <atomic>

#ifdef ROLE_TX
static const bool IS_TX = true;
#else
static const bool IS_TX = false;
#endif

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint32_t KEY = PAIR_KEY;

static LinkMsgCb s_cb;
static void (*s_drain)();
static uint8_t s_peer[6];
static volatile bool s_up;
static volatile uint8_t s_chan = 1;
static MessageBufferHandle_t s_txq, s_rxq;
static SemaphoreHandle_t s_qMtx, s_sendMtx, s_done;
static volatile bool s_sendOk;
static volatile uint32_t s_lastRx, s_lastTx;
static uint32_t s_nSent, s_nFail;      // stats only; a racy count is fine

// Sliding window (go-back-N). Frames are numbered 0,1,2..; the wire carries (n % 255) + 1, 0 = ack-only.
// Every frame also carries the receiver's cumulative ack, so delivery is confirmed by the peer, not the MAC.
#define WIN     8
#define MAX_OUT 4                       // ESP-NOW sends outstanding at the radio
#define RTO_MS  30
static uint8_t s_fr[WIN][LINK_MTU];
static uint16_t s_frLen[WIN];
static volatile uint32_t s_sn, s_acked; // frames built / acknowledged by the peer
static uint32_t s_snd;                  // next frame to (re)transmit
static uint32_t s_rn;                   // frames accepted in order from the peer
static std::atomic<int> s_outCb{0};     // sends whose callback has not run yet
static volatile bool s_raw, s_ackPending, s_rewind;
static volatile uint32_t s_gen;         // bumped on every link-up so the sender restarts its window
static inline uint8_t wire(uint32_t n) { return n % 255 + 1; }
static inline uint8_t ackWire() { return s_rn ? wire(s_rn - 1) : 0; }

// RX-side pairing state
enum { ST_SCAN, ST_HELLO };
static int s_state = ST_SCAN;
static uint32_t s_stateT;
static uint8_t s_hopCh;
static uint32_t s_hopAt;                      // RX: millis() at which to follow a CHSW (0 = none)
static uint32_t s_lastHop;                    // TX

static void setChannel(uint8_t ch) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    s_chan = ch;
}

static bool sameMac(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

// ── ESP-NOW plumbing ─────────────────────────────────────────────────────────
static void onSent(const esp_now_send_info_t*, esp_now_send_status_t st) {
    s_sendOk = (st == ESP_NOW_SEND_SUCCESS);
    s_outCb--;
    if (s_raw) xSemaphoreGive(s_done);
    else if (!s_sendOk) { s_nFail++; s_rewind = true; }     // a data frame failed: resend the window
}

// WiFi task: only copy [src mac][payload] out.
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* d, int n) {
    if (n < 1 || n > 1500) return;
    uint8_t tmp[6 + 1500];
    memcpy(tmp, info->src_addr, 6);
    memcpy(tmp + 6, d, n);
    xMessageBufferSend(s_rxq, tmp, 6 + n, 0);
}

// Control frames (beacon, hello, channel switch): drains the data frames first, then sends one and
// waits for its result. True once the peer's MAC acknowledged it.
static bool rawSend(const uint8_t* mac, const uint8_t* d, size_t n) {
    xSemaphoreTake(s_sendMtx, portMAX_DELAY);
    for (uint32_t t = millis(); s_outCb > 0 && millis() - t < 200;) vTaskDelay(1);
    xSemaphoreTake(s_done, 0);
    s_raw = true;
    s_outCb++;
    bool ok = false;
    if (esp_now_send(mac, d, n) == ESP_OK) ok = xSemaphoreTake(s_done, pdMS_TO_TICKS(100)) == pdTRUE && s_sendOk;
    else s_outCb--;
    s_raw = false;
    xSemaphoreGive(s_sendMtx);
    return ok;
}

static void addPeer(const uint8_t* mac, bool fast) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.ifidx = WIFI_IF_STA;
    p.channel = 0;                            // follow the current channel
    if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
    esp_now_add_peer(&p);
    if (fast) {
        esp_now_rate_config_t rc = {WIFI_PHY_MODE_11G, LINK_RATE, false, false};
        esp_now_set_peer_rate_config(mac, &rc);
    }
}

static void emit(uint8_t t) { if (s_cb) s_cb(t, nullptr, 0); }

static void linkUp(const uint8_t* mac) {
    memcpy(s_peer, mac, 6);
    addPeer(mac, true);
    xMessageBufferReset(s_txq);
    s_rn = 0;
    s_ackPending = false;
    s_gen++;
    s_lastRx = s_lastTx = millis();
    s_nSent = s_nFail = 0;
    s_lastHop = millis();
    s_up = true;
    LOGI("link UP, peer %02X:%02X:%02X:%02X:%02X:%02X, channel %u", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], s_chan);
    emit(M_LINK_UP);
}

static void linkDown(const char* why) {
    if (!s_up) return;
    s_up = false;
    esp_now_del_peer(s_peer);
    s_state = ST_SCAN;
    s_stateT = millis();
    LOGI("link DOWN (%s)", why);
    emit(M_LINK_DOWN);
}

// ── Channel choice (TX) ──────────────────────────────────────────────────────
// Scores each channel by the Wi-Fi networks overlapping it (weighted by signal); lower is quieter.
static void scanScores(int score[CH_MAX + 1]) {
    for (int i = 0; i <= CH_MAX; i++) score[i] = 0;
    int n = WiFi.scanNetworks(false, true, false, 80);
    for (int i = 0; i < n; i++) {
        int ap = WiFi.channel(i), w = constrain(100 + WiFi.RSSI(i), 1, 100);
        for (int c = 1; c <= CH_MAX; c++) {
            int d = abs(c - ap);
            if (d <= 4) score[c] += w * (5 - d);
        }
    }
    WiFi.scanDelete();
}

static uint8_t bestChannel(const int* score) {
    uint8_t b = 1;
    for (int c = 2; c <= CH_MAX; c++) if (score[c] < score[b]) b = c;
    return b;
}

// Move both ends to `ch`: the countdown in each CHSW lets the RX switch at the same instant.
static void hopTo(uint8_t ch) {
    LOGI("channel %u -> %u", s_chan, ch);
    uint8_t f[8] = {F_CHSW};
    memcpy(f + 1, &KEY, 4);
    f[5] = ch;
    for (int i = 0; i < 5; i++) {
        uint16_t rem = 100 - i * 20;
        memcpy(f + 6, &rem, 2);
        rawSend(s_peer, f, 8);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    setChannel(ch);
}

static void maybeHop() {
    static uint32_t winT;
    if (millis() - winT < 2000) return;
    winT = millis();
    uint32_t sent = s_nSent, fail = s_nFail;
    s_nSent = s_nFail = 0;
    if (!s_up || sent < HOP_MIN_FRAMES || fail * 100 < sent * HOP_FAIL_PCT || millis() - s_lastHop < HOP_COOLDOWN_MS) return;
    s_lastHop = millis();
    LOGI("link quality bad (%u/%u frames failed), scanning", (unsigned)fail, (unsigned)sent);
    int score[CH_MAX + 1];
    scanScores(score);
    setChannel(s_chan);                       // the scan leaves the radio on another channel
    uint8_t best = bestChannel(score);
    if (best != s_chan && score[s_chan] - score[best] >= HOP_MIN_GAIN) hopTo(best);
}

// ── Frame handling (rx task) ─────────────────────────────────────────────────
static bool keyOk(const uint8_t* d) {
    uint32_t k;
    memcpy(&k, d, 4);
    return k == KEY;
}

static void handleFrame(const uint8_t* src, const uint8_t* d, int n) {
    switch (d[0]) {
    case F_BEACON:
        if (!IS_TX && !s_up && s_state == ST_SCAN && n >= 6 && keyOk(d + 1)) {
            setChannel(d[5]);
            addPeer(src, false);
            uint8_t f[5] = {F_HELLO};
            memcpy(f + 1, &KEY, 4);
            memcpy(s_peer, src, 6);
            rawSend(src, f, 5);
            s_state = ST_HELLO;
            s_stateT = millis();
        }
        break;
    case F_HELLO:
        if (IS_TX && n >= 5 && keyOk(d + 1) && (!s_up || sameMac(src, s_peer))) {
            addPeer(src, false);
            uint8_t f[5] = {F_ACCEPT};
            memcpy(f + 1, &KEY, 4);
            rawSend(src, f, 5);
            if (s_up) { s_up = false; emit(M_LINK_DOWN); }   // peer restarted: start a clean session
            linkUp(src);
        }
        break;
    case F_ACCEPT:
        if (!IS_TX && !s_up && s_state == ST_HELLO && n >= 5 && sameMac(src, s_peer) && keyOk(d + 1))
            linkUp(src);
        break;
    case F_CHSW:
        if (!IS_TX && s_up && n >= 8 && sameMac(src, s_peer) && keyOk(d + 1)) {
            uint16_t rem;
            memcpy(&rem, d + 6, 2);
            s_hopCh = d[5];
            s_hopAt = millis() + rem;
        }
        break;
    case F_DATA: {
        if (!s_up || n < 3 || !sameMac(src, s_peer)) break;
        s_lastRx = millis();
        uint8_t a = d[2];
        if (a)                                               // cumulative ack: free what the peer has received
            for (uint32_t m = s_acked; m < s_sn; m++)
                if (wire(m) == a) { s_acked = m + 1; break; }
        if (d[1] == 0) break;                                // ack-only frame
        if (d[1] == wire(s_rn)) {                            // in order; anything else is a resend or follows a gap
            s_rn++;
            int o = 3;
            while (o + 3 <= n) {
                uint16_t len = d[o] | (d[o + 1] << 8);
                if (len < 1 || o + 2 + len > n) break;
                if (d[o + 2] != M_PING && s_cb) s_cb(d[o + 2], d + o + 3, len - 1);
                o += 2 + len;
            }
        }
        s_ackPending = true;                                 // always answer so the sender learns our position
        break;
    }
    }
}

static void housekeeping() {
    uint32_t now = millis();
    if (IS_TX) {
        if (!s_up) {
            static uint32_t t;
            if (now - t >= 100) {
                t = now;
                uint8_t f[6] = {F_BEACON};
                memcpy(f + 1, &KEY, 4);
                f[5] = s_chan;
                rawSend(BCAST, f, 6);
            }
        } else if (now - s_lastRx > LINK_TIMEOUT_MS) linkDown("timeout");
        else maybeHop();
    } else {
        if (s_up) {
            if (s_hopAt && (int32_t)(now - s_hopAt) >= 0) { setChannel(s_hopCh); s_hopAt = 0; s_lastRx = now; }
            if (now - s_lastRx > LINK_TIMEOUT_MS) linkDown("timeout");
        } else if (s_state == ST_HELLO) {
            if (now - s_stateT > 300) { esp_now_del_peer(s_peer); s_state = ST_SCAN; s_stateT = now; }
        } else if (now - s_stateT >= 110) {              // listen ~110 ms per channel; TX beacons every 100 ms
            s_stateT = now;
            setChannel(s_chan % CH_MAX + 1);
        }
    }
}

static void rxTask(void*) {
    static uint8_t buf[6 + 1500];
    for (;;) {
        size_t n = xMessageBufferReceive(s_rxq, buf, sizeof buf, pdMS_TO_TICKS(10));
        if (n >= 7) handleFrame(buf, buf + 6, n - 6);
        housekeeping();
    }
}

static bool radioSend(const uint8_t* d, size_t n) {
    xSemaphoreTake(s_sendMtx, portMAX_DELAY);
    s_outCb++;
    bool ok = esp_now_send(s_peer, d, n) == ESP_OK;
    if (!ok) s_outCb--;
    xSemaphoreGive(s_sendMtx);
    return ok;
}

// Packs queued records into numbered frames and keeps up to WIN of them in flight. Nothing waits for a
// callback: frames are freed by the peer's cumulative ack and resent from the oldest unacked one on a
// MAC failure or after RTO_MS without progress.
static void sendTask(void*) {
    uint32_t gen = 0, lastSendT = 0;
    for (;;) {
        if (!s_up) { xMessageBufferReset(s_txq); vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (gen != s_gen) { gen = s_gen; s_sn = s_acked = s_snd = 0; s_rewind = false; }
        uint32_t now = millis();

        if (s_sn - s_acked < WIN) {                          // room in the window: build the next frame
            uint8_t* fr = s_fr[s_sn % WIN];
            size_t used = 3;
            size_t r = xMessageBufferReceive(s_txq, fr + used + 2, LINK_MTU - used - 2, 0);
            if (r == 0 && s_acked == s_sn && now - s_lastTx >= LINK_PING_MS) { fr[used + 2] = M_PING; r = 1; }
            if (r) {
                for (;;) {
                    fr[used] = r & 0xFF;
                    fr[used + 1] = r >> 8;
                    used += 2 + r;
                    r = xMessageBufferReceive(s_txq, fr + used + 2, LINK_MTU - used - 2, 0);
                    if (r == 0) break;                       // empty, or the next record does not fit
                }
                fr[0] = F_DATA;
                fr[1] = wire(s_sn);
                s_frLen[s_sn % WIN] = used;
                s_sn++;
                if (s_drain) s_drain();
            }
        }

        if (s_snd < s_acked) s_snd = s_acked;
        if (s_rewind || (s_acked < s_sn && s_snd == s_sn && now - lastSendT > RTO_MS)) {
            s_rewind = false;
            s_snd = s_acked;
            lastSendT = now;
        }

        bool did = false;
        if (s_snd < s_sn && s_outCb < MAX_OUT) {
            uint8_t* fr = s_fr[s_snd % WIN];
            fr[2] = ackWire();
            if (radioSend(fr, s_frLen[s_snd % WIN])) {
                s_snd++;
                s_nSent++;
                s_ackPending = false;
                lastSendT = s_lastTx = now;
                did = true;
            }
        } else if (s_ackPending && s_outCb < MAX_OUT) {
            uint8_t a[3] = {F_DATA, 0, ackWire()};
            s_ackPending = false;
            did = radioSend(a, 3);
        }
        if (!did) vTaskDelay(1);
    }
}

// ── Public API ───────────────────────────────────────────────────────────────
void link_begin(LinkMsgCb cb) {
    s_cb = cb;
    s_txq = xMessageBufferCreate(16 * 1024);
    s_rxq = xMessageBufferCreate(16 * 1024);
    s_qMtx = xSemaphoreCreateMutex();
    s_sendMtx = xSemaphoreCreateMutex();
    s_done = xSemaphoreCreateBinary();

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);

    uint8_t ch = 1;
    if (IS_TX) {
        int score[CH_MAX + 1];
        scanScores(score);
        ch = bestChannel(score);
    }
    setChannel(ch);
    if (esp_now_init() != ESP_OK) { LOGI("esp_now_init failed"); return; }
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);
    if (IS_TX) addPeer(BCAST, false);
    LOGI("%s link ready on channel %u, MAC %s", IS_TX ? "TX" : "RX", s_chan, WiFi.macAddress().c_str());

    xTaskCreatePinnedToCore(rxTask, "link_rx", 6144, nullptr, 6, nullptr, 0);
    xTaskCreatePinnedToCore(sendTask, "link_tx", 4096, nullptr, 6, nullptr, 0);
}

bool link_up() { return s_up; }
uint8_t link_channel() { return s_chan; }
void link_on_drain(void (*cb)()) { s_drain = cb; }
size_t link_free() { return xMessageBufferSpacesAvailable(s_txq); }

bool link_send(uint8_t type, const void* a, uint16_t la, const void* b, uint16_t lb) {
    size_t total = 1 + la + lb;
    if (!s_up || total > LINK_MTU - 8) return false;
    uint8_t tmp[LINK_MTU];
    tmp[0] = type;
    if (la) memcpy(tmp + 1, a, la);
    if (lb) memcpy(tmp + 1 + la, b, lb);
    bool ok = false;
    if (xSemaphoreTake(s_qMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        ok = xMessageBufferSend(s_txq, tmp, total, 0) == total;
        xSemaphoreGive(s_qMtx);
    }
    return ok;
}
