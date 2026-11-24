#pragma once
#include <Arduino.h>

#if defined(ROLE_TX) == defined(ROLE_RX)
#error "Define exactly one of ROLE_TX (USB host side) or ROLE_RX (USB device side)"
#endif

#ifndef PAIR_KEY
#define PAIR_KEY 0x50444C31u
#endif

// ESP-NOW v2 payload limit is 1470; keep room for the frame header.
#define LINK_MTU          1400
#define LINK_RATE         WIFI_PHY_RATE_36M   // data frames; beacons use the default low rate
#define LINK_TIMEOUT_MS   2000                // no frame from the peer for this long = link down
#define LINK_PING_MS      400
#define LINK_RETRIES      6                   // app-level resends of one frame after MAC retries failed
#define CH_MAX            13

// Channel hopping (TX decides)
#define HOP_FAIL_PCT      15                  // % failed frames in a 2 s window that triggers a rescan
#define HOP_MIN_FRAMES    30
#define HOP_COOLDOWN_MS   20000
#define HOP_MIN_GAIN      150                 // scan score the new channel must beat the current one by

// Flow control
#define IN_WINDOW         4096                // bytes of IN data the TX may have unacknowledged per endpoint
#define OUT_CREDITS       4                   // OUT packets the RX may have unacknowledged per endpoint

#define LOGI(fmt, ...) Serial.printf("[%lu] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
