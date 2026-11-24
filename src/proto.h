#pragma once
#include <stdint.h>

// Frame types (first byte of every ESP-NOW payload)
enum : uint8_t {
    F_BEACON = 0xB0,   // TX -> broadcast  [type][channel][paired u8]
    F_HELLO  = 0xB1,   // RX -> TX         [type]
    F_ACCEPT = 0xB2,   // TX -> RX         [type]
    F_CHSW   = 0xB3,   // TX -> RX         [type][channel][remaining ms u16]
    F_DATA   = 0xD0,   // both             [type][seq] then records
};

// Records inside F_DATA: [len u16 = 1 + payload][type][payload]
enum : uint8_t {
    M_PING = 1,
    M_REQ_ATTACH,   // RX -> TX : (re)send the descriptors
    M_ATTACH,       // TX -> RX : [devdesc 18][cfgdesc ...]
    M_DETACH,       // TX -> RX
    M_IN,           // TX -> RX : [ep][data]            (len 0 = zero-length packet)
    M_OUT,          // RX -> TX : [ep][data]
    M_CREDIT,       // both     : [ep][n u16]           IN ep: bytes consumed, OUT ep: packets completed
    M_CTRL_REQ,     // RX -> TX : [id u16][setup 8][data for an OUT data stage]
    M_CTRL_RSP,     // TX -> RX : [id u16][ok u8][data for an IN data stage]

    M_LINK_UP   = 0xF1,   // local only, delivered to the message callback
    M_LINK_DOWN = 0xF2,
};
