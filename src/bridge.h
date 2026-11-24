#pragma once
#include <stdint.h>

// Role-specific USB side (usb_tx.cpp = USB host, usb_rx.cpp = USB device).
void usb_begin();
void usb_poll();                                                 // Arduino loop task, every ~20 ms
void usb_on_msg(uint8_t type, const uint8_t* p, uint16_t len);   // link rx task

// IN flow control: each IN record costs its length plus this much, on both ends,
// so a flood of tiny packets cannot outgrow the RX buffer.
static inline uint16_t in_cost(uint16_t n) { return n + 16; }
