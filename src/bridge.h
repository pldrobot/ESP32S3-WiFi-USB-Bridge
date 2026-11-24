#pragma once
#include <stdint.h>

// Role-specific USB side (usb_host.cpp = USB host, usb_device.cpp = USB device).
void usb_begin();
bool usb_present();    // host: peripheral plugged in. device: the host reports a peripheral.
void usb_poll();                                                 // Arduino loop task, every ~20 ms
void usb_on_msg(uint8_t type, const uint8_t* p, uint16_t len);   // link rx task

// IN flow control: each IN record costs its length plus this much, on both ends,
// so a flood of tiny packets cannot outgrow the RX buffer.
static inline uint16_t in_cost(uint16_t n) { return n + 16; }
