#pragma once
#include <stdint.h>
#include <stddef.h>

// Ordered record pipe between the two ESPs (ESP-NOW, auto pairing, channel hopping).
// The message callback runs on the link rx task: it must not block for long.
typedef void (*LinkMsgCb)(uint8_t type, const uint8_t* p, uint16_t len);

void     link_begin(LinkMsgCb cb);
bool     link_up();
uint8_t  link_channel();
// Queue one record [type][a][b]. False when the link is down or the queue is full.
bool     link_send(uint8_t type, const void* a = nullptr, uint16_t la = 0, const void* b = nullptr, uint16_t lb = 0);
enum LinkState { LS_AVAILABLE, LS_PAIRING, LS_SEARCHING, LS_CONNECTED };
LinkState link_state();
void     link_poll_button();                // call from loop(): hold BOOT 3 s = erase NVS + reboot (re-pair)
size_t   link_free();                        // bytes left in the send queue
void     link_on_drain(void (*cb)());       // called after each frame left the queue (sender task)
