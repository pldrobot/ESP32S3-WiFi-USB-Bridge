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
size_t   link_free();                       // bytes left in the send queue
void     link_on_drain(void (*cb)());       // called after each frame left the queue (sender task)
