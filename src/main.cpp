#include <Arduino.h>
#include "config.h"
#include "bridge.h"
#include "link.h"
#include "led.h"

void setup() {
    Serial.begin(115200);
    delay(300);
#ifdef ROLE_HOST
    LOGI("ESP32-S3 WiFi USB Bridge: HOST (USB host, peripheral side)");
#else
    LOGI("ESP32-S3 WiFi USB Bridge: DEVICE (USB device, PC side)");
#endif
    usb_begin();                      // before the link: the link callbacks use its state
    link_begin(usb_on_msg);
}

void loop() {
    usb_poll();
    link_poll_button();
    led_poll();
    static uint32_t t;
    if (millis() - t >= 10000) {
        t = millis();
        LOGI("link %s, channel %u, free heap %u", link_up() ? "up" : "down", link_channel(), (unsigned)ESP.getFreeHeap());
    }
    delay(20);
}
