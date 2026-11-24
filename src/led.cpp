#include <Arduino.h>
#include "led.h"
#include "link.h"
#include "bridge.h"

// Board LED, chosen by build flag: LED_WS2812_PIN (addressable) or LED_R/LED_G/LED_B (plain RGB, optional LED_ACTIVE_LOW).
//   Available to pair   blue  slow blink
//   Pairing             blue  fast blink
//   Searching (paired)  green slow blink
//   Connected           green solid
//   Connected, no USB   red   solid
static void show(bool r, bool g, bool b) {
#if defined(LED_WS2812_PIN)
    rgbLedWrite(LED_WS2812_PIN, r * 32, g * 32, b * 32);
#elif defined(LED_R) && defined(LED_G) && defined(LED_B)
    static bool init;
    if (!init) { init = true; pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT); }
#ifdef LED_ACTIVE_LOW
    r = !r; g = !g; b = !b;
#endif
    digitalWrite(LED_R, r); digitalWrite(LED_G, g); digitalWrite(LED_B, b);
#endif
}

void led_poll() {
    uint32_t t = millis();
    bool slow = (t / 500) & 1, fast = (t / 100) & 1;
    switch (link_state()) {
    case LS_AVAILABLE: show(0, 0, slow); break;
    case LS_PAIRING:   show(0, 0, fast); break;
    case LS_SEARCHING: show(0, slow, 0); break;
    default:           usb_present() ? show(0, 1, 0) : show(1, 0, 0); break;
    }
}
