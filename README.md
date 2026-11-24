# ESP32S3-WiFi-USB-Bridge

Extends a USB device over WiFi using two ESP32-S3 boards and no router.

```
 [USB peripheral] -- ESP "TX" (USB host)  ~~ ESP-NOW ~~  ESP "RX" (USB device) -- [PC]
```

- **TX** is the USB host: plug the real peripheral into its OTG port.
- **RX** is the USB device: plug it into the PC. It copies the peripheral's descriptors and relays
  control, bulk and interrupt traffic, so the PC sees the original device (full speed, 12 Mbps bus on both ends).
- One code base; the role is picked by the PlatformIO environment.

| env | board | role |
|-----|-------|------|
| `n16r8_tx` / `n16r8_rx` | ESP32-S3-N16R8 DevKitC-1 | host / device |
| `supermini_tx` / `supermini_rx` | ESP32-S3 Super Mini (4 MB) | host / device |

Use the **OTG** USB-C port for the USB side. Logs are on UART0 (GPIO43/44, 115200), not USB.
On the Super Mini the single USB-C port is the OTG port: hold BOOT while plugging in to re-flash.

## Pairing and channel

No buttons. TX scans for the quietest 2.4 GHz channel at boot and broadcasts a beacon on it. RX hops
channels 1-13 until it hears a beacon with the same `PAIR_KEY` (platformio.ini), replies, and the link is up.
Change `PAIR_KEY` to keep several bridges apart. If the link drops, RX goes back to scanning.

While running, TX counts failed frames. Above 15 % over 2 s it rescans, and if a clearly quieter channel
exists it tells RX to switch at a countdown instant (at most once every 20 s).

## Limits

- Bulk and interrupt endpoints only: no isochronous (audio/video), no hubs, no alternate settings > 0.
- Throughput is whatever ESP-NOW gives (a few Mbps), not 12 Mbps; fine for serial, HID, small bulk devices.
- ESP-NOW traffic is not encrypted.
- Control requests cost one WiFi round trip each.
