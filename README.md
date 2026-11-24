# WiFi Direct Bridge

Share a USB device over WiFi, without a router or network infrastructure.

Two ESP32-S3 boards communicate directly over ESP-NOW. The USB device plugs into one board (the host) and the
other board (the device) plugs into the PC, which then sees the original device as if it were connected locally.

```
 [USB peripheral] ── Host board  ~~ ESP-NOW ~~  Device board ── [PC]
```

| Role | Function |
|------|----------|
| Host | Acts as the USB host. The peripheral connects to its OTG port. |
| Device | Acts as a USB device. It mirrors the peripheral's descriptors and relays control, bulk and interrupt transfers to it. |

Both boards run at USB full speed (12 Mbps).

## Supported devices

Both modes support:

- HID: keyboards, mice, gamepads, barcode scanners
- CDC serial and USB-serial adapters (FTDI, CP210x, CH340), including programmers and debug probes
- Printers and other vendor-specific devices

Not supported: isochronous devices (webcams, audio) and hubs. These device classes are expected to work but have not yet been tested.

## Hardware

- ESP32-S3-N16R8 DevKitC-1
- ESP32-S3 Super Mini (4 MB)

Connect the USB side to the board's **OTG** USB-C port. Both boards use the on-board WS2812 LED on GPIO48.
Firmware logs are written to UART0 (GPIO43/44, 115200 baud), not over USB.

On the Super Mini the single USB-C port is the OTG port. Hold BOOT while plugging in to re-flash.

## Build and flash

Requires [PlatformIO](https://platformio.org/). Set `upload_port` and `monitor_port` in `platformio.ini`, then:

```
pio run -e host_n16r8 -t upload
pio run -e host_supermini -t upload
pio run -e device_n16r8 -t upload
pio run -e device_supermini -t upload
```

Serial monitor (replace the environment as needed):

```
pio device monitor -e host_n16r8
```

## Pairing

A new or erased board starts in pairing mode. The host selects the quietest 2.4 GHz channel and advertises on it.
The device scans channels 1-13 and pairs with the strongest (nearest) unpaired host. Both boards then store the
peer's MAC address in NVS and only communicate with each other. If the link drops, the device resumes scanning but
accepts only its paired host.

To re-pair, hold **BOOT (GPIO0) for 3 seconds on both boards**, then release. NVS is erased and the board reboots
into pairing mode. Keep both boards close together and away from other unpaired bridges while pairing.

## LED status

| Status | LED |
|--------|-----|
| Available to pair | Blue, slow blink |
| Pairing | Blue, fast blink |
| Searching (paired, peer not found) | Green, slow blink |
| Connected | Green, solid |
| Connected, no USB device | Red, solid |

## Channel management

The host monitors the failure rate of transmitted frames. If more than 15 % fail over a 2-second window, it scans
for a quieter channel and, if one is clearly better, instructs the device to switch at a synchronised instant
(at most once every 20 seconds).

## Limitations

- Control, bulk and interrupt endpoints only: no isochronous transfers, hubs or alternate settings above 0.
- Throughput is limited by ESP-NOW (8-frame sliding window, 4 sends outstanding at the radio), not by USB full speed. It has not been measured yet.
- ESP-NOW traffic is not encrypted.
- Each control request costs one WiFi round trip.

## TODO

- Mass storage (flash drives, card readers) does not work yet. Investigate and fix.
