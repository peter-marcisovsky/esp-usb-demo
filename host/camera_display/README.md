| Supported Targets | ESP32-P4 |
| ----------------- | -------- |

# Dual USB Camera and display demo

## Overview

This demo is using a following (sub)components:
- [USB Host stack](https://github.com/espressif/esp-idf/tree/master/components/usb)
- [USB UVC driver](https://github.com/espressif/esp-usb/tree/master/host/class/uvc/usb_host_uvc)
- [External USB Hub driver](https://github.com/espressif/esp-idf/tree/master/components/usb)
- [Hardware JPEG Decoder](https://github.com/espressif/esp-idf/tree/master/components/esp_driver_jpeg)
- [PPA (Per Pixel Accelerator)](https://github.com/espressif/esp-idf/tree/master/components/esp_driver_ppa)
- [LCD Display driver](https://github.com/espressif/esp-iot-solution/tree/master/components/display/lcd/esp_lcd_ek79007)


Up to 2 USB Cameras, connected via an external USB hub into a HS USB port of [ESP32-P4-Function-EV-Board](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/index.html), are streaming JPEG encoded video frames over USB Bus.

`The ESP32-P4:`
- Acting as HS USB Host
- Receives video streams from both cameras
- Decodes the JPEG decoded frames to RG565 bit array, using Hardware JPEG Decoder
- Rotates the RGB565 bit arrays in 90 degrees (if needed) using PPA
- Finally, the both images are drawn to a display

## Console control

User can interact with the demo using console.

```
esp32p4> help
help  [<string>] [-v <0|1>]
  Print the summary of all registered commands if no arguments are given,
  otherwise print summary of given command.
      <string>  Name of command
  -v, --verbose=<0|1>  If specified, list console commands with given verbose level

stream  [-S <stream_id>] [-T <stream_id>]
  Video stream control
  -S, --start=<stream_id>  Start video stream
  -T, --stop=<stream_id>  Stop video stream
```

User can start or stop a specific video stream

```
esp32p4> stream -T 0
I (92728) esp-usb-demo: Console: stream stop
I (92828) esp-usb-demo: Stream 0 stopped
esp32p4> stream -S 0
I (97698) esp-usb-demo: Console: stream start
I (97738) esp-usb-demo: Stream 0 started
esp32p4> stream -S 0
I (102228) esp-usb-demo: Console: stream start
W (102238) esp-usb-demo: Stream 0: already streaming
```

## Demo configuration

Specific PID, VID and video stream interface settings (Resolution, FPS) are configured via menuconfig.

## Operation modes

- Single camera: One USB Camera connected to the esp32p4 HS USB Port (with or without the External USB Hub)
  - Resolution `1024x576`
  - FPS `30`

- Dual camera: Two USB Cameras connected to the esp32p4 HS USB Port via External USB Hub
  - Resolution `640x480` (each video stream)
  - FSP `20` (each video stream)

Camera resolution in both modes are selected in respect to the [ESP32-P4-Function-EV-Board's](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/index.html) display resolution, being `1024x600`.

## Sample output

Below is a sample output of the demo, with an external USB Hub and both USB cameras already connected to the `esp32p4` HS USB port. The USB cameras are then unplugged and plugged back in.

```
I (1316) main_task: Started on CPU0
I (1326) esp_psram: Reserving pool of 32K of internal memory for DMA/internal allocations
I (1326) main_task: Calling app_main()
I (1326) ESP32_P4_EV: MIPI DSI PHY Powered on
I (1326) ESP32_P4_EV: Install MIPI DSI LCD control panel
I (1326) ESP32_P4_EV: Install EK79007 LCD control panel
I (1326) ek79007: version: 1.0.2
I (1326) gpio: GPIO[27]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (1496) ESP32_P4_EV: Display initialized with resolution 1024x600
I (1506) ESP32_P4_EV: Setting LCD backlight: 100%
I (1506) esp-usb-demo: Installing USB Host
I (1536) esp-usb-demo: Installing UVC driver

Type 'help' to get the list of commands.
Use UP/DOWN arrows to navigate through command history.
Press TAB when typing command name to auto-complete.
esp32p4>
W (1916) uvc: USB device with addr(1) is not UVC device
I (2486) esp-usb-demo: UVC Device connected -> Open the UVC stream
I (2486) esp-usb-demo: Opening UVC device 0x046D:0x094C	1024x576@30.0FPS...
I (2686) esp-usb-demo: Opening UVC device 0x046D:0x082C	1024x576@30.0FPS...
I (2746) esp-usb-demo: UVC device opened, opening stream
I (2776) esp-usb-demo: Setting resolution to full
I (2816) esp-usb-demo: Stream opened, streaming...
I (2876) esp-usb-demo: UVC Device connected -> Open the UVC stream
I (2876) esp-usb-demo: Opening UVC device 0x046D:0x094C	640x480@20.0FPS...
I (3216) esp-usb-demo: Opening UVC device 0x046D:0x082C	640x480@20.0FPS...
I (3266) esp-usb-demo: Setting resolution to half
I (3316) esp-usb-demo: Stream opened, streaming...
I (3316) esp-usb-demo: UVC device opened, opening stream
I (3326) esp-usb-demo: Setting resolution to half
I (3366) esp-usb-demo: Stream opened, streaming...
...

First camera disconnected from external USB Hub

...
I (6426) esp-usb-demo: Device suddenly disconnected
I (6426) esp-usb-demo: UVC Device disconnected -> Close the UVC stream
I (6426) esp-usb-demo: Reopening video stream of device 1 from HALF to FULL screen
I (6576) esp-usb-demo: Opening UVC device 0x046D:0x082C	1024x576@30.0FPS...
I (6636) esp-usb-demo: Setting resolution to full
I (6686) esp-usb-demo: Stream opened, streaming...
...

Second camera disconnected from external USB Hub

...
I (10846) esp-usb-demo: Device suddenly disconnected
I (10846) esp-usb-demo: UVC Device disconnected -> Close the UVC stream
...

FIrst camera connected to external USB Hub

...
I (14196) esp-usb-demo: UVC Device connected -> Open the UVC stream
I (14196) esp-usb-demo: Opening UVC device 0x046D:0x094C	1024x576@30.0FPS...
I (14396) esp-usb-demo: Opening UVC device 0x046D:0x082C	1024x576@30.0FPS...
I (14476) esp-usb-demo: UVC device opened, opening stream
I (14506) esp-usb-demo: Setting resolution to full
I (14546) esp-usb-demo: Stream opened, streaming...
...

Second camera connected to external USB Hub

...
I (17526) esp-usb-demo: UVC Device connected -> Open the UVC stream
I (17536) esp-usb-demo: Opening UVC device 0x046D:0x094C	640x480@20.0FPS...
I (17916) esp-usb-demo: Opening UVC device 0x046D:0x082C	640x480@20.0FPS...
I (17966) esp-usb-demo: Setting resolution to half
I (18016) esp-usb-demo: Stream opened, streaming...
I (18016) esp-usb-demo: UVC device opened, opening stream
I (18026) esp-usb-demo: Setting resolution to half
I (18066) esp-usb-demo: Stream opened, streaming...
