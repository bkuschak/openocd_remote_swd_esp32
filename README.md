## remote_swd for OpenOCD

This is a remote-side implementation of the new OpenOCD remote_swd protocol,
using an ESP32 as the remote programmer. This device can be used to program the
flash memory of an ARM microcontroller using the two-wire SWD interface.
OpenOCD connects to this programmer using TCP/IP over WiFi, allowing remote
flashing and debugging of the ARM target. 

The remote_swd driver is intended as a faster alternative to the OpenOCD
remote_bitbang driver. Whereas remote_bitbang sends a separate TCP/IP packet
for each transition of the SWDIO and SWCLK pins, remote_swd sends at most one 
packet for each 32-bit read or write. In most cases it combines many writes
into a single TCP/IP packet, improving performance significantly. After
receiving the packets, the ESP32 locally bit-bangs the SWDIO and SWCLK pins to
perform each 32-bit SWD transfer.

![diagram](img/remote_swd_diagram.svg)

- Tested with the XIAO ESP32C6 development board as the programmer, and STM32
  Blue Pill as the target.
- Two GPIO pins are needed for SWDIO and SWCLK. Pull up/down resistors are
  recommended: 100K pullup on SWDIO, 100K pulldown on SWCLK.
- An optional GPIO pin can be used to drive the SRST (NRST#) line, but this is
  typically not required.
- A separate GPIO may be used to control an activity LED.
- As a point of reference, flashing a 64KB firmware image to the STM32F103
  completes in about 6.7 seconds, for a throughput of 9.5 KB/sec. (The same
  operation using remote_bitbang takes more than 20 minutes).

## Limitations

The current software has some limitations:

- There is no support for adjusting the SWCLK frequency. It was measured at
  about 900 KHz.
- The WiFi credentials are currently hardcoded. The software must be rebuilt if
  you want to change them.

To configure your WiFi SSID and password, copy the
```wifi_password_example.h``` file to ```wifi_password.h``` and edit it
appropriately. For privacy, do not commit this file.

## OpenOCD implementation

The remote_swd driver is committed to a fork of the OpenOCD repo, on the
remote_swd branch.  Refer to the
[implementation](https://github.com/bkuschak/openocd/blob/remote_swd/src/jtag/drivers/remote_swd.c)
in this repo:
```git@github.com:bkuschak/openocd.git```

Configure and build OpenOCD as usual, while enabling the remote_swd driver:

```./configure --enable-remote-swd```

An OpenOCD configuration file has been provided for convenience.
Update your ```tcl/interface/remote_swd.cfg``` configuration file to point to
your ESP32's IP address:

```
adapter driver remote_swd
remote_swd host 192.168.2.144
remote_swd port 5253
reset_config none
```

To flash an STM32, run the following command from your OpenOCD build directory.
Replace ```firmware.elf``` with the name of your ELF file, and
```stm32f1x.cfg``` with the appropriate file for your microcontroller.

```
./src/openocd --search tcl \
              -f tcl/interface/remote_swd.cfg \
              -f tcl/target/stm32f1x.cfg \
              -c "program firmware.elf verify reset exit"
```

## Building

The [Arduino-Timer library](https://github.com/contrem/arduino-timer) is
a required prerequisite. Use the Arduino library manager to install it first.
(Currently it is only used for monitoring the WiFi status and controlling the
LED).

This build was tested with Arduino IDE v1.8.13, using the following settings:

- Board: Adafruit Feather ESP32-C6
- CPU Frequency: 160 MHz (Wifi)
- Flash Frequency: 80 MHz
- Flash mode: QIO
- Partition scheme: Default 4MB with SPIFFS (1.2MB APP / 1.5MB SPIFFS)
- Core debug level: None
- Erase all flash before sketch upload: Enabled
- JTAG adapter: Integrated USB JTAG

Flash it onto the ESP32 using USB. This project doesn't currently support OTA
updates, but this feature can be added if necessary.

## Operation

The ESP32 prints status and error messages to the USB serial port during
operation. A message is printed whenever the OpenOCD client connects or
disconnects. Only one active connection is allowed.

```
ESP32 remote_swd booting (HW version 1.0, SW version 0x0100) ...
Attempting to connect to SSID 'SomeWifiRouter'
....
Connected to WiFi:
SSID:        SomeWifiRouter
RSSI:        -51 dBm
IP Address:  192.168.2.144
remote_swd server listening on port 5253.

Client connected.
Client disconnected.
```

A single SWD 32-bit transfer completes in about 66 microseconds, with an SWCLK
clock rate of approximately 1 MHz. Yellow is SWCLK. Red is SWDIO.

![scopeshot1](img/scopeshot1.png)

A set of three back-to-back transfers complete in about 300 microseconds. The
following scope capture shows a bit better than 50% utilization of the SWD
link.

![scopeshot1](img/scopeshot2.png)
