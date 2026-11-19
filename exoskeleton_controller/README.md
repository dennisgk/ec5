# BrachiGoon EC5 firmware

ESP-IDF 5.5.2, ESP32-C6. Native USB Serial/JTAG replaces EC4 Wi-Fi/TCP/UDP. Optional NimBLE control and SD autoconfig are supported; see [BLE_PROTOCOL.md](BLE_PROTOCOL.md).
Canonical wire specification: [PROTOCOL.md](PROTOCOL.md).
Use the matching EC5 Rust application.

## Build and operator update

```powershell
cd C:\Users\Owner\Desktop\Compositions\BME490\Arduino\ec5\exoskeleton_controller
# In an ESP-IDF PowerShell terminal:
idf.py build
# Operator action only, after closing Rust and IDF Monitor:
idf.py -p COM_PORT flash
```

The normal firmware build embeds the LP program. No separate LP flash, SD
upload, modules, extra partitions or external RAM are required. SD is optional for autoconfig. Native USB is
the same USB Serial/JTAG interface used for flashing (C6 GPIO12/13).
Do not use UART0 on the motor CAN pins.

## Acquisition and motor

LP exclusively owns 400 kHz I²C on SDA6/SCL7, two nominal 1000 SPS ADCs,
optional hardware resend, and the 4096-record/64 KiB shared HP-SRAM ring.
HP drains records, applies notch banks, rectified weighted integration,
256-point overlapped FFT and graded EMG motor decisions. None waits for serial.
Defaults remain gains 1/1 and analog filters 20/450 Hz, matching sketch_sep30a.
Missing boards warn while healthy boards and manual motor control continue.

CAN remains TX16/RX17, 1 Mbit/s, motor 0x7F, host 0xFD. Startup retries,
zero-speed enabling, fresh feedback and RobStride faults are retained.
All CAN operations remain in the motor task.

## Serial lifecycle

GOON version 1 uses ASCII hexadecimal fields, CRC32 and independently Base64
encoded chunks. Maximum line 512 bytes including LF; decoded chunk 351 bytes;
wire offsets support 65535 bytes; firmware command receive capacity is 1024 bytes.
SD profiles are uploaded as bounded 150-byte command chunks. HELLO uses session 0 and an empty payload.
Argument length and integrity checks precede command side effects.

EC5_SERIAL_WINDOW in menuconfig defaults to four, range 1..16. Rust has the
corresponding SERIAL_WINDOW constant; INIT exchanges receiver limits.
ACK timeout is 250 ms, with three retransmissions. Cached/stale commands never
execute twice. ACK messages never require acknowledgements.

Separate RX, command, TX and watchdog tasks keep USB waits off acquisition and
motor paths. The sole TX writer serializes protocol frames and application logs.
Logs buffer/drop without a session. Driver RX/TX buffers are 8192/4096 bytes.
Queues, reassembly, result caches and telemetry buffers are bounded; firmware
does not allocate per sample. Congested telemetry batches are discarded.

After two seconds without valid PC traffic, the watchdog retires the serial session.
Device settings and the local motor mode continue through USB/BLE disconnects.
Saved manual velocity and EMG ownership can resume at boot; Bluetooth on/off is
saved too. Angle hold requires recalibration after reboot. Explicit Stop, USB
Bluetooth takeover, CAN faults, and stale feedback still stop/inhibit output.
Reconnect reads current state and never replays motion automatically.

## Local verification and hardware acceptance

`cmd /c tests\run-tests.cmd` runs C DSP/conversion, shared-ring/mailbox,
golden-frame, generic/full-size and firmware-limited reassembly, overlap,
deduplication, watchdog timing, angle math and bounded autoconfig validation tests.
These tests never connect to hardware.

After operator flashing, connect Rust to the native COM port and check:

1. HELLO, acquisition and CAN diagnostics, including missing-board warnings.
2. Raw/filtered/integrated/Fourier plots and confirmed settings for each board.
3. Applied ignore masks and graded onboard motor detection.
4. Manual/automatic motor behavior under appropriate operator supervision.
5. Rust close/crash preserves the local motor mode; reconnect reads current settings.
6. Save/Load restores an available saved EMG controller last. Loading can request
   motion, including a saved manual velocity. Verify SD startup behavior separately.

Host tests/builds do not verify physical ADC integrity, USB throughput or motor
response. Those require operator hardware testing.
