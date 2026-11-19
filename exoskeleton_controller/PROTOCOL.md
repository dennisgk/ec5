# EC5 GOON serial protocol, version 1

The wire is ASCII over the ESP32-C6 native USB Serial/JTAG COM port.
Application operations and records are explicitly encoded binary before Base64.
Neither Wi-Fi nor an external USB/UART bridge is required.

## Frames

`GOON|V|S|IIIIIIII|BBBB|EEEE|TTTT|CCCCCCCC|BASE64\n`

V is literal `1`. S is one alphanumeric session character. Session `0` is
reserved for HELLO. ID is uint32 hex; START, END, TOTAL are uint16 hex. Use uppercase
hex. START is inclusive, END exclusive, TOTAL is the complete decoded message
size. Decode exactly END-START bytes. Require START <= END <= TOTAL.
Offsets count decoded bytes, never Base64 characters or frame numbers.

Maximum complete frame is 512 bytes including LF. Fixed overhead is 43 bytes.
Maximum decoded chunk is 351 bytes (468 Base64 characters, 511-byte frame).
Standard padded RFC4648 Base64, no whitespace or wrapping. Smaller chunks are
allowed. Wire offsets allow at most 65535 bytes; INIT advertises the firmware's
actual command receive capacity (currently 1024 bytes). SD JSON uses multiple
small application commands, so it does not reserve a 64 KiB receive buffer.
Require complete
coverage [0,TOTAL) before delivery. Identical overlaps/retransmissions are
acknowledged; conflicting overlaps and inconsistent TOTAL are protocol errors.
Ordinary messages must be nonempty. Only HELLO has START=END=TOTAL=0.

CRC32 is CRC32/ISO-HDLC (reflected polynomial EDB88320, initial and final XOR
FFFFFFFF; check value CBF43926 for ASCII 123456789). Hash every ASCII byte except
the eight CRC characters and LF, including both pipes surrounding CRC. CRC is
eight uppercase hex characters. Firmware uses esp_rom_crc32_le, initial argument 0.
An incremental call over the two spans gives the same result as concatenation.
Malformed lines, non-GOON logs, overlong lines, bad CRC and stale sessions are
discarded without executing commands. A newline resets the bounded line parser.

## HELLO, session, IDs

PC sends session 0, a new PC ID, and an empty payload. ESP replies with the same
ID and empty payload, replacing session 0 with its assigned session. No ACK is
required for either HELLO frame. Retrying the same HELLO ID returns the cached
session response without resetting again. A different HELLO retires the old
transport session and cancels pending transport work before replying. Device settings and local motor mode remain active.
Use nonzero alphanumeric sessions, choosing a different character each new session.
The single character is a testing/reconnection marker, not authentication or
permanent uniqueness. Clear pending work on reconnect.

PC IDs are 00000000..7FFFFFFF, ESP IDs 80000000..FFFFFFFF. IDs increase within a
session. All chunks/retries of a message share its ID. Normal responses and ACKs
have their own ID and reference the other message inside their payload. Only HELLO
echoes an ID. Renew the session before exhaustion; do not replay motion commands.

## Decoded payloads

All integers are little endian; floats are IEEE754 float32. No compiler padding.

| Kind | Value | Remaining bytes |
|---|---|---|
| Command | 1 | operation u8, arguments |
| Response | 2 | request ID u32, operation u8, status u8=0, body |
| Error | 3 | request ID u32, operation u8, nonzero status u8, body |
| Telemetry | 4 | stream u8, epoch u32, record count u16, records |
| Chunk ACK | 5 | message ID u32, START u16, END u16 |
| Keepalive | 6 | none |
| Disconnect | 7 | none |

ACKs are always one frame and never themselves acknowledged. Keepalive and
Disconnect receive chunk ACKs, not application responses.
Application responses indicate completion; a chunk ACK only indicates receipt.

## Window, bounds and failure

Each direction starts at window 1. INIT exchanges receiver windows (1..16),
and each sender uses min(local transmit limit, advertised receiver limit).
Firmware Kconfig EC5_SERIAL_WINDOW and Rust SERIAL_WINDOW default to 4.
Every validated chunk is ACKed independently. Retransmit after 250 ms, at most
three times. Exhausted retries retire the session. Repeated commands return
their cached result and never reexecute; stale command IDs cannot reexecute
after cache eviction. Stop cancels older pending motion requests; disconnect retires transport work without stopping the local mode.

Firmware has one 1024-byte normal receive buffer plus a 128-byte coverage
bitmap. Complete single-frame priority/control messages use a separate slot.
Telemetry queues, receive slots and result caches are bounded. Firmware caches
16 completed command results, each at most 1024 bytes. No unvalidated length
causes allocation. ACKs, stop, disconnect and responses precede telemetry.
A command waits at most 5 seconds without progress / 15 seconds total.

Rust sends Keepalive after 500 ms without other outgoing session traffic.
An independent HP watchdog invalidates the transport session after two seconds
without valid PC traffic. Settings and the local motor mode remain active.
The receive/transmit pump continues during GUI dialogs and command waits.
USB cable presence is not application liveness. Graceful disconnect also preserves device state.

See [BLE_PROTOCOL.md](BLE_PROTOCOL.md) for BLE, persistent motor ownership, SD autoconfig,
and USB extension operations 28–34. These supersede the original disconnect-reset policy.

## Application operations

Reuse EC4 operation IDs and explicitly encoded argument/result layouts for
motor (2), acquisition/integration/Fourier (3/4/5), hardware (6), integration
window (7), Fourier settings (8), acquisition/motor diagnostics (12/13),
hardware resend (14), CAN diagnostics/retry (18/19), ignore ranges (22),
notch banks (23), motor curve (24), configuration (25), hardware failures (27).
Unsupported legacy query commands are rejected, not polled.

INIT (1): request [receive_window u8]. Response body: protocol u16=1,
sample_rate u16=1000, FFT_size u16=256, raw_capacity u16=1024,
session_cookie u32=ASCII session character, streams u8=10,
record_sizes u16[10], receive_window u8, max_command_receive u16=1024.
SUBSCRIBE (26): mask u16 (bits 0..9); response session_cookie u32, epoch u32.
No IP address or destination port exists.

Every request argument below follows its operation byte. Every successful result
is the response body after request-ID, operation and status. Booleans are 0/1;
boards are 0/1. Unknown operations and wrong argument lengths return status 1
without applying side effects.

| Operation | Arguments | Successful body |
|---|---|---|
| 2 Motor | velocity i16, tenths rad/s, -500..500 | empty |
| 3/4/5 Acquisition/integration/Fourier | enabled u8 | empty |
| 6 Hardware property | board u8, property u8, gain u8 or frequency u16 | requested u16, applied/quantized f32 |
| 7 Integration window | board u8, milliseconds u16 | empty |
| 8 Fourier | board u8, hop u16, window u8, remove_mean u8 | empty |
| 12 Acquisition diagnostics | empty | 27 u32 words |
| 13 Motor status | empty | flags/fault/inhibit u32; requested/commanded/position/velocity f32; feedback time u64; reason[96], inhibit_reason[96] |
| 14 Hardware resend | board u8, interval u16, 0 disables | empty |
| 18 CAN diagnostics | empty | 36 u32 words, failure time u64 |
| 19 CAN retry | empty | empty, motor remains stopped |
| 22 Ignore ranges | global enabled u8, count u8, rows {enabled u8, low f32, high f32} | empty |
| 23 Notch bank | board u8, count u8, rows {enabled u8, center u16, width u16, attenuation i16} | empty |
| 24 Motor curve | board u8, enabled u8, minimum/maximum f32, minimum/maximum velocity i16 | empty |
| 25 Configuration | empty | length u16, snapshot below |
| 27 Hardware failures | empty | 2 boards x 6 u32: operation/device/register/expected/actual/error |

Hardware properties: 0 amplifier gain, 1 ADC PGA, 2 high pass, 3 low pass.
Gains 1/2/4/8/16/32/64/128. Frequencies in tenths Hz, HP 33..709, LP 1520..4500.
Integration 10..1000 ms, FFT hop 64/128/256, window 0 rectangular / 1 Hann.
Ignore count <=16, inclusive 0 <= low <= high <=500 Hz. Notch count <=8,
center 10..4990, width 1..1000 (tenths Hz), attenuation -600..-1 (tenths dB).
Nominal notch edges must remain inside Nyquist.

Configuration snapshot: epoch u32; acquisition/integration/Fourier/owner u8
(owner 255=None); global-ignore enabled/count u8; ignore rows; excluded-bin
mask[17]. Then two boards, each: amplifier/ADC gain u8; HP/LP/resend/window_ms/hop
u16; FFT window/remove_mean/notch_count u8; notch rows; minimum/maximum f32;
minimum/maximum velocity i16. Final availability u8 mask (bits 0/1).
Availability and epoch are runtime data, not saved settings.

Acquisition diagnostics word order: state, fault, generation, capacity, occupancy,
high_water, configured_mask, heartbeat; then two-board arrays conversions,
ADC-gap events, read errors, published, discarded, max-read-us, reserved raw
overwrites, processing overruns; late-passes, max-pass-us, max-drain-us.
CAN word order: flags, driver-state, TX/RX pending, TX/RX errors, TX failed,
RX missed, RX overrun, arbitration lost, bus errors, alerts, queued, confirmed,
rejected, received, matching, last-error, stage, ID, parameter, failure-state,
failure-TX-errors, failure-RX-errors, failure-bus-errors, motor-faults,
motor-warnings, last-RX-ID, feedback-faults, observed-feedback-faults,
feedback-mode, fault-frames, observed-motor-faults, observed-motor-warnings,
temperature-tenths, feedback-frames.

Statuses: 0 OK; 1 invalid; 2 INIT required; 3/4/5 board0/board1/both hardware
configuration failure (nonfatal); 6 not ready; 7 motor fault; 8 incompatible
capabilities; 9 internal; 10 LP unavailable; 11 manual control forbidden while
EMG owns motor; 12 motion canceled by newer Stop. Missing-board warnings do not
close the session. Hardware diagnostics operations: 0 none, 1 write, 2 read,
3 readback mismatch; error is the ESP-IDF error number.

Telemetry streams / record sizes are unchanged from EC4:
1/2 raw 0/1 (16), 3/4 filtered 0/1 (16), 5/6 integration 0/1 (24),
7/8 FFT 0/1 (536), 9/10 detection 0/1 (24).
Raw: time u64, ADC i32, sequence u32.
Filtered: time u64, mV f32, sequence u32.
Integration: time u64, integral/envelope f32, window/reserved u16, sequence u32.
FFT: time u64, frame u32, rate/size/hop u16, window/remove_mean u8, PSD f32[129].
Detection: time u64, frame u32, peak f32, flags u32, velocity f32.
Flags: detected=1, selected=2, requested=4. Stream only explicit subscriptions.
Batch around 20 ms; telemetry may be discarded under congestion, never delay LP.
Epoch marks configuration boundaries, not exact per-sample revision attribution.

## Output ownership and configuration

One writer owns USB output. Application log lines enter that same bounded writer;
logs cannot interrupt protocol lines. Boot/panic lines are ignored by the PC parser.
Never open IDF Monitor and Rust on the same COM port simultaneously.
Use nominal 115200 8N1 without flow control; native USB does not serialize bytes
at a UART baud-rate limit. Do not pulse boot/reset control lines.

EC5 JSON profiles retain EC4 device/plot fields, replace IP/port with serial_port,
and require project="ec5", schema_version=1 or 2. New saves use version 2.
Reject EC4 files explicitly.
Save confirmed device state; Load disarms, applies/verifies, subscribes, then
restores an available saved EMG controller last. It may request motion.
Missing boards warn without disconnecting or blocking healthy boards/manual control.

## Examples and acceptance

An empty HELLO is GOON|1|0|0000002A|0000|0000|0000|CRC| followed by LF.
Calculated HELLO and motor-command vectors (each terminated by LF):

```text
GOON|1|0|0000002A|0000|0000|0000|72454372|
GOON|1|K|0000002A|0000|0000|0000|436DEED1|
GOON|1|K|0000002B|0000|0004|0004|19CD7F2E|AQIUAA==
```
A 500-byte message can use ranges [0,351) and [351,500), TOTAL=500.
Golden frame/checksum vectors are exercised by C and Rust tests.
Test noise, fragmented reads, CRC failures, overlap, 65535-byte assembly, lost
ACKs, duplicate commands, simultaneous traffic, optional EMGs and watchdog expiry.
Build both projects. Do not flash or operate hardware during local verification.
