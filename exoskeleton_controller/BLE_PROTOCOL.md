# EC5 BLE, ownership, and autoconfig

## Control lifetime

Motor output is local: stopped/manual velocity, one EMG controller, or angle hold.
Transport sessions do not own its lifetime. USB HELLO/disconnect/watchdog expiry
and BLE disconnect preserve acquisition, DSP settings, motor mode and target.
Explicit Stop, ownership handoff, Delete autoconfig, CAN faults and stale motor
feedback still stop/inhibit output. Reconnection never replays a motion command.

USB always permits diagnostics, settings, SD save/delete and BLE enable/disable.
While Bluetooth owns motion, nonzero USB velocity and EMG arming are rejected
with status 13. Either transport can explicitly stop. USB Disable Bluetooth
stops motion, terminates the BLE link and restores USB motor access. BLE enable
also stops earlier manual/EMG motion. A connection alone never enables hold.

## GATT v1

Service UUID: `7c38a100-e82d-4ec1-a271-f2186a40c005`.
Characteristic UUIDs share the suffix and differ in the first word:

| UUID first word | Characteristic | Access | Bytes |
|---|---|---|---|
| 7c38a101 | Identity | Read | 8 |
| 7c38a102 | Command | Write with response | 4–20 |
| 7c38a103 | Result | Notify | 5 |
| 7c38a104 | State | Read / Notify | 20 |
| 7c38a105 | Mechanics | Read | 16 |
| 7c38a106 | Dynamics | Read | 16 |

All multibyte values are little endian. Floats are finite IEEE754 f32. No native
struct padding, ASCII framing, Base64, or JSON is used in BLE messages. Every
value fits the minimum 23-byte ATT MTU. BLE has one permitted connection.

Identity: `[version=1 u8, factory MAC u8[6], signature=0xC5 u8]`.
Advertised name: `BrachiGoon-` plus the final three MAC bytes as hex. Filter on
the service UUID, require explicit device selection, discover the expected
service/characteristics and verify identity before issuing commands. This is
accidental-device filtering, not authenticated pairing or encryption.

Command: `[version=1 u8, operation u8, request_id u16, payload...]`.
Result: `[version=1 u8, operation u8, request_id u16, status u8]`.
One application command is outstanding per client. IDs increase; reconnect
before u16 exhaustion. GATT Write completion is receipt, not command success;
wait for the matching Result. Do not blindly replay motion after a timeout.
The worker caches its last result and rejects ID reuse with different bytes.

| Operation | Payload | Effect |
|---|---|---|
| 1 | empty | Stop; obtain fresh zero-speed feedback; calibrate current encoder as 0° |
| 2 | minimum f32, maximum f32 | Set elbow limits, hold must be off |
| 3 | motor/elbow ratio f32, direction i8 (+1/-1) | Set mechanics, invalidates calibration if changed |
| 4 | gain f32, speed f32, acceleration f32, deadband f32 | Set dynamics, hold must be off |
| 5 | target elbow degrees f32 | Clamp target to limits; requires enabled hold |
| 6 | enabled u8 (0/1) | Enable at measured position, or stop |
| 7 | empty | Stop from either ownership mode |

State: `[version u8, bluetooth_owner u8, flags u8, motor_fault u8,
measured_degrees f32, target_degrees f32, motor_velocity_rad_s f32, sequence u32]`.
Flags: bit 0 hold enabled; bit 1 calibrated. Notifications are bounded-rate
snapshots, not EMG/sample telemetry. There are no notification retries or queues
of old state. The current state remains readable.

Mechanics: `[ratio f32, direction f32, minimum f32, maximum f32]`.
Dynamics: `[gain f32, speed f32, acceleration f32, deadband f32]`.

Status codes reuse GOON: 0 success, 1 invalid, 6 not ready, 7 motor fault,
12 canceled, 13 competing motor owner, 14 SD failure. Advertising begins only
when Bluetooth control is enabled, including a saved startup setting.

## Angle loop

The motor task computes position feedback at 100 Hz; no BLE callbacks do motor
or DSP work. Ratio is **motor rotations per elbow rotation**. Elbow degrees =
`(unwrapped_encoder - zero) * 180/pi * direction / ratio`. RobStride feedback
wraps over [-4pi,+4pi]; continuous hold unwraps consecutive feedback positions.
Startup or changed mechanics invalidate calibration.

The outer position loop uses measured motor velocity, converted to elbow degrees/s,
to preview 150 ms of motion before braking. Its requested elbow velocity is
`gain / (1 + gain * 0.15) * (position_error - measured_elbow_velocity * 0.15)`.
This bounds the effective outer-loop gain even with existing aggressive profiles.
Speed and stopping-distance limits still apply; motor velocity remains capped at
±50 rad/s. Acceleration limits increases in command magnitude. Braking commands
decrease immediately, and reversals pass through zero before accelerating again.
These are command limits, not a measurement of the arm's physical braking ability.
Encoder tracking also updates while hold is off
whenever fresh feedback is available.
Outward motion at an elbow limit is suppressed. Stale/non-finite feedback
requests zero; the existing CAN task's feedback timeout still faults/inhibits.
Defaults: ratio 1, direction +1, limits 0–120°, gain 40/s, speed 720°/s,
acceleration 7200°/s², deadband 0.5°. Configurable speed is up to 720°/s and
acceleration up to 10000°/s². Existing SD configurations retain their saved
values. Do not assume these establish physical limits or guarantee settling time.

## USB extension operations

GOON framing remains unchanged. Argument validation precedes side effects.

| Operation | Arguments | Successful result body |
|---|---|---|
| 28 Begin autoconfig | JSON length u16, 1–32768 | empty |
| 29 Write autoconfig | next offset u16, 1–158 data bytes | empty |
| 30 Commit autoconfig | empty | empty |
| 31 Delete autoconfig | empty | empty |
| 32 Bluetooth | enabled u8 (0/1) | empty |
| 33 Control state | empty | fixed 76 bytes below |
| 34 Angle configuration | mechanics then dynamics, eight f32 | empty |

Uploads are sequential and volatile until Commit. Transport retirement cancels
an unfinished upload. Explicit Save never changes live device settings.
Desktop chunks currently contain 150 JSON bytes each. Command replay protection
applies to every chunk and Commit.

Control state: Bluetooth u8, hold enabled u8, calibrated u8, mode u8
(0 stopped/1 manual/2 EMG/3 angle); manual velocity f32; eight angle config f32;
eight BLE diagnostic u32; last SD/config error u32.
BLE diagnostics: available, host synced, last error, queue-full count,
notification failure count, free internal heap, minimum internal heap, worker
stack high-water bytes. SD errors may be ESP error codes, errno, or validation
status; an absent SD card does not disable USB or BLE.

## SD persistence

SDSPI: MOSI20, MISO21, SCK19, CS18, initially 10 MHz. The firmware never formats
the card. `/sdcard/autoconfig.json` is an EC5 JSON profile, at most 32 KiB.
Version 2 adds `motor: { bluetooth, manual_velocity, angle: { ratio, direction,
minimum, maximum, gain, speed, acceleration, deadband } }` to the existing
`project`, `schema_version`, `device`, and `display` fields. Version 1 imports
with Bluetooth off, zero manual velocity, and default angle parameters.

The desktop obtains confirmed device/motor settings, then uploads the JSON.
Editor drafts are excluded. JSON is parsed only during Save validation/startup;
BLE callbacks, acquisition and control never parse JSON.

Commit validates the complete profile, writes `.tmp`, flushes/syncs/closes,
verifies the bytes, moves the prior file to `.bak`, then promotes the temp file.
Startup tries the primary and then a validated backup. This is bounded recovery,
not a guarantee against arbitrary FAT/card failure or removal during a write.

Startup restores hardware/DSP settings and saved BLE on/off. Saved manual
velocity or EMG ownership may activate once motor/configuration prerequisites
are ready, without a USB connection. A Bluetooth-enabled profile starts stopped
and advertising; angle hold needs calibration and enable after reboot. No SD
writes happen automatically on settings changes or while dragging.

Delete first stops and resets runtime settings, then removes primary, backup
and temp files. A deletion failure is reported: the old file might otherwise
restore settings on the next boot. No SD card is needed for ordinary operation.

## Failure containment and acceptance

NimBLE has one connection and fixed pools. Application requests use an eight-job
queue of at most 20 bytes each. Callbacks only copy/validate or read cached
snapshots. The low-priority worker handles commands; notifications use NimBLE
mbufs and release them through its send API. Errors are counted, not retried in
a tight loop. The host runs below acquisition priorities.

Essential USB queues/driver and static USB task stacks are reserved before
optional BLE. BLE checks internal heap/headroom and task creation failures;
failed initialization reports diagnostics rather than calling ESP_ERROR_CHECK.
No application can guarantee a bug-free third-party stack. Before relying on
hardware operation, test repeated connections, malformed/overloaded writes,
low heap, USB takeover, SD removal, continued local mode after disconnect,
calibration direction/ratio, limit behavior, CAN faults and LP timing counters.
Do not interpret successful builds/host tests as those hardware checks passing.
