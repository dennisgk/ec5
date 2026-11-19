# BrachiGoon EC5

Dark wgpu/egui controller with EC4 controls over USB serial. Windows uses DirectX 12.
See the [GOON protocol](../exoskeleton_controller/PROTOCOL.md).

```powershell
cd C:\Users\Owner\Desktop\Compositions\BME490\Arduino\ec5\exoskeleton_controller_rust
cargo run --release --locked
```

Or run `target/release/BrachiGoon.exe`. Build only:
`cargo build --release --locked`.

## Connect

Select the ESP32-C6 native USB Serial/JTAG COM port and Connect. Refresh ports
lists names without opening/probing devices. Close IDF Monitor and other serial
applications before connecting; close Rust before flashing. No Wi-Fi/firewall
setup is needed. Nominal 115200 does not impose a UART baud-rate limit on native
USB. Flow control is disabled; the app does not pulse boot/reset lines.

GOON HELLO precedes commands. The serial pump remains active during rendering,
file dialogs and command waits. Four outstanding chunks per direction are the
default; SERIAL_WINDOW controls Rust's limit, negotiated with firmware.
Explicit Stop bypasses a busy command worker and receives transport priority.

## Controls

Acquire EMGs enables available boards. Missing boards warn without disconnecting
or blocking healthy boards/manual motor control. Hardware resend is optional,
off by default, and runs on LP. Hardware/DSP settings remain drafts until Apply.

Preserved functionality includes up to eight notch rows per board, sixteen global
Fourier ignore ranges, integration, 256-point Fourier, and graded onboard motor
control. Filtered/integrated/Fourier processing uses enabled notches; raw is
unchanged. Plots and detection use the same acknowledged ignore mask.
Notch width is measured at half the center attenuation in dB.

Only checked plots subscribe. Intervals are laptop display refresh (20–5000 ms),
not acquisition rates. Autoscroll X, 0.01–120 second X span, manual navigation
and Auto-fit Y are retained. Congestion may discard telemetry batches rather
than block acquisition; sequence gaps break raw/filtered plot lines.

One EMG can control the motor. The maximum eligible FFT peak maps linearly between
minimum/maximum thresholds and their signed endpoint velocities, stopping below
minimum and clamping above maximum. All decisions run on HP. Manual controls
are disabled while an EMG owns control. Disable that controller or use Stop. Disconnect preserves the local mode.
CAN diagnostics, retry and feedback protections remain available.

## Save / Load

EC5 JSON retains EC4 device/plot fields, replaces IP/port with serial_port, and
requires project="ec5", schema_version=1 or 2. New saves use version 2. EC4 files are explicitly rejected.
Save uses confirmed settings. Load validates, disarms, applies/verifies settings,
restores subscriptions, then arms an available saved EMG controller last.
Loading can therefore request motion, including a saved manual velocity. Version 2
stores confirmed motor velocity, Bluetooth on/off, and angle parameters. Sessions,
histories and runtime availability are not persisted.

Graceful disconnect and serial timeout preserve device settings and local motor
logic. Idle keepalives maintain only transport liveness. Reconnect reads current
state. **Save autoconfig to SD** stores confirmed settings, including manual motor
velocity and Bluetooth on/off, to the optional SD card. Unsent editor drafts are
excluded. **Delete autoconfig** stops/resets and deletes the saved profile.

**Enable Bluetooth control** stops current motion and advertises the EC5 BLE
service. USB settings and diagnostics remain available; competing USB motor
commands are rejected. **Disable Bluetooth control** stops and takes ownership
back. **Stop** remains available in either mode. BLE/SD diagnostics expose stack,
heap and error counters. Use the new BrachiFlex mobile application for angle
control. See [BLE_PROTOCOL.md](../exoskeleton_controller/BLE_PROTOCOL.md).

## Checks

```powershell
cargo fmt --check
cargo clippy --locked --all-targets -- -D warnings
cargo test --locked
```

Tests cover shared C/Rust frame vectors, maximum-size assembly, chunk loss,
CRC/noise recovery, windows, profile compatibility, and an in-memory serial peer
exercising the real pump with commands, raw/FFT telemetry and keepalives.
They do not open physical serial ports or operate hardware.
