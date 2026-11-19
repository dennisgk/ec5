# BrachiFlex EC5

Rust egui/wgpu BLE application for Windows and iPhone. Connect, Config,
Angle Control, Auto, and Signals are the five pages. Config, Auto, and Signals
are intentionally empty. Only Angle Control commands the arm in this version.

## Windows

```powershell
cargo run --release --locked --bin BrachiFlex
```

Use the matching EC5 firmware and updated USB app. Connect USB, click
**Enable Bluetooth control**, then scan in BrachiFlex and select the intended
`BrachiGoon-XXXXXX`. Scanning filters the EC5 service; connection verifies the
service, identity signature and protocol before exposing controls. There is no
automatic connection or automatic motor enable. Identity filtering prevents
accidental device selection; it is not cryptographic authentication.

In Angle Control, stop hold before applying settings. Set the motor turns per
elbow turn, direction, and elbow limits. Calibrate at the position you designate
0°, then Enable hold. Drag the forearm to change the target. Calibration/enable
may briefly enable the motor at zero speed to obtain fresh encoder feedback.
The firmware computes position control at 100 Hz; touch/mouse targets are
coalesced at up to 20 Hz. No raw EMG or FFT data is streamed over BLE.

Disconnecting or backgrounding the phone does **not** stop hold. USB remains
available for settings and for **Disable Bluetooth control**, which stops motion
and returns ownership to USB. Stop also remains available from either app.
CAN faults and stale motor feedback still inhibit output. After a reboot,
Angle Control must be recalibrated and enabled again.

The arm is the strap-free, optimized GLB in `assets/`. The original and edited
Blender working copies and exporter live in the parent EC5 `work/` directory;
they are not required to build this repository. The fixed and moving assemblies
and elbow pivot are retained as named nodes. Rendering uses DirectX 12 on
Windows and Metal on Apple platforms.

## iPhone / IPA on a Mac

Install Xcode, select its command-line tools, install Rust and XcodeGen
(`brew install xcodegen`), then clone/sync this repository.

```bash
# Compile and link without signing (also used in CI):
bash scripts/build-ios.sh --unsigned

# Archive with your Apple team and Xcode-managed signing:
export APPLE_TEAM_ID=YOUR_TEAM_ID
bash scripts/build-ios.sh

# Copy ios/ExportOptions.example.plist to an ignored local file, set the team,
# and select the export method supported by your provisioning/account:
bash scripts/export-ipa.sh /absolute/path/ExportOptions.local.plist
```

The output IPA is in `ios/build/ipa/`. Alternatively export the archive from
Xcode Organizer. A target triple alone cannot create an installable, signed IPA;
Apple signing/provisioning and a Mac are required. The included example uses
development/debugging export; distribution methods depend on your account.
Set a different bundle identifier in `ios/project.yml` if your team needs it.

The small Objective-C executable enters Rust on the main thread. winit owns
UIApplication and egui owns the UI; there is no WebView. The Info.plist includes
Bluetooth usage descriptions. BLE permission refusal is reported in Connect.
No background Bluetooth entitlement is requested: the controller holds locally
while iOS suspends the app, and reconnection reads current state.

Windows builds/tests do not establish that an iOS SDK, signing setup, physical
iPhone BLE connection, or real motor works. Run the unsigned Mac CI job and then
test a signed application on the intended iPhone/Xcode combination.

## Checks

```powershell
cargo fmt --check
cargo clippy --locked --all-targets -- -D warnings
cargo test --locked
cargo build --release --locked
# Optional offscreen renderer check; no Bluetooth access:
cargo test --locked --lib render_arm_without_ble -- --ignored
```

No check scans for or connects to a physical BLE device. The GitHub workflow also
builds and links an unsigned iOS application on macOS. Hardware acceptance should
cover wrong-device rejection, permission denial, repeated connect/disconnect,
USB takeover while dragging, continued local hold without the phone, CAN feedback
loss, and calibration with the actual transmission ratio and direction.

The included [BLE protocol](docs/BLE_PROTOCOL.md) defines all GATT UUIDs and byte
layouts. It is copied from the matching firmware's canonical `BLE_PROTOCOL.md`.
