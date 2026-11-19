# Exoskeleton Controller 5

This repository contains the complete EC5 project:

| Directory | Purpose |
|---|---|
| `exoskeleton_controller` | ESP-IDF 5.5.2 ESP32-C6 firmware, LP acquisition, motor/DSP control, USB, BLE and SD autoconfig |
| `exoskeleton_controller_rust` | Windows egui/wgpu USB configuration and signal application |
| `exoskeleton_controller_mobile` | Windows/iPhone egui/wgpu BLE angle controller |
| `work` | Original and strap-free Blender working models and GLB exporter |

Generated builds, local SDK configuration, logs and signing credentials are
excluded. Firmware build defaults, Rust lockfiles and application model assets
are included. The mobile application's earlier commits are retained in history;
the current tree places all projects together under this repository root.

## Windows

In an ESP-IDF 5.5.2 terminal:

```powershell
cd exoskeleton_controller
idf.py build
```

Use separate terminals for the two applications:

```powershell
cd exoskeleton_controller_rust
cargo run --release --locked
```

```powershell
cd exoskeleton_controller_mobile
cargo run --release --locked --bin BrachiFlex
```

Close the USB application before flashing or opening IDF Monitor. Bluetooth
advertising is enabled through the USB application's Enable Bluetooth control
button or a saved autoconfig setting.

## iPhone build on a Mac

Install full Xcode, Rust 1.93+ and XcodeGen (`brew install xcodegen`). From a
clone of this repository:

```bash
cd exoskeleton_controller_mobile
bash scripts/build-ios.sh --unsigned

# Signed archive and IPA, automatically detecting your Apple developer team:
bash scripts/build-ios.sh
bash scripts/export-ipa.sh
```

See the [mobile README](exoskeleton_controller_mobile/README.md) for signing and
IPA export. The iPhone build requires verification on macOS and the intended
device; Windows tests do not validate iOS linking or signing.

## Control behavior

Settings and active local motor logic persist across USB/BLE disconnects.
Explicit Stop, control handoff and motor faults still stop/inhibit output.
Saved manual/EMG control can resume at boot. Angle Hold needs calibration after
a restart. Saving autoconfig is explicit; old profiles retain their saved
response values. Select Fast and Apply angle settings to replace an old gentle
response, then save autoconfig again if desired.

Detailed documentation:

- [Firmware and hardware](exoskeleton_controller/README.md)
- [USB application](exoskeleton_controller_rust/README.md)
- [BLE application](exoskeleton_controller_mobile/README.md)
- [BLE protocol and persistence](exoskeleton_controller/BLE_PROTOCOL.md)
- [GOON USB framing](exoskeleton_controller/PROTOCOL.md)
