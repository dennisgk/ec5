use crate::protocol::MotorCan;
use eframe::egui;

// RS06 manufacturer manual (251112), fault frame 0x15 and feedback 0x02/0x18.
// Preserve unknown bits: other motor models/firmware revisions use other maps.
const RS06_FAULTS: &[(u32, &str)] = &[
    (1 << 0, "Motor overtemperature"),
    (1 << 1, "Driver chip fault"),
    (1 << 2, "Undervoltage"),
    (1 << 3, "Overvoltage"),
    (1 << 7, "Encoder not calibrated"),
    (1 << 14, "Stall / overload"),
];
const RS06_FEEDBACK_FAULTS: &[(u32, &str)] = &[
    (1 << 0, "Undervoltage"),
    (1 << 1, "Overcurrent"),
    (1 << 2, "Overtemperature"),
    (1 << 3, "Magnetic encoder fault"),
    (1 << 4, "Stall / overload"),
    (1 << 5, "Not calibrated"),
];
const RS06_WARNINGS: &[(u32, &str)] = &[(1, "Motor overtemperature warning")];

fn decode_bits(value: u32, definitions: &[(u32, &str)]) -> String {
    if value == 0 {
        return "None".into();
    }
    let mut names = Vec::new();
    let mut known = 0;
    for (mask, name) in definitions {
        known |= mask;
        if value & mask != 0 {
            names.push((*name).to_owned());
        }
    }
    let unknown = value & !known;
    if unknown != 0 {
        names.push(format!("Unknown bits 0x{unknown:08X}"));
    }
    names.join(", ")
}

fn state_name(state: u32) -> &'static str {
    match state {
        0 => "Stopped",
        1 => "Running",
        2 => "Bus-off",
        3 => "Recovering",
        _ => "Unavailable",
    }
}

fn stage_name(stage: u32) -> &'static str {
    match stage {
        1 => "Zero velocity",
        2 => "Disable",
        3 => "Speed mode (0x7005)",
        4 => "Current limit (0x7018)",
        5 => "Torque limit (0x7022)",
        6 => "Feedback reporting",
        7 => "Runtime command",
        8 => "Driver install",
        9 => "Driver start",
        10 => "Driver reset",
        11 => "CAN pin initialization",
        _ => "None",
    }
}

fn error_name(error: u32) -> &'static str {
    match error {
        0 => "ESP_OK",
        0xFFFFFFFF => "ESP_FAIL: CAN transmission failed",
        0x101 => "ESP_ERR_NO_MEM",
        0x102 => "ESP_ERR_INVALID_ARG",
        0x103 => "ESP_ERR_INVALID_STATE",
        0x107 => "ESP_ERR_TIMEOUT",
        _ => "ESP error",
    }
}

pub fn show(ui: &mut egui::Ui, d: &MotorCan) {
    ui.label(format!(
        "At Refresh: {} · configured {} · configuring {}",
        state_name(d.state),
        d.flags & 2 != 0,
        d.flags & 4 != 0
    ));
    ui.small("CAN TX16 / RX17 · 1 Mbit/s · motor 0x7F · host 0xFD");
    if d.flags & 0x100 != 0 {
        ui.label(format!(
            "TX GPIO output enabled {} · routed to TWAI {} · OE source {}",
            d.flags & 8 != 0,
            d.flags & 32 != 0,
            if d.flags & 64 != 0 {
                "Peripheral"
            } else {
                "GPIO register"
            }
        ));
        ui.label(format!(
            "RX input enabled {} · sampled RX level {}",
            d.flags & 16 != 0,
            if d.flags & 128 != 0 { "High" } else { "Low" }
        ));
    }
    egui::Grid::new("motor_can_diagnostics")
        .num_columns(2)
        .show(ui, |ui| {
            for (label, value) in [
                (
                    "TX / RX queued",
                    format!("{} / {}", d.tx_pending, d.rx_pending),
                ),
                (
                    "TX / RX error counters",
                    format!("{} / {}", d.tx_errors, d.rx_errors),
                ),
                (
                    "Bus errors / arbitration losses",
                    format!("{} / {}", d.bus_errors, d.arbitration_lost),
                ),
                (
                    "Failed TX / missed RX / RX overruns",
                    format!("{} / {} / {}", d.tx_failed, d.rx_missed, d.rx_overrun),
                ),
                (
                    "Frames queued / configuration CAN-ACKs",
                    format!("{} / {}", d.tx_queued, d.tx_confirmed),
                ),
                ("Rejected / failed send attempts", d.tx_rejected.to_string()),
                (
                    "Received / matching motor frames",
                    format!("{} / {}", d.rx_frames, d.rx_motor_frames),
                ),
                (
                    "Motor fault / warning bits",
                    format!("0x{:08X} / 0x{:08X}", d.motor_faults, d.motor_warnings),
                ),
                ("Last received frame ID", format!("0x{:08X}", d.last_rx_id)),
                ("Observed TWAI alerts", format!("0x{:08X}", d.alerts)),
            ] {
                ui.label(label);
                ui.monospace(value);
                ui.end_row();
            }
        });
    ui.separator();
    ui.strong("RobStride 06 fault reports");
    if d.fault_frames == 0 {
        ui.label("No detailed fault frame (0x15) received");
    } else {
        ui.label(format!(
            "Last fault word: {}",
            decode_bits(d.motor_faults, RS06_FAULTS)
        ));
        ui.label(format!(
            "Last warning word: {}",
            decode_bits(d.motor_warnings, RS06_WARNINGS)
        ));
    }
    if d.feedback_frames == 0 {
        ui.label("No feedback frame (0x02 / 0x18) received");
    } else {
        ui.label(format!(
            "Last feedback faults: {}",
            decode_bits(d.feedback_faults, RS06_FEEDBACK_FAULTS)
        ));
        let mode = match d.feedback_mode {
            0 => "Reset",
            1 => "Calibration",
            2 => "Run",
            _ => "Unknown",
        };
        ui.label(format!(
            "Last mode: {mode} · temperature {:.1} °C",
            d.temperature_tenths as f32 / 10.0
        ));
    }
    if d.motor_faults_seen != 0 || d.feedback_faults_seen != 0 {
        ui.label(format!(
            "Observed detailed faults: {}",
            decode_bits(d.motor_faults_seen, RS06_FAULTS)
        ));
        ui.label(format!(
            "Observed feedback faults: {}",
            decode_bits(d.feedback_faults_seen, RS06_FEEDBACK_FAULTS)
        ));
    }
    if d.motor_warnings_seen != 0 {
        ui.label(format!(
            "Observed warnings: {}",
            decode_bits(d.motor_warnings_seen, RS06_WARNINGS)
        ));
    }
    if d.last_error != 0 {
        ui.colored_label(
            egui::Color32::LIGHT_RED,
            format!(
                "Failed stage: {} · {} (0x{:08X})",
                stage_name(d.last_stage),
                error_name(d.last_error),
                d.last_error
            ),
        );
        ui.monospace(format!(
            "Frame 0x{:08X} · parameter 0x{:04X} · at {:.3} s",
            d.last_id,
            d.last_parameter,
            d.failure_time_us as f64 / 1e6
        ));
        ui.label(format!(
            "At failure: {} · TEC {} · REC {} · bus errors {}",
            state_name(d.failure_state),
            d.failure_tx_errors,
            d.failure_rx_errors,
            d.failure_bus_errors
        ));
    }
    if d.failure_tx_errors > 0 && d.rx_frames == 0 {
        ui.label("CAN errors with no received frames: check motor/transceiver power, CAN wiring, termination and matching bitrate. These counters do not identify the exact electrical cause.");
    } else if d.rx_frames > 0 && d.rx_motor_frames == 0 {
        ui.label("CAN frames are arriving, but none match motor 0x7F → host 0xFD.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rs06_fault_words_and_feedback_flags_have_distinct_bit_maps() {
        assert_eq!(decode_bits(4, RS06_FAULTS), "Undervoltage");
        assert_eq!(decode_bits(1, RS06_FEEDBACK_FAULTS), "Undervoltage");
        assert_eq!(decode_bits(1, RS06_FAULTS), "Motor overtemperature");
        assert_eq!(
            decode_bits(1, RS06_WARNINGS),
            "Motor overtemperature warning"
        );
        let faults = decode_bits(0x4085, RS06_FAULTS);
        for name in [
            "Motor overtemperature",
            "Undervoltage",
            "Encoder not calibrated",
            "Stall / overload",
        ] {
            assert!(faults.contains(name));
        }
        assert_eq!(decode_bits(1 << 16, RS06_FAULTS), "Unknown bits 0x00010000");
        assert_eq!(decode_bits(0, RS06_FEEDBACK_FAULTS), "None");
    }
}
