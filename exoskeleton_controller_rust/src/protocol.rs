use std::io::Read;
#[cfg(test)]
use std::io::Write;

pub const VERSION: u16 = 1;
pub const RAW_CAPACITY: usize = 1024;
pub const FFT_BINS: usize = 129;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Raw = 0,
    Integration = 1,
    Fourier = 2,
    Filtered = 3,
    Detection = 4,
}

impl Kind {
    pub const ALL: [Self; 5] = [
        Self::Raw,
        Self::Integration,
        Self::Fourier,
        Self::Filtered,
        Self::Detection,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Raw => "Raw EMG",
            Self::Integration => "Integration",
            Self::Fourier => "Fourier",
            Self::Filtered => "Filtered EMG",
            Self::Detection => "Detection",
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Hardware {
    pub ina_gain: u8,
    pub adc_gain: u8,
    pub high_pass: u16,
    pub low_pass: u16,
}

impl Default for Hardware {
    fn default() -> Self {
        Self {
            ina_gain: 1,
            adc_gain: 1,
            high_pass: 200,
            low_pass: 4500,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FourierConfig {
    pub hop: u16,
    pub window: u8,
    pub remove_mean: bool,
}

impl Default for FourierConfig {
    fn default() -> Self {
        Self {
            hop: 128,
            window: 1,
            remove_mean: true,
        }
    }
}

#[derive(Clone, Debug)]
pub enum Action {
    Ignore(crate::fourier_view::FourierIgnore),
    NotchBank {
        board: u8,
        filters: Vec<NotchConfig>,
    },
    Motor(i16),
    Activate(Kind, bool),
    Property {
        board: u8,
        property: u8,
        value: u16,
    },
    Integration {
        board: u8,
        milliseconds: u16,
    },
    Fourier {
        board: u8,
        config: FourierConfig,
    },
    ResendHardware {
        board: u8,
        milliseconds: u16,
    },
    Notch {
        board: u8,
        config: NotchConfig,
    },
    RetryMotorCan,
    EmgControl {
        board: u8,
        enabled: bool,
        config: EmgMotorConfig,
    },
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EmgMotorConfig {
    pub threshold: f32,
    pub velocity: i16,
    pub maximum: f32,
    pub maximum_velocity: i16,
}

impl Default for EmgMotorConfig {
    fn default() -> Self {
        Self {
            threshold: 0.001,
            velocity: 0,
            maximum: 0.01,
            maximum_velocity: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NotchConfig {
    pub enabled: bool,
    pub frequency: u16,
    pub bandwidth: u16,
    pub attenuation: i16, // signed tenths of dB at center frequency
}

impl Default for NotchConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            frequency: 600,
            bandwidth: 20,
            attenuation: -30,
        }
    }
}

impl Action {
    pub fn description(&self) -> String {
        match self {
            Self::Ignore(_) => "Fourier ignore segments".into(),
            Self::NotchBank { board, .. } => format!("EMG {board}: notch filters"),
            Self::Motor(v) => format!("Motor {:.1} rad/s", *v as f32 / 10.0),
            Self::RetryMotorCan => "Retry Motor CAN (motor stays stopped)".into(),
            Self::EmgControl { board, .. } => format!("EMG {board}: motor control settings"),
            Self::Activate(kind, enabled) => {
                format!("{} {}", kind.name(), if *enabled { "on" } else { "off" })
            }
            Self::Property {
                board, property, ..
            } => format!(
                "EMG {board}: {}",
                ["amplifier gain", "ADC gain", "high-pass", "low-pass"][*property as usize]
            ),
            Self::Integration { board, .. } => format!("EMG {board}: integration settings"),
            Self::Fourier { board, .. } => format!("EMG {board}: Fourier settings"),
            Self::Notch { board, .. } => format!("EMG {board}: software notch filter"),
            Self::ResendHardware { board, .. } => {
                format!("EMG {board}: resend hardware parameters")
            }
        }
    }

    pub fn encode(&self) -> Result<Vec<u8>, String> {
        let mut out = Vec::new();
        match self {
            Self::Ignore(config) => {
                config.validate()?;
                out.extend([22, config.enabled as u8, config.segments.len() as u8]);
                for row in &config.segments {
                    out.push(row.enabled as u8);
                    out.extend(row.low_hz.to_le_bytes());
                    out.extend(row.high_hz.to_le_bytes());
                }
            }
            Self::NotchBank { board, filters } => {
                validate_board(*board)?;
                if filters.len() > 8 {
                    return Err("At most 8 notch filters per EMG".into());
                }
                out.extend([23, *board, filters.len() as u8]);
                for config in filters {
                    let encoded = Self::Notch {
                        board: *board,
                        config: config.clone(),
                    }
                    .encode()?;
                    out.extend_from_slice(&encoded[2..]);
                }
            }
            Self::RetryMotorCan => out.push(19),
            Self::EmgControl {
                board,
                enabled,
                config,
            } => {
                validate_board(*board)?;
                if !config.threshold.is_finite()
                    || config.threshold <= 0.0
                    || !config.maximum.is_finite()
                    || config.maximum <= config.threshold
                    || !(-500..=500).contains(&config.maximum_velocity)
                    || !(-500..=500).contains(&config.velocity)
                {
                    return Err(
                        "Require finite 0 < minimum peak < maximum peak and velocities within -50..50 rad/s".into(),
                    );
                }
                out.extend([24, *board, *enabled as u8]);
                out.extend(config.threshold.to_le_bytes());
                out.extend(config.maximum.to_le_bytes());
                out.extend(config.velocity.to_le_bytes());
                out.extend(config.maximum_velocity.to_le_bytes());
            }
            Self::Motor(value) => {
                if !(-500..=500).contains(value) {
                    return Err("Velocity must be −50.0 to +50.0 rad/s".into());
                }
                out.push(2);
                out.extend(value.to_le_bytes());
            }
            Self::Activate(kind, enabled) => {
                if matches!(*kind, Kind::Filtered | Kind::Detection) {
                    return Err("Use notch settings to enable filtered EMG".into());
                }
                out.extend([3 + *kind as u8, *enabled as u8]);
            }
            Self::Property {
                board,
                property,
                value,
            } => {
                validate_board(*board)?;
                let valid = match property {
                    0 | 1 => *value <= 128 && value.is_power_of_two(),
                    2 => (33..=709).contains(value),
                    3 => (1520..=4500).contains(value),
                    _ => false,
                };
                if !valid {
                    return Err("Invalid EMG property value".into());
                }
                out.extend([6, *board, *property]);
                if *property <= 1 {
                    out.push(*value as u8);
                } else {
                    out.extend(value.to_le_bytes());
                }
            }
            Self::Integration {
                board,
                milliseconds,
            } => {
                validate_board(*board)?;
                if !(10..=1000).contains(milliseconds) {
                    return Err("Integration window must be 10–1000 ms".into());
                }
                out.extend([7, *board]);
                out.extend(milliseconds.to_le_bytes());
            }
            Self::Fourier { board, config } => {
                validate_board(*board)?;
                if ![64, 128, 256].contains(&config.hop) || config.window > 1 {
                    return Err("Invalid Fourier settings".into());
                }
                out.extend([8, *board]);
                out.extend(config.hop.to_le_bytes());
                out.extend([config.window, config.remove_mean as u8]);
            }
            Self::ResendHardware {
                board,
                milliseconds,
            } => {
                validate_board(*board)?;
                if *milliseconds != 0 && *milliseconds < 100 {
                    return Err("Resend interval must be at least 100 ms".into());
                }
                out.extend([14, *board]);
                out.extend(milliseconds.to_le_bytes());
            }
            Self::Notch { board, config } => {
                validate_board(*board)?;
                if !(10..=4990).contains(&config.frequency)
                    || !(1..=1000).contains(&config.bandwidth)
                    || config.bandwidth >= 2 * config.frequency
                    || config.bandwidth >= 2 * (5000 - config.frequency)
                {
                    return Err("Notch: center 1–499 Hz, bandwidth 0.1–100 Hz; keep band edges inside 0–500 Hz".into());
                }
                if !(-600..=-1).contains(&config.attenuation) {
                    return Err("Center attenuation must be negative: −60.0 to −0.1 dB".into());
                }
                out.extend([17, *board, config.enabled as u8]);
                out.extend(config.frequency.to_le_bytes());
                out.extend(config.bandwidth.to_le_bytes());
                out.extend(config.attenuation.to_le_bytes());
            }
        }
        Ok(out)
    }
}

fn validate_board(board: u8) -> Result<(), String> {
    if board > 1 {
        Err("EMG index must be 0 or 1".into())
    } else {
        Ok(())
    }
}

#[derive(Debug)]
pub enum Failure {
    Device(u8),
    Rejected(String),
    Transport(String),
}

impl Failure {
    pub fn message(&self) -> String {
        match self {
            Self::Transport(message) | Self::Rejected(message) => message.clone(),
            Self::Device(code) => match code {
                1 => "Value rejected; settings are outside their allowed range",
                2 => "Connection has not completed INIT",
                3 => "EMG 0 configuration write/readback failed",
                4 => "EMG 1 configuration write/readback failed",
                5 => "Both EMG boards failed configuration write/readback",
                6 => {
                    "Not ready; acquisition must be enabled and processing needs continuous samples"
                }
                7 => "Motor unavailable: CAN is not ready or a motor fault is latched",
                8 => "EC5 protocol version mismatch",
                10 => "LP acquisition core unavailable; Refresh Acquisition Diagnostics, then explicitly retry Acquire EMGs",
                11 => "Manual motor commands are disabled while an EMG controls the motor",
                12 => "Motion request canceled by a newer Stop command",
                13 => "Motor is owned by Bluetooth or Angle Control; disable Bluetooth control to take over",
                14 => "SD configuration operation failed; check SD card and refresh BLE/SD diagnostics",
                _ => "Device reported an internal error",
            }
            .into(),
        }
    }
}

impl From<std::io::Error> for Failure {
    fn from(error: std::io::Error) -> Self {
        Self::Transport(error.to_string())
    }
}

#[derive(Clone, Debug)]
pub struct RawSample {
    pub time_us: u64,
    pub raw: i32,
    pub sequence: u32,
}

#[derive(Debug)]
pub struct RawBatch {
    pub discontinuity: bool,
    pub active: bool,
    pub configured: bool,
    pub hardware: Hardware,
    pub actual_hp: f32,
    pub actual_lp: f32,
    pub samples: Vec<RawSample>,
}

#[derive(Debug)]
pub struct Integration {
    pub time_us: u64,
    pub window_ms: u16,
    pub integral: f32,
    pub envelope: f32,
}

#[derive(Clone, Debug)]
pub struct Spectrum {
    pub time_us: u64,
    pub sample_rate: u16,
    pub config: FourierConfig,
    pub frame: u32,
    pub power: Vec<f32>,
}

#[derive(Debug)]
pub struct Acquisition {
    pub state: u32,
    pub fault: u32,
    pub generation: u32,
    pub capacity: u32,
    pub occupancy: u32,
    pub high_water: u32,
    pub configured_mask: u32,
    pub heartbeat: u32,
    pub conversions: [u32; 2],
    pub adc_gaps: [u32; 2],
    pub read_errors: [u32; 2],
    pub published: [u32; 2],
    pub discarded: [u32; 2],
    pub max_read_us: [u32; 2],
    pub raw_overwrites: [u32; 2],
    pub processing_overruns: [u32; 2],
    pub late_passes: u32,
    pub max_pass_us: u32,
    pub max_drain_us: u32,
}

#[derive(Debug)]
pub enum Reply {
    Control(crate::autoconfig::ControlState),
    HardwareErrors([[u32; 6]; 2]),
    Configuration(Box<crate::configuration::DeviceConfig>),
    Subscribed {
        session: u32,
        epoch: u32,
    },
    Motor(MotorStatus),
    MotorCan(MotorCan),
    Detection {
        overflow: bool,
        owner: Option<u8>,
        ready: bool,
        records: Vec<DetectionRecord>,
    },
    Init,
    Ack,
    Adjusted(f32),
    Raw(RawBatch),
    Filtered {
        discontinuity: bool,
        samples: Vec<FilteredSample>,
    },
    Integration(Integration),
    Fourier(Spectrum),
    Acquisition(Acquisition),
}

#[derive(Debug)]
pub struct DetectionRecord {
    pub time_us: u64,
    pub frame: u32,
    pub peak: f32,
    pub flags: u32,
    pub velocity: f32,
}

#[derive(Debug)]
pub struct FilteredSample {
    pub time_us: u64,
    pub millivolts: f32,
    pub sequence: u32,
}

#[derive(Debug)]
pub struct MotorStatus {
    pub flags: u32,
    pub fault_code: u32,
    pub inhibit_code: u32,
    pub requested: f32,
    pub commanded: f32,
    pub position: f32,
    pub velocity: f32,
    pub feedback_time_us: u64,
    pub reason: String,
    pub inhibit_reason: String,
}

#[derive(Debug)]
pub struct MotorCan {
    pub flags: u32,
    pub state: u32,
    pub tx_pending: u32,
    pub rx_pending: u32,
    pub tx_errors: u32,
    pub rx_errors: u32,
    pub tx_failed: u32,
    pub rx_missed: u32,
    pub rx_overrun: u32,
    pub arbitration_lost: u32,
    pub bus_errors: u32,
    pub alerts: u32,
    pub tx_queued: u32,
    pub tx_confirmed: u32,
    pub tx_rejected: u32,
    pub rx_frames: u32,
    pub rx_motor_frames: u32,
    pub last_error: u32,
    pub last_stage: u32,
    pub last_id: u32,
    pub last_parameter: u32,
    pub failure_state: u32,
    pub failure_tx_errors: u32,
    pub failure_rx_errors: u32,
    pub failure_bus_errors: u32,
    pub motor_faults: u32,
    pub motor_warnings: u32,
    pub last_rx_id: u32,
    pub feedback_faults: u32,
    pub feedback_faults_seen: u32,
    pub feedback_mode: u32,
    pub fault_frames: u32,
    pub motor_faults_seen: u32,
    pub motor_warnings_seen: u32,
    pub temperature_tenths: u32,
    pub feedback_frames: u32,
    pub failure_time_us: u64,
}

pub fn adc_mv_scale(gain: u8) -> f64 {
    2048.0 / (8_388_608.0 * gain as f64)
}

fn bytes<const N: usize>(io: &mut impl Read) -> Result<[u8; N], Failure> {
    let mut out = [0; N];
    io.read_exact(&mut out)?;
    Ok(out)
}
fn u16_value(io: &mut impl Read) -> Result<u16, Failure> {
    Ok(u16::from_le_bytes(bytes(io)?))
}
fn u32_value(io: &mut impl Read) -> Result<u32, Failure> {
    Ok(u32::from_le_bytes(bytes(io)?))
}
fn u64_value(io: &mut impl Read) -> Result<u64, Failure> {
    Ok(u64::from_le_bytes(bytes(io)?))
}
fn float_value(io: &mut impl Read) -> Result<f32, Failure> {
    let value = f32::from_le_bytes(bytes(io)?);
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Failure::Transport("Non-finite device result".into()))
    }
}

#[cfg(test)]
pub fn exchange(io: &mut (impl Read + Write), request: &[u8]) -> Result<Reply, Failure> {
    io.write_all(request)?;
    decode_stream(io, request)
}

pub fn decode_response(request: &[u8], response: &[u8]) -> Result<Reply, Failure> {
    let mut input = std::io::Cursor::new(response);
    let reply = decode_stream(&mut input, request)?;
    if input.position() != response.len() as u64 {
        return Err(Failure::Transport(
            "Trailing bytes in serial response".into(),
        ));
    }
    Ok(reply)
}

fn decode_stream(io: &mut impl Read, request: &[u8]) -> Result<Reply, Failure> {
    let response = bytes::<2>(io)?;
    if response[0] != request[0] {
        return Err(Failure::Transport(
            "Unexpected serial response command".into(),
        ));
    }
    if response[1] != 0 {
        return Err(Failure::Device(response[1]));
    }
    match request[0] {
        28..=32 | 34 => Ok(Reply::Ack),
        33 => {
            let body = bytes::<76>(io)?;
            Ok(Reply::Control(crate::autoconfig::ControlState::decode(
                &body,
            )?))
        }
        27 => {
            let mut boards = [[0; 6]; 2];
            for board in &mut boards {
                for word in board {
                    *word = u32_value(io)?;
                }
            }
            Ok(Reply::HardwareErrors(boards))
        }
        1 => {
            let version = u16_value(io)?;
            let rate = u16_value(io)?;
            let size = u16_value(io)?;
            let capacity = u16_value(io)?;
            if (version, rate, size, capacity) != (VERSION, 1000, 256, RAW_CAPACITY as u16) {
                return Err(Failure::Transport("Incompatible EC5 capabilities".into()));
            }
            let _session = u32_value(io)?;
            if bytes::<1>(io)?[0] != 10 {
                return Err(Failure::Transport("Invalid stream count".into()));
            }
            for size in crate::telemetry::RECORD_SIZES {
                if u16_value(io)? as usize != size {
                    return Err(Failure::Transport("Incompatible telemetry layout".into()));
                }
            }
            let window = bytes::<1>(io)?[0];
            let maximum = u16_value(io)?;
            if !(1..=16).contains(&window) || maximum < 163 {
                return Err(Failure::Transport(
                    "Incompatible EC5 serial capabilities".into(),
                ));
            }
            Ok(Reply::Init)
        }
        25 => {
            let length = u16_value(io)? as usize;
            if length > 900 {
                return Err(Failure::Transport("Configuration exceeds capacity".into()));
            }
            let mut data = vec![0; length];
            io.read_exact(&mut data)?;
            crate::configuration::DeviceConfig::decode(&data)
                .map(|c| Reply::Configuration(Box::new(c)))
                .map_err(Failure::Transport)
        }
        26 => Ok(Reply::Subscribed {
            session: u32_value(io)?,
            epoch: u32_value(io)?,
        }),
        6 => {
            let value = u16_value(io)?;
            let expected = if request[2] <= 1 {
                request[3] as u16
            } else {
                u16::from_le_bytes([request[3], request[4]])
            };
            if value != expected {
                return Err(Failure::Transport(
                    "Adjustment reply does not match request".into(),
                ));
            }
            Ok(Reply::Adjusted(float_value(io)?))
        }
        9 => {
            let discontinuity = bytes::<1>(io)?[0] != 0;
            let count = u16_value(io)? as usize;
            if count > RAW_CAPACITY {
                return Err(Failure::Transport("Raw batch exceeds capacity".into()));
            }
            let config = bytes::<4>(io)?;
            if config[0] > 1
                || config[1] > 1
                || !config[2].is_power_of_two()
                || !config[3].is_power_of_two()
            {
                return Err(Failure::Transport(
                    "Invalid board configuration in raw reply".into(),
                ));
            }
            let hardware = Hardware {
                ina_gain: config[2],
                adc_gain: config[3],
                high_pass: u16_value(io)?,
                low_pass: u16_value(io)?,
            };
            let actual_hp = float_value(io)?;
            let actual_lp = float_value(io)?;
            let mut packed = vec![0; count * 16];
            io.read_exact(&mut packed)?;
            let mut records = packed.as_slice();
            let mut samples = Vec::with_capacity(count);
            for _ in 0..count {
                samples.push(RawSample {
                    time_us: u64_value(&mut records)?,
                    raw: i32::from_le_bytes(bytes(&mut records)?),
                    sequence: u32_value(&mut records)?,
                });
            }
            Ok(Reply::Raw(RawBatch {
                discontinuity,
                active: config[0] != 0,
                configured: config[1] != 0,
                hardware,
                actual_hp,
                actual_lp,
                samples,
            }))
        }
        11 => {
            if bytes::<1>(io)?[0] != 1 {
                return Err(Failure::Transport("Invalid integration readiness".into()));
            }
            Ok(Reply::Integration(Integration {
                time_us: u64_value(io)?,
                window_ms: u16_value(io)?,
                integral: float_value(io)?,
                envelope: float_value(io)?,
            }))
        }
        10 => {
            if bytes::<1>(io)?[0] != 1 {
                return Err(Failure::Transport("Invalid Fourier readiness".into()));
            }
            let time_us = u64_value(io)?;
            let sample_rate = u16_value(io)?;
            let size = u16_value(io)?;
            let hop = u16_value(io)?;
            let flags = bytes::<2>(io)?;
            let frame = u32_value(io)?;
            if sample_rate != 1000
                || size != 256
                || ![64, 128, 256].contains(&hop)
                || flags[0] > 1
                || flags[1] > 1
            {
                return Err(Failure::Transport("Invalid Fourier layout".into()));
            }
            let packed = bytes::<{ FFT_BINS * 4 }>(io)?;
            let mut records = packed.as_slice();
            let mut power = Vec::with_capacity(FFT_BINS);
            for _ in 0..FFT_BINS {
                power.push(float_value(&mut records)?);
            }
            Ok(Reply::Fourier(Spectrum {
                time_us,
                sample_rate,
                config: FourierConfig {
                    hop,
                    window: flags[0],
                    remove_mean: flags[1] != 0,
                },
                frame,
                power,
            }))
        }
        12 => {
            let mut w = [0u32; 27];
            for word in &mut w {
                *word = u32_value(io)?;
            }
            if w[0] > 3 || w[3] != 4096 || w[4] > w[3] || w[5] > w[3] {
                return Err(Failure::Transport(
                    "Invalid acquisition diagnostics layout".into(),
                ));
            }
            Ok(Reply::Acquisition(Acquisition {
                state: w[0],
                fault: w[1],
                generation: w[2],
                capacity: w[3],
                occupancy: w[4],
                high_water: w[5],
                configured_mask: w[6],
                heartbeat: w[7],
                conversions: [w[8], w[9]],
                adc_gaps: [w[10], w[11]],
                read_errors: [w[12], w[13]],
                published: [w[14], w[15]],
                discarded: [w[16], w[17]],
                max_read_us: [w[18], w[19]],
                raw_overwrites: [w[20], w[21]],
                processing_overruns: [w[22], w[23]],
                late_passes: w[24],
                max_pass_us: w[25],
                max_drain_us: w[26],
            }))
        }
        13 => {
            let flags = u32_value(io)?;
            let fault_code = u32_value(io)?;
            let inhibit_code = u32_value(io)?;
            let requested = float_value(io)?;
            let commanded = float_value(io)?;
            let position = float_value(io)?;
            let velocity = float_value(io)?;
            let feedback_time_us = u64_value(io)?;
            let reason = bytes::<96>(io)?;
            let inhibit = bytes::<96>(io)?;
            let text = |bytes: &[u8]| {
                let end = bytes
                    .iter()
                    .position(|byte| *byte == 0)
                    .unwrap_or(bytes.len());
                String::from_utf8_lossy(&bytes[..end]).into_owned()
            };
            Ok(Reply::Motor(MotorStatus {
                flags,
                fault_code,
                inhibit_code,
                requested,
                commanded,
                position,
                velocity,
                feedback_time_us,
                reason: text(&reason),
                inhibit_reason: text(&inhibit),
            }))
        }
        21 => {
            let overflow = bytes::<1>(io)?[0];
            let count = u16_value(io)? as usize;
            let context = bytes::<2>(io)?;
            if overflow > 1 || count > 128 || ![0, 1, 255].contains(&context[0]) || context[1] > 1 {
                return Err(Failure::Transport("Invalid detection batch header".into()));
            }
            let mut packed = vec![0; count * 24];
            io.read_exact(&mut packed)?;
            let mut data = packed.as_slice();
            let mut records = Vec::with_capacity(count);
            for _ in 0..count {
                let record = DetectionRecord {
                    time_us: u64_value(&mut data)?,
                    frame: u32_value(&mut data)?,
                    peak: float_value(&mut data)?,
                    flags: u32_value(&mut data)?,
                    velocity: float_value(&mut data)?,
                };
                if record.flags & !7 != 0 {
                    return Err(Failure::Transport("Invalid detection flags".into()));
                }
                records.push(record);
            }
            Ok(Reply::Detection {
                overflow: overflow != 0,
                owner: if context[0] == 255 {
                    None
                } else {
                    Some(context[0])
                },
                ready: context[1] != 0,
                records,
            })
        }
        18 => {
            let mut w = [0; 36];
            for word in &mut w {
                *word = u32_value(io)?;
            }
            Ok(Reply::MotorCan(MotorCan {
                flags: w[0],
                state: w[1],
                tx_pending: w[2],
                rx_pending: w[3],
                tx_errors: w[4],
                rx_errors: w[5],
                tx_failed: w[6],
                rx_missed: w[7],
                rx_overrun: w[8],
                arbitration_lost: w[9],
                bus_errors: w[10],
                alerts: w[11],
                tx_queued: w[12],
                tx_confirmed: w[13],
                tx_rejected: w[14],
                rx_frames: w[15],
                rx_motor_frames: w[16],
                last_error: w[17],
                last_stage: w[18],
                last_id: w[19],
                last_parameter: w[20],
                failure_state: w[21],
                failure_tx_errors: w[22],
                failure_rx_errors: w[23],
                failure_bus_errors: w[24],
                motor_faults: w[25],
                motor_warnings: w[26],
                last_rx_id: w[27],
                feedback_faults: w[28],
                feedback_faults_seen: w[29],
                feedback_mode: w[30],
                fault_frames: w[31],
                motor_faults_seen: w[32],
                motor_warnings_seen: w[33],
                temperature_tenths: w[34],
                feedback_frames: w[35],
                failure_time_us: u64_value(io)?,
            }))
        }
        16 => {
            let overflow = bytes::<1>(io)?[0];
            let count = u16_value(io)? as usize;
            if overflow > 1 || count > RAW_CAPACITY {
                return Err(Failure::Transport("Invalid filtered batch header".into()));
            }
            let mut packed = vec![0; count * 16];
            io.read_exact(&mut packed)?;
            let mut records = packed.as_slice();
            let mut samples = Vec::with_capacity(count);
            for _ in 0..count {
                samples.push(FilteredSample {
                    time_us: u64_value(&mut records)?,
                    millivolts: float_value(&mut records)?,
                    sequence: u32_value(&mut records)?,
                });
            }
            Ok(Reply::Filtered {
                discontinuity: overflow != 0,
                samples,
            })
        }
        2..=5 | 7 | 8 | 14 | 15 | 17 | 19 | 20 | 22..=24 => Ok(Reply::Ack),
        _ => Err(Failure::Transport("Unknown response command".into())),
    }
}

#[cfg(test)]
pub fn query(kind: Kind, board: u8) -> [u8; 2] {
    [
        match kind {
            Kind::Raw => 9,
            Kind::Integration => 11,
            Kind::Fourier => 10,
            Kind::Filtered => 16,
            Kind::Detection => 21,
        },
        board,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn hardware_error_snapshot_has_two_complete_board_records() {
        let expected = [[1u32, 0x20, 1, 0x58, 0, 0x108], [3, 0x41, 0x24, 0xc8, 0, 0]];
        let mut response = vec![27, 0];
        for board in expected {
            for value in board {
                response.extend(value.to_le_bytes());
            }
        }
        let mut io = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        match exchange(&mut io, &[27]).unwrap() {
            Reply::HardwareErrors(actual) => assert_eq!(actual, expected),
            _ => panic!("Expected both hardware failure records"),
        }
    }

    struct Fragmented {
        input: Cursor<Vec<u8>>,
        written: Vec<u8>,
    }
    impl Read for Fragmented {
        fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
            let size = out.len().min(1);
            self.input.read(&mut out[..size])
        }
    }
    impl Write for Fragmented {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.written.push(bytes[0]);
            Ok(1)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn emg_control_and_detection_match_fixed_binary_protocol() {
        for velocity in [-500i16, -20, 0, 20, 500] {
            let config = EmgMotorConfig {
                velocity,
                ..EmgMotorConfig::default()
            };
            let encoded = Action::EmgControl {
                board: 1,
                enabled: true,
                config,
            }
            .encode()
            .unwrap();
            assert_eq!(encoded.len(), 15);
            assert_eq!(&encoded[..3], &[24, 1, 1]);
            assert_eq!(&encoded[3..7], &0.001f32.to_le_bytes());
            assert_eq!(&encoded[7..11], &0.01f32.to_le_bytes());
            assert_eq!(&encoded[11..13], &velocity.to_le_bytes());
            assert_eq!(&encoded[13..15], &0i16.to_le_bytes());
        }
        for threshold in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(
                Action::EmgControl {
                    board: 0,
                    enabled: true,
                    config: EmgMotorConfig {
                        threshold,
                        ..EmgMotorConfig::default()
                    }
                }
                .encode()
                .is_err()
            );
        }
        let mut response = vec![21, 0, 0, 1, 0, 1, 1];
        response.extend(1234u64.to_le_bytes());
        response.extend(9u32.to_le_bytes());
        response.extend(0.01f32.to_le_bytes());
        response.extend(7u32.to_le_bytes());
        response.extend((-2f32).to_le_bytes());
        let mut io = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        match exchange(&mut io, &[21, 1]).unwrap() {
            Reply::Detection {
                owner,
                ready,
                records,
                ..
            } => {
                assert_eq!(owner, Some(1));
                assert!(ready);
                assert_eq!(records.len(), 1);
                assert_eq!(records[0].frame, 9);
                assert_eq!(records[0].flags, 7);
                assert_eq!(records[0].velocity, -2.0);
            }
            _ => panic!("Expected detection records"),
        }
    }

    #[test]
    fn motor_can_diagnostics_and_explicit_retry_use_fixed_binary_layouts() {
        assert_eq!(Action::RetryMotorCan.encode().unwrap(), [19]);
        let mut ack = Fragmented {
            input: Cursor::new(vec![19, 0]),
            written: Vec::new(),
        };
        assert!(matches!(exchange(&mut ack, &[19]), Ok(Reply::Ack)));
        let mut response = vec![18, 0];
        let mut words = [0u32; 36];
        words[0] = 1;
        words[1] = 1;
        words[4] = 128;
        words[13] = 2;
        words[17] = 0x107;
        words[18] = 3;
        words[19] = 0x1200FD7F;
        words[20] = 0x7005;
        words[21] = 1;
        words[22] = 128;
        words[24] = 17;
        words[28] = 3;
        words[29] = 7;
        words[31] = 1;
        words[32] = 0x4085;
        for word in words {
            response.extend(word.to_le_bytes());
        }
        response.extend(1234567u64.to_le_bytes());
        let mut io = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        match exchange(&mut io, &[18]).unwrap() {
            Reply::MotorCan(d) => {
                assert_eq!(d.state, 1);
                assert_eq!(d.tx_errors, 128);
                assert_eq!(d.tx_confirmed, 2);
                assert_eq!(d.last_error, 0x107);
                assert_eq!(d.last_stage, 3);
                assert_eq!(d.last_id, 0x1200FD7F);
                assert_eq!(d.last_parameter, 0x7005);
                assert_eq!(d.failure_bus_errors, 17);
                assert_eq!(d.feedback_faults, 3);
                assert_eq!(d.feedback_faults_seen, 7);
                assert_eq!(d.fault_frames, 1);
                assert_eq!(d.motor_faults_seen, 0x4085);
                assert_eq!(d.failure_time_us, 1234567);
            }
            _ => panic!("Expected CAN diagnostics"),
        }
    }

    #[test]
    fn notch_center_attenuation_requires_negative_signed_db() {
        for attenuation in [-1i16, -30, -90, -600] {
            let config = NotchConfig {
                enabled: true,
                attenuation,
                ..NotchConfig::default()
            };
            let request = Action::Notch { board: 0, config }.encode().unwrap();
            let [low, high] = attenuation.to_le_bytes();
            assert_eq!(request, [17, 0, 1, 88, 2, 20, 0, low, high]);
            let mut io = Fragmented {
                input: Cursor::new(vec![17, 0]),
                written: Vec::new(),
            };
            assert!(matches!(exchange(&mut io, &request), Ok(Reply::Ack)));
        }
        for attenuation in [0, 1, 90, -601, i16::MIN, i16::MAX] {
            assert!(
                Action::Notch {
                    board: 0,
                    config: NotchConfig {
                        attenuation,
                        ..NotchConfig::default()
                    },
                }
                .encode()
                .is_err()
            );
        }
    }

    #[test]
    fn notch_settings_and_filtered_records_match_firmware_wire_layout() {
        let config = NotchConfig {
            enabled: true,
            ..NotchConfig::default()
        };
        assert_eq!(
            Action::Notch { board: 1, config }.encode().unwrap(),
            [17, 1, 1, 88, 2, 20, 0, 226, 255]
        );
        for (frequency, bandwidth) in [(0, 20), (5000, 20), (600, 0), (600, 1001), (10, 20)] {
            assert!(
                Action::Notch {
                    board: 0,
                    config: NotchConfig {
                        enabled: true,
                        frequency,
                        bandwidth,
                        attenuation: -30,
                    },
                }
                .encode()
                .is_err()
            );
        }
        let mut response = vec![16, 0, 1, 1, 0];
        response.extend(123456u64.to_le_bytes());
        response.extend((-2.5f32).to_le_bytes());
        response.extend(u32::MAX.to_le_bytes());
        let mut io = Fragmented {
            input: Cursor::new(response.clone()),
            written: Vec::new(),
        };
        match exchange(&mut io, &query(Kind::Filtered, 1)).unwrap() {
            Reply::Filtered {
                discontinuity,
                samples,
            } => {
                assert!(discontinuity);
                assert_eq!(samples.len(), 1);
                assert_eq!(samples[0].time_us, 123456);
                assert_eq!(samples[0].millivolts, -2.5);
                assert_eq!(samples[0].sequence, u32::MAX);
            }
            _ => panic!("Expected filtered EMG"),
        }
        response.pop();
        let mut truncated = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        assert!(exchange(&mut truncated, &[16, 1]).is_err());
        let mut oversized = Fragmented {
            input: Cursor::new(vec![16, 0, 0, 1, 4]),
            written: Vec::new(),
        };
        assert!(exchange(&mut oversized, &[16, 0]).is_err());
    }

    #[test]
    fn hardware_resend_encodes_intervals_and_accepts_empty_ack() {
        for board in [0, 1] {
            for milliseconds in [0u16, 100, 1000, u16::MAX] {
                let request = Action::ResendHardware {
                    board,
                    milliseconds,
                }
                .encode()
                .unwrap();
                let [low, high] = milliseconds.to_le_bytes();
                assert_eq!(request, [14, board, low, high]);
                let mut io = Fragmented {
                    input: Cursor::new(vec![14, 0]),
                    written: Vec::new(),
                };
                assert!(matches!(exchange(&mut io, &request), Ok(Reply::Ack)));
                assert_eq!(io.written, request);
            }
        }
        for (board, milliseconds) in [(2, 1000), (0, 1), (1, 99)] {
            assert!(
                Action::ResendHardware {
                    board,
                    milliseconds
                }
                .encode()
                .is_err()
            );
        }
    }

    #[test]
    fn commands_match_c_wire_vectors_and_validate_limits() {
        for value in [-500i16, -1, 0, 1, 500] {
            let encoded = Action::Motor(value).encode().unwrap();
            assert_eq!(encoded[0], 2);
            assert_eq!(&encoded[1..], &value.to_le_bytes());
        }
        assert!(Action::Motor(501).encode().is_err());
        assert_eq!(
            Action::Property {
                board: 1,
                property: 2,
                value: 200
            }
            .encode()
            .unwrap(),
            [6, 1, 2, 200, 0]
        );
        assert_eq!(
            Action::Property {
                board: 0,
                property: 0,
                value: 128
            }
            .encode()
            .unwrap(),
            [6, 0, 0, 128]
        );
        assert!(
            Action::Property {
                board: 0,
                property: 0,
                value: 256
            }
            .encode()
            .is_err()
        );
        assert!(
            Action::Property {
                board: 2,
                property: 0,
                value: 1
            }
            .encode()
            .is_err()
        );
        assert!(
            Action::Integration {
                board: 0,
                milliseconds: 0
            }
            .encode()
            .is_err()
        );
    }

    #[test]
    fn fragmented_reads_and_writes_preserve_response_boundaries() {
        let mut response = vec![1, 0, 1, 0, 232, 3, 0, 1, 0, 4];
        response.extend(42u32.to_le_bytes());
        response.push(10);
        for size in crate::telemetry::RECORD_SIZES {
            response.extend((size as u16).to_le_bytes());
        }
        response.extend([4, 255, 255]);
        response.extend([2, 0]);
        let mut io = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        assert!(matches!(exchange(&mut io, &[1, 4]), Ok(Reply::Init)));
        assert!(matches!(exchange(&mut io, &[2, 0, 0]), Ok(Reply::Ack)));
        assert_eq!(io.written, [1, 4, 2, 0, 0]);
    }

    #[test]
    fn rejects_wrong_headers_oversized_batches_and_truncated_results() {
        let mut wrong = Fragmented {
            input: Cursor::new(vec![8, 0]),
            written: Vec::new(),
        };
        assert!(matches!(
            exchange(&mut wrong, &[2, 0, 0]),
            Err(Failure::Transport(_))
        ));
        let mut large = Fragmented {
            input: Cursor::new(vec![9, 0, 0, 1, 4]),
            written: Vec::new(),
        };
        assert!(matches!(
            exchange(&mut large, &[9, 0]),
            Err(Failure::Transport(_))
        ));
        let mut short = Fragmented {
            input: Cursor::new(vec![11, 0, 1]),
            written: Vec::new(),
        };
        assert!(matches!(
            exchange(&mut short, &[11, 0]),
            Err(Failure::Transport(_))
        ));
    }

    #[test]
    fn acquisition_diagnostics_use_27_binary_words_in_firmware_order() {
        let words: [u32; 27] = [
            2, 0, 7, 4096, 12, 100, 3, 2000, 1000, 999, 1, 2, 3, 4, 990, 989, 5, 6, 180, 190, 50,
            60, 0, 1, 8, 400, 1100,
        ];
        let mut response = vec![12, 0];
        for word in words {
            response.extend(word.to_le_bytes());
        }
        let mut io = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        match exchange(&mut io, &[12]).unwrap() {
            Reply::Acquisition(d) => {
                assert_eq!(d.state, 2);
                assert_eq!(d.capacity, 4096);
                assert_eq!(d.read_errors, [3, 4]);
                assert_eq!(d.discarded, [5, 6]);
                assert_eq!(d.max_drain_us, 1100);
            }
            _ => panic!("Expected acquisition diagnostics"),
        }
        assert_eq!(io.written, [12]);
    }

    #[test]
    fn rejects_ec3_firmware_during_init() {
        let mut io = Fragmented {
            input: Cursor::new(vec![1, 0, 1, 0, 232, 3, 0, 1, 0, 4]),
            written: Vec::new(),
        };
        assert!(matches!(
            exchange(&mut io, &[1, 2, 0]),
            Err(Failure::Transport(_))
        ));
    }

    #[test]
    fn raw_adc_scaling_matches_reference_sketch_in_millivolts() {
        for gain in [1u8, 2, 4, 8, 16, 32, 64, 128] {
            for raw in [
                -8_388_608i32,
                -4_096_000,
                -256,
                -1,
                0,
                1,
                256,
                4_096_000,
                8_388_607,
            ] {
                let reference = (raw as f64 / 8_388_608.0) * (2.048 / gain as f64) * 1000.0;
                assert!((raw as f64 * adc_mv_scale(gain) - reference).abs() < 1e-9);
            }
        }
        assert_eq!(4_096_000.0 * adc_mv_scale(1), 1000.0);
        assert_eq!(-4_096_000.0 * adc_mv_scale(1), -1000.0);
    }

    #[test]
    fn motor_status_decodes_actual_fault_and_feedback_without_json() {
        let mut response = vec![13, 0];
        for value in [11u32, 6, 1] {
            response.extend(value.to_le_bytes());
        }
        for value in [1.5f32, 0.0, 0.25, 0.0] {
            response.extend(value.to_le_bytes());
        }
        response.extend(123456u64.to_le_bytes());
        let mut reason = [0u8; 96];
        let message = b"Motor feedback not received after zero-speed enable";
        reason[..message.len()].copy_from_slice(message);
        response.extend(reason);
        response.extend([0; 96]);
        assert_eq!(response.len(), 230);
        let mut io = Fragmented {
            input: Cursor::new(response),
            written: Vec::new(),
        };
        match exchange(&mut io, &[13]).unwrap() {
            Reply::Motor(status) => {
                assert_eq!(status.fault_code, 6);
                assert_eq!(status.requested, 1.5);
                assert_eq!(status.commanded, 0.0);
                assert_eq!(status.feedback_time_us, 123456);
                assert!(status.reason.contains("zero-speed enable"));
            }
            _ => panic!("Expected motor status"),
        }
    }
}
