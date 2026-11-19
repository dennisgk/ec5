use crate::{
    connection::Poll,
    fourier_view::{FourierIgnore, IgnoreSegment},
    protocol::{
        Action, EmgMotorConfig, Failure, FourierConfig, Hardware, Kind, NotchConfig, Reply,
    },
    serial_io,
};
use serde::{Deserialize, Serialize};
use std::{
    io::{Cursor, Read},
    path::Path,
};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeviceBoard {
    pub hardware: Hardware,
    pub resend_ms: u16,
    pub integration_ms: u16,
    pub fourier: FourierConfig,
    pub notches: Vec<NotchConfig>,
    pub curve: EmgMotorConfig,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeviceConfig {
    #[serde(skip)]
    pub available: [bool; 2],
    #[serde(skip)]
    pub revision: u32,
    pub enabled: [bool; 3],
    pub owner: Option<u8>,
    pub ignore: FourierIgnore,
    pub boards: [DeviceBoard; 2],
}

struct Decoder<'a>(Cursor<&'a [u8]>);
impl Decoder<'_> {
    fn bytes<const N: usize>(&mut self) -> Result<[u8; N], String> {
        let mut b = [0; N];
        self.0.read_exact(&mut b).map_err(|e| e.to_string())?;
        Ok(b)
    }
    fn byte(&mut self) -> Result<u8, String> {
        Ok(self.bytes::<1>()?[0])
    }
    fn boolean(&mut self) -> Result<bool, String> {
        match self.byte()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err("Invalid configuration boolean".into()),
        }
    }
    fn short(&mut self) -> Result<u16, String> {
        Ok(u16::from_le_bytes(self.bytes()?))
    }
    fn float(&mut self) -> Result<f32, String> {
        let x = f32::from_le_bytes(self.bytes()?);
        if x.is_finite() {
            Ok(x)
        } else {
            Err("Non-finite configuration value".into())
        }
    }
}

impl DeviceConfig {
    pub fn decode(data: &[u8]) -> Result<Self, String> {
        let mut d = Decoder(Cursor::new(data));
        let revision = u32::from_le_bytes(d.bytes()?);
        let enabled = [d.boolean()?, d.boolean()?, d.boolean()?];
        let owner = match d.byte()? {
            255 => None,
            b @ 0..=1 => Some(b),
            _ => return Err("Invalid controller owner".into()),
        };
        let mut ignore = FourierIgnore {
            enabled: d.boolean()?,
            segments: Vec::new(),
            mask: [0; 17],
        };
        let count = d.byte()?;
        if count > 16 {
            return Err("Too many ignore segments".into());
        }
        for _ in 0..count {
            ignore.segments.push(IgnoreSegment {
                enabled: d.boolean()?,
                low_hz: d.float()?,
                high_hz: d.float()?,
            });
        }
        ignore.mask = d.bytes()?;
        let mut boards = Vec::new();
        for _ in 0..2 {
            let hardware = Hardware {
                ina_gain: d.byte()?,
                adc_gain: d.byte()?,
                high_pass: d.short()?,
                low_pass: d.short()?,
            };
            let resend_ms = d.short()?;
            let integration_ms = d.short()?;
            let fourier = FourierConfig {
                hop: d.short()?,
                window: d.byte()?,
                remove_mean: d.boolean()?,
            };
            let count = d.byte()?;
            if count > 8 {
                return Err("Too many notch filters".into());
            }
            let mut notches = Vec::new();
            for _ in 0..count {
                notches.push(NotchConfig {
                    enabled: d.boolean()?,
                    frequency: d.short()?,
                    bandwidth: d.short()?,
                    attenuation: d.short()? as i16,
                });
            }
            let curve = EmgMotorConfig {
                threshold: d.float()?,
                maximum: d.float()?,
                velocity: d.short()? as i16,
                maximum_velocity: d.short()? as i16,
            };
            boards.push(DeviceBoard {
                hardware,
                resend_ms,
                integration_ms,
                fourier,
                notches,
                curve,
            });
        }
        let available = if d.0.position() + 1 == data.len() as u64 {
            let mask = d.byte()?;
            if mask & !3 != 0 {
                return Err("Invalid EMG availability mask".into());
            }
            [mask & 1 != 0, mask & 2 != 0]
        } else {
            // Older protocol-3 snapshots did not include availability.
            [enabled[0]; 2]
        };
        if d.0.position() != data.len() as u64 {
            return Err("Trailing configuration bytes".into());
        }
        let result = Self {
            available,
            revision,
            enabled,
            owner,
            ignore,
            boards: boards.try_into().map_err(|_| "Missing boards")?,
        };
        result.validate()?;
        Ok(result)
    }

    pub fn validate(&self) -> Result<(), String> {
        self.ignore.validate()?;
        if self.owner.is_some_and(|b| b > 1)
            || (self.owner.is_some() && (!self.enabled[0] || !self.enabled[2]))
            || (!self.enabled[0] && (self.enabled[1] || self.enabled[2]))
        {
            return Err("Acquisition/Fourier must be enabled for the saved EMG controller".into());
        }
        for (board, b) in self.boards.iter().enumerate() {
            for action in self.board_actions(board as u8, b) {
                action.encode()?;
            }
        }
        Ok(())
    }

    fn board_actions(&self, board: u8, b: &DeviceBoard) -> Vec<Action> {
        let h = &b.hardware;
        let mut actions = Vec::new();
        for (property, value) in [
            h.ina_gain as u16,
            h.adc_gain as u16,
            h.high_pass,
            h.low_pass,
        ]
        .into_iter()
        .enumerate()
        {
            actions.push(Action::Property {
                board,
                property: property as u8,
                value,
            });
        }
        actions.extend([
            Action::NotchBank {
                board,
                filters: b.notches.clone(),
            },
            Action::Integration {
                board,
                milliseconds: b.integration_ms,
            },
            Action::Fourier {
                board,
                config: b.fourier.clone(),
            },
            Action::EmgControl {
                board,
                enabled: false,
                config: b.curve.clone(),
            },
            Action::ResendHardware {
                board,
                milliseconds: b.resend_ms,
            },
        ]);
        actions
    }

    // Ordered and bounded: acquisition is paused, motor ownership is restored
    // only after every setting has been applied and independently read back.
    pub fn apply_inactive(
        &self,
        socket: &mut impl crate::serial_io::CommandConnection,
    ) -> Result<Vec<String>, Failure> {
        let mut warnings = Vec::new();
        self.validate().map_err(Failure::Rejected)?;
        send(socket, &Action::Motor(0))?;
        send(socket, &Action::Activate(Kind::Raw, false))?;
        send(socket, &Action::Ignore(self.ignore.clone()))?;
        for (board, b) in self.boards.iter().enumerate() {
            for action in self.board_actions(board as u8, b) {
                if !matches!(action, Action::ResendHardware { .. }) {
                    send(socket, &action)?;
                }
            }
        }
        if self.enabled[0] {
            match serial_io::exchange(socket, &[3, 1]) {
                Ok(_) => {}
                Err(Failure::Device(code @ 3..=5)) => {
                    let mut warning = Failure::Device(code).message();
                    if let Reply::HardwareErrors(boards) = serial_io::exchange(socket, &[27])? {
                        warning.push_str(&crate::connection::hardware_failure_details(boards));
                    }
                    warnings.push(warning);
                }
                Err(error) => return Err(error),
            }
            for (kind, on) in [
                (Kind::Integration, self.enabled[1]),
                (Kind::Fourier, self.enabled[2]),
            ] {
                send(socket, &Action::Activate(kind, on))?;
            }
        }
        for (board, b) in self.boards.iter().enumerate() {
            send(
                socket,
                &Action::ResendHardware {
                    board: board as u8,
                    milliseconds: b.resend_ms,
                },
            )?;
        }
        let confirmed = read(socket)?;
        let mut expected = self.clone();
        expected.owner = None;
        expected.revision = 0;
        expected.ignore.mask = [0; 17];
        expected.available = [false; 2];
        let mut actual = confirmed;
        actual.revision = 0;
        actual.ignore.mask = [0; 17];
        actual.available = [false; 2];
        if expected != actual {
            return Err(Failure::Rejected(
                "Loaded configuration readback differs; motor remains disarmed".into(),
            ));
        }
        Ok(warnings)
    }
}

pub fn send(
    socket: &mut impl crate::serial_io::CommandConnection,
    action: &Action,
) -> Result<(), Failure> {
    let request = action.encode().map_err(Failure::Rejected)?;
    serial_io::exchange(socket, &request)
        .map(|_| ())
        .map_err(|error| match error {
            Failure::Device(code) => Failure::Rejected(format!(
                "{}: {}",
                action.description(),
                Failure::Device(code).message()
            )),
            other => other,
        })
}

pub fn read(
    socket: &mut impl crate::serial_io::CommandConnection,
) -> Result<DeviceConfig, Failure> {
    match serial_io::exchange(socket, &[25])? {
        Reply::Configuration(config) => Ok(*config),
        _ => Err(Failure::Transport("Missing configuration response".into())),
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DisplayConfig {
    #[serde(default)]
    pub serial_port: String,
    pub plots: [[Poll; 5]; 2],
    pub x_seconds: f64,
    pub autoscroll: bool,
    pub autofit_y: bool,
    pub envelope: [bool; 2],
    pub manual_velocity: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Profile {
    #[serde(default)]
    pub motor: crate::autoconfig::MotorConfig,
    #[serde(default)]
    pub project: String,
    pub schema_version: u32,
    pub device: DeviceConfig,
    pub display: DisplayConfig,
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::*;

    pub fn configuration() -> DeviceConfig {
        DeviceConfig {
            available: [true; 2],
            revision: 2,
            enabled: [true, true, true],
            owner: None,
            ignore: FourierIgnore::default(),
            boards: std::array::from_fn(|_| DeviceBoard {
                hardware: Hardware::default(),
                resend_ms: 0,
                integration_ms: 100,
                fourier: FourierConfig::default(),
                notches: Vec::new(),
                curve: EmgMotorConfig::default(),
            }),
        }
    }

    // Independent byte fixture in the documented C encoder order.
    pub fn response(c: &DeviceConfig) -> Vec<u8> {
        let mut blob = c.revision.to_le_bytes().to_vec();
        blob.extend(c.enabled.map(u8::from));
        blob.push(c.owner.unwrap_or(255));
        blob.extend([c.ignore.enabled as u8, c.ignore.segments.len() as u8]);
        for row in &c.ignore.segments {
            blob.push(row.enabled as u8);
            blob.extend(row.low_hz.to_le_bytes());
            blob.extend(row.high_hz.to_le_bytes());
        }
        blob.extend(c.ignore.mask);
        for b in &c.boards {
            blob.extend([b.hardware.ina_gain, b.hardware.adc_gain]);
            for value in [
                b.hardware.high_pass,
                b.hardware.low_pass,
                b.resend_ms,
                b.integration_ms,
                b.fourier.hop,
            ] {
                blob.extend(value.to_le_bytes());
            }
            blob.extend([
                b.fourier.window,
                b.fourier.remove_mean as u8,
                b.notches.len() as u8,
            ]);
            for n in &b.notches {
                blob.push(n.enabled as u8);
                blob.extend(n.frequency.to_le_bytes());
                blob.extend(n.bandwidth.to_le_bytes());
                blob.extend(n.attenuation.to_le_bytes());
            }
            blob.extend(b.curve.threshold.to_le_bytes());
            blob.extend(b.curve.maximum.to_le_bytes());
            blob.extend(b.curve.velocity.to_le_bytes());
            blob.extend(b.curve.maximum_velocity.to_le_bytes());
        }
        blob.push(c.available[0] as u8 | ((c.available[1] as u8) << 1));
        let mut response = vec![25, 0];
        response.extend((blob.len() as u16).to_le_bytes());
        response.extend(blob);
        response
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_support::*;

    #[test]
    fn configuration_roundtrip_validates_banks_masks_and_trailing_data() {
        let mut expected = configuration();
        expected.ignore.enabled = true;
        expected.ignore.mask[2] = 1;
        expected.boards[0].notches = vec![NotchConfig::default(); 8];
        let wire = response(&expected);
        assert_eq!(DeviceConfig::decode(&wire[4..]).unwrap(), expected);
        let mut bad = wire[4..].to_vec();
        bad.push(0);
        assert!(DeviceConfig::decode(&bad).is_err());
        expected.boards[0].curve.maximum = expected.boards[0].curve.threshold;
        assert!(expected.validate().is_err());
    }

    #[test]
    fn config_load_continues_with_missing_boards_and_keeps_motor_disarmed() {
        struct Fake {
            config: DeviceConfig,
            commands: Vec<Vec<u8>>,
        }
        impl crate::serial_io::CommandConnection for Fake {
            fn transact(&mut self, request: &[u8]) -> Result<Reply, Failure> {
                self.commands.push(request.to_vec());
                match request[0] {
                    25 => Ok(Reply::Configuration(Box::new(self.config.clone()))),
                    27 => Ok(Reply::HardwareErrors([[1, 0x20, 1, 0x58, 0, 0x108]; 2])),
                    3 if request[1] == 1 && self.config.available != [true, true] => {
                        let failed = u8::from(!self.config.available[0])
                            | (u8::from(!self.config.available[1]) << 1);
                        Err(Failure::Device(2 + failed))
                    }
                    _ => Ok(Reply::Ack),
                }
            }
        }
        for available in [[true, true], [true, false], [false, true], [false, false]] {
            let mut config = configuration();
            config.available = available;
            let mut fake = Fake {
                config: config.clone(),
                commands: Vec::new(),
            };
            config.owner = Some(1);
            let warnings = config.apply_inactive(&mut fake).unwrap();
            assert_eq!(warnings.is_empty(), available == [true, true]);
            assert_eq!(fake.commands[0], [2, 0, 0]);
            assert!(fake.commands.iter().all(|c| c[0] != 24 || c[2] == 0));
            assert_eq!(fake.commands.last().unwrap(), &[25]);
        }
    }

    #[test]
    fn json_keeps_settings_but_excludes_session_and_derived_mask() {
        let c = configuration();
        let json = serde_json::to_string(&c).unwrap();
        assert!(!json.contains("revision"));
        assert!(!json.contains("mask"));
        let mut decoded: DeviceConfig = serde_json::from_str(&json).unwrap();
        decoded.revision = c.revision;
        decoded.available = c.available;
        assert_eq!(decoded, c);
    }

    #[test]
    fn ec5_profile_roundtrips_and_explicitly_rejects_ec4_files() {
        let file = std::env::temp_dir().join(format!("ec5-profile-{}.json", std::process::id()));
        let profile = Profile {
            motor: Default::default(),
            project: "ec5".into(),
            schema_version: 1,
            device: configuration(),
            display: DisplayConfig {
                serial_port: "COM7".into(),
                plots: [[Poll::default(); 5]; 2],
                x_seconds: 10.0,
                autoscroll: true,
                autofit_y: true,
                envelope: [true; 2],
                manual_velocity: 2.0,
            },
        };
        let json = serde_json::to_value(&profile).unwrap();
        std::fs::write(&file, serde_json::to_vec(&json).unwrap()).unwrap();
        let loaded = Profile::load(&file).unwrap();
        assert_eq!(loaded.display.serial_port, "COM7");
        assert_eq!(loaded.device.boards, profile.device.boards);
        let mut old = json;
        old.as_object_mut().unwrap().remove("project");
        old["display"]
            .as_object_mut()
            .unwrap()
            .remove("serial_port");
        old["display"]["ip"] = serde_json::json!("192.168.4.1");
        old["display"]["port"] = serde_json::json!(3333);
        std::fs::write(&file, serde_json::to_vec(&old).unwrap()).unwrap();
        assert!(Profile::load(&file).unwrap_err().contains("Not an EC5"));
        std::fs::remove_file(file).unwrap();
    }
}

impl Profile {
    pub fn load(path: &Path) -> Result<Self, String> {
        let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
        if file.metadata().map_err(|e| e.to_string())?.len() > 1_048_576 {
            return Err("Configuration file exceeds 1 MiB".into());
        }
        let profile: Self = serde_json::from_reader(file).map_err(|e| e.to_string())?;
        if profile.project != "ec5" || !(1..=2).contains(&profile.schema_version) {
            return Err("Not an EC5 version-1/2 configuration".into());
        }
        profile.device.validate()?;
        profile.motor.validate()?;
        let d = &profile.display;
        if !d.x_seconds.is_finite()
            || !(0.01..=120.0).contains(&d.x_seconds)
            || d.serial_port.len() > 256
            || !d.manual_velocity.is_finite()
            || !(-50.0..=50.0).contains(&d.manual_velocity)
            || d.plots
                .iter()
                .flatten()
                .any(|p| !(20..=5000).contains(&p.milliseconds))
        {
            return Err("Invalid saved plot or connection settings".into());
        }
        Ok(profile)
    }
}
