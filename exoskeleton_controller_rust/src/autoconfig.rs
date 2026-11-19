use crate::{
    configuration::{self, DisplayConfig, Profile},
    protocol::{Failure, Reply},
    serial_io::{self, SerialConnection},
};

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct AngleConfig {
    pub ratio: f32,
    pub direction: f32,
    pub minimum: f32,
    pub maximum: f32,
    pub gain: f32,
    pub speed: f32,
    pub acceleration: f32,
    pub deadband: f32,
}
impl Default for AngleConfig {
    fn default() -> Self {
        Self {
            ratio: 1.0,
            direction: 1.0,
            minimum: 0.0,
            maximum: 120.0,
            gain: 40.0,
            speed: 720.0,
            acceleration: 7200.0,
            deadband: 0.5,
        }
    }
}
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct MotorConfig {
    pub bluetooth: bool,
    pub manual_velocity: f32,
    pub angle: AngleConfig,
}
impl MotorConfig {
    pub fn validate(&self) -> Result<(), String> {
        let a = &self.angle;
        if !self.manual_velocity.is_finite()
            || self.manual_velocity.abs() > 50.0
            || ![
                a.ratio,
                a.direction,
                a.minimum,
                a.maximum,
                a.gain,
                a.speed,
                a.acceleration,
                a.deadband,
            ]
            .iter()
            .all(|n| n.is_finite())
            || !(0.01..=1000.0).contains(&a.ratio)
            || ![-1.0, 1.0].contains(&a.direction)
            || a.minimum < -180.0
            || a.maximum > 180.0
            || a.minimum >= a.maximum
            || !(0.0..=50.0).contains(&a.gain)
            || a.gain == 0.0
            || !(0.0..=720.0).contains(&a.speed)
            || a.speed == 0.0
            || !(0.0..=10000.0).contains(&a.acceleration)
            || a.acceleration == 0.0
            || !(0.0..=10.0).contains(&a.deadband)
        {
            return Err("Invalid saved motor/angle configuration".into());
        }
        Ok(())
    }
}
#[derive(Debug)]
pub struct ControlState {
    pub motor: MotorConfig,
    pub angle_enabled: bool,
    pub calibrated: bool,
    pub mode: u8,
    pub diagnostics: [u32; 9],
}
impl ControlState {
    pub fn decode(bytes: &[u8]) -> Result<Self, Failure> {
        if bytes.len() != 76 || bytes[0..3].iter().any(|b| *b > 1) || bytes[3] > 3 {
            return Err(Failure::Transport("Invalid control-state layout".into()));
        }
        let float = |offset| f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
        let motor = MotorConfig {
            bluetooth: bytes[0] != 0,
            manual_velocity: float(4),
            angle: AngleConfig {
                ratio: float(8),
                direction: float(12),
                minimum: float(16),
                maximum: float(20),
                gain: float(24),
                speed: float(28),
                acceleration: float(32),
                deadband: float(36),
            },
        };
        motor.validate().map_err(Failure::Rejected)?;
        let mut diagnostics = [0; 9];
        for (i, n) in diagnostics.iter_mut().enumerate() {
            *n = u32::from_le_bytes(bytes[40 + i * 4..44 + i * 4].try_into().unwrap());
        }
        Ok(Self {
            motor,
            angle_enabled: bytes[1] != 0,
            calibrated: bytes[2] != 0,
            mode: bytes[3],
            diagnostics,
        })
    }
}
pub fn read(connection: &mut SerialConnection) -> Result<ControlState, Failure> {
    match serial_io::exchange(connection, &[33])? {
        Reply::Control(state) => Ok(state),
        _ => Err(Failure::Transport("Missing control state".into())),
    }
}
pub fn profile(
    connection: &mut SerialConnection,
    display: DisplayConfig,
) -> Result<Profile, Failure> {
    let device = configuration::read(connection)?;
    let motor = read(connection)?.motor;
    Ok(Profile {
        project: "ec5".into(),
        schema_version: 2,
        device,
        display,
        motor,
    })
}
pub fn save(connection: &mut SerialConnection, display: DisplayConfig) -> Result<(), Failure> {
    let profile = profile(connection, display)?;
    let bytes =
        serde_json::to_vec_pretty(&profile).map_err(|e| Failure::Rejected(e.to_string()))?;
    if bytes.is_empty() || bytes.len() > 32768 {
        return Err(Failure::Rejected("Autoconfig exceeds 32 KiB".into()));
    }
    let mut begin = vec![28];
    begin.extend((bytes.len() as u16).to_le_bytes());
    serial_io::exchange(connection, &begin)?;
    for (i, chunk) in bytes.chunks(150).enumerate() {
        let mut request = vec![29];
        request.extend(((i * 150) as u16).to_le_bytes());
        request.extend(chunk);
        serial_io::exchange(connection, &request)?;
    }
    serial_io::exchange(connection, &[30])?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn motor_defaults_and_invalid_settings() {
        let mut config = MotorConfig::default();
        assert!(config.validate().is_ok());
        config.bluetooth = true;
        config.manual_velocity = -12.3;
        let encoded = serde_json::to_string(&config).unwrap();
        assert!(
            serde_json::from_str::<MotorConfig>(&encoded)
                .unwrap()
                .bluetooth
        );
        config.angle.ratio = f32::NAN;
        assert!(config.validate().is_err());
        config.angle.ratio = 1.;
        config.angle.minimum = config.angle.maximum;
        assert!(config.validate().is_err());
    }
    #[test]
    fn control_state_matches_firmware_layout() {
        let a = AngleConfig::default();
        let mut bytes = vec![1, 0, 1, 1];
        for value in [
            -2.5,
            a.ratio,
            a.direction,
            a.minimum,
            a.maximum,
            a.gain,
            a.speed,
            a.acceleration,
            a.deadband,
        ] {
            bytes.extend(value.to_le_bytes());
        }
        for value in 0_u32..9 {
            bytes.extend(value.to_le_bytes());
        }
        let state = ControlState::decode(&bytes).unwrap();
        assert!(state.motor.bluetooth && state.calibrated);
        assert_eq!(state.motor.manual_velocity, -2.5);
        assert_eq!(state.diagnostics, [0, 1, 2, 3, 4, 5, 6, 7, 8]);
        bytes.push(0);
        assert!(ControlState::decode(&bytes).is_err());
    }
}
