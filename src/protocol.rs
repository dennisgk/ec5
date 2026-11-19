use uuid::{Uuid, uuid};
pub const SERVICE: Uuid = uuid!("7c38a100-e82d-4ec1-a271-f2186a40c005");
pub const IDENTITY: Uuid = uuid!("7c38a101-e82d-4ec1-a271-f2186a40c005");
pub const COMMAND: Uuid = uuid!("7c38a102-e82d-4ec1-a271-f2186a40c005");
pub const RESULT: Uuid = uuid!("7c38a103-e82d-4ec1-a271-f2186a40c005");
pub const STATE: Uuid = uuid!("7c38a104-e82d-4ec1-a271-f2186a40c005");
pub const MECHANICS: Uuid = uuid!("7c38a105-e82d-4ec1-a271-f2186a40c005");
pub const DYNAMICS: Uuid = uuid!("7c38a106-e82d-4ec1-a271-f2186a40c005");

#[derive(Clone, Copy, Default, Debug)]
pub struct State {
    pub bluetooth: bool,
    pub holding: bool,
    pub calibrated: bool,
    pub fault: bool,
    pub measured: f32,
    pub target: f32,
    pub velocity: f32,
}
impl State {
    pub fn decode(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() != 20 || bytes[0] != 1 || bytes[1] > 1 || bytes[2] & !3 != 0 || bytes[3] > 1
        {
            return Err("Incompatible BLE state record".into());
        }
        let float = |n| f32::from_le_bytes(bytes[n..n + 4].try_into().unwrap());
        let state = Self {
            bluetooth: bytes[1] != 0,
            holding: bytes[2] & 1 != 0,
            calibrated: bytes[2] & 2 != 0,
            fault: bytes[3] != 0,
            measured: float(4),
            target: float(8),
            velocity: float(12),
        };
        if ![state.measured, state.target, state.velocity]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err("Non-finite BLE state".into());
        }
        Ok(state)
    }
}
pub fn identity(bytes: &[u8]) -> Result<String, String> {
    if bytes.len() != 8 || bytes[0] != 1 || bytes[7] != 0xC5 {
        return Err("Not a compatible EC5 controller".into());
    }
    Ok(bytes[1..7].iter().map(|b| format!("{b:02X}")).collect())
}
pub fn command(operation: u8, id: u16, payload: &[u8]) -> Result<Vec<u8>, String> {
    let length = match operation {
        1 | 7 => 0,
        2 => 8,
        3 => 5,
        4 => 16,
        5 => 4,
        6 => 1,
        _ => return Err("Unknown BLE operation".into()),
    };
    if payload.len() != length {
        return Err("Invalid BLE command size".into());
    }
    let mut bytes = vec![1, operation];
    bytes.extend(id.to_le_bytes());
    bytes.extend(payload);
    Ok(bytes)
}
pub fn floats(values: &[f32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

pub fn validate_settings(s: &[f32; 8]) -> Result<(), String> {
    if !s.iter().all(|value| value.is_finite())
        || !(0.01..=1000.).contains(&s[0])
        || ![-1., 1.].contains(&s[1])
        || s[2] < -180.
        || s[3] > 180.
        || s[2] >= s[3]
        || s[4] <= 0.
        || s[4] > 50.
        || s[5] <= 0.
        || s[5] > 180.
        || s[6] <= 0.
        || s[6] > 1000.
        || !(0.0..=10.).contains(&s[7])
    {
        return Err(
            "Invalid angle settings: check ratio, direction, limits and positive dynamics".into(),
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn commands_fit_minimum_att_and_identity_is_strict() {
        assert!(validate_settings(&[1., 1., 0., 120., 2., 10., 30., 1.]).is_ok());
        assert!(validate_settings(&[1., 1., 120., 0., 2., 10., 30., 1.]).is_err());
        assert!(validate_settings(&[1., 0., 0., 120., 2., 10., 30., 1.]).is_err());
        assert_eq!(
            command(4, 0x1234, &floats(&[2., 10., 30., 1.]))
                .unwrap()
                .len(),
            20
        );
        assert_eq!(&command(7, 0x1234, &[]).unwrap(), &[1, 7, 0x34, 0x12]);
        assert!(command(5, 1, &[]).is_err());
        assert!(identity(&[1, 1, 2, 3, 4, 5, 6, 0xC5]).is_ok());
        assert!(identity(&[1, 1, 2, 3, 4, 5, 6, 0]).is_err());
    }
    #[test]
    fn state_rejects_truncation_and_non_finite_values() {
        let mut bytes = [0; 20];
        bytes[0] = 1;
        assert!(State::decode(&bytes).is_ok());
        assert!(State::decode(&bytes[..19]).is_err());
        bytes[4..8].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(State::decode(&bytes).is_err());
    }
}
