use crate::{configuration::DeviceConfig, protocol::*};
use std::io::{Cursor, Read};
pub const RECORD_SIZES: [usize; 10] = [16, 16, 16, 16, 24, 24, 536, 536, 24, 24];
fn read<const N: usize>(data: &mut Cursor<&[u8]>) -> Result<[u8; N], Failure> {
    let mut b = [0; N];
    data.read_exact(&mut b)?;
    Ok(b)
}
fn u64v(d: &mut Cursor<&[u8]>) -> Result<u64, Failure> {
    Ok(u64::from_le_bytes(read(d)?))
}
fn u32v(d: &mut Cursor<&[u8]>) -> Result<u32, Failure> {
    Ok(u32::from_le_bytes(read(d)?))
}
fn u16v(d: &mut Cursor<&[u8]>) -> Result<u16, Failure> {
    Ok(u16::from_le_bytes(read(d)?))
}
fn float(d: &mut Cursor<&[u8]>) -> Result<f32, Failure> {
    let f = f32::from_le_bytes(read(d)?);
    if f.is_finite() {
        Ok(f)
    } else {
        Err(Failure::Transport("Non-finite telemetry".into()))
    }
}

pub fn decode(stream: usize, payload: &[u8], config: &DeviceConfig) -> Result<Vec<Reply>, Failure> {
    let size = *RECORD_SIZES
        .get(stream)
        .ok_or_else(|| Failure::Transport("Invalid stream".into()))?;
    if payload.is_empty() || !payload.len().is_multiple_of(size) {
        return Err(Failure::Transport("Partial telemetry record".into()));
    }
    let board = &config.boards[stream % 2];
    let mut replies = Vec::new();
    let mut raw = Vec::new();
    let mut filtered = Vec::new();
    let mut detection = Vec::new();
    for bytes in payload.chunks_exact(size) {
        let mut d = Cursor::new(bytes);
        let time_us = u64v(&mut d)?;
        match stream / 2 {
            0 => raw.push(RawSample {
                time_us,
                raw: u32v(&mut d)? as i32,
                sequence: u32v(&mut d)?,
            }),
            1 => filtered.push(FilteredSample {
                time_us,
                millivolts: float(&mut d)?,
                sequence: u32v(&mut d)?,
            }),
            2 => {
                let integral = float(&mut d)?;
                let envelope = float(&mut d)?;
                let window_ms = u16v(&mut d)?;
                let _reserved = u16v(&mut d)?;
                let _sequence = u32v(&mut d)?;
                replies.push(Reply::Integration(Integration {
                    time_us,
                    window_ms,
                    integral,
                    envelope,
                }));
            }
            3 => {
                let frame = u32v(&mut d)?;
                let sample_rate = u16v(&mut d)?;
                let fft = u16v(&mut d)?;
                let hop = u16v(&mut d)?;
                let flags = read::<2>(&mut d)?;
                if sample_rate != 1000
                    || fft != 256
                    || ![64, 128, 256].contains(&hop)
                    || flags[0] > 1
                    || flags[1] > 1
                {
                    return Err(Failure::Transport(
                        "Invalid Fourier telemetry layout".into(),
                    ));
                }
                let power = (0..FFT_BINS)
                    .map(|_| float(&mut d))
                    .collect::<Result<Vec<_>, _>>()?;
                replies.push(Reply::Fourier(Spectrum {
                    time_us,
                    sample_rate,
                    config: FourierConfig {
                        hop,
                        window: flags[0],
                        remove_mean: flags[1] != 0,
                    },
                    frame,
                    power,
                }));
            }
            4 => {
                let record = DetectionRecord {
                    time_us,
                    frame: u32v(&mut d)?,
                    peak: float(&mut d)?,
                    flags: u32v(&mut d)?,
                    velocity: float(&mut d)?,
                };
                if record.flags & !7 != 0 {
                    return Err(Failure::Transport("Invalid detection flags".into()));
                }
                detection.push(record);
            }
            _ => unreachable!(),
        }
    }
    match stream / 2 {
        0 => replies.push(Reply::Raw(RawBatch {
            discontinuity: false,
            active: config.enabled[0],
            configured: true,
            hardware: board.hardware.clone(),
            actual_hp: quantized_frequency(board.hardware.high_pass, 470e-9),
            actual_lp: quantized_frequency(board.hardware.low_pass, 10e-9),
            samples: raw,
        })),
        1 => replies.push(Reply::Filtered {
            discontinuity: false,
            samples: filtered,
        }),
        4 => replies.push(Reply::Detection {
            overflow: false,
            owner: config.owner,
            ready: true,
            records: detection,
        }),
        _ => {}
    }
    Ok(replies)
}

fn quantized_frequency(tenths_hz: u16, capacitance: f32) -> f32 {
    let tau = std::f32::consts::TAU;
    let resistance = 1.0 / (tau * (tenths_hz as f32 / 10.0) * capacitance) - 4775.0;
    let code = (resistance / 100_000.0 * 256.0).round();
    1.0 / (tau * (4775.0 + code / 256.0 * 100_000.0) * capacitance)
}
