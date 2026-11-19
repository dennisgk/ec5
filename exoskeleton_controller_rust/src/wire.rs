use base64::{Engine, engine::general_purpose::STANDARD};
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

pub const CHUNK: usize = 351;
pub const MAX_MESSAGE: usize = 65535;
pub const SERIAL_WINDOW: usize = 4;
pub const ACK_TIMEOUT: Duration = Duration::from_millis(250);
pub const COMMAND: u8 = 1;
pub const RESPONSE: u8 = 2;
pub const ERROR: u8 = 3;
pub const DATA: u8 = 4;
pub const ACK: u8 = 5;
pub const KEEPALIVE: u8 = 6;
pub const DISCONNECT: u8 = 7;

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub session: u8,
    pub id: u32,
    pub start: u16,
    pub end: u16,
    pub total: u16,
    pub data: Vec<u8>,
}

impl Frame {
    pub fn hello(session: u8, id: u32) -> Self {
        Self {
            session,
            id,
            start: 0,
            end: 0,
            total: 0,
            data: Vec::new(),
        }
    }
    pub fn encode(&self) -> Result<Vec<u8>, String> {
        if !self.session.is_ascii_alphanumeric()
            || self.start > self.end
            || self.end > self.total
            || self.data.len() != (self.end - self.start) as usize
            || self.data.len() > CHUNK
        {
            return Err("Invalid frame range".into());
        }
        let prefix = format!(
            "GOON|1|{}|{:08X}|{:04X}|{:04X}|{:04X}|",
            self.session as char, self.id, self.start, self.end, self.total
        );
        let suffix = format!("|{}", STANDARD.encode(&self.data));
        let mut crc = crc32fast::Hasher::new();
        crc.update(prefix.as_bytes());
        crc.update(suffix.as_bytes());
        Ok(format!("{prefix}{:08X}{suffix}\n", crc.finalize()).into_bytes())
    }
    pub fn decode(line: &[u8]) -> Result<Self, String> {
        if line.len() < 43 || line.len() > 512 || line.last() != Some(&b'\n') {
            return Err("Invalid frame length".into());
        }
        let text = std::str::from_utf8(&line[..line.len() - 1]).map_err(|_| "Non-ASCII frame")?;
        if !text.is_ascii() {
            return Err("Non-ASCII frame".into());
        }
        let fields: Vec<_> = text.split('|').collect();
        if fields.len() != 9
            || fields[0] != "GOON"
            || fields[1] != "1"
            || fields[2].len() != 1
            || !fields[2].as_bytes()[0].is_ascii_alphanumeric()
        {
            return Err("Invalid frame identifier/version".into());
        }
        let number = |s: &str, width: usize| -> Result<u32, String> {
            if s.len() != width
                || !s
                    .bytes()
                    .all(|b| b.is_ascii_digit() || (b'A'..=b'F').contains(&b))
            {
                return Err("Invalid hexadecimal field".into());
            }
            u32::from_str_radix(s, 16).map_err(|_| "Invalid hexadecimal field".into())
        };
        let id = number(fields[3], 8)?;
        let start = number(fields[4], 4)? as u16;
        let end = number(fields[5], 4)? as u16;
        let total = number(fields[6], 4)? as u16;
        let expected = number(fields[7], 8)?;
        let mut crc = crc32fast::Hasher::new();
        crc.update(&line[..33]);
        crc.update(&line[41..line.len() - 1]);
        if crc.finalize() != expected {
            return Err("CRC32 mismatch".into());
        }
        if start > end || end > total || (end - start) as usize > CHUNK {
            return Err("Invalid byte range".into());
        }
        let data = STANDARD.decode(fields[8]).map_err(|_| "Invalid Base64")?;
        if data.len() != (end - start) as usize {
            return Err("Decoded byte count differs from END-START".into());
        }
        Ok(Self {
            session: fields[2].as_bytes()[0],
            id,
            start,
            end,
            total,
            data,
        })
    }
}

pub struct Lines {
    bytes: Vec<u8>,
    overflow: bool,
}
impl Lines {
    pub fn new() -> Self {
        Self {
            bytes: Vec::with_capacity(512),
            overflow: false,
        }
    }
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        for &byte in bytes {
            if self.bytes.len() < 512 {
                self.bytes.push(byte);
            } else {
                self.overflow = true;
            }
            if byte == b'\n' {
                if !self.overflow
                    && let Ok(frame) = Frame::decode(&self.bytes)
                {
                    frames.push(frame);
                }
                self.bytes.clear();
                self.overflow = false;
            }
        }
        frames
    }
}

struct Assembly {
    bytes: Vec<u8>,
    seen: Vec<bool>,
    received: usize,
}
pub struct Reassembly {
    pending: HashMap<u32, Assembly>,
    completed: VecDeque<(u32, u32)>,
}
impl Reassembly {
    pub fn new() -> Self {
        Self {
            pending: HashMap::new(),
            completed: VecDeque::new(),
        }
    }
    pub fn insert(&mut self, frame: &Frame) -> Result<Option<Vec<u8>>, String> {
        if frame.total == 0 || frame.start >= frame.end {
            return Err("Empty ordinary message".into());
        }
        if self.completed.iter().any(|(id, _)| *id == frame.id) {
            return Ok(None);
        }
        if !self.pending.contains_key(&frame.id) {
            if self.pending.len() >= 4 {
                return Err("Receive reassembly slots exhausted".into());
            }
            self.pending.insert(
                frame.id,
                Assembly {
                    bytes: vec![0; frame.total as usize],
                    seen: vec![false; frame.total as usize],
                    received: 0,
                },
            );
        }
        let a = self.pending.get_mut(&frame.id).unwrap();
        if a.bytes.len() != frame.total as usize {
            return Err("TOTAL changed within message".into());
        }
        for (offset, &byte) in frame.data.iter().enumerate() {
            let at = frame.start as usize + offset;
            if a.seen[at] && a.bytes[at] != byte {
                return Err("Conflicting byte overlap".into());
            }
            if !a.seen[at] {
                a.bytes[at] = byte;
                a.seen[at] = true;
                a.received += 1;
            }
        }
        if a.received == a.bytes.len() {
            let a = self.pending.remove(&frame.id).unwrap();
            self.completed
                .push_back((frame.id, crc32fast::hash(&a.bytes)));
            while self.completed.len() > 64 {
                self.completed.pop_front();
            }
            return Ok(Some(a.bytes));
        }
        Ok(None)
    }
}

struct Sent {
    start: u16,
    end: u16,
    when: Instant,
    retries: u8,
}
pub struct Outgoing {
    pub id: u32,
    pub data: Vec<u8>,
    next: usize,
    pending: Vec<Sent>,
}
impl Outgoing {
    pub fn new(id: u32, data: Vec<u8>) -> Result<Self, String> {
        if data.is_empty() || data.len() > MAX_MESSAGE {
            return Err("Invalid message length".into());
        }
        Ok(Self {
            id,
            data,
            next: 0,
            pending: Vec::new(),
        })
    }
    pub fn acknowledged(&mut self, start: u16, end: u16) {
        self.pending.retain(|p| p.start != start || p.end != end);
    }
    pub fn pending(&self) -> usize {
        self.pending.len()
    }
    pub fn complete(&self) -> bool {
        self.next == self.data.len() && self.pending.is_empty()
    }
    pub fn frames(
        &mut self,
        session: u8,
        capacity: usize,
        now: Instant,
    ) -> Result<Vec<Frame>, String> {
        let mut frames = Vec::new();
        for p in &mut self.pending {
            if now.duration_since(p.when) >= ACK_TIMEOUT {
                if p.retries >= 3 {
                    return Err(format!(
                        "Serial message {:08X} ACK timed out for {}..{}",
                        self.id, p.start, p.end
                    ));
                }
                p.retries += 1;
                p.when = now;
                frames.push(Frame {
                    session,
                    id: self.id,
                    start: p.start,
                    end: p.end,
                    total: self.data.len() as u16,
                    data: self.data[p.start as usize..p.end as usize].to_vec(),
                });
            }
        }
        for _ in 0..capacity {
            if self.next == self.data.len() {
                break;
            }
            let end = (self.next + CHUNK).min(self.data.len());
            let p = Sent {
                start: self.next as u16,
                end: end as u16,
                when: now,
                retries: 0,
            };
            frames.push(Frame {
                session,
                id: self.id,
                start: p.start,
                end: p.end,
                total: self.data.len() as u16,
                data: self.data[self.next..end].to_vec(),
            });
            self.pending.push(p);
            self.next = end;
        }
        Ok(frames)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c_and_rust_use_identical_golden_frames() {
        assert_eq!(crc32fast::hash(b"123456789"), 0xCBF43926);
        let vectors = include_str!("../../exoskeleton_controller/tests/protocol_vectors.txt");
        for text in vectors.lines() {
            let line = format!("{text}\n");
            let frame = Frame::decode(line.as_bytes()).unwrap();
            assert_eq!(frame.encode().unwrap(), line.as_bytes());
            let mut damaged = line.into_bytes();
            damaged[33] ^= 1;
            assert!(Frame::decode(&damaged).is_err());
        }
    }

    #[test]
    fn line_parser_recovers_from_logs_overflow_and_split_reads() {
        let expected = Frame::hello(b'K', 42).encode().unwrap();
        let mut bytes = b"Boot log without protocol\n".to_vec();
        bytes.extend(vec![b'X'; 900]);
        bytes.push(b'\n');
        bytes.extend(&expected);
        let mut lines = Lines::new();
        let mut frames = Vec::new();
        for fragment in bytes.chunks(7) {
            frames.extend(lines.push(fragment));
        }
        assert_eq!(frames, vec![Frame::hello(b'K', 42)]);
    }

    #[test]
    fn full_u16_message_assembles_with_holes_duplicates_and_variable_ranges() {
        let expected: Vec<_> = (0..65535).map(|n| n as u8).collect();
        let mut reassembly = Reassembly::new();
        let mut end = expected.len();
        let mut result = None;
        while end > 0 {
            let start = end.saturating_sub(317);
            let f = Frame {
                session: b'K',
                id: 44,
                start: start as u16,
                end: end as u16,
                total: 65535,
                data: expected[start..end].to_vec(),
            };
            let encoded = f.encode().unwrap();
            assert!(encoded.len() <= 512);
            result = reassembly
                .insert(&Frame::decode(&encoded).unwrap())
                .unwrap();
            if start > 0 {
                assert!(result.is_none());
                assert!(reassembly.insert(&f).unwrap().is_none());
            }
            end = start;
        }
        assert_eq!(result.unwrap(), expected);
        let mut partial = Frame {
            session: b'K',
            id: 45,
            start: 0,
            end: 2,
            total: 3,
            data: vec![1, 2],
        };
        assert!(reassembly.insert(&partial).unwrap().is_none());
        partial.data[1] = 9;
        assert!(reassembly.insert(&partial).is_err());
    }

    #[test]
    fn four_chunk_window_retransmits_only_unacknowledged_ranges_then_stops() {
        let now = Instant::now();
        let mut message = Outgoing::new(45, vec![7; 2000]).unwrap();
        let first = message.frames(b'K', 4, now).unwrap();
        assert_eq!(first.len(), 4);
        assert_eq!(message.pending(), 4);
        message.acknowledged(first[1].start, first[1].end);
        message.acknowledged(first[2].start, first[2].end);
        message.acknowledged(first[3].start, first[3].end);
        let retry = message.frames(b'K', 0, now + ACK_TIMEOUT).unwrap();
        assert_eq!(retry, vec![first[0].clone()]);
        message.acknowledged(first[0].start, first[0].end);
        let remaining = message.frames(b'K', 4, now + ACK_TIMEOUT).unwrap();
        for f in remaining {
            message.acknowledged(f.start, f.end);
        }
        assert!(message.complete());
        let mut lost = Outgoing::new(46, vec![1]).unwrap();
        for n in 0..=3 {
            assert_eq!(
                lost.frames(b'K', usize::from(n == 0), now + ACK_TIMEOUT * n)
                    .unwrap()
                    .len(),
                1
            );
        }
        assert!(lost.frames(b'K', 0, now + ACK_TIMEOUT * 4).is_err());
    }
}
