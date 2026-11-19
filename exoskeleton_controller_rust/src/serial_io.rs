use crate::{
    configuration::DeviceConfig,
    connection::Event,
    protocol::{self, Failure, Reply},
    telemetry,
    wire::{self, Frame, Lines, Outgoing, Reassembly},
};
use crossbeam_channel::{Receiver, Sender, bounded};
use serialport::{ClearBuffer, FlowControl, SerialPort};
use std::{
    collections::{HashMap, VecDeque},
    io::{Read, Write},
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

enum ReplyEvent {
    Progress,
    Complete(Result<Vec<u8>, String>),
}
enum Work {
    Command(Vec<u8>, Sender<ReplyEvent>),
    EmergencyStop,
    Close,
}
#[derive(Clone)]
pub struct EmergencyHandle {
    work: Sender<Work>,
}
impl EmergencyHandle {
    pub fn stop(&self) {
        let _ = self.work.try_send(Work::EmergencyStop);
    }
    pub fn close(&self) {
        let _ = self.work.try_send(Work::Close);
    }
}
pub struct SerialConnection {
    work: Sender<Work>,
    pub config: Arc<RwLock<Option<DeviceConfig>>>,
    alive: Arc<AtomicBool>,
}
impl Drop for SerialConnection {
    fn drop(&mut self) {
        let _ = self.work.try_send(Work::Close);
    }
}
impl SerialConnection {
    pub fn emergency_handle(&self) -> EmergencyHandle {
        EmergencyHandle {
            work: self.work.clone(),
        }
    }
    pub fn connect(name: &str, data: Sender<(u32, Event)>) -> Result<Self, Failure> {
        let (work, jobs) = bounded(32);
        let (ready, wait) = bounded(1);
        let config = Arc::new(RwLock::new(None));
        let alive = Arc::new(AtomicBool::new(true));
        let shared = config.clone();
        let running = alive.clone();
        let name = name.to_owned();
        thread::spawn(move || {
            let result = open_and_pump(&name, jobs, ready.clone(), data, shared);
            running.store(false, Ordering::Release);
            if let Err(error) = result {
                let _ = ready.try_send(Err(error.clone()));
                eprintln!("Serial: {error}");
            }
        });
        wait.recv_timeout(Duration::from_secs(5))
            .map_err(|_| Failure::Transport("Serial HELLO timed out".into()))?
            .map_err(Failure::Transport)?;
        Ok(Self {
            work,
            config,
            alive,
        })
    }
    pub fn is_alive(&self) -> bool {
        self.alive.load(Ordering::Acquire)
    }
    pub fn close(&self) {
        let _ = self.work.try_send(Work::Close);
    }
    pub fn set_configuration(&self, c: DeviceConfig) {
        *self.config.write().unwrap() = Some(c);
    }
}

pub trait CommandConnection {
    fn transact(&mut self, request: &[u8]) -> Result<Reply, Failure>;
}
impl CommandConnection for SerialConnection {
    fn transact(&mut self, request: &[u8]) -> Result<Reply, Failure> {
        real_exchange(self, request)
    }
}
pub fn exchange(connection: &mut impl CommandConnection, request: &[u8]) -> Result<Reply, Failure> {
    connection.transact(request)
}
fn real_exchange(connection: &mut SerialConnection, request: &[u8]) -> Result<Reply, Failure> {
    let (tx, rx) = bounded(64);
    connection
        .work
        .send(Work::Command(request.to_vec(), tx))
        .map_err(|_| Failure::Transport("Serial connection closed".into()))?;
    let started = Instant::now();
    let mut progressed = started;
    let wire = loop {
        if !connection.is_alive() {
            return Err(Failure::Transport(
                "Serial connection lost; device settings and motor mode remain active".into(),
            ));
        }
        if progressed.elapsed() >= Duration::from_secs(5)
            || started.elapsed() >= Duration::from_secs(15)
        {
            connection.close();
            return Err(Failure::Transport(format!(
                "Serial operation {} response timed out",
                request[0]
            )));
        }
        match rx.recv_timeout(Duration::from_millis(20)) {
            Ok(ReplyEvent::Progress) => progressed = Instant::now(),
            Ok(ReplyEvent::Complete(result)) => break result.map_err(Failure::Transport)?,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(_) => {
                return Err(Failure::Transport(
                    "Serial connection closed during command".into(),
                ));
            }
        }
    };
    protocol::decode_response(request, &wire)
}

fn write_frame(port: &mut impl Write, frame: &Frame) -> Result<(), String> {
    port.write_all(&frame.encode()?).map_err(|e| e.to_string())
}
fn allocate(next: &mut u32) -> Result<u32, String> {
    if *next >= 0x80000000 {
        return Err("Serial ID range exhausted; reconnect to establish a new session".into());
    }
    let id = *next;
    *next += 1;
    Ok(id)
}
fn short(session: u8, id: u32, payload: Vec<u8>) -> Frame {
    Frame {
        session,
        id,
        start: 0,
        end: payload.len() as u16,
        total: payload.len() as u16,
        data: payload,
    }
}

trait BytePort: Read + Write {
    fn available(&mut self) -> Result<usize, String>;
}
struct UsbPort(Box<dyn SerialPort>);
impl Read for UsbPort {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(out)
    }
}
impl Write for UsbPort {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        self.0.write(bytes)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.0.flush()
    }
}
impl BytePort for UsbPort {
    fn available(&mut self) -> Result<usize, String> {
        self.0
            .bytes_to_read()
            .map(|n| n as usize)
            .map_err(|e| e.to_string())
    }
}

fn open_and_pump(
    name: &str,
    jobs: Receiver<Work>,
    ready: Sender<Result<(), String>>,
    data: Sender<(u32, Event)>,
    config: Arc<RwLock<Option<DeviceConfig>>>,
) -> Result<(), String> {
    let port = serialport::new(name, 115200)
        .flow_control(FlowControl::None)
        .timeout(Duration::from_millis(20))
        .open()
        .map_err(|e| format!("Open {name}: {e}"))?;
    port.clear(ClearBuffer::Input).map_err(|e| e.to_string())?;
    pump(&mut UsbPort(port), jobs, ready, data, config)
}
fn pump(
    port: &mut impl BytePort,
    jobs: Receiver<Work>,
    ready: Sender<Result<(), String>>,
    data: Sender<(u32, Event)>,
    config: Arc<RwLock<Option<DeviceConfig>>>,
) -> Result<(), String> {
    let hello_id = (SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .subsec_nanos()
        & 0x3fffffff)
        .max(1);
    let hello = Frame::hello(b'0', hello_id);
    let mut lines = Lines::new();
    let mut bytes = [0; 2048];
    let mut session = 0;
    let started = Instant::now();
    let mut hello_sent = Instant::now() - Duration::from_secs(1);
    while session == 0 {
        if started.elapsed() > Duration::from_secs(3) {
            return Err("No valid GOON version 1 HELLO response on selected COM port".into());
        }
        if hello_sent.elapsed() >= Duration::from_millis(500) {
            write_frame(&mut *port, &hello)?;
            hello_sent = Instant::now();
        }
        let available = port.available()?;
        if available > 0 {
            let capacity = bytes.len().min(available);
            let count = port
                .read(&mut bytes[..capacity])
                .map_err(|e| e.to_string())?;
            for f in lines.push(&bytes[..count]) {
                if f.id == hello_id
                    && f.session != b'0'
                    && f.total == 0
                    && f.start == 0
                    && f.end == 0
                {
                    session = f.session;
                    break;
                }
            }
        } else {
            thread::sleep(Duration::from_millis(1));
        }
    }
    ready.send(Ok(())).map_err(|_| "Connection canceled")?;
    let mut next = hello_id + 1;
    let mut window = 1usize;
    let mut outgoing: VecDeque<Outgoing> = VecDeque::new();
    let mut pending: HashMap<u32, Sender<ReplyEvent>> = HashMap::new();
    let mut assembly = Reassembly::new();
    let mut last_send = Instant::now();
    let mut last_receive = Instant::now();
    let mut closing: Option<Instant> = None;
    loop {
        let available = port.available()?;
        if available > 0 {
            let capacity = bytes.len().min(available);
            let count = port
                .read(&mut bytes[..capacity])
                .map_err(|e| e.to_string())?;
            for f in lines.push(&bytes[..count]) {
                if f.session != session || f.id < 0x80000000 || f.total == 0 {
                    continue;
                }
                last_receive = Instant::now();
                if f.start == 0 && f.end == f.total && f.data.first() == Some(&wire::ACK) {
                    if f.data.len() != 9 {
                        return Err("Invalid serial acknowledgement".into());
                    }
                    let reference = u32::from_le_bytes(f.data[1..5].try_into().unwrap());
                    let start = u16::from_le_bytes(f.data[5..7].try_into().unwrap());
                    let end = u16::from_le_bytes(f.data[7..9].try_into().unwrap());
                    if let Some(message) = outgoing.iter_mut().find(|m| m.id == reference) {
                        message.acknowledged(start, end);
                    }
                    if let Some(waiter) = pending.get(&reference) {
                        let _ = waiter.try_send(ReplyEvent::Progress);
                    }
                    continue;
                }
                let mut ack = vec![wire::ACK];
                ack.extend(f.id.to_le_bytes());
                ack.extend(f.start.to_le_bytes());
                ack.extend(f.end.to_le_bytes());
                // ACKs bypass the window and never request an ACK themselves.
                write_frame(&mut *port, &short(session, allocate(&mut next)?, ack))?;
                last_send = Instant::now();
                if f.start == 0
                    && f.data.len() >= 5
                    && matches!(f.data[0], wire::RESPONSE | wire::ERROR)
                {
                    let reference = u32::from_le_bytes(f.data[1..5].try_into().unwrap());
                    if let Some(waiter) = pending.get(&reference) {
                        let _ = waiter.try_send(ReplyEvent::Progress);
                    }
                }
                if let Some(payload) = assembly.insert(&f)? {
                    match payload[0] {
                        wire::RESPONSE | wire::ERROR if payload.len() >= 7 => {
                            let reference = u32::from_le_bytes(payload[1..5].try_into().unwrap());
                            let response = payload[5..].to_vec();
                            if response[0] == 1 && response[1] == 0 && response.len() >= 38 {
                                let advertised = response[35] as usize;
                                if !(1..=16).contains(&advertised) {
                                    return Err("Invalid receiver window".into());
                                }
                                window = wire::SERIAL_WINDOW.min(advertised);
                            }
                            outgoing.retain(|m| m.id != reference);
                            if let Some(waiter) = pending.remove(&reference) {
                                let _ = waiter.send(ReplyEvent::Complete(Ok(response)));
                            }
                        }
                        wire::DATA if payload.len() >= 8 => {
                            let stream = payload[1]
                                .checked_sub(1)
                                .ok_or("Invalid telemetry stream")?
                                as usize;
                            let epoch = u32::from_le_bytes(payload[2..6].try_into().unwrap());
                            let count =
                                u16::from_le_bytes(payload[6..8].try_into().unwrap()) as usize;
                            let snapshot = config.read().unwrap();
                            if let Some(c) = snapshot.as_ref()
                                && epoch == c.revision
                                && stream < 10
                                && count * telemetry::RECORD_SIZES[stream] + 8 == payload.len()
                            {
                                match telemetry::decode(stream, &payload[8..], c) {
                                    Ok(replies) => {
                                        for reply in replies {
                                            let _ = data.try_send((
                                                epoch,
                                                Event::Data((stream % 2) as u8, reply),
                                            ));
                                        }
                                    }
                                    Err(e) => eprintln!("Discarded telemetry: {}", e.message()),
                                }
                            }
                        }
                        _ => return Err("Unexpected serial message kind".into()),
                    }
                }
            }
        }
        while let Ok(work) = jobs.try_recv() {
            match work {
                Work::Command(request, waiter) => {
                    if closing.is_some() {
                        let _ = waiter.send(ReplyEvent::Complete(Err("Disconnecting".into())));
                        continue;
                    }
                    let id = allocate(&mut next)?;
                    let mut payload = vec![wire::COMMAND];
                    payload.extend(request);
                    if pending.len() >= 8 {
                        return Err("Command slots exhausted".into());
                    }
                    pending.insert(id, waiter);
                    outgoing.push_back(Outgoing::new(id, payload)?);
                }
                Work::Close => {
                    outgoing.clear();
                    outgoing
                        .push_front(Outgoing::new(allocate(&mut next)?, vec![wire::DISCONNECT])?);
                    closing = Some(Instant::now());
                }
                Work::EmergencyStop => {
                    let id = allocate(&mut next)?;
                    outgoing.push_front(Outgoing::new(id, vec![wire::COMMAND, 2, 0, 0])?);
                }
            }
        }
        outgoing.retain(|m| !m.complete());
        if closing
            .is_some_and(|at| outgoing.is_empty() || at.elapsed() >= Duration::from_millis(200))
        {
            return Ok(());
        }
        if last_send.elapsed() >= Duration::from_millis(500) && closing.is_none() {
            outgoing.push_back(Outgoing::new(allocate(&mut next)?, vec![wire::KEEPALIVE])?);
        }
        let mut credit =
            window.saturating_sub(outgoing.iter().map(Outgoing::pending).sum::<usize>());
        for message in &mut outgoing {
            let before = message.pending();
            let frames = message.frames(session, credit, Instant::now())?;
            credit = credit.saturating_sub(message.pending() - before);
            for frame in frames {
                write_frame(&mut *port, &frame)?;
                last_send = Instant::now();
            }
        }
        if last_receive.elapsed() >= Duration::from_secs(2) {
            return Err("ESP32 serial session stopped responding".into());
        }
        thread::sleep(Duration::from_millis(1));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::configuration::test_support;

    struct FakePort {
        rx: Receiver<Vec<u8>>,
        tx: Sender<Vec<u8>>,
        input: VecDeque<u8>,
    }
    impl Read for FakePort {
        fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
            let n = out.len().min(self.input.len()).min(37);
            for byte in &mut out[..n] {
                *byte = self.input.pop_front().unwrap();
            }
            Ok(n)
        }
    }
    impl Write for FakePort {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            let n = bytes.len().min(53);
            self.tx
                .send(bytes[..n].to_vec())
                .map_err(|_| std::io::ErrorKind::BrokenPipe)?;
            Ok(n)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    impl BytePort for FakePort {
        fn available(&mut self) -> Result<usize, String> {
            while let Ok(bytes) = self.rx.try_recv() {
                self.input.extend(bytes);
            }
            Ok(self.input.len())
        }
    }
    fn send_message(tx: &Sender<Vec<u8>>, id: u32, payload: &[u8]) {
        for (index, chunk) in payload.chunks(wire::CHUNK).enumerate().rev() {
            tx.send(
                Frame {
                    session: b'K',
                    id,
                    start: (index * wire::CHUNK) as u16,
                    end: (index * wire::CHUNK + chunk.len()) as u16,
                    total: payload.len() as u16,
                    data: chunk.to_vec(),
                }
                .encode()
                .unwrap(),
            )
            .unwrap();
        }
    }

    #[test]
    fn pump_handles_hello_commands_streams_keepalive_and_disconnect_without_hardware() {
        let (to_peer, from_pc) = bounded(256);
        let (to_pc, from_peer) = bounded(256);
        let (work, jobs) = bounded(32);
        let (ready, ready_rx) = bounded(1);
        let (data, received) = bounded(64);
        let config = Arc::new(RwLock::new(None));
        let shared = config.clone();
        let alive = Arc::new(AtomicBool::new(true));
        let running = alive.clone();
        let pump_thread = thread::spawn(move || {
            let result = pump(
                &mut FakePort {
                    rx: from_peer,
                    tx: to_peer,
                    input: VecDeque::new(),
                },
                jobs,
                ready,
                data,
                shared,
            );
            running.store(false, Ordering::Release);
            result
        });
        let peer = thread::spawn(move || {
            let mut lines = Lines::new();
            let mut assembly = Reassembly::new();
            let mut next = 0x80000000u32;
            let mut keepalives = 0;
            let mut commands = 0;
            let mut first_hello = None;
            loop {
                let bytes = from_pc.recv_timeout(Duration::from_secs(3)).unwrap();
                for f in lines.push(&bytes) {
                    if f.session == b'0' {
                        if first_hello.is_none() {
                            first_hello = Some(f.id);
                            continue; // Lose the first reply; PC must retry the same HELLO ID.
                        }
                        assert_eq!(first_hello, Some(f.id));
                        to_pc
                            .send(b"I (10) ordinary startup log\n".to_vec())
                            .unwrap();
                        to_pc
                            .send(Frame::hello(b'K', f.id).encode().unwrap())
                            .unwrap();
                        continue;
                    }
                    if f.start == 0 && f.data.first() == Some(&wire::ACK) {
                        continue;
                    }
                    let mut ack = vec![wire::ACK];
                    ack.extend(f.id.to_le_bytes());
                    ack.extend(f.start.to_le_bytes());
                    ack.extend(f.end.to_le_bytes());
                    send_message(&to_pc, next, &ack);
                    next += 1;
                    if let Some(payload) = assembly.insert(&f).unwrap() {
                        if payload[0] == wire::DISCONNECT {
                            return (commands, keepalives);
                        }
                        if payload[0] == wire::KEEPALIVE {
                            keepalives += 1;
                            continue;
                        }
                        assert_eq!(payload[0], wire::COMMAND);
                        commands += 1;
                        let op = payload[1];
                        let body = match op {
                            1 => {
                                assert_eq!(payload, [wire::COMMAND, 1, 4]);
                                let mut response = vec![1, 0, 1, 0, 232, 3, 0, 1, 0, 4];
                                response.extend((b'K' as u32).to_le_bytes());
                                response.push(10);
                                for size in telemetry::RECORD_SIZES {
                                    response.extend((size as u16).to_le_bytes());
                                }
                                response.extend([4, 255, 255]);
                                response
                            }
                            25 => {
                                let mut configuration = test_support::configuration();
                                configuration.available = [true, false];
                                test_support::response(&configuration)
                            }
                            26 => {
                                assert_eq!(payload, [wire::COMMAND, 26, 65, 0]);
                                let mut response = vec![26, 0];
                                response.extend((b'K' as u32).to_le_bytes());
                                response.extend(2u32.to_le_bytes());
                                response
                            }
                            2 => vec![2, 0],
                            _ => panic!("Unexpected operation {op}"),
                        };
                        let mut response = vec![wire::RESPONSE];
                        response.extend(f.id.to_le_bytes());
                        response.extend(body);
                        send_message(&to_pc, next, &response);
                        next += 1;
                        if op == 26 {
                            let mut raw = vec![wire::DATA, 1];
                            raw.extend(2u32.to_le_bytes());
                            raw.extend(2u16.to_le_bytes());
                            for sequence in [11u32, 12] {
                                raw.extend((sequence as u64 * 1000).to_le_bytes());
                                raw.extend((-123i32).to_le_bytes());
                                raw.extend(sequence.to_le_bytes());
                            }
                            send_message(&to_pc, next, &raw);
                            next += 1;
                            let mut fft = vec![wire::DATA, 7];
                            fft.extend(2u32.to_le_bytes());
                            fft.extend(1u16.to_le_bytes());
                            fft.extend(12000u64.to_le_bytes());
                            fft.extend(1u32.to_le_bytes());
                            for value in [1000u16, 256, 128] {
                                fft.extend(value.to_le_bytes());
                            }
                            fft.extend([1, 1]);
                            for _ in 0..129 {
                                fft.extend(0.1f32.to_le_bytes());
                            }
                            send_message(&to_pc, next, &fft);
                            next += 1;
                        }
                    }
                }
            }
        });
        ready_rx
            .recv_timeout(Duration::from_secs(3))
            .unwrap()
            .unwrap();
        let mut connection = SerialConnection {
            work,
            config,
            alive,
        };
        assert!(matches!(
            exchange(&mut connection, &[1, 4]).unwrap(),
            Reply::Init
        ));
        let c = crate::configuration::read(&mut connection).unwrap();
        assert_eq!(c.available, [true, false]);
        connection.set_configuration(c);
        assert!(matches!(
            exchange(&mut connection, &[26, 65, 0]).unwrap(),
            Reply::Subscribed { .. }
        ));
        let mut raw = false;
        let mut fft = false;
        for _ in 0..2 {
            let (epoch, event) = received.recv_timeout(Duration::from_secs(3)).unwrap();
            assert_eq!(epoch, 2);
            match event {
                Event::Data(0, Reply::Raw(batch)) => {
                    assert_eq!(batch.samples[1].sequence, 12);
                    raw = true;
                }
                Event::Data(0, Reply::Fourier(s)) => {
                    assert_eq!(s.power.len(), 129);
                    fft = true;
                }
                _ => panic!("Unexpected telemetry"),
            }
        }
        assert!(raw && fft);
        thread::sleep(Duration::from_millis(650));
        assert!(matches!(
            exchange(&mut connection, &[2, 0, 0]).unwrap(),
            Reply::Ack
        ));
        connection.close();
        let (commands, keepalives) = peer.join().unwrap();
        assert_eq!(commands, 4);
        assert!(keepalives >= 1);
        pump_thread.join().unwrap().unwrap();
    }
}
