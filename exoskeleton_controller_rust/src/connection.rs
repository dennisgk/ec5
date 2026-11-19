use crate::{
    configuration::{self, DisplayConfig, Profile},
    protocol::{Action, Failure, Kind, Reply},
    serial_io::{self, SerialConnection},
    wire::SERIAL_WINDOW,
};
use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Poll {
    pub enabled: bool,
    pub milliseconds: u32,
}
impl Default for Poll {
    fn default() -> Self {
        Self {
            enabled: false,
            milliseconds: 500,
        }
    }
}
pub enum Work {
    Connect(String),
    Ports,
    Disconnect,
    Command(Action),
    Polls([[Poll; 5]; 2]),
    Diagnostics,
    MotorStatus,
    Quit,
    Save(std::path::PathBuf, DisplayConfig),
    Load(std::path::PathBuf),
    SaveSd(DisplayConfig),
    Device(u8),
}
pub enum Event {
    Notice(String),
    Connected,
    Control(crate::autoconfig::ControlState),
    Ports(Vec<String>),
    Disconnected(String),
    Applied(Action, Option<f32>),
    Error(String),
    Data(u8, Reply),
    Configuration(Box<configuration::DeviceConfig>, bool),
    ProfileDone(Option<Box<Profile>>),
}
pub struct Connection {
    pub commands: Sender<Work>,
    pub urgent: Sender<Work>,
    pub events: Receiver<Event>,
    pub telemetry: Receiver<(u32, Event)>,
}
impl Connection {
    pub fn new() -> Self {
        let (commands, requests) = bounded(64);
        let (urgent, interrupts) = bounded(8);
        let (forward, priority) = bounded(8);
        let fast: Arc<Mutex<Option<serial_io::EmergencyHandle>>> = Arc::new(Mutex::new(None));
        let emergency = fast.clone();
        thread::spawn(move || {
            while let Ok(work) = interrupts.recv() {
                if let Some(handle) = emergency.lock().unwrap().as_ref() {
                    match &work {
                        Work::Command(Action::Motor(0)) => handle.stop(),
                        Work::Disconnect | Work::Quit => handle.close(),
                        _ => {}
                    }
                }
                if forward.send(work).is_err() {
                    break;
                }
            }
        });
        let (events, results) = unbounded();
        let (data, telemetry) = bounded(4096);
        thread::spawn(move || worker(requests, priority, events, data, fast));
        let result = Self {
            commands,
            urgent,
            events: results,
            telemetry,
        };
        let _ = result.commands.try_send(Work::Ports);
        result
    }
}
fn close(socket: &mut Option<SerialConnection>, events: &Sender<Event>, reason: String) {
    if let Some(socket) = socket.take() {
        socket.close();
    }
    let _ = events.send(Event::Disconnected(reason));
}
fn motor_snapshots(socket: &mut SerialConnection, events: &Sender<Event>) -> Result<(), Failure> {
    for command in [13, 18] {
        let reply = serial_io::exchange(socket, &[command])?;
        let _ = events.send(Event::Data(0, reply));
    }
    Ok(())
}
fn worker(
    requests: Receiver<Work>,
    priority: Receiver<Work>,
    events: Sender<Event>,
    data: Sender<(u32, Event)>,
    fast: Arc<Mutex<Option<serial_io::EmergencyHandle>>>,
) {
    let mut socket: Option<SerialConnection> = None;
    let mut polls = [[Poll::default(); 5]; 2];
    let mut pending = VecDeque::new();
    let mut checked = Instant::now();
    loop {
        let next = priority
            .try_recv()
            .ok()
            .or_else(|| pending.pop_front())
            .or_else(|| requests.try_recv().ok());
        if let Some(work) = next {
            match work {
                Work::Ports => match serialport::available_ports() {
                    Ok(ports) => {
                        let mut names: Vec<_> = ports.into_iter().map(|p| p.port_name).collect();
                        names.sort();
                        let _ = events.send(Event::Ports(names));
                    }
                    Err(e) => {
                        let _ = events.send(Event::Error(format!("List serial ports: {e}")));
                    }
                },
                Work::Connect(name) => {
                    if socket.is_some() {
                        close(&mut socket, &events, String::new());
                    }
                    polls = [[Poll::default(); 5]; 2];
                    let result = SerialConnection::connect(&name, data.clone()).and_then(
                        |mut connection| {
                            serial_io::exchange(&mut connection, &[1, SERIAL_WINDOW as u8])?;
                            Ok(connection)
                        },
                    );
                    match result {
                        Ok(connection) => {
                            *fast.lock().unwrap() = Some(connection.emergency_handle());
                            socket = Some(connection);
                            let _ = events.send(Event::Connected);
                            if let Err(e) =
                                update_configuration(socket.as_mut().unwrap(), &events, true)
                            {
                                close(&mut socket, &events, e.message());
                            }
                        }
                        Err(e) => {
                            let _ = events.send(Event::Disconnected(e.message()));
                        }
                    }
                }
                Work::Disconnect | Work::Quit => {
                    let quit = matches!(work, Work::Quit);
                    pending.clear();
                    while requests.try_recv().is_ok() {}
                    close(&mut socket, &events, String::new());
                    if quit {
                        return;
                    }
                }
                Work::Polls(updated) => {
                    polls = updated;
                    if let Some(connection) = &mut socket
                        && let Err(e) = subscribe(connection, &polls, &events, false)
                    {
                        if matches!(e, Failure::Transport(_)) {
                            close(&mut socket, &events, e.message());
                        }
                        let _ = events.send(Event::Error(e.message()));
                    }
                }
                Work::SaveSd(display) => {
                    let result = socket
                        .as_mut()
                        .ok_or_else(|| Failure::Rejected("Not connected".into()))
                        .and_then(|connection| crate::autoconfig::save(connection, display));
                    if let Err(e) = result {
                        let _ =
                            events.send(Event::Error(format!("Save autoconfig: {}", e.message())));
                    } else {
                        let _ = events.send(Event::Notice("Autoconfig saved to SD".into()));
                    }
                    let _ = events.send(Event::ProfileDone(None));
                }
                Work::Device(operation) => {
                    if let Some(connection) = &mut socket {
                        let request = match operation {
                            0 => vec![32, 0],
                            1 => vec![32, 1],
                            2 => vec![31],
                            _ => vec![33],
                        };
                        match serial_io::exchange(connection, &request)
                            .and_then(|_| crate::autoconfig::read(connection))
                        {
                            Ok(state) => {
                                let _ = events.send(Event::Control(state));
                                match update_configuration(connection, &events, true) {
                                    Ok(()) => {
                                        let _ = events.send(Event::Notice(
                                            match operation {
                                                0 => {
                                                    "Bluetooth disabled; USB motor control restored"
                                                }
                                                1 => "Bluetooth control enabled",
                                                2 => "Autoconfig deleted; settings reset",
                                                _ => "Diagnostics refreshed",
                                            }
                                            .into(),
                                        ));
                                    }
                                    Err(e) => {
                                        let _ = events.send(Event::Error(e.message()));
                                    }
                                }
                            }
                            Err(e) => {
                                let _ = events.send(Event::Error(e.message()));
                            }
                        }
                    }
                    let _ = events.send(Event::ProfileDone(None));
                }
                Work::Save(path, display) => {
                    if let Some(connection) = &mut socket {
                        let result =
                            crate::autoconfig::profile(connection, display).and_then(|profile| {
                                let text = serde_json::to_string_pretty(&profile)
                                    .map_err(|e| Failure::Rejected(e.to_string()))?;
                                std::fs::write(path, text)
                                    .map_err(|e| Failure::Rejected(e.to_string()))
                            });
                        if let Err(e) = result {
                            if matches!(e, Failure::Transport(_)) {
                                close(&mut socket, &events, e.message());
                            }
                            let _ =
                                events.send(Event::Error(format!("Save Config: {}", e.message())));
                        }
                    }
                    let _ = events.send(Event::ProfileDone(None));
                }
                Work::Load(path) => {
                    let mut started = false;
                    let result: Result<Profile, Failure> = (|| {
                        let profile = Profile::load(&path).map_err(Failure::Rejected)?;
                        let connection = socket
                            .as_mut()
                            .ok_or_else(|| Failure::Transport("Not connected".into()))?;
                        started = true;
                        serial_io::exchange(connection, &[32, 0])?;
                        subscribe(connection, &[[Poll::default(); 5]; 2], &events, false)?;
                        let mut warnings = profile.device.apply_inactive(connection)?;
                        let a = &profile.motor.angle;
                        let mut request = vec![34];
                        for value in [
                            a.ratio,
                            a.direction,
                            a.minimum,
                            a.maximum,
                            a.gain,
                            a.speed,
                            a.acceleration,
                            a.deadband,
                        ] {
                            request.extend(value.to_le_bytes());
                        }
                        serial_io::exchange(connection, &request)?;
                        polls = profile.display.plots;
                        subscribe(connection, &polls, &events, false)?;
                        if let Ok(work) = priority.try_recv() {
                            pending.push_front(work);
                            return Err(Failure::Rejected(
                                "Load canceled before restoring motor control".into(),
                            ));
                        }
                        let confirmed = configuration::read(connection)?;
                        if !profile.motor.bluetooth
                            && let Some(board) = profile.device.owner
                        {
                            if confirmed.available[board as usize] {
                                configuration::send(
                                    connection,
                                    &Action::EmgControl {
                                        board,
                                        enabled: true,
                                        config: profile.device.boards[board as usize].curve.clone(),
                                    },
                                )?;
                            } else {
                                warnings.push(format!(
                                    "EMG {board} unavailable; saved motor control remains disabled",
                                ));
                            }
                        }
                        if profile.motor.bluetooth {
                            serial_io::exchange(connection, &[32, 1])?;
                        } else if profile.device.owner.is_none()
                            && profile.motor.manual_velocity != 0.0
                        {
                            configuration::send(
                                connection,
                                &Action::Motor(
                                    (profile.motor.manual_velocity * 10.0).round() as i16
                                ),
                            )?;
                        }
                        update_configuration(connection, &events, true)?;
                        if !warnings.is_empty() {
                            let _ = events.send(Event::Error(warnings.join("\n")));
                        }
                        Ok(profile)
                    })();
                    match result {
                        Ok(profile) => {
                            let _ = events.send(Event::ProfileDone(Some(Box::new(profile))));
                        }
                        Err(e) => {
                            if matches!(e, Failure::Transport(_)) {
                                close(&mut socket, &events, e.message());
                            } else if started && let Some(connection) = &mut socket {
                                let _ = serial_io::exchange(connection, &[2, 0, 0]);
                                let _ = update_configuration(connection, &events, true);
                            }
                            let _ =
                                events.send(Event::Error(format!("Load Config: {}", e.message())));
                            let _ = events.send(Event::ProfileDone(None));
                        }
                    }
                }
                query @ (Work::Diagnostics | Work::MotorStatus) => {
                    if let Some(connection) = &mut socket {
                        let result = if matches!(query, Work::Diagnostics) {
                            serial_io::exchange(connection, &[12]).map(|reply| {
                                let _ = events.send(Event::Data(0, reply));
                            })
                        } else {
                            motor_snapshots(connection, &events)
                        };
                        if let Err(e) = result {
                            if matches!(e, Failure::Transport(_)) {
                                close(&mut socket, &events, e.message());
                            }
                            let _ = events.send(Event::Error(e.message()));
                        }
                    }
                }
                Work::Command(action) => {
                    if matches!(
                        action,
                        Action::Motor(0) | Action::RetryMotorCan | Action::EmgControl { .. }
                    ) {
                        pending.retain(|w| !matches!(w, Work::Command(Action::Motor(_))));
                        while let Ok(work) = requests.try_recv() {
                            if !matches!(work, Work::Command(Action::Motor(_))) {
                                pending.push_back(work);
                            }
                        }
                    }
                    if let Some(connection) = &mut socket {
                        let result = action
                            .encode()
                            .map_err(Failure::Rejected)
                            .and_then(|request| serial_io::exchange(connection, &request));
                        match result {
                            Ok(reply) => {
                                let retry = matches!(action, Action::RetryMotorCan);
                                let actual = if let Reply::Adjusted(v) = reply {
                                    Some(v)
                                } else {
                                    None
                                };
                                let _ = events.send(Event::Applied(action, actual));
                                let result = update_configuration(connection, &events, false)
                                    .and_then(|()| {
                                        if retry {
                                            motor_snapshots(connection, &events)
                                        } else {
                                            Ok(())
                                        }
                                    });
                                if let Err(e) = result {
                                    close(&mut socket, &events, e.message());
                                }
                            }
                            Err(Failure::Device(code)) => {
                                if (3..=5).contains(&code)
                                    && matches!(action, Action::Activate(Kind::Raw, true))
                                {
                                    let _ = events.send(Event::Applied(action.clone(), None));
                                }
                                let mut message = format!(
                                    "{}: {}",
                                    action.description(),
                                    Failure::Device(code).message()
                                );
                                let result: Result<(), Failure> = (|| {
                                    if (3..=5).contains(&code) {
                                        if let Reply::HardwareErrors(boards) =
                                            serial_io::exchange(connection, &[27])?
                                        {
                                            message.push_str(&hardware_failure_details(boards));
                                        }
                                        update_configuration(connection, &events, false)?;
                                    }
                                    if matches!(
                                        action,
                                        Action::Motor(_)
                                            | Action::RetryMotorCan
                                            | Action::EmgControl { .. }
                                    ) {
                                        motor_snapshots(connection, &events)?;
                                    }
                                    Ok(())
                                })(
                                );
                                let _ = events.send(Event::Error(message));
                                if let Err(e) = result
                                    && matches!(e, Failure::Transport(_))
                                {
                                    close(&mut socket, &events, e.message());
                                }
                            }
                            Err(e) => {
                                if matches!(e, Failure::Transport(_)) {
                                    close(&mut socket, &events, e.message());
                                }
                                let _ = events.send(Event::Error(e.message()));
                            }
                        }
                    }
                }
            }
        } else {
            thread::sleep(Duration::from_millis(2));
        }
        if checked.elapsed() >= Duration::from_millis(100) {
            checked = Instant::now();
            if socket.as_ref().is_some_and(|s| !s.is_alive()) {
                close(
                    &mut socket,
                    &events,
                    "Serial connection lost; device settings and motor mode remain active".into(),
                );
            }
        }
        if socket.is_none() {
            *fast.lock().unwrap() = None;
        }
    }
}
pub(crate) fn hardware_failure_details(boards: [[u32; 6]; 2]) -> String {
    let mut text = String::new();
    for (board, [operation, device, register, expected, actual, error]) in
        boards.into_iter().enumerate()
    {
        if operation == 0 {
            continue;
        }
        if operation == 3 {
            text.push_str(&format!("\nEMG {board}: I²C 0x{device:02X} register 0x{register:02X}: expected 0x{expected:X}, received 0x{actual:X}"));
        } else {
            text.push_str(&format!(
                "\nEMG {board}: I²C {} 0x{device:02X} register 0x{register:02X}: {} (0x{error:X})",
                if operation == 1 { "write" } else { "read" },
                match error {
                    0x107 => "timeout",
                    0x108 => "NACK",
                    _ => "driver error",
                }
            ));
        }
    }
    text
}
fn update_configuration(
    connection: &mut SerialConnection,
    events: &Sender<Event>,
    drafts: bool,
) -> Result<(), Failure> {
    let control = crate::autoconfig::read(connection)?;
    let _ = events.send(Event::Control(control));
    let config = configuration::read(connection)?;
    connection.set_configuration(config.clone());
    let _ = events.send(Event::Configuration(Box::new(config), drafts));
    Ok(())
}
fn subscribe(
    connection: &mut SerialConnection,
    polls: &[[Poll; 5]; 2],
    events: &Sender<Event>,
    drafts: bool,
) -> Result<(), Failure> {
    let mut mask = 0u16;
    for (board, plots) in polls.iter().enumerate() {
        for (kind, plot) in plots.iter().enumerate() {
            if plot.enabled {
                mask |= 1 << ([0, 4, 6, 2, 8][kind] + board);
            }
        }
    }
    let mut request = vec![26];
    request.extend(mask.to_le_bytes());
    match serial_io::exchange(connection, &request)? {
        Reply::Subscribed { session, epoch } if session != 0 && epoch != 0 => {}
        _ => {
            return Err(Failure::Transport(
                "Invalid serial subscription response".into(),
            ));
        }
    }
    update_configuration(connection, events, drafts)
}
