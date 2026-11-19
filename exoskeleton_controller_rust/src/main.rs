mod autoconfig;
mod configuration;
mod connection;
mod fourier_view;
mod motor_diagnostics;
mod protocol;
mod serial_io;
mod telemetry;
mod wire;

use connection::{Connection, Event, Poll, Work};
use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints, Points};
use protocol::{Action, EmgMotorConfig, FourierConfig, Hardware, Kind, NotchConfig, Reply};
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

#[derive(Clone, Copy)]
struct Point {
    time: f64,
    value: f64,
    discontinuity: bool,
}

struct PlotCache {
    histories: [VecDeque<Point>; 6],
    spectrum: Option<protocol::Spectrum>,
    updated: [Instant; 5],
}

impl Default for PlotCache {
    fn default() -> Self {
        Self {
            histories: std::array::from_fn(|_| VecDeque::new()),
            spectrum: None,
            updated: [Instant::now() - Duration::from_secs(5); 5],
        }
    }
}

struct Board {
    available: bool,
    display: PlotCache,
    control: EmgMotorConfig,
    applied_control: EmgMotorConfig,
    detection: VecDeque<Point>,
    motor_requests: VecDeque<Point>,
    detection_frame: Option<u32>,
    detection_peak: Option<f32>,
    detection_velocity: f32,
    resend_enabled: bool,
    resend_ms: u16,
    notches: Vec<NotchConfig>,
    applied_notches: Vec<NotchConfig>,
    applied_notch: NotchConfig,
    filtered: VecDeque<Point>,
    filtered_sequence: Option<u32>,
    filtered_notice: String,
    latest_raw: Option<i32>,
    draft: Hardware,
    applied: Hardware,
    actual_hp: Option<f32>,
    actual_lp: Option<f32>,
    integration_ms: u16,
    applied_integration_ms: u16,
    fourier: FourierConfig,
    applied_fourier: FourierConfig,
    polls: [Poll; 5],
    raw: VecDeque<Point>,
    envelope: VecDeque<Point>,
    integral: VecDeque<Point>,
    spectrum: Option<protocol::Spectrum>,
    last_sequence: Option<u32>,
    waiting: [bool; 5],
    notice: String,
    show_envelope: bool,
}

impl Board {
    fn refresh_display(&mut self) {
        for kind in Kind::ALL {
            let slot = kind as usize;
            if !self.polls[slot].enabled
                || self.display.updated[slot].elapsed()
                    < Duration::from_millis(self.polls[slot].milliseconds as u64)
            {
                continue;
            }
            self.display.updated[slot] = Instant::now();
            match kind {
                Kind::Raw => self.display.histories[0] = self.raw.clone(),
                Kind::Filtered => self.display.histories[1] = self.filtered.clone(),
                Kind::Integration => {
                    self.display.histories[2] = self.envelope.clone();
                    self.display.histories[3] = self.integral.clone();
                }
                Kind::Fourier => self.display.spectrum = self.spectrum.clone(),
                Kind::Detection => {
                    self.display.histories[4] = self.detection.clone();
                    self.display.histories[5] = self.motor_requests.clone();
                }
            }
        }
    }
}

impl Default for Board {
    fn default() -> Self {
        Self {
            available: false,
            display: PlotCache::default(),
            control: EmgMotorConfig::default(),
            applied_control: EmgMotorConfig::default(),
            detection: VecDeque::new(),
            motor_requests: VecDeque::new(),
            detection_frame: None,
            detection_peak: None,
            detection_velocity: 0.0,
            resend_enabled: false,
            resend_ms: 1000,
            notches: vec![NotchConfig::default()],
            applied_notches: Vec::new(),
            applied_notch: NotchConfig::default(),
            filtered: VecDeque::new(),
            filtered_sequence: None,
            filtered_notice: String::new(),
            latest_raw: None,
            draft: Hardware::default(),
            applied: Hardware::default(),
            actual_hp: None,
            actual_lp: None,
            integration_ms: 100,
            applied_integration_ms: 100,
            fourier: FourierConfig::default(),
            applied_fourier: FourierConfig::default(),
            polls: [Poll::default(); 5],
            raw: VecDeque::new(),
            envelope: VecDeque::new(),
            integral: VecDeque::new(),
            spectrum: None,
            last_sequence: None,
            waiting: [false; 5],
            notice: String::new(),
            show_envelope: true,
        }
    }
}

struct App {
    notice: String,
    connection: Connection,
    serial_port: String,
    ports: Vec<String>,
    connected: bool,
    bluetooth: bool,
    angle_active: bool,
    control_diagnostics: String,
    connecting: bool,
    enabled: [bool; 3],
    velocity: f32,
    commanded_velocity: f32,
    boards: [Board; 2],
    error: String,
    diagnostics: Option<protocol::Acquisition>,
    motor_status: Option<protocol::MotorStatus>,
    motor_can: Option<protocol::MotorCan>,
    motor_retry_pending: bool,
    emg_owner: Option<u8>,
    emg_control_pending: bool,
    config_busy: bool,
    telemetry_epoch: u32,
    applied_ignore: fourier_view::FourierIgnore,
    autoscroll: bool,
    autofit_y: bool,
    x_seconds: f64,
    fourier_ignore: fourier_view::FourierIgnore,
}

impl App {
    fn new() -> Self {
        Self {
            connection: Connection::new(),
            notice: String::new(),
            serial_port: String::new(),
            ports: Vec::new(),
            connected: false,
            bluetooth: false,
            angle_active: false,
            control_diagnostics: String::new(),
            connecting: false,
            enabled: [false; 3],
            velocity: 0.0,
            commanded_velocity: 0.0,
            boards: [Board::default(), Board::default()],
            error: String::new(),
            diagnostics: None,
            motor_status: None,
            motor_can: None,
            motor_retry_pending: false,
            emg_owner: None,
            emg_control_pending: false,
            config_busy: false,
            telemetry_epoch: 0,
            applied_ignore: fourier_view::FourierIgnore::default(),
            autoscroll: true,
            autofit_y: true,
            x_seconds: 10.0,
            fourier_ignore: fourier_view::FourierIgnore::default(),
        }
    }

    fn send(&mut self, work: Work) {
        if matches!(
            &work,
            Work::Command(_) | Work::SaveSd(_) | Work::Device(_) | Work::Save(..) | Work::Load(_)
        ) {
            self.error.clear();
            self.notice.clear();
        }
        let urgent = matches!(
            &work,
            Work::Disconnect
                | Work::Quit
                | Work::Command(
                    Action::Motor(0) | Action::RetryMotorCan | Action::EmgControl { .. }
                )
        );
        let sender = if urgent {
            &self.connection.urgent
        } else {
            &self.connection.commands
        };
        if sender.try_send(work).is_err() {
            self.config_busy = false;
            self.motor_retry_pending = false;
            self.emg_control_pending = false;
            self.error = "Command queue is full; wait for the current request".into();
        }
    }

    fn sync_polls(&mut self) {
        self.send(Work::Polls([self.boards[0].polls, self.boards[1].polls]));
    }

    fn reset(&mut self) {
        self.connected = false;
        self.connecting = false;
        self.enabled = [false; 3];
        self.velocity = 0.0;
        self.commanded_velocity = 0.0;
        self.boards = [Board::default(), Board::default()];
        self.diagnostics = None;
        self.motor_status = None;
        self.motor_can = None;
        self.motor_retry_pending = false;
        self.emg_owner = None;
        self.emg_control_pending = false;
        self.config_busy = false;
    }

    fn apply(&mut self, action: Action, actual: Option<f32>) {
        match action {
            Action::Ignore(_) => {}
            Action::NotchBank { board, .. } => {
                let board = &mut self.boards[board as usize];
                board.filtered.clear();
                board.envelope.clear();
                board.integral.clear();
                board.spectrum = None;
                board.detection.clear();
                board.motor_requests.clear();
            }
            Action::RetryMotorCan => {
                self.emg_owner = None;
                self.emg_control_pending = false;
                self.config_busy = false;
                self.motor_retry_pending = false;
                self.velocity = 0.0;
                self.commanded_velocity = 0.0;
                self.motor_status = None;
            }
            Action::Motor(velocity) => {
                self.commanded_velocity = velocity as f32 / 10.0;
                if velocity == 0 {
                    self.emg_owner = None;
                }
            }
            Action::EmgControl {
                board,
                enabled,
                config,
            } => {
                self.emg_control_pending = false;
                if enabled {
                    self.emg_owner = Some(board);
                } else if self.emg_owner == Some(board) {
                    self.emg_owner = None;
                }
                self.boards[board as usize].applied_control = config;
                self.velocity = 0.0;
                self.commanded_velocity = 0.0;
            }
            Action::Activate(kind, enabled) => {
                self.enabled[kind as usize] = enabled;
                if !enabled {
                    if matches!(kind, Kind::Raw | Kind::Fourier) {
                        self.emg_owner = None;
                        self.emg_control_pending = false;
                        self.config_busy = false;
                    }
                    if kind == Kind::Raw {
                        self.enabled = [false; 3];
                    }
                    for board in &mut self.boards {
                        if kind == Kind::Raw {
                            board.resend_enabled = false;
                        }
                        for index in 0..5 {
                            if kind == Kind::Raw
                                || index == kind as usize
                                || (kind == Kind::Fourier && index == 4)
                            {
                                board.polls[index].enabled = false;
                            }
                        }
                        board.raw.clear();
                        board.filtered.clear();
                        board.filtered_sequence = None;
                        board.filtered_notice.clear();
                        board.latest_raw = None;
                        board.integral.clear();
                        board.envelope.clear();
                        board.spectrum = None;
                        board.last_sequence = None;
                        board.waiting = [false; 5];
                        board.detection.clear();
                        board.motor_requests.clear();
                        board.detection_frame = None;
                    }
                    self.sync_polls();
                }
            }
            Action::Property {
                board,
                property,
                value,
            } => {
                let board = &mut self.boards[board as usize];
                match property {
                    0 => board.applied.ina_gain = value as u8,
                    1 => board.applied.adc_gain = value as u8,
                    2 => {
                        board.applied.high_pass = value;
                        board.actual_hp = actual;
                    }
                    3 => {
                        board.applied.low_pass = value;
                        board.actual_lp = actual;
                    }
                    _ => {}
                }
                board.raw.clear();
                board.filtered.clear();
                board.filtered_sequence = None;
                board.latest_raw = None;
                board.integral.clear();
                board.envelope.clear();
                board.spectrum = None;
                board.last_sequence = None;
            }
            Action::Integration {
                board,
                milliseconds,
            } => {
                let board = &mut self.boards[board as usize];
                board.applied_integration_ms = milliseconds;
                board.integral.clear();
                board.envelope.clear();
            }
            Action::Fourier { board, config } => {
                let board = &mut self.boards[board as usize];
                board.applied_fourier = config;
                board.spectrum = None;
            }
            Action::ResendHardware {
                board,
                milliseconds,
            } => {
                let board = &mut self.boards[board as usize];
                board.resend_enabled = milliseconds != 0;
                if milliseconds != 0 {
                    board.resend_ms = milliseconds;
                }
            }
            Action::Notch { board, config } => {
                let board = &mut self.boards[board as usize];
                board.applied_notch = config;
                board.filtered.clear();
                board.filtered_sequence = None;
                board.filtered_notice.clear();
                board.integral.clear();
                board.envelope.clear();
                board.spectrum = None;
                board.waiting[3] = false;
                if !board.applied_notch.enabled {
                    board.polls[3].enabled = false;
                }
                self.sync_polls();
            }
        }
    }

    fn events(&mut self) {
        let mut events: Vec<_> = self.connection.events.try_iter().collect();
        // Configuration events precede telemetry, so old epochs never change
        // plots or motor ownership after an acknowledged settings change.
        let mut queued = Some(
            self.connection
                .telemetry
                .try_iter()
                .take(4096)
                .collect::<Vec<_>>(),
        );
        let mut telemetry_loaded = false;
        loop {
            for event in events.drain(..) {
                match event {
                    Event::Notice(message) => {
                        self.notice = message;
                        self.error.clear();
                    }
                    Event::Control(state) => {
                        self.bluetooth = state.motor.bluetooth;
                        self.angle_active = state.angle_enabled;
                        self.commanded_velocity = state.motor.manual_velocity;
                        self.control_diagnostics = format!(
                            "Mode {} · calibrated {} · BLE available/synced {}/{} · last BLE error 0x{:X} · queue full {} · notify failures {} · heap {}/{} bytes · BLE stack {} · last SD error 0x{:X}",
                            state.mode,
                            state.calibrated,
                            state.diagnostics[0],
                            state.diagnostics[1],
                            state.diagnostics[2],
                            state.diagnostics[3],
                            state.diagnostics[4],
                            state.diagnostics[5],
                            state.diagnostics[6],
                            state.diagnostics[7],
                            state.diagnostics[8]
                        );
                    }
                    Event::Ports(ports) => {
                        if self.serial_port.is_empty() {
                            self.serial_port = ports.first().cloned().unwrap_or_default();
                        }
                        self.ports = ports;
                    }
                    Event::Connected => {
                        self.reset();
                        self.connected = true;
                        self.error.clear();
                    }
                    Event::Disconnected(reason) => {
                        self.reset();
                        self.error = reason;
                    }
                    Event::Applied(action, actual) => {
                        self.apply(action, actual);
                    }
                    Event::Error(message) => {
                        self.notice.clear();
                        self.emg_control_pending = false;
                        self.motor_retry_pending = false;
                        eprintln!("{message}");
                        self.error = message;
                    }
                    Event::Configuration(config, drafts) => self.configuration(*config, drafts),
                    Event::ProfileDone(profile) => {
                        self.config_busy = false;
                        if let Some(profile) = profile {
                            let d = profile.display;
                            self.serial_port = d.serial_port;
                            self.autoscroll = d.autoscroll;
                            self.autofit_y = d.autofit_y;
                            self.x_seconds = d.x_seconds;
                            self.velocity = d.manual_velocity;
                            for (b, board) in self.boards.iter_mut().enumerate() {
                                board.polls = d.plots[b];
                                board.show_envelope = d.envelope[b];
                            }
                        }
                    }
                    Event::Data(index, reply) => {
                        let board = &mut self.boards[index as usize];
                        match reply {
                            Reply::Motor(result) => self.motor_status = Some(result),
                            Reply::Detection {
                                overflow,
                                owner,
                                ready,
                                records,
                            } => {
                                self.emg_owner = owner;
                                board.waiting[4] = !ready;
                                let mut gap = overflow;
                                for record in records {
                                    gap |= board
                                        .detection_frame
                                        .is_some_and(|old| record.frame != old.wrapping_add(1));
                                    let time = record.time_us as f64 / 1e6;
                                    board.detection.push_back(Point {
                                        time,
                                        value: if record.flags & 1 != 0 { 1.0 } else { 0.0 },
                                        discontinuity: gap,
                                    });
                                    board.motor_requests.push_back(Point {
                                        time,
                                        value: if record.flags & 4 != 0 { 1.0 } else { 0.0 },
                                        discontinuity: gap,
                                    });
                                    board.detection_frame = Some(record.frame);
                                    board.detection_peak = Some(record.peak);
                                    board.detection_velocity = record.velocity;
                                    gap = false;
                                }
                            }
                            Reply::MotorCan(result) => self.motor_can = Some(result),
                            Reply::Acquisition(result) => self.diagnostics = Some(result),
                            Reply::Raw(batch) => {
                                board.waiting[0] = !batch.active || !batch.configured;
                                if batch.hardware != board.applied {
                                    board.raw.clear();
                                    board.latest_raw = None;
                                    board.last_sequence = None;
                                }
                                board.applied = batch.hardware;
                                if batch.configured {
                                    board.actual_hp = Some(batch.actual_hp);
                                    board.actual_lp = Some(batch.actual_lp);
                                }
                                board.notice = if !batch.active {
                                    "Acquisition off".into()
                                } else if !batch.configured {
                                    "Board configuration failed".into()
                                } else if batch.discontinuity {
                                    "Raw buffer filled; older samples were discarded".into()
                                } else {
                                    String::new()
                                };
                                let scale = protocol::adc_mv_scale(board.applied.adc_gain);
                                for sample in batch.samples {
                                    board.latest_raw = Some(sample.raw);
                                    let gap = board.last_sequence.is_some_and(|previous| {
                                        sample.sequence != previous.wrapping_add(1)
                                    });
                                    board.raw.push_back(Point {
                                        time: sample.time_us as f64 / 1e6,
                                        value: sample.raw as f64 * scale,
                                        discontinuity: gap,
                                    });
                                    board.last_sequence = Some(sample.sequence);
                                }
                            }
                            Reply::Filtered {
                                discontinuity,
                                samples,
                            } => {
                                board.waiting[3] = samples.is_empty();
                                board.filtered_notice = if discontinuity {
                                    "Filtered buffer filled; older samples were discarded".into()
                                } else {
                                    String::new()
                                };
                                let mut gap = discontinuity;
                                for sample in samples {
                                    let missed = board.filtered_sequence.is_some_and(|previous| {
                                        sample.sequence != previous.wrapping_add(1)
                                    });
                                    board.filtered.push_back(Point {
                                        time: sample.time_us as f64 / 1e6,
                                        value: sample.millivolts as f64,
                                        discontinuity: gap || missed,
                                    });
                                    board.filtered_sequence = Some(sample.sequence);
                                    gap = false;
                                }
                            }
                            Reply::Integration(result) => {
                                let was_waiting = board.waiting[1];
                                board.waiting[1] = false;
                                board.applied_integration_ms = result.window_ms;
                                let time = result.time_us as f64 / 1e6;
                                if board.envelope.back().is_none_or(|point| point.time != time) {
                                    let gap = was_waiting
                                        || board.envelope.back().is_some_and(|previous| {
                                            time - previous.time
                                                > board.polls[1].milliseconds as f64 / 1000.0 * 3.0
                                        });
                                    board.envelope.push_back(Point {
                                        time,
                                        value: result.envelope as f64,
                                        discontinuity: gap,
                                    });
                                    board.integral.push_back(Point {
                                        time,
                                        value: result.integral as f64,
                                        discontinuity: gap,
                                    });
                                }
                            }
                            Reply::Fourier(result) => {
                                board.waiting[2] = false;
                                board.applied_fourier = result.config.clone();
                                board.spectrum = Some(result);
                            }
                            _ => {}
                        }
                    }
                }
            }
            if telemetry_loaded {
                break;
            }
            telemetry_loaded = true;
            events.extend(
                queued
                    .take()
                    .unwrap_or_default()
                    .into_iter()
                    .filter(|(epoch, _)| *epoch == self.telemetry_epoch)
                    .map(|(_, event)| event),
            );
        }
        for board in &mut self.boards {
            for history in [
                &mut board.raw,
                &mut board.filtered,
                &mut board.envelope,
                &mut board.integral,
                &mut board.detection,
                &mut board.motor_requests,
            ] {
                if let Some(latest) = history.back().map(|point| point.time) {
                    while history
                        .front()
                        .is_some_and(|point| latest - point.time > self.x_seconds.max(10.0))
                        || history.len() > 120_000
                    {
                        history.pop_front();
                    }
                }
            }
        }
    }

    fn connection_panel(&mut self, ui: &mut egui::Ui) {
        if !self.notice.is_empty() {
            ui.label(&self.notice);
        }
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(
                    self.connected && !self.config_busy,
                    egui::Button::new("Save autoconfig to SD"),
                )
                .on_hover_text(
                    "Saves confirmed device settings. Unsent editor drafts are excluded.",
                )
                .clicked()
            {
                self.config_busy = true;
                self.send(Work::SaveSd(self.display_configuration()));
            }
            if ui
                .add_enabled(
                    self.connected && !self.config_busy,
                    egui::Button::new("Delete autoconfig"),
                )
                .clicked()
            {
                self.config_busy = true;
                self.send(Work::Device(2));
            }
            if ui
                .add_enabled(
                    self.connected && !self.config_busy,
                    egui::Button::new(if self.bluetooth {
                        "Disable Bluetooth control"
                    } else {
                        "Enable Bluetooth control"
                    }),
                )
                .clicked()
            {
                self.config_busy = true;
                self.send(Work::Device(u8::from(!self.bluetooth)));
            }
            if ui
                .add_enabled(
                    self.connected,
                    egui::Button::new("Refresh BLE/SD diagnostics"),
                )
                .clicked()
            {
                self.send(Work::Device(3));
            }
        });
        if self.bluetooth {
            ui.label("Bluetooth owns motor control; USB settings remain available.");
        }
        egui::CollapsingHeader::new("BLE / SD diagnostics").show(ui, |ui| {
            ui.label(&self.control_diagnostics);
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    self.connected && !self.config_busy,
                    egui::Button::new("Save Config"),
                )
                .clicked()
                && let Some(path) = rfd::FileDialog::new()
                    .add_filter("JSON config", &["json"])
                    .set_file_name("ec5-config.json")
                    .save_file()
            {
                self.config_busy = true;
                self.send(Work::Save(path, self.display_configuration()));
            }
            if ui
                .add_enabled(
                    self.connected && !self.config_busy,
                    egui::Button::new("Load Config"),
                )
                .clicked()
                && let Some(path) = rfd::FileDialog::new()
                    .add_filter("JSON config", &["json"])
                    .pick_file()
            {
                self.config_busy = true;
                self.send(Work::Load(path));
            }
            if self.config_busy {
                ui.spinner();
            }
            ui.label("Serial port");
            ui.add_enabled_ui(!self.connected && !self.connecting, |ui| {
                egui::ComboBox::from_id_salt("serial_port")
                    .selected_text(if self.serial_port.is_empty() {
                        "Select port"
                    } else {
                        &self.serial_port
                    })
                    .show_ui(ui, |ui| {
                        for port in &self.ports {
                            ui.selectable_value(&mut self.serial_port, port.clone(), port);
                        }
                    });
                if ui.button("Refresh ports").clicked() {
                    self.send(Work::Ports);
                }
            });
            if self.connected {
                ui.colored_label(egui::Color32::LIGHT_GREEN, "Connected");
                if ui.button("Disconnect").clicked() {
                    self.send(Work::Disconnect);
                }
            } else if self.connecting {
                ui.spinner();
                ui.label("Connecting");
            } else if ui.button("Connect").clicked() {
                if self.serial_port.is_empty() {
                    self.error = "Select a serial port".into();
                } else {
                    self.error.clear();
                    self.connecting = true;
                    self.send(Work::Connect(self.serial_port.clone()));
                }
            }
        });
        if !self.error.is_empty() {
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(egui::Color32::LIGHT_RED, &self.error);
                if ui.small_button("Dismiss").clicked() {
                    self.error.clear();
                }
            });
        }
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        let manual = self.emg_owner.is_none()
            && !self.emg_control_pending
            && !self.motor_retry_pending
            && !self.bluetooth
            && !self.angle_active;
        ui.horizontal(|ui| {
            ui.label("Motor velocity");
            ui.add_enabled(
                manual,
                egui::DragValue::new(&mut self.velocity)
                    .range(-50.0..=50.0)
                    .speed(0.1)
                    .fixed_decimals(1)
                    .suffix(" rad/s"),
            );
            if ui.add_enabled(manual, egui::Button::new("Send")).clicked()
                && self.velocity.is_finite()
            {
                self.send(Work::Command(Action::Motor(
                    (self.velocity * 10.0).round() as i16
                )));
            }
            if ui.button("Stop").clicked() {
                self.velocity = 0.0;
                self.send(Work::Command(Action::Motor(0)));
            }
            ui.label(format!("Requested: {:.1} rad/s", self.commanded_velocity));
            if ui.button("Motor Status").clicked() {
                self.send(Work::MotorStatus);
            }
            if ui
                .add_enabled(
                    !self.motor_retry_pending,
                    egui::Button::new("Retry Motor CAN"),
                )
                .clicked()
            {
                self.motor_retry_pending = true;
                self.emg_owner = None;
                self.emg_control_pending = false;
                self.config_busy = false;
                self.velocity = 0.0;
                self.send(Work::Command(Action::RetryMotorCan));
            }
            if self.motor_retry_pending {
                ui.label("Reconfiguring at zero velocity…");
            }
        });
        if let Some(status) = &self.motor_status {
            let state = if status.flags & 8 != 0 {
                "Fault"
            } else if status.flags & 4 != 0 {
                "Running"
            } else {
                "Stopped"
            };
            ui.label(format!(
                "At Refresh: {state} · ready {} · CAN {} · requested {:.1} · commanded {:.1} rad/s",
                status.flags & 1 != 0,
                status.flags & 2 != 0,
                status.requested,
                status.commanded
            ));
            if !status.reason.is_empty() {
                ui.colored_label(
                    egui::Color32::LIGHT_RED,
                    format!("Motor fault {}: {}", status.fault_code, status.reason),
                );
            } else if !status.inhibit_reason.is_empty() {
                ui.label(format!(
                    "Motor inhibit {}: {}",
                    status.inhibit_code, status.inhibit_reason
                ));
            }
            if status.feedback_time_us == 0 {
                ui.label("No motor feedback received");
            } else {
                ui.label(format!(
                    "Feedback: {:.3} rad · {:.3} rad/s · device {:.3} s",
                    status.position,
                    status.velocity,
                    status.feedback_time_us as f64 / 1e6
                ));
            }
        }
        egui::CollapsingHeader::new("Motor CAN Diagnostics").show(ui, |ui| {
            if ui.button("Refresh CAN Diagnostics").clicked() {
                self.send(Work::MotorStatus);
            }
            if let Some(diagnostics) = &self.motor_can {
                motor_diagnostics::show(ui, diagnostics);
            }
        });
        ui.horizontal(|ui| {
            for kind in [Kind::Raw, Kind::Integration, Kind::Fourier] {
                let mut enabled = self.enabled[kind as usize];
                let allowed = kind == Kind::Raw || self.enabled[0];
                let label = if kind == Kind::Raw {
                    "Acquire EMGs"
                } else {
                    kind.name()
                };
                if ui
                    .add_enabled(allowed, egui::Checkbox::new(&mut enabled, label))
                    .changed()
                {
                    self.send(Work::Command(Action::Activate(kind, enabled)));
                }
            }
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.autoscroll, "Autoscroll X");
            ui.checkbox(&mut self.autofit_y, "Auto-fit Y");
            ui.label("X window");
            ui.add(
                egui::DragValue::new(&mut self.x_seconds)
                    .range(0.01..=120.0)
                    .speed(0.1)
                    .suffix(" s"),
            );
        });
        ui.separator();
    }

    fn fourier_ignore_controls(&mut self, ui: &mut egui::Ui) {
        ui.checkbox(&mut self.fourier_ignore.enabled, "Ignore Fourier segments");
        let mut remove = None;
        for (index, row) in self.fourier_ignore.segments.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.checkbox(&mut row.enabled, "");
                ui.label("Low");
                ui.add(
                    egui::DragValue::new(&mut row.low_hz)
                        .range(0.0..=500.0)
                        .suffix(" Hz"),
                );
                ui.label("High");
                ui.add(
                    egui::DragValue::new(&mut row.high_hz)
                        .range(0.0..=500.0)
                        .suffix(" Hz"),
                );
                if ui.small_button("Remove").clicked() {
                    remove = Some(index);
                }
            });
        }
        if let Some(index) = remove {
            self.fourier_ignore.segments.remove(index);
        }
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    self.fourier_ignore.segments.len() < 16,
                    egui::Button::new("Add segment"),
                )
                .clicked()
            {
                self.fourier_ignore.segments.push(Default::default());
            }
            if ui.button("Apply ignore segments").clicked() {
                self.send(Work::Command(Action::Ignore(self.fourier_ignore.clone())));
            }
            if ui.button("Discard").clicked() {
                self.fourier_ignore = self.applied_ignore.clone();
            }
        });
        ui.separator();
    }

    fn configuration(&mut self, config: configuration::DeviceConfig, drafts: bool) {
        self.telemetry_epoch = config.revision;
        self.enabled = config.enabled;
        self.emg_owner = config.owner;
        self.applied_ignore = config.ignore.clone();
        if drafts {
            self.fourier_ignore = config.ignore;
        }
        for (index, (board, settings)) in self.boards.iter_mut().zip(config.boards).enumerate() {
            board.available = config.available[index];
            board.display = PlotCache::default();
            board.applied = settings.hardware.clone();
            board.applied_control = settings.curve.clone();
            board.applied_fourier = settings.fourier.clone();
            board.applied_integration_ms = settings.integration_ms;
            board.applied_notches = settings.notches.clone();
            board.applied_notch.enabled = settings.notches.iter().any(|n| n.enabled);
            board.resend_enabled = settings.resend_ms != 0;
            if drafts {
                board.draft = settings.hardware;
                board.control = settings.curve;
                board.fourier = settings.fourier;
                board.integration_ms = settings.integration_ms;
                board.notches = settings.notches;
                board.resend_ms = if settings.resend_ms == 0 {
                    1000
                } else {
                    settings.resend_ms
                };
                board.raw.clear();
                board.filtered.clear();
                board.integral.clear();
                board.envelope.clear();
                board.spectrum = None;
                board.detection.clear();
                board.motor_requests.clear();
                board.last_sequence = None;
                board.filtered_sequence = None;
                board.detection_frame = None;
            }
        }
    }

    fn display_configuration(&self) -> configuration::DisplayConfig {
        configuration::DisplayConfig {
            serial_port: self.serial_port.clone(),
            plots: [self.boards[0].polls, self.boards[1].polls],
            x_seconds: self.x_seconds,
            autoscroll: self.autoscroll,
            autofit_y: self.autofit_y,
            envelope: [self.boards[0].show_envelope, self.boards[1].show_envelope],
            manual_velocity: self.velocity,
        }
    }

    fn acquisition_panel(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Acquisition Diagnostics").show(ui, |ui| {
            if ui.button("Refresh").clicked() {
                self.send(Work::Diagnostics);
            }
            if let Some(d) = &self.diagnostics {
                let state = ["Starting", "Paused", "Running", "Fault"][d.state as usize];
                ui.label(format!(
                    "LP at Refresh: {state} · generation {} · activity {}",
                    d.generation, d.heartbeat
                ));
                if d.fault != 0 {
                    let message = match d.fault {
                        1 => "LP mailbox timed out",
                        2 => "LP acquisition stopped making progress",
                        3 => "LP boot / I²C initialization failed",
                        _ => "LP mailbox state was invalid",
                    };
                    ui.colored_label(egui::Color32::LIGHT_RED, message);
                }
                ui.label(format!(
                    "Shared ring: {} / {} · high-water {}",
                    d.occupancy, d.capacity, d.high_water
                ));
                egui::Grid::new("capture_diagnostics").show(ui, |ui| {
                    ui.label("");
                    ui.strong("EMG 0");
                    ui.strong("EMG 1");
                    ui.end_row();
                    for (label, values) in [
                        (
                            "Configured",
                            [d.configured_mask & 1, (d.configured_mask >> 1) & 1],
                        ),
                        ("Conversions read", d.conversions),
                        ("ADC counter gap events", d.adc_gaps),
                        ("I²C read errors", d.read_errors),
                        ("Shared records published", d.published),
                        ("Shared-ring discarded records", d.discarded),
                        ("Max I²C read (µs)", d.max_read_us),
                        ("Raw-query overwritten records", d.raw_overwrites),
                        (
                            "Processing-buffer overwritten records",
                            d.processing_overruns,
                        ),
                    ] {
                        ui.label(label);
                        ui.monospace(values[0].to_string());
                        ui.monospace(values[1].to_string());
                        ui.end_row();
                    }
                });
                ui.label(format!(
                    "Late LP passes: {} · max pass: {} µs · max HP drain interval: {} µs",
                    d.late_passes, d.max_pass_us, d.max_drain_us
                ));
            }
        });
        ui.separator();
    }

    fn board_panel(&mut self, ui: &mut egui::Ui, index: usize) {
        let mut actions = Vec::new();
        let mut polling_changed = false;
        let board = &mut self.boards[index];
        board.refresh_display();
        ui.heading(format!("EMG {index}"));
        if self.enabled[0] && !board.available {
            ui.colored_label(
                egui::Color32::YELLOW,
                "EMG unavailable — acquisition continues on other available boards",
            );
        }
        egui::Grid::new(("hardware", index))
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("Amplifier gain");
                gain_combo(ui, (index, "ina"), &mut board.draft.ina_gain);
                ui.end_row();
                ui.label("ADC PGA gain");
                gain_combo(ui, (index, "adc"), &mut board.draft.adc_gain);
                ui.end_row();
                ui.label("High-pass");
                frequency(ui, &mut board.draft.high_pass, 3.3..=70.9);
                ui.end_row();
                ui.label("Low-pass");
                frequency(ui, &mut board.draft.low_pass, 152.0..=450.0);
                ui.end_row();
            });
        ui.horizontal(|ui| {
            if ui.button("Apply EMG Settings").clicked() {
                let requested = [
                    board.draft.ina_gain as u16,
                    board.draft.adc_gain as u16,
                    board.draft.high_pass,
                    board.draft.low_pass,
                ];
                let applied = [
                    board.applied.ina_gain as u16,
                    board.applied.adc_gain as u16,
                    board.applied.high_pass,
                    board.applied.low_pass,
                ];
                for property in 0..4 {
                    if requested[property] != applied[property] {
                        actions.push(Action::Property {
                            board: index as u8,
                            property: property as u8,
                            value: requested[property],
                        });
                    }
                }
            }
            if ui.button("Discard").clicked() {
                board.draft = board.applied.clone();
            }
        });
        if let (Some(hp), Some(lp)) = (board.actual_hp, board.actual_lp) {
            ui.label(format!("Quantized filters: {hp:.2} / {lp:.2} Hz"));
        }
        ui.horizontal(|ui| {
            let mut enabled = board.resend_enabled;
            if ui
                .checkbox(&mut enabled, "Resend hardware parameters")
                .changed()
            {
                actions.push(Action::ResendHardware {
                    board: index as u8,
                    milliseconds: if enabled { board.resend_ms } else { 0 },
                });
            }
            ui.add(
                egui::DragValue::new(&mut board.resend_ms)
                    .range(100..=65535)
                    .speed(100)
                    .suffix(" ms"),
            );
            if ui
                .add_enabled(board.resend_enabled, egui::Button::new("Apply interval"))
                .clicked()
            {
                actions.push(Action::ResendHardware {
                    board: index as u8,
                    milliseconds: board.resend_ms,
                });
            }
        });
        ui.label("Software notch filters");
        let mut remove = None;
        for (row, notch) in board.notches.iter_mut().enumerate() {
            ui.horizontal_wrapped(|ui| {
                ui.checkbox(&mut notch.enabled, "Enabled");
                ui.label("Frequency");
                frequency(ui, &mut notch.frequency, 1.0..=499.0);
                ui.label("Width")
                    .on_hover_text("Width at half the center attenuation in dB");
                frequency(ui, &mut notch.bandwidth, 0.1..=100.0);
                ui.label("Center");
                let mut db = notch.attenuation as f32 / 10.0;
                if ui
                    .add(
                        egui::DragValue::new(&mut db)
                            .range(-60.0..=-0.1)
                            .speed(0.1)
                            .suffix(" dB"),
                    )
                    .changed()
                {
                    notch.attenuation = (db * 10.0).round() as i16;
                }
                if ui.small_button("Remove").clicked() {
                    remove = Some(row);
                }
            });
        }
        if let Some(row) = remove {
            board.notches.remove(row);
        }
        ui.horizontal(|ui| {
            if ui
                .add_enabled(board.notches.len() < 8, egui::Button::new("Add notch"))
                .clicked()
            {
                board.notches.push(NotchConfig::default());
            }
            if ui.button("Apply notches").clicked() {
                actions.push(Action::NotchBank {
                    board: index as u8,
                    filters: board.notches.clone(),
                });
            }
            if ui.button("Discard").clicked() {
                board.notches = board.applied_notches.clone();
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Integration window");
            ui.add(
                egui::DragValue::new(&mut board.integration_ms)
                    .range(10..=1000)
                    .suffix(" ms"),
            );
            if ui.button("Apply").clicked() {
                actions.push(Action::Integration {
                    board: index as u8,
                    milliseconds: board.integration_ms,
                });
            }
            if ui.small_button("Discard").clicked() {
                board.integration_ms = board.applied_integration_ms;
            }
        });
        ui.horizontal(|ui| {
            ui.label("FFT hop");
            egui::ComboBox::from_id_salt((index, "hop"))
                .selected_text(board.fourier.hop.to_string())
                .show_ui(ui, |ui| {
                    for hop in [64, 128, 256] {
                        ui.selectable_value(&mut board.fourier.hop, hop, hop.to_string());
                    }
                });
            egui::ComboBox::from_id_salt((index, "window"))
                .selected_text(if board.fourier.window == 1 {
                    "Hann"
                } else {
                    "Rectangular"
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut board.fourier.window, 1, "Hann");
                    ui.selectable_value(&mut board.fourier.window, 0, "Rectangular");
                });
        });
        ui.horizontal(|ui| {
            ui.checkbox(&mut board.fourier.remove_mean, "Remove mean");
            if ui.button("Apply Fourier Settings").clicked() {
                actions.push(Action::Fourier {
                    board: index as u8,
                    config: board.fourier.clone(),
                });
            }
            if ui.small_button("Discard").clicked() {
                board.fourier = board.applied_fourier.clone();
            }
        });
        ui.separator();
        for kind in Kind::ALL
            .into_iter()
            .filter(|kind| *kind != Kind::Detection)
        {
            let slot = kind as usize;
            ui.horizontal(|ui| {
                polling_changed |= ui
                    .add_enabled(
                        if kind == Kind::Filtered {
                            self.enabled[0] && board.applied_notch.enabled
                        } else {
                            self.enabled[slot]
                        },
                        egui::Checkbox::new(
                            &mut board.polls[slot].enabled,
                            format!("Plot {}", kind.name()),
                        ),
                    )
                    .changed();
                polling_changed |= ui
                    .add(
                        egui::DragValue::new(&mut board.polls[slot].milliseconds)
                            .range(20..=5000)
                            .speed(20)
                            .suffix(" ms"),
                    )
                    .changed();
                if board.polls[slot].enabled && board.waiting[slot] {
                    ui.label("Waiting for continuous samples");
                }
            });
            if !board.polls[slot].enabled {
                continue;
            }
            match kind {
                Kind::Detection => continue, // Rendered in the EMG control section below.
                Kind::Raw => {
                    if let Some(raw) = board.latest_raw {
                        ui.monospace(format!(
                            "ADC 0x{:06X} · {} counts · {:.5} mV · PGA {}×",
                            (raw as u32) & 0xFFFFFF,
                            raw,
                            raw as f64 * protocol::adc_mv_scale(board.applied.adc_gain),
                            board.applied.adc_gain
                        ));
                    }
                    if !board.notice.is_empty() {
                        ui.colored_label(egui::Color32::YELLOW, &board.notice);
                    }
                    time_plot(
                        ui,
                        (index, slot),
                        &board.display.histories[0],
                        "mV",
                        PlotControls {
                            autoscroll: self.autoscroll,
                            autofit_y: self.autofit_y,
                            seconds: self.x_seconds,
                        },
                        egui::Color32::LIGHT_BLUE,
                    );
                }
                Kind::Filtered => {
                    if !board.filtered_notice.is_empty() {
                        ui.colored_label(egui::Color32::YELLOW, &board.filtered_notice);
                    }
                    time_plot(
                        ui,
                        (index, slot),
                        &board.display.histories[1],
                        "mV",
                        PlotControls {
                            autoscroll: self.autoscroll,
                            autofit_y: self.autofit_y,
                            seconds: self.x_seconds,
                        },
                        egui::Color32::LIGHT_RED,
                    );
                }
                Kind::Integration => {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut board.show_envelope, true, "Envelope (mV)");
                        ui.selectable_value(&mut board.show_envelope, false, "Integral (mV·s)");
                    });
                    let (history, unit) = if board.show_envelope {
                        (&board.display.histories[2], "mV")
                    } else {
                        (&board.display.histories[3], "mV·s")
                    };
                    time_plot(
                        ui,
                        (index, slot),
                        history,
                        unit,
                        PlotControls {
                            autoscroll: self.autoscroll,
                            autofit_y: self.autofit_y,
                            seconds: self.x_seconds,
                        },
                        egui::Color32::LIGHT_GREEN,
                    );
                }
                Kind::Fourier => {
                    Plot::new((index, "spectrum"))
                        .height(200.0)
                        .x_axis_label("Hz")
                        .y_axis_label("mV²/Hz")
                        .show(ui, |plot| {
                            if let Some(spectrum) = &board.display.spectrum {
                                let visible = self
                                    .applied_ignore
                                    .plot_data(&spectrum.power, spectrum.sample_rate);
                                plot.set_plot_bounds_y(0.0..=visible.y_max);
                                for segment in visible.segments {
                                    if segment.len() == 1 {
                                        plot.points(
                                            Points::new("PSD", PlotPoints::from(segment))
                                                .color(egui::Color32::GOLD),
                                        );
                                    } else {
                                        plot.line(
                                            Line::new("PSD", PlotPoints::from(segment))
                                                .color(egui::Color32::GOLD),
                                        );
                                    }
                                }
                            }
                        });
                    if let Some(spectrum) = &board.display.spectrum {
                        ui.small(format!(
                            "256 samples · frame {} · {:.3} s",
                            spectrum.frame,
                            spectrum.time_us as f64 / 1e6
                        ));
                    }
                }
            }
        }
        ui.separator();
        ui.heading("Control Motor by EMG");
        let mut controls = self.emg_owner == Some(index as u8);
        let allowed = !self.emg_control_pending
            && !self.motor_retry_pending
            && (controls || (board.available && self.enabled[0] && self.enabled[2]));
        let toggled = ui
            .add_enabled(allowed, egui::Checkbox::new(&mut controls, "Enabled"))
            .changed();
        ui.horizontal_wrapped(|ui| {
            ui.label("Minimum FFT peak");
            ui.add(
                egui::DragValue::new(&mut board.control.threshold)
                    .range(0.000001..=1e6)
                    .speed(0.0001)
                    .max_decimals(6)
                    .suffix(" mV²/Hz"),
            );
            ui.label("Velocity");
            let mut velocity = board.control.velocity as f32 / 10.0;
            if ui
                .add(
                    egui::DragValue::new(&mut velocity)
                        .range(-50.0..=50.0)
                        .speed(0.1)
                        .fixed_decimals(1)
                        .suffix(" rad/s"),
                )
                .changed()
            {
                board.control.velocity = (velocity * 10.0).round() as i16;
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("Maximum FFT peak");
            ui.add(
                egui::DragValue::new(&mut board.control.maximum)
                    .range(0.000001..=1e6)
                    .speed(0.0001)
                    .max_decimals(6)
                    .suffix(" mV²/Hz"),
            );
            ui.label("Velocity");
            let mut v = board.control.maximum_velocity as f32 / 10.0;
            if ui
                .add(
                    egui::DragValue::new(&mut v)
                        .range(-50.0..=50.0)
                        .speed(0.1)
                        .fixed_decimals(1)
                        .suffix(" rad/s"),
                )
                .changed()
            {
                board.control.maximum_velocity = (v * 10.0).round() as i16;
            }
        });
        let apply = ui
            .add_enabled(
                !self.emg_control_pending,
                egui::Button::new("Apply control settings"),
            )
            .clicked();
        if toggled || apply {
            let config = board.control.clone();
            actions.push(Action::EmgControl {
                board: index as u8,
                enabled: controls,
                config,
            });
        }
        if !self.enabled[0] || !self.enabled[2] {
            ui.label("Enable Acquire EMGs and Fourier to control the motor");
        }
        if self.emg_owner == Some(index as u8) {
            ui.label(format!(
                "Applied: peak ≥ {:.6} mV²/Hz → {:.1} rad/s",
                board.applied_control.threshold,
                board.applied_control.velocity as f32 / 10.0
            ));
            ui.small(format!(
                "Maximum {:.6} → {:.1} rad/s; applied ignore segments excluded",
                board.applied_control.maximum,
                board.applied_control.maximum_velocity as f32 / 10.0
            ));
        }
        ui.horizontal(|ui| {
            polling_changed |= ui
                .add_enabled(
                    self.enabled[2],
                    egui::Checkbox::new(&mut board.polls[4].enabled, "Plot Detection"),
                )
                .changed();
            polling_changed |= ui
                .add(
                    egui::DragValue::new(&mut board.polls[4].milliseconds)
                        .range(20..=5000)
                        .suffix(" ms"),
                )
                .changed();
        });
        if board.polls[4].enabled {
            if board.waiting[4] {
                ui.label("Waiting for continuous FFT samples");
            }
            if let Some(peak) = board.detection_peak {
                ui.label(format!(
                    "Last eligible peak: {peak:.6} mV²/Hz · motor request {:.1} rad/s",
                    board.detection_velocity
                ));
            }
            detection_plot(
                ui,
                index,
                &board.display.histories[4],
                &board.display.histories[5],
                self.autoscroll,
                self.x_seconds,
            );
        }
        for action in actions {
            if matches!(action, Action::EmgControl { .. }) {
                self.emg_control_pending = true;
            }
            self.send(Work::Command(action));
        }
        if polling_changed {
            self.sync_polls();
        }
    }
}

fn detection_plot(
    ui: &mut egui::Ui,
    board: usize,
    detected: &VecDeque<Point>,
    requested: &VecDeque<Point>,
    autoscroll: bool,
    seconds: f64,
) {
    Plot::new((board, "detection"))
        .height(170.0)
        .x_axis_label("Device time (s)")
        .y_axis_label("0 / 1")
        .legend(egui_plot::Legend::default())
        .show(ui, |plot| {
            plot.set_auto_bounds(egui::Vec2b::new(false, false));
            plot.set_plot_bounds_y(-0.05..=1.05);
            let key = plot.response().id.with("span");
            let old = plot.ctx().data_mut(|data| {
                let old = data.get_temp::<f64>(key);
                if !detected.is_empty() {
                    data.insert_temp(key, seconds);
                }
                old
            });
            if (autoscroll || old.is_none())
                && let Some(last) = detected.back()
            {
                let right = last.time.max(seconds);
                plot.set_plot_bounds_x(right - seconds..=right);
            } else if old.is_some_and(|previous| previous != seconds) {
                let bounds = plot.plot_bounds();
                let center = (bounds.min()[0] + bounds.max()[0]) * 0.5;
                plot.set_plot_bounds_x(center - seconds / 2.0..=center + seconds / 2.0);
            }
            for (history, label, color) in [
                (detected, "Detected", egui::Color32::GOLD),
                (requested, "Motor request", egui::Color32::LIGHT_GREEN),
            ] {
                let mut segment: Vec<[f64; 2]> = Vec::new();
                for point in history {
                    if point.discontinuity && !segment.is_empty() {
                        plot.line(
                            Line::new(label, PlotPoints::from(std::mem::take(&mut segment)))
                                .color(color),
                        );
                    }
                    if let Some(previous) = segment.last() {
                        segment.push([point.time, previous[1]]);
                    }
                    segment.push([point.time, point.value]);
                }
                if !segment.is_empty() {
                    plot.line(Line::new(label, PlotPoints::from(segment)).color(color));
                }
            }
        });
}

fn gain_combo(ui: &mut egui::Ui, id: impl std::hash::Hash, value: &mut u8) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(format!("{value}×"))
        .show_ui(ui, |ui| {
            for gain in [1, 2, 4, 8, 16, 32, 64, 128] {
                ui.selectable_value(value, gain, format!("{gain}×"));
            }
        });
}

fn frequency(ui: &mut egui::Ui, value: &mut u16, range: std::ops::RangeInclusive<f32>) {
    let mut hz = *value as f32 / 10.0;
    if ui
        .add(
            egui::DragValue::new(&mut hz)
                .range(range)
                .speed(0.1)
                .fixed_decimals(1)
                .suffix(" Hz"),
        )
        .changed()
    {
        *value = (hz * 10.0).round() as u16;
    }
}

struct PlotControls {
    autoscroll: bool,
    autofit_y: bool,
    seconds: f64,
}

fn time_plot(
    ui: &mut egui::Ui,
    id: impl std::hash::Hash,
    history: &VecDeque<Point>,
    unit: &str,
    controls: PlotControls,
    color: egui::Color32,
) {
    let PlotControls {
        autoscroll,
        autofit_y,
        seconds,
    } = controls;
    Plot::new(id)
        .height(200.0)
        .x_axis_label("Device time (s)")
        .y_axis_label(unit)
        .show(ui, |plot| {
            plot.set_auto_bounds(egui::Vec2b::new(false, autofit_y));
            let key = plot.response().id.with("span");
            let previous = plot.ctx().data_mut(|data| {
                let previous = data.get_temp::<f64>(key);
                if !history.is_empty() {
                    data.insert_temp(key, seconds);
                }
                previous
            });
            if (autoscroll || previous.is_none()) && !history.is_empty() {
                let right = history.back().unwrap().time.max(seconds);
                plot.set_plot_bounds_x(right - seconds..=right);
            } else if previous.is_some_and(|old| old != seconds) {
                let bounds = plot.plot_bounds();
                let center = (bounds.min()[0] + bounds.max()[0]) * 0.5;
                plot.set_plot_bounds_x(center - seconds / 2.0..=center + seconds / 2.0);
            }
            let mut segment = Vec::new();
            let draw = |plot: &mut egui_plot::PlotUi<'_>, segment: &mut Vec<[f64; 2]>| {
                if segment.len() == 1 {
                    plot.points(
                        Points::new(unit, PlotPoints::from(std::mem::take(segment)))
                            .radius(1.5)
                            .color(color),
                    );
                } else if !segment.is_empty() {
                    plot.line(
                        Line::new(unit, PlotPoints::from(std::mem::take(segment))).color(color),
                    );
                }
            };
            for point in history {
                if point.discontinuity {
                    draw(plot, &mut segment);
                }
                segment.push([point.time, point.value]);
            }
            draw(plot, &mut segment);
        });
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.events();
        egui::TopBottomPanel::top("connection").show(ctx, |ui| self.connection_panel(ui));
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.add_enabled_ui(self.connected && !self.config_busy, |ui| {
                self.controls(ui);
                self.fourier_ignore_controls(ui);
                self.acquisition_panel(ui);
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.columns(2, |columns| {
                        self.board_panel(&mut columns[0], 0);
                        self.board_panel(&mut columns[1], 1);
                    });
                });
            });
        });
        ctx.request_repaint_after(Duration::from_millis(33));
    }

    fn on_exit(&mut self) {
        let _ = self.connection.urgent.try_send(Work::Quit);
    }
}

fn main() -> eframe::Result {
    let mut options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1360.0, 900.0])
            .with_min_inner_size([1100.0, 600.0]),
        ..Default::default()
    };
    #[cfg(target_os = "windows")]
    if let eframe::egui_wgpu::WgpuSetup::CreateNew(setup) = &mut options.wgpu_options.wgpu_setup {
        setup.instance_descriptor.backends = eframe::wgpu::Backends::DX12;
    }
    eframe::run_native(
        "BrachiGoon EC5",
        options,
        Box::new(|creation| {
            creation.egui_ctx.set_visuals(egui::Visuals::dark());
            Ok(Box::new(App::new()))
        }),
    )
}
