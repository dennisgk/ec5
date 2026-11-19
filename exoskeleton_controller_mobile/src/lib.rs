mod ble;
mod interaction;
mod model;
mod protocol;
use eframe::egui;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, PartialEq)]
enum Page {
    Connect,
    Config,
    Angle,
    Auto,
    Signals,
}
struct App {
    ble: ble::Ble,
    page: Page,
    devices: Vec<ble::Device>,
    selected: String,
    connected: Option<String>,
    busy: bool,
    error: String,
    state: protocol::State,
    ratio: f32,
    direction: i8,
    minimum: f32,
    maximum: f32,
    applied_limits: [f32; 2],
    dynamics: [f32; 4],
    model: Option<model::Model>,
    target: f32,
    drag_start: Option<(interaction::SurfacePoint, f32)>,
    drag_marker: Option<interaction::SurfacePoint>,
    last_target: Instant,
}
impl App {
    fn new(context: &eframe::CreationContext<'_>) -> Self {
        context.egui_ctx.set_visuals(egui::Visuals::dark());
        context.egui_ctx.style_mut(|style| {
            for (text, size) in [
                (egui::TextStyle::Body, 18.),
                (egui::TextStyle::Button, 18.),
                (egui::TextStyle::Heading, 24.),
                (egui::TextStyle::Small, 16.),
                (egui::TextStyle::Monospace, 18.),
            ] {
                style.text_styles.insert(
                    text,
                    egui::FontId::new(size, egui::FontFamily::Proportional),
                );
            }
            style.spacing.interact_size = egui::vec2(48., 48.);
            style.spacing.button_padding = egui::vec2(14., 10.);
            style.spacing.item_spacing = egui::vec2(12., 10.);
            style.spacing.scroll.bar_width = 14.;
            style.spacing.scroll.handle_min_length = 48.;
        });
        let result = context
            .wgpu_render_state
            .as_ref()
            .ok_or("wgpu renderer unavailable".to_owned())
            .and_then(model::Model::new);
        let (model, error) = match result {
            Ok(model) => (Some(model), String::new()),
            Err(e) => (None, e),
        };
        Self {
            ble: ble::Ble::new(),
            page: Page::Connect,
            devices: Vec::new(),
            selected: String::new(),
            connected: None,
            busy: false,
            error,
            state: Default::default(),
            ratio: 1.,
            direction: 1,
            minimum: 0.,
            maximum: 120.,
            applied_limits: [0., 120.],
            dynamics: [40., 720., 7200., 0.5],
            model,
            target: 0.,
            drag_start: None,
            drag_marker: None,
            last_target: Instant::now(),
        }
    }
    fn send(&mut self, work: ble::Work) {
        match self.ble.send(work) {
            Ok(()) => {
                self.busy = true;
                self.error.clear();
            }
            Err(e) => self.error = e,
        }
    }
    fn events(&mut self) {
        for event in self.ble.events.try_iter().collect::<Vec<_>>() {
            match event {
                ble::Event::Devices(devices) => {
                    self.devices = devices;
                }
                ble::Event::Connected(identity) => {
                    self.connected = Some(identity);
                    self.error.clear();
                }
                ble::Event::Settings(settings) => {
                    self.ratio = settings[0];
                    self.direction = settings[1] as i8;
                    self.minimum = settings[2];
                    self.maximum = settings[3];
                    self.applied_limits = [settings[2], settings[3]];
                    self.dynamics.copy_from_slice(&settings[4..8]);
                }
                ble::Event::Disconnected => {
                    self.connected = None;
                    self.page = Page::Connect;
                    self.busy = false;
                    self.drag_start = None;
                }
                ble::Event::State(state) => {
                    self.state = state;
                    if self.drag_start.is_none() {
                        self.target = state.target;
                    }
                }
                ble::Event::Error(e) => {
                    self.error = e;
                }
                ble::Event::Done => self.busy = false,
            }
        }
    }
    fn connect_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(
                    !self.busy && self.connected.is_none(),
                    egui::Button::new("Scan EC5 devices"),
                )
                .clicked()
            {
                self.send(ble::Work::Scan);
            }
            if self.busy {
                ui.spinner();
            }
            if let Some(identity) = &self.connected {
                ui.label(format!("Connected · {identity}"));
            }
        });
        if self.connected.is_none() {
            for device in &self.devices {
                ui.radio_value(&mut self.selected, device.key.clone(), &device.name);
            }
            if ui
                .add_enabled(
                    !self.busy && !self.selected.is_empty(),
                    egui::Button::new("Connect"),
                )
                .clicked()
            {
                self.send(ble::Work::Connect(self.selected.clone()));
            }
        } else if ui.button("Disconnect").clicked() {
            self.send(ble::Work::Disconnect);
        }
    }
    fn angle_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(format!(
                "Encoder {:.1}° · target {:.1}°",
                self.state.measured, self.target
            ));
            ui.label(if self.state.holding {
                "Holding"
            } else {
                "Stopped"
            });
            if self.state.fault {
                ui.colored_label(egui::Color32::LIGHT_RED, "Motor fault");
            }
            if ui.button("Stop").clicked() {
                self.send(ble::Work::Command(7, vec![]));
            }
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(
                    !self.busy && !self.state.holding,
                    egui::Button::new("Calibrate 0°"),
                )
                .clicked()
            {
                self.send(ble::Work::Command(1, vec![]));
            }
            if ui
                .add_enabled(
                    !self.busy
                        && self.state.calibrated
                        && !self.state.holding
                        && self.state.bluetooth
                        && !self.state.fault,
                    egui::Button::new("Enable hold"),
                )
                .clicked()
            {
                self.send(ble::Work::Command(6, vec![1]));
            }
        });
        egui::CollapsingHeader::new("Angle settings").show(ui, |ui| {
            ui.add_enabled_ui(!self.state.holding && !self.busy, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Response");
                    let name = if self.dynamics == [40., 720., 7200., 0.5] {
                        "Fast"
                    } else if self.dynamics == [2., 10., 30., 1.] {
                        "Gentle"
                    } else {
                        "Custom"
                    };
                    egui::ComboBox::from_id_salt("response_preset")
                        .selected_text(name)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.dynamics,
                                [40., 720., 7200., 0.5],
                                "Fast",
                            );
                            ui.selectable_value(&mut self.dynamics, [2., 10., 30., 1.], "Gentle");
                        });
                });
                egui::Grid::new("angle_settings")
                    .num_columns(2)
                    .show(ui, |ui| {
                        ui.label("Motor turns / elbow turn");
                        ui.add(
                            egui::DragValue::new(&mut self.ratio)
                                .range(0.01..=1000.)
                                .speed(0.01),
                        );
                        ui.end_row();
                        ui.label("Direction");
                        ui.horizontal(|ui| {
                            ui.radio_value(&mut self.direction, 1, "+");
                            ui.radio_value(&mut self.direction, -1, "−");
                        });
                        ui.end_row();
                        ui.label("Minimum angle");
                        ui.add(
                            egui::DragValue::new(&mut self.minimum)
                                .range(-180.0..=180.0)
                                .suffix("°"),
                        );
                        ui.end_row();
                        ui.label("Maximum angle");
                        ui.add(
                            egui::DragValue::new(&mut self.maximum)
                                .range(-180.0..=180.0)
                                .suffix("°"),
                        );
                        ui.end_row();
                        for (i, label) in [
                            "Position gain (1/s)",
                            "Speed limit (°/s)",
                            "Acceleration (°/s²)",
                            "Deadband (°)",
                        ]
                        .iter()
                        .enumerate()
                        {
                            ui.label(*label);
                            ui.add(egui::DragValue::new(&mut self.dynamics[i]).speed(0.1));
                            ui.end_row();
                        }
                    });
                if ui.button("Apply angle settings").clicked() {
                    if self.minimum >= self.maximum {
                        self.error = "Minimum must be below maximum".into();
                    } else {
                        self.send(ble::Work::Configure {
                            ratio: self.ratio,
                            direction: self.direction,
                            minimum: self.minimum,
                            maximum: self.maximum,
                            dynamics: self.dynamics,
                        });
                    }
                }
            });
        });
        if !self.state.calibrated {
            ui.label("Calibrate 0°, then enable hold to drag.");
        } else if !self.state.holding {
            ui.label("Enable hold to drag the forearm or rotation handle.");
        }
        let size = ui.available_size().min_elem().clamp(100., 1024.);
        if let Some(model) = &self.model {
            // Never animate the requested target as if the motor had reached it.
            model.draw(self.state.measured);
            let response = ui.add(
                egui::Image::new((model.texture, egui::vec2(size, size)))
                    .sense(egui::Sense::drag()),
            );
            let geometry = &model.geometry;
            let enabled =
                self.state.holding && self.state.bluetooth && !self.state.fault && !self.busy;
            let painter = ui.painter().with_clip_rect(response.rect);
            draw_angle_dial(
                &painter,
                geometry,
                response.rect,
                self.state.measured,
                self.target,
                self.applied_limits,
                enabled,
            );
            let pointer = response.hover_pos();
            let hovered =
                pointer.and_then(|p| geometry.pick(p, response.rect, self.state.measured));
            let on_handle = |p: egui::Pos2| {
                p.distance(geometry.handle_position(self.target, response.rect)) <= 24.
                    || p.distance(geometry.handle_position(self.state.measured, response.rect))
                        <= 24.
            };
            let interactive =
                pointer.is_some_and(on_handle) || hovered.is_some_and(|point| point.moving);
            if response.hovered() {
                ui.ctx().set_cursor_icon(if !enabled {
                    egui::CursorIcon::NotAllowed
                } else if self.drag_start.is_some() {
                    egui::CursorIcon::Grabbing
                } else if interactive {
                    egui::CursorIcon::Grab
                } else {
                    egui::CursorIcon::Crosshair
                });
            }
            if !enabled {
                self.drag_start = None;
            } else if response.drag_started_by(egui::PointerButton::Primary)
                && let Some(origin) = ui.input(|input| input.pointer.press_origin())
            {
                self.drag_marker = geometry
                    .pick(origin, response.rect, self.state.measured)
                    .filter(|point| point.moving);
                // Point the forearm toward the cursor's intersection with its
                // rotation plane, including drags that start in empty space.
                self.drag_start = Some((geometry.handle(), self.state.measured));
            }
            if let Some((point, reference)) = self.drag_start {
                if let Some(position) = response.interact_pointer_pos()
                    && let Some(target) =
                        geometry.target_at_pointer(point, position, response.rect, reference)
                {
                    self.target = target.clamp(self.applied_limits[0], self.applied_limits[1]);
                    if self.last_target.elapsed() >= Duration::from_millis(50) {
                        self.ble.target(self.target);
                        self.last_target = Instant::now();
                    }
                }
                let marker = geometry.project(
                    geometry.world_point(self.drag_marker.unwrap_or(point), self.state.measured),
                    response.rect,
                );
                painter.circle_filled(marker, 7., egui::Color32::LIGHT_GREEN);
                painter.circle_stroke(marker, 9., egui::Stroke::new(2_f32, egui::Color32::WHITE));
            } else if let Some(point) = hovered {
                let marker = geometry.project(
                    geometry.world_point(point, self.state.measured),
                    response.rect,
                );
                let color = if point.moving {
                    egui::Color32::LIGHT_GREEN
                } else {
                    egui::Color32::LIGHT_GRAY
                };
                painter.circle_filled(marker, 5., color);
                painter.circle_stroke(marker, 7., egui::Stroke::new(1.5_f32, egui::Color32::WHITE));
            }
            if response.drag_stopped_by(egui::PointerButton::Primary) {
                if self.drag_start.is_some() && enabled {
                    self.ble.target(self.target);
                }
                self.drag_start = None;
            }
        }
    }
}
fn draw_angle_dial(
    painter: &egui::Painter,
    geometry: &interaction::Geometry,
    rect: egui::Rect,
    measured: f32,
    target: f32,
    limits: [f32; 2],
    enabled: bool,
) {
    let center = geometry.dial_center(rect);
    let points: Vec<_> = (0..=96)
        .map(|i| {
            geometry.handle_position(limits[0] + (limits[1] - limits[0]) * i as f32 / 96., rect)
        })
        .collect();
    painter.add(egui::Shape::line(
        points,
        egui::Stroke::new(3_f32, egui::Color32::GRAY),
    ));
    let actual = geometry.handle_position(measured, rect);
    let requested = geometry.handle_position(target, rect);
    let cyan = egui::Color32::from_rgb(85, 205, 255);
    let yellow = if enabled {
        egui::Color32::from_rgb(255, 205, 75)
    } else {
        egui::Color32::GRAY
    };
    painter.line_segment([center, actual], egui::Stroke::new(3_f32, cyan));
    painter.circle_filled(actual, 7., cyan);
    painter.line_segment([center, requested], egui::Stroke::new(1.5, yellow));
    painter.circle_stroke(requested, 12., egui::Stroke::new(3_f32, yellow));
    painter.circle_filled(center, 5., cyan);
    painter.text(
        center + egui::vec2(14., -22.),
        egui::Align2::LEFT_BOTTOM,
        format!("{measured:.1}°"),
        egui::FontId::proportional(20.),
        cyan,
    );
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.events();
        egui::TopBottomPanel::top("pages").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (page, title) in [
                    (Page::Connect, "Connect"),
                    (Page::Config, "Config"),
                    (Page::Angle, "Angle Control"),
                    (Page::Auto, "Auto"),
                    (Page::Signals, "Signals"),
                ] {
                    if ui
                        .add_enabled(
                            page == Page::Connect || self.connected.is_some(),
                            egui::Button::new(title).selected(self.page == page),
                        )
                        .clicked()
                    {
                        self.page = page;
                    }
                }
            });
        });
        egui::CentralPanel::default().show(ctx, |ui| {
            if !self.error.is_empty() {
                ui.horizontal_wrapped(|ui| {
                    ui.colored_label(egui::Color32::LIGHT_RED, &self.error);
                    if ui.small_button("Dismiss").clicked() {
                        self.error.clear();
                    }
                });
            }
            egui::ScrollArea::vertical().show(ui, |ui| match self.page {
                Page::Connect => self.connect_ui(ui),
                Page::Angle => self.angle_ui(ui),
                _ => {}
            });
        });
        ctx.request_repaint_after(Duration::from_millis(33));
    }
}
pub fn run() -> eframe::Result {
    let mut options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default().with_inner_size([720., 1000.]),
        ..Default::default()
    };
    if let eframe::egui_wgpu::WgpuSetup::CreateNew(setup) = &mut options.wgpu_options.wgpu_setup {
        #[cfg(target_os = "windows")]
        {
            setup.instance_descriptor.backends = eframe::wgpu::Backends::DX12;
        }
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        {
            setup.instance_descriptor.backends = eframe::wgpu::Backends::METAL;
        }
    }
    eframe::run_native(
        "BrachiFlex",
        options,
        Box::new(|context| Ok(Box::new(App::new(context)))),
    )
}
// Called by the tiny iOS executable on the main thread. winit owns UIApplication.
#[unsafe(no_mangle)]
pub extern "C" fn brachiflex_main() -> i32 {
    match std::panic::catch_unwind(run) {
        Ok(Ok(())) => 0,
        Ok(Err(error)) => {
            eprintln!("BrachiFlex: {error}");
            1
        }
        Err(_) => 1,
    }
}
