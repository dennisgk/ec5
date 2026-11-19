use crate::protocol::{self, State};
use btleplug::{
    api::{Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType},
    platform::{Manager, Peripheral},
};
use crossbeam_channel::{Receiver, Sender, bounded};
use futures::StreamExt;
use std::{collections::BTreeMap, time::Duration};
use tokio::sync::{mpsc, watch};

#[derive(Clone, Debug)]
pub struct Device {
    pub key: String,
    pub name: String,
}
pub enum Work {
    Scan,
    Connect(String),
    Disconnect,
    Command(u8, Vec<u8>),
    Configure {
        ratio: f32,
        direction: i8,
        minimum: f32,
        maximum: f32,
        dynamics: [f32; 4],
    },
}
pub enum Event {
    Devices(Vec<Device>),
    Connected(String),
    Settings([f32; 8]),
    Disconnected,
    State(State),
    Error(String),
    Done,
}
pub struct Ble {
    requests: mpsc::Sender<Work>,
    target: watch::Sender<Option<f32>>,
    pub events: Receiver<Event>,
}
impl Ble {
    pub fn new() -> Self {
        let (requests, jobs) = mpsc::channel(16);
        let (target, targets) = watch::channel(None);
        let (events, received) = bounded(64);
        std::thread::spawn(move || {
            match tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
            {
                Ok(runtime) => runtime.block_on(worker(jobs, targets, events)),
                Err(error) => {
                    let _ = events.try_send(Event::Error(error.to_string()));
                }
            }
        });
        Self {
            requests,
            target,
            events: received,
        }
    }
    pub fn send(&self, work: Work) -> Result<(), String> {
        self.requests
            .try_send(work)
            .map_err(|_| "Bluetooth command queue is busy".into())
    }
    pub fn target(&self, degrees: f32) {
        self.target.send_replace(Some(degrees));
    }
}
struct Link {
    peripheral: Peripheral,
    command: Characteristic,
    notifications:
        std::pin::Pin<Box<dyn futures::Stream<Item = btleplug::api::ValueNotification> + Send>>,
    next_id: u16,
}
impl Link {
    async fn send(
        &mut self,
        operation: u8,
        payload: &[u8],
        events: &Sender<Event>,
    ) -> Result<(), String> {
        self.next_id = self
            .next_id
            .checked_add(1)
            .ok_or("BLE command IDs exhausted; reconnect")?;
        let id = self.next_id;
        let bytes = protocol::command(operation, id, payload)?;
        tokio::time::timeout(
            Duration::from_secs(3),
            self.exchange_result(&bytes, operation, id, events),
        )
        .await
        .map_err(|_| {
            "BLE command result timed out; read current state before retrying".to_owned()
        })?
    }

    async fn exchange_result(
        &mut self,
        bytes: &[u8],
        operation: u8,
        id: u16,
        events: &Sender<Event>,
    ) -> Result<(), String> {
        self.peripheral
            .write(&self.command, bytes, WriteType::WithResponse)
            .await
            .map_err(|e| e.to_string())?;
        while let Some(notification) = self.notifications.next().await {
            if notification.uuid == protocol::STATE {
                if let Ok(state) = State::decode(&notification.value) {
                    let _ = events.try_send(Event::State(state));
                }
                continue;
            }
            if notification.uuid != protocol::RESULT {
                continue;
            }
            let result = notification.value;
            if result.len() != 5 || result[0] != 1 {
                return Err("Invalid BLE command result".into());
            }
            if u16::from_le_bytes([result[2], result[3]]) != id {
                continue;
            }
            if result[1] != operation {
                return Err("BLE response operation mismatch".into());
            }
            if result[4] == 0 {
                return Ok(());
            }
            let reason = match result[4] {
                1 => "Invalid angle settings or target",
                6 => {
                    "Not ready: stop hold before applying settings; calibrate with fresh CAN feedback before enabling"
                }
                7 => "Motor/CAN fault",
                12 => "Request superseded by a newer command",
                13 => "Enable Bluetooth control from USB first",
                _ => "Controller rejected command",
            };
            return Err(format!("{reason} (code {})", result[4]));
        }
        Err("Bluetooth connection closed; onboard mode remains active".into())
    }
}
async fn connect(peripheral: Peripheral) -> Result<(Link, String, State, [f32; 8]), String> {
    let result = async {
        peripheral.connect().await.map_err(|e| e.to_string())?;
        peripheral
            .discover_services()
            .await
            .map_err(|e| e.to_string())?;
        if !peripheral
            .services()
            .iter()
            .any(|s| s.uuid == protocol::SERVICE)
        {
            return Err("Selected device is not an EC5 controller".into());
        }
        let characteristics = peripheral.characteristics();
        let get = |id| {
            characteristics
                .iter()
                .find(|c| c.uuid == id && c.service_uuid == protocol::SERVICE)
                .cloned()
                .ok_or_else(|| "Incomplete EC5 BLE service".to_owned())
        };
        let identity = peripheral
            .read(&get(protocol::IDENTITY)?)
            .await
            .map_err(|e| e.to_string())?;
        let name = protocol::identity(&identity)?;
        let command = get(protocol::COMMAND)?;
        let notifications = peripheral
            .notifications()
            .await
            .map_err(|e| e.to_string())?;
        peripheral
            .subscribe(&get(protocol::RESULT)?)
            .await
            .map_err(|e| e.to_string())?;
        peripheral
            .subscribe(&get(protocol::STATE)?)
            .await
            .map_err(|e| e.to_string())?;
        let state = State::decode(
            &peripheral
                .read(&get(protocol::STATE)?)
                .await
                .map_err(|e| e.to_string())?,
        )?;
        let mut settings = [0.; 8];
        for (index, uuid) in [protocol::MECHANICS, protocol::DYNAMICS]
            .into_iter()
            .enumerate()
        {
            let bytes = peripheral
                .read(&get(uuid)?)
                .await
                .map_err(|e| e.to_string())?;
            if bytes.len() != 16 {
                return Err("Invalid angle settings record".into());
            }
            for i in 0..4 {
                let value = f32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap());
                if !value.is_finite() {
                    return Err("Invalid angle settings value".into());
                }
                settings[index * 4 + i] = value;
            }
        }
        protocol::validate_settings(&settings)?;
        Ok((
            Link {
                peripheral: peripheral.clone(),
                command,
                notifications,
                next_id: 0,
            },
            name,
            state,
            settings,
        ))
    }
    .await;
    if result.is_err() {
        let _ = peripheral.disconnect().await;
    }
    result
}
async fn worker(
    mut jobs: mpsc::Receiver<Work>,
    mut targets: watch::Receiver<Option<f32>>,
    events: Sender<Event>,
) {
    let mut devices = BTreeMap::<String, Peripheral>::new();
    let mut link: Option<Link> = None;
    let mut health = tokio::time::interval(Duration::from_secs(1));
    loop {
        // Keep transport work off the egui/main thread, including iOS permission prompts.
        let work = if let Some(active) = &mut link {
            tokio::select! {
                biased;
                job = jobs.recv() => job,
                _ = health.tick() => {
                    if !active.peripheral.is_connected().await.unwrap_or(false) {
                        link = None;
                        let _ = events.try_send(Event::Disconnected);
                    }
                    continue;
                }
                changed = targets.changed() => {
                    if changed.is_err() { return; }
                    let target = *targets.borrow_and_update();
                    if let Some(target) = target
                        && let Err(e) = active.send(5, &protocol::floats(&[target]), &events).await {
                        let _ = events.try_send(Event::Error(e));
                    }
                    continue;
                }
                notification = active.notifications.next() => {
                    match notification {
                        Some(n) if n.uuid == protocol::STATE => match State::decode(&n.value) {
                            Ok(state) => { let _ = events.try_send(Event::State(state)); }
                            Err(e) => { let _ = events.try_send(Event::Error(e)); }
                        },
                        Some(_) => {},
                        None => { link = None; let _ = events.try_send(Event::Disconnected); }
                    }
                    continue;
                }
            }
        } else {
            jobs.recv().await
        };
        let Some(work) = work else {
            return;
        };
        let result: Result<(), String> = match work {
            Work::Scan => {
                async {
                    let manager = Manager::new().await.map_err(|e| e.to_string())?;
                    let adapters = manager.adapters().await.map_err(|e| e.to_string())?;
                    let adapter = adapters
                        .into_iter()
                        .next()
                        .ok_or("No Bluetooth adapter; enable Bluetooth and grant permission")?;
                    adapter
                        .start_scan(ScanFilter {
                            services: vec![protocol::SERVICE],
                        })
                        .await
                        .map_err(|e| e.to_string())?;
                    tokio::time::sleep(Duration::from_secs(4)).await;
                    let result = adapter.peripherals().await;
                    let _ = adapter.stop_scan().await;
                    let peripherals = result.map_err(|e| e.to_string())?;
                    devices.clear();
                    let mut list = Vec::new();
                    for peripheral in peripherals {
                        if let Some(properties) =
                            peripheral.properties().await.map_err(|e| e.to_string())?
                        {
                            if !properties.services.contains(&protocol::SERVICE) {
                                continue;
                            }
                            let key = peripheral.id().to_string();
                            list.push(Device {
                                key: key.clone(),
                                name: properties
                                    .local_name
                                    .unwrap_or_else(|| "EC5 controller".into()),
                            });
                            devices.insert(key, peripheral);
                        }
                    }
                    let _ = events.try_send(Event::Devices(list));
                    Ok(())
                }
                .await
            }
            Work::Connect(key) => {
                if let Some(old) = link.take() {
                    let _ = old.peripheral.disconnect().await;
                }
                if let Some(peripheral) = devices.get(&key) {
                    match tokio::time::timeout(Duration::from_secs(15), connect(peripheral.clone()))
                        .await
                    {
                        Ok(Ok((new, name, state, settings))) => {
                            link = Some(new);
                            targets.borrow_and_update();
                            let _ = events.try_send(Event::Connected(name));
                            let _ = events.try_send(Event::State(state));
                            let _ = events.try_send(Event::Settings(settings));
                            Ok(())
                        }
                        Ok(Err(e)) => Err(e),
                        Err(_) => {
                            let _ = peripheral.disconnect().await;
                            Err("Bluetooth connection timed out".into())
                        }
                    }
                } else {
                    Err("Scan and select an EC5 controller first".into())
                }
            }
            Work::Disconnect => {
                if let Some(old) = link.take() {
                    let _ = old.peripheral.disconnect().await;
                }
                let _ = events.try_send(Event::Disconnected);
                Ok(())
            }
            Work::Command(op, data) => match &mut link {
                Some(active) => active.send(op, &data, &events).await,
                None => Err("Not connected".into()),
            },
            Work::Configure {
                ratio,
                direction,
                minimum,
                maximum,
                dynamics,
            } => {
                if let Some(active) = &mut link {
                    let settings = [
                        ratio,
                        f32::from(direction),
                        minimum,
                        maximum,
                        dynamics[0],
                        dynamics[1],
                        dynamics[2],
                        dynamics[3],
                    ];
                    if let Err(error) = protocol::validate_settings(&settings) {
                        let _ = events.try_send(Event::Error(error));
                        let _ = events.try_send(Event::Done);
                        continue;
                    }
                    let mut mechanics = protocol::floats(&[ratio]);
                    mechanics.push(direction as u8);
                    async {
                        active.send(3, &mechanics, &events).await?;
                        active
                            .send(2, &protocol::floats(&[minimum, maximum]), &events)
                            .await?;
                        active
                            .send(4, &protocol::floats(&dynamics), &events)
                            .await?;
                        let _ = events.try_send(Event::Settings([
                            ratio,
                            f32::from(direction),
                            minimum,
                            maximum,
                            dynamics[0],
                            dynamics[1],
                            dynamics[2],
                            dynamics[3],
                        ]));
                        Ok(())
                    }
                    .await
                } else {
                    Err("Not connected".into())
                }
            }
        };
        if let Err(error) = result {
            let _ = events.try_send(Event::Error(error));
        }
        let _ = events.try_send(Event::Done);
    }
}
