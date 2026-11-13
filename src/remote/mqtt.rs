//! WiFi and MQTT plumbing (ESP only), with the thread layout proven in the esp32-mqtt-relay
//! project:
//!
//! - WiFi watchdog: esp-idf-svc doesn't reconnect the station by itself, so it checks every
//!   `WIFI_CHECK_SECS` and reconnects after a dropout.
//! - Connection drainer: pumps `connection.next()` continuously, or the client's internal
//!   state machine never progresses (esp-rs/esp-idf-svc#441). It hands messages to
//!   `on_message` and must never call `publish()`/`subscribe()` itself (that deadlocks), so
//!   `on_message` must only forward work over channels.
//! - Subscriber: while not subscribed (boot, or after any disconnect, since the client uses a
//!   clean session), retries the subscriptions every `SUBSCRIBE_RETRY_MS`. It polls a flag
//!   rather than waiting for a Connected event, which has been seen to hang.
//! - Publisher: the only thread that calls `publish()`; it sends each `Outgoing` from
//!   `out_rx` to `<prefix>/status` or `<prefix>/pose`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use anyhow::anyhow;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::mqtt::client::{EspMqttClient, EventPayload, MqttClientConfiguration, QoS};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};

use super::command::{POSE_TOPIC, STATUS_TOPIC, SUBSCRIBE_TOPICS};

const SUBSCRIBE_RETRY_MS: u64 = 500;
const WIFI_CHECK_SECS: u64 = 5;
/// The default pthread stack (3 KiB) is too small for Debug-formatted error logs and
/// command parsing.
const THREAD_STACK: usize = 8 * 1024;

pub struct NetConfig {
    pub wifi_ssid: &'static str,
    pub wifi_pass: &'static str,
    pub mqtt_url: &'static str,
    pub mqtt_client_id: &'static str,
    pub topic_prefix: &'static str,
}

type Client = Arc<Mutex<EspMqttClient<'static>>>;

/// A message for the publisher thread.
pub enum Outgoing {
    /// Status JSON (`remote::status`), sent at least once.
    Status(String),
    /// Pose JSON (`remote::pose`), sent at most once: a lost frame is soon replaced.
    Pose(String),
}

/// Brings WiFi up and starts the MQTT threads, then returns. A failed WiFi connect at boot is
/// only logged; the watchdog keeps retrying.
///
/// `on_message(topic, payload)` runs on the drainer thread for every received message.
/// `on_connection(up)` runs with `true` once the subscriptions are in place and `false` when
/// the broker connection drops.
pub fn start(
    modem: Modem<'static>,
    cfg: NetConfig,
    out_rx: Receiver<Outgoing>,
    mut on_message: impl FnMut(&str, &[u8]) + Send + 'static,
    on_connection: impl Fn(bool) + Send + Sync + 'static,
) -> anyhow::Result<()> {
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sys_loop.clone(), Some(nvs))?, sys_loop)?;
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: cfg
            .wifi_ssid
            .try_into()
            .map_err(|_| anyhow!("wifi_ssid is too long"))?,
        password: cfg
            .wifi_pass
            .try_into()
            .map_err(|_| anyhow!("wifi_pass is too long"))?,
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    }))?;
    wifi.start()?;
    match connect_wifi(&mut wifi) {
        Ok(()) => log::info!("WiFi connected"),
        Err(e) => log::warn!("WiFi connect failed at boot: {e:?}, watchdog will retry"),
    }
    spawn("wifi_watchdog", move || wifi_watchdog(wifi))?;

    let mqtt_config = MqttClientConfiguration {
        client_id: Some(cfg.mqtt_client_id),
        ..Default::default()
    };
    let (client, mut connection) = EspMqttClient::new(cfg.mqtt_url, &mqtt_config)?;
    let client: Client = Arc::new(Mutex::new(client));
    let subscribed = Arc::new(AtomicBool::new(false));
    let on_connection = Arc::new(on_connection);

    {
        let subscribed = subscribed.clone();
        let on_connection = on_connection.clone();
        spawn("mqtt_drain", move || {
            while let Ok(event) = connection.next() {
                match event.payload() {
                    EventPayload::Received {
                        topic: Some(topic),
                        data,
                        ..
                    } => on_message(topic, data),
                    EventPayload::Disconnected => {
                        log::warn!("MQTT disconnected");
                        if subscribed.swap(false, Ordering::Relaxed) {
                            on_connection(false);
                        }
                    }
                    _ => {}
                }
            }
            log::warn!("MQTT connection closed");
        })?;
    }

    {
        let client = client.clone();
        let status_topic = format!("{}/{STATUS_TOPIC}", cfg.topic_prefix);
        let pose_topic = format!("{}/{POSE_TOPIC}", cfg.topic_prefix);
        spawn("mqtt_publish", move || {
            while let Ok(msg) = out_rx.recv() {
                let (topic, qos, payload) = match &msg {
                    Outgoing::Status(p) => (&status_topic, QoS::AtLeastOnce, p),
                    Outgoing::Pose(p) => (&pose_topic, QoS::AtMostOnce, p),
                };
                let sent = client.lock().unwrap().publish(topic, qos, false, payload.as_bytes());
                if let Err(e) = sent {
                    log::warn!("Failed to publish to {topic}: {e:?}");
                }
            }
        })?;
    }

    {
        let filters: Vec<String> = SUBSCRIBE_TOPICS
            .iter()
            .map(|t| format!("{}/{t}", cfg.topic_prefix))
            .collect();
        spawn("mqtt_subscribe", move || {
            let mut warned = false;
            loop {
                if !subscribed.load(Ordering::Relaxed) {
                    match subscribe_all(&client, &filters) {
                        Ok(()) => {
                            log::info!("Subscribed to {}", filters.join(", "));
                            subscribed.store(true, Ordering::Relaxed);
                            warned = false;
                            on_connection(true);
                        }
                        // Expected to fail with ESP_FAIL until the connection is up.
                        Err(e) if !warned => {
                            log::warn!("Failed to subscribe: {e:?}, retrying...");
                            warned = true;
                        }
                        Err(_) => {}
                    }
                }
                thread::sleep(Duration::from_millis(SUBSCRIBE_RETRY_MS));
            }
        })?;
    }

    Ok(())
}

fn subscribe_all(client: &Client, filters: &[String]) -> anyhow::Result<()> {
    let mut client = client.lock().unwrap();
    for filter in filters {
        client.subscribe(filter, QoS::AtLeastOnce)?;
    }
    Ok(())
}

fn spawn(name: &str, f: impl FnOnce() + Send + 'static) -> anyhow::Result<()> {
    thread::Builder::new()
        .name(name.into())
        .stack_size(THREAD_STACK)
        .spawn(f)?;
    Ok(())
}

fn wifi_watchdog(mut wifi: BlockingWifi<EspWifi<'static>>) {
    let mut down = false;
    loop {
        if !wifi.is_up().unwrap_or(false) {
            if !down {
                log::warn!("WiFi down, reconnecting...");
                down = true;
            }
            match connect_wifi(&mut wifi) {
                Ok(()) => {
                    log::info!("WiFi reconnected");
                    down = false;
                }
                Err(e) => log::warn!("WiFi reconnect failed: {e:?}"),
            }
        }
        thread::sleep(Duration::from_secs(WIFI_CHECK_SECS));
    }
}

/// Associates with the AP if needed, then waits for an IP. Each step times out after ~15 s
/// inside esp-idf-svc.
fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    if !wifi.is_connected()? {
        wifi.connect()?;
    }
    wifi.wait_netif_up()?;
    Ok(())
}
