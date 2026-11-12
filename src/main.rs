//! Robot controller: ESP32-S3 + PCA9685 + 16 servos, driven over MQTT, with the board's RGB
//! LED as a status light.
//!
//! On boot it waits a few seconds, snaps every joint to neutral, connects to WiFi and the
//! broker, and then runs commands as they arrive (see `remote::command` for the topics).
//! If the PCA9685 doesn't answer at boot it still connects, with the LED red and the state
//! "pca9685_not_found": LED commands work and robot commands are rejected.
//!
//! Threads: the main thread owns the robot and runs one robot command at a time. The MQTT
//! drainer thread parses messages and, without blocking, either rejects them, applies them
//! (LED, stop) or hands them to the main thread. The LED thread owns the WS2812.

mod remote;
mod rgb_led;
mod robot_16_dof;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::peripherals::Peripherals;

use remote::command::{self, Command, RobotCommand};
use remote::mqtt::{self, NetConfig};
use remote::status::{self, Snapshot};
use rgb_led::indicator::{Activity, Indicator, LedMsg};
use rgb_led::ws2812::Ws2812;
use robot_16_dof::esp32::{self as robot_esp32, FaultSignal, Servos};
use robot_16_dof::motion::{Pose, Robot};
use robot_16_dof::moves::{self, DEFAULT_GAIT};
use robot_16_dof::servo::Joint;

/// Read from cfg.toml at build time (see cfg.toml.example).
#[toml_cfg::toml_config]
pub struct Config {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_pass: &'static str,
    #[default("mqtt://broker.example.com:1883")]
    mqtt_url: &'static str,
    #[default("robot-controller")]
    mqtt_client_id: &'static str,
    #[default("robot")]
    topic_prefix: &'static str,
}

/// The joints the `sway` command swings together: hips and ankles, as in the bring-up sway
/// demo.
const SWAY_JOINTS: [Joint; 4] = [
    Joint::RAnkleRoll, // S1
    Joint::RHipRoll,   // S4
    Joint::LHipRoll,   // S29
    Joint::LAnkleRoll, // S32
];

/// Where `sway` goes first. Negative, so the robot leans left first.
const SWAY_FIRST_DEG: f32 = -moves::JOINT_TEST_DEG;

/// Time to get your hands off the robot and the power connected before anything moves.
const START_DELAY_SECS: u32 = 5;

const HEARTBEAT_SECS: u64 = 30;

/// Rainbow fade speed: 360 steps x 10 ms = 3.6 s per turn of the colour wheel.
const RAINBOW_STEP: Duration = Duration::from_millis(10);
const LED_THREAD_STACK: usize = 4 * 1024;

type RobotHw = Robot<Servos<LedLink>>;

/// Sends LED messages to the LED thread.
struct LedLink(Sender<LedMsg>);

impl LedLink {
    fn send(&self, msg: LedMsg) {
        let _ = self.0.send(msg);
    }
}

/// The status light turns red on the first failed servo write and stays red.
impl FaultSignal for LedLink {
    fn fault(&mut self) {
        self.send(LedMsg::Fault);
    }
}

/// State shared between the main thread and the MQTT threads.
#[derive(Clone)]
struct Shared {
    /// True from the moment a robot command is accepted until it has finished, so a second
    /// one is rejected rather than queued.
    busy: Arc<AtomicBool>,
    /// Set by `stop`; the robot checks it every step (see `Servos::should_abort`).
    abort: Arc<AtomicBool>,
    snapshot: Arc<Mutex<Snapshot>>,
    /// False when the PCA9685 didn't answer at boot.
    servos_ok: bool,
    status_tx: Sender<String>,
    led_tx: Sender<LedMsg>,
}

impl Shared {
    fn snapshot(&self) -> Snapshot {
        *self.snapshot.lock().unwrap()
    }

    fn publish(&self, snapshot: Snapshot, reason: &str, error: Option<&str>) {
        let _ = self.status_tx.send(status::json(snapshot, reason, error));
    }

    fn publish_now(&self, reason: &str, error: Option<&str>) {
        self.publish(self.snapshot(), reason, error);
    }

    fn led(&self, msg: LedMsg) {
        let _ = self.led_tx.send(msg);
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let config = CONFIG;
    if config.wifi_ssid.is_empty() {
        anyhow::bail!("wifi_ssid is empty: fill in cfg.toml (see cfg.toml.example) and rebuild");
    }

    let peripherals = Peripherals::take()?;
    let (led_tx, led_rx) = mpsc::channel();
    spawn_led_thread(peripherals.pins.gpio48, led_rx)?;

    // Without the PCA9685 the firmware keeps going, so MQTT and the LED can still be used.
    let pca = robot_esp32::connect_pca9685(
        peripherals.i2c0,
        peripherals.pins.gpio8, // SDA
        peripherals.pins.gpio9, // SCL
    )
    .inspect_err(|e| log::error!("{e:#}; continuing without servos (robot commands rejected)"))
    .ok();

    let (status_tx, status_rx) = mpsc::channel();
    let shared = Shared {
        busy: Arc::new(AtomicBool::new(false)),
        abort: Arc::new(AtomicBool::new(false)),
        snapshot: Arc::new(Mutex::new(if pca.is_some() {
            Snapshot::IDLE
        } else {
            Snapshot::NO_SERVOS
        })),
        servos_ok: pca.is_some(),
        status_tx,
        led_tx: led_tx.clone(),
    };

    let robot = pca.map(|pca| {
        log::info!("PCA9685 ready");
        for s in (1..=START_DELAY_SECS).rev() {
            log::info!("Starting in {s}s - hands off the robot");
            FreeRtos::delay_ms(1000);
        }
        let mut robot = Robot::new(Servos::new(pca, LedLink(led_tx), shared.abort.clone()));
        // The servos' real positions are unknown until first commanded, so this first write
        // snaps every joint to neutral at once.
        robot.snap_to(&Pose::neutral());
        FreeRtos::delay_ms(1000);
        robot
    });
    shared.led(LedMsg::Activity(if robot.is_some() {
        Activity::Idle
    } else {
        Activity::NoServos
    }));

    let (robot_tx, robot_rx) = mpsc::channel();
    let prefix = config.topic_prefix;
    let on_message = {
        let shared = shared.clone();
        move |topic: &str, payload: &[u8]| on_message(&shared, &robot_tx, prefix, topic, payload)
    };
    let on_connection = {
        let shared = shared.clone();
        move |up: bool| {
            shared.led(LedMsg::Connected(up));
            if up {
                shared.publish_now("connected", None);
            }
        }
    };
    mqtt::start(
        peripherals.modem,
        NetConfig {
            wifi_ssid: config.wifi_ssid,
            wifi_pass: config.wifi_pass,
            mqtt_url: config.mqtt_url,
            mqtt_client_id: config.mqtt_client_id,
            topic_prefix: prefix,
        },
        status_rx,
        on_message,
        on_connection,
    )?;

    {
        let shared = shared.clone();
        thread::Builder::new().name("heartbeat".into()).spawn(move || loop {
            thread::sleep(Duration::from_secs(HEARTBEAT_SECS));
            shared.publish_now("heartbeat", None);
        })?;
    }

    match robot {
        Some(mut robot) => run_commands(&mut robot, &shared, robot_rx),
        // Returning from main would end the task and release the LED pin, so idle here.
        None => loop {
            FreeRtos::delay_ms(60_000);
        },
    }
    Ok(())
}

/// Handles one MQTT message on the drainer thread. Never blocks: robot commands go to the
/// main thread, everything else is applied here.
fn on_message(
    shared: &Shared,
    robot_tx: &Sender<RobotCommand>,
    prefix: &str,
    topic: &str,
    payload: &[u8],
) {
    match command::parse(prefix, topic, payload) {
        None => log::warn!("Ignoring message on unexpected topic {topic}"),
        Some(Err(e)) => {
            log::warn!("Rejected message on {topic}: {e}");
            shared.publish_now("rejected_invalid", Some(&e));
        }
        Some(Ok(Command::Led(mode))) => {
            log::info!("LED -> {mode:?}");
            shared.led(LedMsg::Mode(mode));
            shared.publish_now("led", None);
        }
        Some(Ok(Command::Stop)) => {
            if shared.busy.load(Ordering::SeqCst) {
                log::info!("Stopping");
                shared.abort.store(true, Ordering::SeqCst);
                shared.publish_now("stopping", None);
            } else {
                shared.publish_now("nothing_to_stop", None);
            }
        }
        Some(Ok(Command::Robot(cmd))) => {
            if !shared.servos_ok {
                log::warn!("Rejected {}: no PCA9685", cmd.name());
                shared.publish_now(
                    "rejected_no_servos",
                    Some("PCA9685 not found at boot; check its wiring and power, then reset"),
                );
            } else if shared.busy.swap(true, Ordering::SeqCst) {
                let running = shared.snapshot().action.unwrap_or("a command");
                log::warn!("Rejected {}: busy with {running}", cmd.name());
                shared.publish_now("rejected_busy", Some(&format!("busy with {running}")));
            } else {
                let _ = robot_tx.send(cmd);
            }
        }
    }
}

/// The main loop: runs robot commands one at a time as they arrive.
fn run_commands(robot: &mut RobotHw, shared: &Shared, robot_rx: Receiver<RobotCommand>) {
    while let Ok(cmd) = robot_rx.recv() {
        let action = Some(cmd.name());
        let running = Snapshot { state: "busy", action };
        *shared.snapshot.lock().unwrap() = running;
        shared.led(LedMsg::Activity(Activity::Busy));
        shared.publish(running, "started", None);
        log::info!("Running {cmd:?}");

        run(robot, &cmd);

        let stopped = robot.aborted();
        robot.clear_abort();
        shared.abort.store(false, Ordering::SeqCst);
        let (state, activity) = match cmd {
            RobotCommand::StandBy if !stopped => ("standby", Activity::Standby),
            _ => ("idle", Activity::Idle),
        };
        *shared.snapshot.lock().unwrap() = Snapshot { state, action: None };
        shared.led(LedMsg::Activity(activity));
        shared.busy.store(false, Ordering::SeqCst);
        let reason = if stopped { "stopped" } else { "done" };
        log::info!("{} {reason}", cmd.name());
        shared.publish(Snapshot { state, action }, reason, None);
    }
}

fn run(robot: &mut RobotHw, cmd: &RobotCommand) {
    match cmd {
        RobotCommand::Move { targets, ms } => moves::move_joints(robot, targets, *ms),
        RobotCommand::Sway { times } => moves::sway(robot, &SWAY_JOINTS, SWAY_FIRST_DEG, *times),
        RobotCommand::SwayGrip { times } => moves::sway_and_grip(robot, *times),
        RobotCommand::SwayArms { times } => moves::sway_arms_and_grip(robot, *times),
        RobotCommand::Arms { arm, times } => moves::arm_up_down(robot, *arm, *times),
        RobotCommand::Walk { steps } => moves::walk(robot, *steps, &DEFAULT_GAIT),
        RobotCommand::StandUp => moves::stand_up(robot),
        RobotCommand::StandBy => moves::stand_by(robot),
        RobotCommand::Dance { times } => moves::dance(robot, *times),
        RobotCommand::Release => {
            // The next move re-energizes every servo, starting from the last commanded pose.
            if let Err(e) = robot.hardware_mut().pca.all_off() {
                log::error!("Failed to release servos: {e:?}");
            }
        }
        RobotCommand::SelfTest => moves::self_test(robot, |j| log::info!("Testing joint {j:?}")),
        RobotCommand::JointTest { joints, deg, times } => {
            moves::joint_test(robot, joints, *deg, *times)
        }
        RobotCommand::FullRange { joints, times } => moves::full_range_test(robot, joints, *times),
    }
}

/// Starts the thread that owns the WS2812 and shows what `Indicator` works out from the
/// messages it receives.
fn spawn_led_thread(pin: impl OutputPin + 'static, rx: Receiver<LedMsg>) -> anyhow::Result<()> {
    thread::Builder::new()
        .name("led".into())
        .stack_size(LED_THREAD_STACK)
        .spawn(move || {
            let mut led = match Ws2812::new(pin) {
                Ok(led) => led,
                Err(e) => {
                    log::error!("RGB LED init failed: {e:?}");
                    return;
                }
            };
            let mut indicator = Indicator::new();
            let mut shown = indicator.colour();
            led.set(shown);
            loop {
                let wait = if indicator.animating() {
                    RAINBOW_STEP
                } else {
                    Duration::from_secs(1)
                };
                match rx.recv_timeout(wait) {
                    Ok(msg) => indicator.apply(msg),
                    Err(RecvTimeoutError::Timeout) if indicator.animating() => indicator.advance(),
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => return,
                }
                let colour = indicator.colour();
                if colour != shown {
                    led.set(colour);
                    shown = colour;
                }
            }
        })?;
    Ok(())
}
