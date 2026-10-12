//! Hardcoded robot controller: ESP32-S3 + PCA9685 + 16 servos.
//!
//! No networking and no remote control. On boot it waits a few seconds, then runs the
//! demo selected by `DEMO` below, once.

mod motion;
mod moves;
mod pca9685;
mod servo;

use esp_idf_svc::hal::delay::{FreeRtos, TickType, BLOCK};
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::rmt::config::{TransmitConfig, TxChannelConfig};
use esp_idf_svc::hal::rmt::encoder::{BytesEncoder, BytesEncoderConfig};
use esp_idf_svc::hal::rmt::{PinState, Symbol, TxChannelDriver};
use esp_idf_svc::hal::units::Hertz;
use std::time::Duration;

use motion::{Hardware, Pose, Robot};
use moves::{Arm, DEFAULT_GAIT};
use pca9685::Pca9685;
use servo::Joint;

/// What to run after boot. Keep `SelfTest` for the very first flash.
#[allow(dead_code)]
enum Demo {
    /// Moves each joint +15 degrees and back, one at a time, logging its name.
    SelfTest,
    /// Swings each listed joint +30 degrees and back three times, one joint after another,
    /// for checking servos one by one. The log names each joint and its PCA9685 channel.
    Joints(&'static [Joint]),
    /// Like `Joints`, but swings all the listed joints at the same time, e.g. a left/right
    /// pair to check that they agree.
    Together(&'static [Joint]),
    /// Raises and lowers both arms three times.
    Arms,
    /// Walks 10 steps.
    Walk,
    /// Arms (right, left, both), then 10 steps.
    All,
    /// Fades the status LED smoothly around the rainbow forever. Doesn't touch I2C, so it
    /// runs without the PCA9685 connected.
    LedCycle,
}

const DEMO: Demo = Demo::LedCycle;

/// Time to get your hands off the robot and the power connected before anything moves.
const START_DELAY_SECS: u32 = 5;

/// Cut power to all servos when the demo finishes. The robot goes limp and may fall over,
/// so this is off by default; with it off the servos keep holding the final pose.
const RELEASE_AT_END: bool = false;

/// I2C bus speed. 100 kHz rather than 400 kHz because servo current spikes on a shared
/// 5 V supply make the PCA9685 miss writes; the slower bus is less sensitive to that noise.
const I2C_HZ: u32 = 100_000;

/// Status light colours as (red, green, blue). Kept dim: the LED is very bright at full power.
type Rgb = (u8, u8, u8);
const LED_COUNTDOWN: Rgb = (0, 0, 32);
const LED_MOVING: Rgb = (0, 32, 0);
const LED_ERROR: Rgb = (48, 0, 0);
const LED_OFF: Rgb = (0, 0, 0);

/// Rainbow fade for `Demo::LedCycle`: peak brightness, and time per 1-degree hue step
/// (360 steps x 10 ms = 3.6 s per full rainbow).
const RAINBOW_BRIGHTNESS: u8 = 32;
const RAINBOW_STEP_MS: u32 = 10;

/// Colour at `hue` degrees (0 red, 120 green, 240 blue) on the colour wheel, fully
/// saturated, with `max` as the brightest channel.
fn rainbow(hue: u32, max: u8) -> Rgb {
    let hue = hue % 360;
    // Within each 60-degree sector one channel ramps up or down while another sits at max.
    let ramp = |deg: u32| (u32::from(max) * deg / 60) as u8;
    let up = ramp(hue % 60);
    let down = max - up;
    match hue / 60 {
        0 => (max, up, 0),
        1 => (down, max, 0),
        2 => (0, max, up),
        3 => (0, down, max),
        4 => (up, 0, max),
        _ => (max, 0, down),
    }
}

/// RMT clock for the status LED: 10 MHz, so one tick is 0.1 us.
const LED_RMT_HZ: Hertz = Hertz(10_000_000);

/// The board's WS2812 RGB LED on GPIO48, used as a status light: blue during the boot
/// countdown, green while the demo runs, red if the PCA9685 fails, off when finished.
struct StatusLed {
    tx: TxChannelDriver<'static>,
    encoder: BytesEncoder,
}

impl StatusLed {
    fn new(pin: impl OutputPin + 'static) -> anyhow::Result<Self> {
        let tx = TxChannelDriver::new(
            pin,
            &TxChannelConfig {
                resolution: LED_RMT_HZ,
                ..Default::default()
            },
        )?;
        // WS2812 bit timings from its datasheet, sent most significant bit first.
        let bit = |high_ns, low_ns| {
            Symbol::new_with(
                LED_RMT_HZ,
                PinState::High,
                Duration::from_nanos(high_ns),
                PinState::Low,
                Duration::from_nanos(low_ns),
            )
        };
        let encoder = BytesEncoder::with_config(&BytesEncoderConfig {
            bit0: bit(350, 800)?,
            bit1: bit(700, 600)?,
            msb_first: true,
            ..Default::default()
        })?;
        Ok(Self { tx, encoder })
    }

    /// Only logs on failure: a broken status light shouldn't stop the robot.
    fn set(&mut self, (r, g, b): Rgb) {
        // WS2812 takes the bytes in green, red, blue order.
        let sent = self
            .tx
            .send_and_wait(&mut self.encoder, &[g, r, b], &TransmitConfig::default());
        if let Err(e) = sent {
            log::warn!("Status LED write failed: {e:?}");
        }
    }
}

/// I2C (with a delay) for the PCA9685 driver, on top of the ESP-IDF I2C driver.
struct I2cBus(I2cDriver<'static>);

impl pca9685::Transport for I2cBus {
    type Error = esp_idf_svc::sys::EspError;

    /// Retries a failed write once, since servo noise makes the odd write fail.
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0
            .write(addr, bytes, BLOCK)
            .or_else(|_| self.0.write(addr, bytes, BLOCK))
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

/// The real robot: poses become PCA9685 tick values through the calibration table.
struct Servos {
    pca: Pca9685<I2cBus>,
    led: StatusLed,
    /// Set by the first failed write; the LED then stays red for the rest of the run.
    write_failed: bool,
}

impl Hardware for Servos {
    fn write_pose(&mut self, pose: &Pose) {
        let ticks = servo::pose_to_ticks(&pose.0, &servo::CALIBRATION);
        if let Err(e) = self.pca.set_all_ticks(&ticks) {
            log::error!("I2C write to PCA9685 failed: {e:?}");
            if !self.write_failed {
                self.write_failed = true;
                self.led.set(LED_ERROR);
            }
        }
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

/// Logs every I2C address that answers (the PCA9685 should be at 0x40), so a wiring or
/// power fault shows up as "no devices" rather than only as a failed PCA9685 init.
fn log_i2c_scan(i2c: &mut I2cDriver<'static>) {
    let timeout = TickType::new_millis(10).ticks();
    let found: Vec<String> = (0x08u8..=0x77)
        .filter(|&addr| i2c.read(addr, &mut [0u8; 1], timeout).is_ok())
        .map(|addr| format!("0x{addr:02X}"))
        .collect();
    if found.is_empty() {
        log::warn!("I2C scan: no devices on SDA=GPIO8/SCL=GPIO9");
    } else {
        log::info!("I2C scan: found {}", found.join(", "));
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let mut led = StatusLed::new(peripherals.pins.gpio48)?;

    if let Demo::LedCycle = DEMO {
        log::info!("Fading the status LED through the rainbow");
        for hue in (0..360).cycle() {
            led.set(rainbow(hue, RAINBOW_BRIGHTNESS));
            FreeRtos::delay_ms(RAINBOW_STEP_MS);
        }
    }
    let mut i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio8, // SDA
        peripherals.pins.gpio9, // SCL
        &I2cConfig::new().baudrate(Hertz(I2C_HZ)),
    )?;

    log_i2c_scan(&mut i2c);

    let mut pca = Pca9685::new(I2cBus(i2c), pca9685::DEFAULT_ADDR);
    if let Err(e) = pca.init_50hz() {
        led.set(LED_ERROR);
        anyhow::bail!("PCA9685 not responding at 0x40: {e:?}");
    }
    log::info!("PCA9685 ready");

    led.set(LED_COUNTDOWN);
    for s in (1..=START_DELAY_SECS).rev() {
        log::info!("Starting in {s}s - hands off the robot");
        FreeRtos::delay_ms(1000);
    }

    led.set(LED_MOVING);
    let mut robot = Robot::new(Servos {
        pca,
        led,
        write_failed: false,
    });

    // The servos' real positions are unknown until first commanded, so this first write
    // snaps every joint to neutral at once.
    robot.snap_to(&Pose::neutral());
    FreeRtos::delay_ms(1000);

    match DEMO {
        Demo::SelfTest => {
            moves::self_test(&mut robot, |j| log::info!("Testing joint {j:?}"));
        }
        Demo::Joints(joints) => {
            for &joint in joints {
                let channel = servo::CALIBRATION[joint.index()].channel;
                log::info!("Testing joint {joint:?} on channel {channel}");
                moves::joint_test(&mut robot, &[joint], 3);
            }
        }
        Demo::Together(joints) => {
            log::info!("Testing joints {joints:?} together");
            moves::joint_test(&mut robot, joints, 3);
        }
        Demo::Arms => {
            moves::arm_up_down(&mut robot, Arm::Both, 3);
        }
        Demo::Walk => {
            moves::walk(&mut robot, 10, &DEFAULT_GAIT);
        }
        Demo::All => {
            log::info!("Right arm");
            moves::arm_up_down(&mut robot, Arm::Right, 3);
            log::info!("Left arm");
            moves::arm_up_down(&mut robot, Arm::Left, 3);
            log::info!("Both arms");
            moves::arm_up_down(&mut robot, Arm::Both, 3);
            log::info!("Walking 10 steps");
            moves::walk(&mut robot, 10, &DEFAULT_GAIT);
        }
        Demo::LedCycle => unreachable!("handled before the PCA9685 is set up"),
    }

    log::info!("Demo finished");
    let servos = robot.hardware_mut();
    if !servos.write_failed {
        servos.led.set(LED_OFF);
    }
    if RELEASE_AT_END {
        if let Err(e) = robot.hardware_mut().pca.all_off() {
            log::error!("Failed to release servos: {e:?}");
        }
    }

    // Nothing else to do; press reset to run it again.
    loop {
        FreeRtos::delay_ms(1000);
    }
}
