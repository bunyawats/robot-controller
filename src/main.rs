//! Hardcoded robot controller: ESP32-S3 + PCA9685 + 16 servos.
//!
//! No networking and no remote control. On boot it waits a few seconds, then runs the
//! demo selected by `DEMO` below, once.

mod motion;
mod moves;
mod pca9685;
mod servo;

use esp_idf_svc::hal::delay::{FreeRtos, BLOCK};
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::Hertz;

use motion::{Hardware, Pose, Robot};
use moves::{Arm, DEFAULT_GAIT};
use pca9685::Pca9685;

/// What to run after boot. Keep `SelfTest` for the very first flash.
#[allow(dead_code)]
enum Demo {
    /// Moves each joint +15 degrees and back, one at a time, logging its name.
    SelfTest,
    /// Raises and lowers both arms three times.
    Arms,
    /// Walks 10 steps.
    Walk,
    /// Arms (right, left, both), then 10 steps.
    All,
}

const DEMO: Demo = Demo::SelfTest;

/// Time to get your hands off the robot and the power connected before anything moves.
const START_DELAY_SECS: u32 = 5;

/// Cut power to all servos when the demo finishes. The robot goes limp and may fall over,
/// so this is off by default; with it off the servos keep holding the final pose.
const RELEASE_AT_END: bool = false;

/// I2C (with a delay) for the PCA9685 driver, on top of the ESP-IDF I2C driver.
struct I2cBus(I2cDriver<'static>);

impl pca9685::Transport for I2cBus {
    type Error = esp_idf_svc::sys::EspError;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.write(addr, bytes, BLOCK)
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

/// The real robot: poses become PCA9685 tick values through the calibration table.
struct Servos {
    pca: Pca9685<I2cBus>,
}

impl Hardware for Servos {
    fn write_pose(&mut self, pose: &Pose) {
        let ticks = servo::pose_to_ticks(&pose.0, &servo::CALIBRATION);
        if let Err(e) = self.pca.set_all_ticks(&ticks) {
            log::error!("I2C write to PCA9685 failed: {e:?}");
        }
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio8, // SDA
        peripherals.pins.gpio9, // SCL
        &I2cConfig::new().baudrate(Hertz(400_000)),
    )?;

    let mut pca = Pca9685::new(I2cBus(i2c), pca9685::DEFAULT_ADDR);
    pca.init_50hz()
        .map_err(|e| anyhow::anyhow!("PCA9685 not responding at 0x40: {e:?}"))?;
    log::info!("PCA9685 ready");

    for s in (1..=START_DELAY_SECS).rev() {
        log::info!("Starting in {s}s - hands off the robot");
        FreeRtos::delay_ms(1000);
    }

    let mut robot = Robot::new(Servos { pca });

    // The servos' real positions are unknown until first commanded, so this first write
    // snaps every joint to neutral at once.
    robot.snap_to(&Pose::neutral());
    FreeRtos::delay_ms(1000);

    match DEMO {
        Demo::SelfTest => {
            moves::self_test(&mut robot, |j| log::info!("Testing joint {j:?}"));
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
    }

    log::info!("Demo finished");
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
