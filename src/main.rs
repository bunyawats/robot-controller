//! Hardcoded robot controller: ESP32-S3 + PCA9685 + 16 servos, plus the board's RGB LED
//! as a status light.
//!
//! No networking and no remote control. On boot it waits a few seconds, then runs the
//! demo selected by `DEMO` below, once.

mod rgb_led;
mod robot_16_dof;

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::peripherals::Peripherals;

use rgb_led::color::{self, Rgb};
use rgb_led::ws2812::Ws2812;
use robot_16_dof::esp32::{self as robot_esp32, FaultSignal, Servos};
use robot_16_dof::motion::{Pose, Robot};
use robot_16_dof::moves::{self, Arm, DEFAULT_GAIT};
use robot_16_dof::servo::{self, Joint};
/// What to run after boot. Keep `SelfTest` for the very first flash.
#[allow(dead_code)]
enum Demo {
    /// Moves each joint +15 degrees and back, one at a time, logging its name.
    SelfTest,
    /// Swings each listed joint 30 degrees (direction set by `JOINT_TEST_SWING_DEG`) and back
    /// three times, one joint after another, for checking servos one by one. The log names
    /// each joint and its PCA9685 channel.
    Joints(&'static [Joint]),
    /// Like `Joints`, but swings all the listed joints at the same time, e.g. a left/right
    /// pair to check that they agree.
    Together(&'static [Joint]),
    /// Sways the listed joints together to one side and the other three times, starting in
    /// the direction of `SWAY_FIRST_DEG` (e.g. the hip rolls: left, then right).
    Sway(&'static [Joint]),
    /// Sways the hips left and right three times, opening both claws while leaning left and
    /// closing them while leaning right.
    SwayAndGrip,
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

const DEMO: Demo = Demo::SwayAndGrip;

/// Which way `Joints` and `Together` swing: `JOINT_TEST_DEG` is each joint's positive
/// direction (see the sign conventions on `Joint`), `-JOINT_TEST_DEG` the other way.
const JOINT_TEST_SWING_DEG: f32 = moves::JOINT_TEST_DEG;

/// Where `Sway` goes first. Negative, so the hip rolls lean left first.
const SWAY_FIRST_DEG: f32 = -moves::JOINT_TEST_DEG;

/// Time to get your hands off the robot and the power connected before anything moves.
const START_DELAY_SECS: u32 = 5;

/// Cut power to all servos when the demo finishes. The robot goes limp and may fall over,
/// so this is off by default; with it off the servos keep holding the final pose.
const RELEASE_AT_END: bool = false;

/// Status light colours. Kept dim: the LED is very bright at full power.
const LED_COUNTDOWN: Rgb = (0, 0, 32);
const LED_MOVING: Rgb = (0, 32, 0);
const LED_ERROR: Rgb = (48, 0, 0);

/// Rainbow fade for `Demo::LedCycle`: peak brightness, and time per 1-degree hue step
/// (360 steps x 10 ms = 3.6 s per full rainbow).
const RAINBOW_BRIGHTNESS: u8 = 32;
const RAINBOW_STEP_MS: u32 = 10;

/// The status light turns red on the first failed servo write and stays red.
impl FaultSignal for Ws2812 {
    fn fault(&mut self) {
        self.set(LED_ERROR);
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let mut led = Ws2812::new(peripherals.pins.gpio48)?;

    if let Demo::LedCycle = DEMO {
        log::info!("Fading the status LED through the rainbow");
        for hue in (0..360).cycle() {
            led.set(color::rainbow(hue, RAINBOW_BRIGHTNESS));
            FreeRtos::delay_ms(RAINBOW_STEP_MS);
        }
    }

    let pca = robot_esp32::connect_pca9685(
        peripherals.i2c0,
        peripherals.pins.gpio8, // SDA
        peripherals.pins.gpio9, // SCL
    )
    .inspect_err(|_| led.set(LED_ERROR))?;
    log::info!("PCA9685 ready");

    led.set(LED_COUNTDOWN);
    for s in (1..=START_DELAY_SECS).rev() {
        log::info!("Starting in {s}s - hands off the robot");
        FreeRtos::delay_ms(1000);
    }

    led.set(LED_MOVING);
    let mut robot = Robot::new(Servos::new(pca, led));

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
                moves::joint_test(&mut robot, &[joint], JOINT_TEST_SWING_DEG, 3);
            }
        }
        Demo::Together(joints) => {
            log::info!("Testing joints {joints:?} together");
            moves::joint_test(&mut robot, joints, JOINT_TEST_SWING_DEG, 3);
        }
        Demo::Sway(joints) => {
            log::info!("Swaying {joints:?} side to side");
            moves::sway(&mut robot, joints, SWAY_FIRST_DEG, 3);
        }
        Demo::SwayAndGrip => {
            log::info!("Swaying the hips with the grippers opening and closing");
            moves::sway_and_grip(&mut robot, 3);
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
        servos.fault_signal.set(color::OFF);
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
