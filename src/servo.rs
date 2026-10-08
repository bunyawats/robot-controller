//! Joint table and angle -> PCA9685 tick conversion.
//!
//! Pure logic with no hardware dependencies, so it can be unit-tested on a PC.
//!
//! Angles are "logical degrees": 90 is the neutral (centered) position of a joint.
//! Everything else in the firmware talks in logical degrees. Only this module knows
//! about servo direction, trim, safe limits and PWM ticks.

pub const NUM_JOINTS: usize = 16;
pub const NEUTRAL_DEG: f32 = 90.0;

/// PCA9685 output frequency (prescale 121 gives about 50 Hz).
pub const PWM_HZ: f32 = 50.0;
/// The PCA9685 divides each PWM period into 4096 steps.
pub const PCA_STEPS: f32 = 4096.0;
/// Pulse width at servo angle 0 and 180 degrees. SG90/MG90S accept roughly 500-2400 us.
pub const MIN_PULSE_US: f32 = 500.0;
pub const MAX_PULSE_US: f32 = 2400.0;

/// The 16 joints. The order here MUST match the order of `CALIBRATION` below.
///
/// Layout assumed for the 16-servo biped: 3 per arm (two shoulder joints plus a claw),
/// 5 per leg. This is a guess from a photo: run the self-test on first boot, watch which
/// servo moves, and rename or reorder entries here to match the real robot.
/// Sign convention (flip a joint with `invert` in `CALIBRATION` if it moves the wrong way):
///   - gripper: positive = claw opens
///   - shoulder pitch: positive = arm swings forward/up
///   - hip pitch:      positive = thigh swings forward
///   - knee:           positive = knee bends (foot goes backward)
///   - ankle pitch:    positive = toes up
///   - hip/ankle roll: positive = lean toward the robot's right
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Joint {
    RShoulderPitch,
    RShoulderRoll,
    RGripper,
    LShoulderPitch,
    LShoulderRoll,
    LGripper,
    RHipRoll,
    RHipPitch,
    RKnee,
    RAnklePitch,
    RAnkleRoll,
    LHipRoll,
    LHipPitch,
    LKnee,
    LAnklePitch,
    LAnkleRoll,
}

impl Joint {
    pub const ALL: [Joint; NUM_JOINTS] = [
        Joint::RShoulderPitch,
        Joint::RShoulderRoll,
        Joint::RGripper,
        Joint::LShoulderPitch,
        Joint::LShoulderRoll,
        Joint::LGripper,
        Joint::RHipRoll,
        Joint::RHipPitch,
        Joint::RKnee,
        Joint::RAnklePitch,
        Joint::RAnkleRoll,
        Joint::LHipRoll,
        Joint::LHipPitch,
        Joint::LKnee,
        Joint::LAnklePitch,
        Joint::LAnkleRoll,
    ];

    pub const fn index(self) -> usize {
        self as usize
    }
}

/// Per-joint calibration.
#[derive(Clone, Copy, Debug)]
pub struct Calib {
    /// PCA9685 output channel (0-15) this joint's servo is plugged into.
    pub channel: u8,
    /// Safe range in logical degrees. Commands outside it are clamped.
    pub min_deg: f32,
    pub max_deg: f32,
    /// Mechanical trim in degrees, added after inversion, to line the joint up with
    /// its true neutral when the servo horn is not mounted exactly centered.
    pub trim_deg: f32,
    /// Flip the direction of rotation (for servos mounted mirrored).
    pub invert: bool,
}

const fn c(channel: u8, min_deg: f32, max_deg: f32) -> Calib {
    Calib {
        channel,
        min_deg,
        max_deg,
        trim_deg: 0.0,
        invert: false,
    }
}

/// EDIT THIS TABLE FOR YOUR ROBOT.
///
/// The channel numbers are a placeholder (joint order = channel order). Change them to
/// match where each servo is really plugged in. The limits are deliberately conservative;
/// widen them only after checking the joint can move that far without binding.
pub const CALIBRATION: [Calib; NUM_JOINTS] = [
    c(0, 10.0, 170.0),  // RShoulderPitch
    c(1, 20.0, 160.0),  // RShoulderRoll
    c(2, 20.0, 160.0),  // RGripper
    c(3, 10.0, 170.0),  // LShoulderPitch
    c(4, 20.0, 160.0),  // LShoulderRoll
    c(5, 20.0, 160.0),  // LGripper
    c(6, 50.0, 130.0),  // RHipRoll
    c(7, 40.0, 140.0),  // RHipPitch
    c(8, 40.0, 140.0),  // RKnee
    c(9, 40.0, 140.0),  // RAnklePitch
    c(10, 50.0, 130.0), // RAnkleRoll
    c(11, 50.0, 130.0), // LHipRoll
    c(12, 40.0, 140.0), // LHipPitch
    c(13, 40.0, 140.0), // LKnee
    c(14, 40.0, 140.0), // LAnklePitch
    c(15, 50.0, 130.0), // LAnkleRoll
];

/// Converts one logical angle to PCA9685 "off" ticks (0-4095) for a joint.
pub fn angle_to_ticks(cal: &Calib, logical_deg: f32) -> u16 {
    let limited = logical_deg.clamp(cal.min_deg, cal.max_deg);
    let directed = if cal.invert { 180.0 - limited } else { limited };
    let servo_deg = (directed + cal.trim_deg).clamp(0.0, 180.0);
    let pulse_us = MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * servo_deg / 180.0;
    let period_us = 1_000_000.0 / PWM_HZ;
    (pulse_us / period_us * PCA_STEPS).round() as u16
}

/// Converts a whole pose (indexed by joint) into ticks indexed by PCA9685 channel.
pub fn pose_to_ticks(
    pose: &[f32; NUM_JOINTS],
    table: &[Calib; NUM_JOINTS],
) -> [u16; NUM_JOINTS] {
    let mut ticks = [0u16; NUM_JOINTS];
    for (i, cal) in table.iter().enumerate() {
        ticks[cal.channel as usize] = angle_to_ticks(cal, pose[i]);
    }
    ticks
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_is_about_1450_us() {
        // 1450 us of a 20000 us period * 4096 = 296.96
        assert_eq!(angle_to_ticks(&c(0, 0.0, 180.0), 90.0), 297);
    }

    #[test]
    fn end_stops_match_pulse_range() {
        let cal = c(0, 0.0, 180.0);
        assert_eq!(angle_to_ticks(&cal, 0.0), 102); // 500 us
        assert_eq!(angle_to_ticks(&cal, 180.0), 492); // 2400 us
    }

    #[test]
    fn commands_beyond_limits_are_clamped() {
        let cal = c(0, 40.0, 140.0);
        assert_eq!(angle_to_ticks(&cal, 500.0), angle_to_ticks(&cal, 140.0));
        assert_eq!(angle_to_ticks(&cal, -50.0), angle_to_ticks(&cal, 40.0));
    }

    #[test]
    fn invert_mirrors_around_center() {
        let normal = c(0, 0.0, 180.0);
        let inverted = Calib { invert: true, ..normal };
        assert_eq!(angle_to_ticks(&inverted, 60.0), angle_to_ticks(&normal, 120.0));
        assert_eq!(angle_to_ticks(&inverted, 90.0), angle_to_ticks(&normal, 90.0));
    }

    #[test]
    fn trim_shifts_the_output() {
        let normal = c(0, 0.0, 180.0);
        let trimmed = Calib { trim_deg: 10.0, ..normal };
        assert_eq!(angle_to_ticks(&trimmed, 90.0), angle_to_ticks(&normal, 100.0));
    }

    #[test]
    fn ticks_never_zero_or_overflow() {
        for cal in CALIBRATION.iter() {
            for d in [-1000.0f32, 0.0, 90.0, 180.0, 1000.0] {
                let t = angle_to_ticks(cal, d);
                assert!(t > 0 && t < 4096, "ticks {t} out of range at {d}");
            }
        }
    }

    #[test]
    fn calibration_table_is_sane() {
        let mut seen = [false; NUM_JOINTS];
        for cal in CALIBRATION.iter() {
            assert!((cal.channel as usize) < NUM_JOINTS, "channel out of range");
            assert!(!seen[cal.channel as usize], "duplicate channel {}", cal.channel);
            seen[cal.channel as usize] = true;
            assert!(cal.min_deg <= NEUTRAL_DEG && NEUTRAL_DEG <= cal.max_deg);
        }
        assert!(seen.iter().all(|s| *s));
    }

    #[test]
    fn joint_all_matches_enum_order() {
        for (i, j) in Joint::ALL.iter().enumerate() {
            assert_eq!(j.index(), i);
        }
    }

    #[test]
    fn pose_to_ticks_routes_by_channel() {
        let mut table = CALIBRATION;
        // Swap the channels of the first two joints.
        table[0].channel = 1;
        table[1].channel = 0;
        let mut pose = [NEUTRAL_DEG; NUM_JOINTS];
        pose[0] = 150.0;
        let ticks = pose_to_ticks(&pose, &table);
        assert_eq!(ticks[1], angle_to_ticks(&table[0], 150.0));
        assert_eq!(ticks[0], angle_to_ticks(&table[1], NEUTRAL_DEG));
    }
}
