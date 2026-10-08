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

/// The 16 joints, in PCA9685 channel order. The order here MUST match `CALIBRATION` below.
///
/// Layout from the kit manual's "servo number Schematic" (see `Robot Schema.jpg`): 4 servos
/// per arm and 4 per leg. The manual numbers them for a 32-channel controller, S1-S8 on one
/// side and S25-S32 on the other; here S1-S8 go on channels 0-7 and S25-S32 on channels
/// 8-15. The drawing is assumed to be a front view, so S1-S8 are the robot's right side.
/// The self-test confirms this: if a "right" joint moves on the left, swap the two blocks
/// of plugs.
///
/// Each leg has a hip roll, two stacked pitch servos at the knee and an ankle roll. There is
/// no hip pitch or ankle pitch, so the foot only stays flat when the two knee servos turn by
/// equal and opposite amounts.
///
/// Sign convention (flip a joint with `invert` in `CALIBRATION` if it moves the wrong way):
///   - gripper:        positive = claw opens
///   - elbow:          positive = forearm bends inward
///   - shoulder pitch: positive = arm swings forward/up
///   - shoulder roll:  positive = arm swings out sideways
///   - knee upper:     positive = everything below it swings forward
///   - knee lower:     positive = shin swings forward
///   - hip/ankle roll: positive = lean toward the robot's right
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Joint {
    RAnkleRoll,     // S1
    RKneeLower,     // S2
    RKneeUpper,     // S3
    RHipRoll,       // S4
    RShoulderPitch, // S5
    RShoulderRoll,  // S6
    RElbow,         // S7
    RGripper,       // S8
    LGripper,       // S25
    LElbow,         // S26
    LShoulderRoll,  // S27
    LShoulderPitch, // S28
    LHipRoll,       // S29
    LKneeUpper,     // S30
    LKneeLower,     // S31
    LAnkleRoll,     // S32
}

impl Joint {
    pub const ALL: [Joint; NUM_JOINTS] = [
        Joint::RAnkleRoll,
        Joint::RKneeLower,
        Joint::RKneeUpper,
        Joint::RHipRoll,
        Joint::RShoulderPitch,
        Joint::RShoulderRoll,
        Joint::RElbow,
        Joint::RGripper,
        Joint::LGripper,
        Joint::LElbow,
        Joint::LShoulderRoll,
        Joint::LShoulderPitch,
        Joint::LHipRoll,
        Joint::LKneeUpper,
        Joint::LKneeLower,
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
/// Channels follow the manual's numbering (S1-S8 on 0-7, S25-S32 on 8-15). Change them if a
/// servo is plugged in elsewhere. The limits are deliberately conservative; widen them only
/// after checking the joint can move that far without binding.
pub const CALIBRATION: [Calib; NUM_JOINTS] = [
    c(0, 50.0, 130.0),  // RAnkleRoll     S1
    c(1, 40.0, 140.0),  // RKneeLower     S2
    c(2, 40.0, 140.0),  // RKneeUpper     S3
    c(3, 50.0, 130.0),  // RHipRoll       S4
    c(4, 10.0, 170.0),  // RShoulderPitch S5
    c(5, 20.0, 160.0),  // RShoulderRoll  S6
    c(6, 20.0, 160.0),  // RElbow         S7
    c(7, 20.0, 160.0),  // RGripper       S8
    c(8, 20.0, 160.0),  // LGripper       S25
    c(9, 20.0, 160.0),  // LElbow         S26
    c(10, 20.0, 160.0), // LShoulderRoll  S27
    c(11, 10.0, 170.0), // LShoulderPitch S28
    c(12, 50.0, 130.0), // LHipRoll       S29
    c(13, 40.0, 140.0), // LKneeUpper     S30
    c(14, 40.0, 140.0), // LKneeLower     S31
    c(15, 50.0, 130.0), // LAnkleRoll     S32
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
