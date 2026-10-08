//! Hardcoded body movements: self-test, arm up/down, and a simple walk.
//!
//! These are starting points. Servo directions, trims and the gait numbers below all need
//! tuning on the real robot. Do the self-test and fix `CALIBRATION` first.

use crate::motion::{Hardware, Pose, Robot};
use crate::servo::Joint;

// ---------------------------------------------------------------------------------------
// Self-test
// ---------------------------------------------------------------------------------------

/// How far each joint swings during the self-test.
const SELF_TEST_DEG: f32 = 15.0;

/// Moves every joint, one at a time, by +15 degrees and back. `on_joint` is called before
/// each joint so the caller can log which one is about to move.
pub fn self_test<H: Hardware>(robot: &mut Robot<H>, mut on_joint: impl FnMut(Joint)) {
    let home = *robot.current();
    for joint in Joint::ALL {
        on_joint(joint);
        let out = home.add(joint, SELF_TEST_DEG);
        robot.move_to(&out, 500);
        robot.pause(300);
        robot.move_to(&home, 500);
        robot.pause(300);
    }
}

// ---------------------------------------------------------------------------------------
// Arms
// ---------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Arm {
    Right,
    Left,
    Both,
}

/// How far the arm swings up from where it hangs.
const ARM_RAISE_DEG: f32 = 70.0;
const ARM_MOVE_MS: u32 = 800;

/// Raises the arm (shoulder pitch) and lowers it again, `times` times.
pub fn arm_up_down<H: Hardware>(robot: &mut Robot<H>, arm: Arm, times: u32) {
    let down = *robot.current();
    let mut up = down;
    if matches!(arm, Arm::Right | Arm::Both) {
        up = up.add(Joint::RShoulderPitch, ARM_RAISE_DEG);
    }
    if matches!(arm, Arm::Left | Arm::Both) {
        up = up.add(Joint::LShoulderPitch, ARM_RAISE_DEG);
    }
    for _ in 0..times {
        robot.move_to(&up, ARM_MOVE_MS);
        robot.pause(200);
        robot.move_to(&down, ARM_MOVE_MS);
        robot.pause(200);
    }
}

// ---------------------------------------------------------------------------------------
// Walking
// ---------------------------------------------------------------------------------------

/// Gait tuning. Start small and increase slowly.
#[derive(Clone, Copy, Debug)]
pub struct Gait {
    /// Sideways lean over the support foot while the other foot is unloaded.
    pub lean_deg: f32,
    /// Total swing of the knee pair between a leg's backmost and foremost position.
    pub stride_deg: f32,
    /// Extra outward hip roll on the swinging leg so its foot clears the ground.
    pub lift_deg: f32,
    /// Duration of each of the five phases of a step.
    pub phase_ms: u32,
}

pub const DEFAULT_GAIT: Gait = Gait {
    lean_deg: 8.0,
    stride_deg: 30.0,
    lift_deg: 6.0,
    phase_ms: 300,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Right,
    Left,
}

impl Side {
    fn other(self) -> Side {
        match self {
            Side::Right => Side::Left,
            Side::Left => Side::Right,
        }
    }
    fn idx(self) -> usize {
        match self {
            Side::Right => 0,
            Side::Left => 1,
        }
    }
    /// Sign of a roll that tilts this side's leg outward (positive roll = toward the right).
    fn outward(self) -> f32 {
        match self {
            Side::Right => 1.0,
            Side::Left => -1.0,
        }
    }
}

#[derive(Clone, Copy)]
struct Leg {
    side: Side,
    hip_roll: Joint,
    knee_upper: Joint,
    knee_lower: Joint,
    ankle_roll: Joint,
}

const RIGHT_LEG: Leg = Leg {
    side: Side::Right,
    hip_roll: Joint::RHipRoll,
    knee_upper: Joint::RKneeUpper,
    knee_lower: Joint::RKneeLower,
    ankle_roll: Joint::RAnkleRoll,
};

const LEFT_LEG: Leg = Leg {
    side: Side::Left,
    hip_roll: Joint::LHipRoll,
    knee_upper: Joint::LKneeUpper,
    knee_lower: Joint::LKneeLower,
    ankle_roll: Joint::LAnkleRoll,
};

/// Commands for one leg, in degrees from neutral.
#[derive(Clone, Copy)]
struct LegCmd {
    /// Knee pair swing: positive moves the foot forward.
    stride: f32,
    /// Outward hip roll on top of the body lean.
    lift: f32,
}

/// Builds a standing-style pose from per-leg commands. The two knee servos turn by equal
/// and opposite amounts so the shin stays parallel to the thigh and the foot stays flat
/// front to back, and each ankle roll cancels its hip roll so the foot stays flat side to
/// side.
fn frame(roll: f32, right: LegCmd, left: LegCmd) -> Pose {
    let mut pose = Pose::neutral();
    for (leg, cmd) in [(RIGHT_LEG, right), (LEFT_LEG, left)] {
        let hip_roll = roll + leg.side.outward() * cmd.lift;
        pose = pose
            .with(leg.knee_upper, cmd.stride)
            .with(leg.knee_lower, -cmd.stride)
            .with(leg.hip_roll, hip_roll)
            .with(leg.ankle_roll, -hip_roll);
    }
    pose
}

fn step_frame(roll: f32, swing: Side, s_stride: f32, s_lift: f32, p_stride: f32) -> Pose {
    let swing_cmd = LegCmd { stride: s_stride, lift: s_lift };
    let support_cmd = LegCmd { stride: p_stride, lift: 0.0 };
    match swing {
        Side::Right => frame(roll, swing_cmd, support_cmd),
        Side::Left => frame(roll, support_cmd, swing_cmd),
    }
}

/// Standing straight on both feet (legs only; arms are left as they are).
fn stand_pose() -> Pose {
    let zero = LegCmd { stride: 0.0, lift: 0.0 };
    frame(0.0, zero, zero)
}

/// Walks `steps` steps forward, starting with the right foot, then stands still.
///
/// The legs have no hip or ankle pitch, so this is a shuffle: lean onto the support foot,
/// tilt the unloaded leg outward to clear the ground, move it forward with the knee pair,
/// plant it while the support leg's knee pair moves back (which carries the body forward),
/// and straighten up.
pub fn walk<H: Hardware>(robot: &mut Robot<H>, steps: u32, g: &Gait) {
    let half = g.stride_deg / 2.0;
    let mut stride = [0.0f32; 2]; // current knee pair offset of [right, left]

    robot.move_to(&stand_pose(), 600);
    robot.pause(300);

    for n in 0..steps {
        let swing = if n % 2 == 0 { Side::Right } else { Side::Left };
        let support = swing.other();
        let (s0, p0) = (stride[swing.idx()], stride[support.idx()]);
        let (s1, p1) = (half, -half);

        // Lean toward the support side.
        let roll = support.outward() * g.lean_deg;

        // 1. lean over the support foot
        robot.move_to(&step_frame(roll, swing, s0, 0.0, p0), g.phase_ms);
        // 2. lift the swing foot and start moving it forward
        let s_mid = s0 + (s1 - s0) * 0.3;
        robot.move_to(&step_frame(roll, swing, s_mid, g.lift_deg, p0), g.phase_ms);
        // 3. swing it fully forward, still lifted
        robot.move_to(&step_frame(roll, swing, s1, g.lift_deg, p0), g.phase_ms);
        // 4. plant the foot while the support leg moves back, which moves the body forward
        robot.move_to(&step_frame(roll, swing, s1, 0.0, p1), g.phase_ms);
        // 5. straighten up
        robot.move_to(&step_frame(0.0, swing, s1, 0.0, p1), g.phase_ms);

        stride[swing.idx()] = s1;
        stride[support.idx()] = p1;
    }

    robot.move_to(&stand_pose(), 600);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::test_support::MockHw;
    use crate::servo::{CALIBRATION, NEUTRAL_DEG};

    fn robot() -> Robot<MockHw> {
        Robot::new(MockHw::default())
    }

    #[test]
    fn self_test_visits_every_joint_once_and_returns_home() {
        let mut r = robot();
        let mut visited = Vec::new();
        self_test(&mut r, |j| visited.push(j));
        assert_eq!(visited, Joint::ALL.to_vec());
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn self_test_moves_only_one_joint_at_a_time() {
        let mut r = robot();
        self_test(&mut r, |_| {});
        for pose in &r.hardware_mut().poses {
            let moved = (0..16).filter(|i| (pose.0[*i] - NEUTRAL_DEG).abs() > 1e-3).count();
            assert!(moved <= 1, "more than one joint away from neutral: {moved}");
        }
    }

    #[test]
    fn arm_up_down_raises_only_the_requested_arm_and_comes_back() {
        let mut r = robot();
        arm_up_down(&mut r, Arm::Right, 3);
        let poses = &r.hardware_mut().poses;
        let max_r = poses.iter().map(|p| p.get(Joint::RShoulderPitch)).fold(0.0, f32::max);
        let max_l = poses.iter().map(|p| p.get(Joint::LShoulderPitch)).fold(0.0, f32::max);
        assert!((max_r - (NEUTRAL_DEG + ARM_RAISE_DEG)).abs() < 1e-3);
        assert!((max_l - NEUTRAL_DEG).abs() < 1e-3);
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn arm_up_down_repeats_the_requested_number_of_times() {
        let mut r = robot();
        arm_up_down(&mut r, Arm::Both, 4);
        let tops = r
            .hardware_mut()
            .poses
            .iter()
            .filter(|p| (p.get(Joint::RShoulderPitch) - (NEUTRAL_DEG + ARM_RAISE_DEG)).abs() < 1e-3)
            .count();
        assert_eq!(tops, 4);
    }

    #[test]
    fn walk_ends_standing_with_arms_untouched() {
        let mut r = robot();
        walk(&mut r, 10, &DEFAULT_GAIT);
        assert_eq!(*r.current(), stand_pose());
        assert_eq!(r.current().get(Joint::RShoulderPitch), NEUTRAL_DEG);
    }

    #[test]
    fn walk_stays_inside_every_joints_calibrated_limits() {
        let mut r = robot();
        walk(&mut r, 10, &DEFAULT_GAIT);
        for pose in &r.hardware_mut().poses {
            for (i, cal) in CALIBRATION.iter().enumerate() {
                let v = pose.0[i];
                assert!(
                    v >= cal.min_deg && v <= cal.max_deg,
                    "joint {:?} reached {v}, outside {}..{}",
                    Joint::ALL[i],
                    cal.min_deg,
                    cal.max_deg
                );
            }
        }
    }

    #[test]
    fn walk_alternates_feet_and_lifts_each_one_per_step() {
        let mut r = robot();
        walk(&mut r, 10, &DEFAULT_GAIT);
        let poses = &r.hardware_mut().poses;
        // A foot is lifted while the hip rolls are spread apart; the lifted leg is the one
        // whose knee pair moves forward during that time.
        let mut lifted = Vec::new();
        let mut start: Option<&Pose> = None;
        for w in poses.windows(2) {
            let spread = |p: &Pose| (p.get(Joint::RHipRoll) - p.get(Joint::LHipRoll)).abs();
            if spread(&w[1]) > 2.0 && start.is_none() {
                start = Some(&w[1]);
            }
            if spread(&w[1]) <= 2.0 {
                if let Some(s) = start.take() {
                    let moved = |j: Joint| w[0].get(j) - s.get(j);
                    let side = if moved(Joint::RKneeUpper) > moved(Joint::LKneeUpper) {
                        Side::Right
                    } else {
                        Side::Left
                    };
                    lifted.push(side);
                }
            }
        }
        let expected: Vec<Side> =
            (0..10).map(|n| if n % 2 == 0 { Side::Right } else { Side::Left }).collect();
        assert!(lifted == expected, "lift order was wrong ({} lifts)", lifted.len());
    }

    #[test]
    fn walk_takes_a_sensible_amount_of_time() {
        let mut r = robot();
        walk(&mut r, 10, &DEFAULT_GAIT);
        let secs = r.hardware_mut().elapsed_ms as f32 / 1000.0;
        assert!(secs > 10.0 && secs < 30.0, "walk took {secs}s");
    }

    #[test]
    fn zero_steps_just_stands() {
        let mut r = robot();
        walk(&mut r, 0, &DEFAULT_GAIT);
        assert_eq!(*r.current(), stand_pose());
    }

    #[test]
    fn feet_stay_flat_knee_pair_and_rolls_cancel() {
        let p = step_frame(8.0, Side::Right, 15.0, 6.0, -15.0);
        for leg in [RIGHT_LEG, LEFT_LEG] {
            let off = |j: Joint| p.get(j) - NEUTRAL_DEG;
            assert_eq!(off(leg.knee_upper), -off(leg.knee_lower));
            assert_eq!(off(leg.hip_roll), -off(leg.ankle_roll));
        }
        // the swinging right leg is tilted outward (toward the right) by the lift
        assert_eq!(p.get(Joint::RHipRoll), NEUTRAL_DEG + 8.0 + 6.0);
        assert_eq!(p.get(Joint::LHipRoll), NEUTRAL_DEG + 8.0);
    }
}
