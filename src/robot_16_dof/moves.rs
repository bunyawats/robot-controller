//! Hardcoded body movements: self-test, arm up/down, and a simple walk.
//!
//! These are starting points. Servo directions, trims and the gait numbers below all need
//! tuning on the real robot. Do the self-test and fix `CALIBRATION` first.

use crate::robot_16_dof::motion::{Hardware, Pose, Robot};
use crate::robot_16_dof::servo::{Joint, CALIBRATION, NEUTRAL_DEG};

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

/// How far `joint_test` swings the joints. Fits inside every joint's calibrated limits in
/// both directions.
pub const JOINT_TEST_DEG: f32 = 30.0;

/// Swings the given joints together by `swing_deg` (normally `JOINT_TEST_DEG`, or its
/// negative for the other direction) and back, `times` times, with a hold at each end. One
/// direction per run, so the first movement shows which way the sign turns (compare with
/// the sign convention on `Joint`; flip `invert` if it is wrong). Moving a left/right pair
/// together shows at a glance whether they agree.
pub fn joint_test<H: Hardware>(
    robot: &mut Robot<H>,
    joints: &[Joint],
    swing_deg: f32,
    times: u32,
) {
    let home = *robot.current();
    let out = joints.iter().fold(home, |pose, &j| pose.add(j, swing_deg));
    for _ in 0..times {
        robot.move_to(&out, 600);
        robot.pause(700);
        robot.move_to(&home, 600);
        robot.pause(700);
    }
}

/// Swings each given joint, one after another, through its whole calibrated range: from
/// neutral slowly to its max limit, then to its min limit, then back to neutral, `times`
/// times. Shows the full travel of a joint, and whether it hits anything on the way.
pub fn full_range_test<H: Hardware>(robot: &mut Robot<H>, joints: &[Joint], times: u32) {
    for &joint in joints {
        let cal = &CALIBRATION[joint.index()];
        let home = *robot.current();
        let at = |deg: f32| home.with(joint, deg - NEUTRAL_DEG);
        for _ in 0..times {
            robot.move_to(&at(cal.max_deg), 2000);
            robot.pause(1000);
            robot.move_to(&at(cal.min_deg), 3000);
            robot.pause(1000);
        }
        robot.move_to(&home, 2000);
        robot.pause(500);
    }
}

/// Swings the given joints together to `swing_deg`, then through neutral to `-swing_deg`,
/// `times` times, and back to where they started. E.g. the two hip rolls with a negative
/// `swing_deg` sway the robot left, then right.
pub fn sway<H: Hardware>(robot: &mut Robot<H>, joints: &[Joint], swing_deg: f32, times: u32) {
    let home = *robot.current();
    let side = |deg: f32| joints.iter().fold(home, |pose, &j| pose.add(j, deg));
    alternate(robot, &side(swing_deg), &side(-swing_deg), times);
}

/// Sways the hips left and right `times` times, opening both claws after leaning left and
/// closing them after leaning right. The hips and claws take turns rather than moving at
/// once, so each current surge only powers two servos (the shared 5 V supply can't start
/// four together).
pub fn sway_and_grip<H: Hardware>(robot: &mut Robot<H>, times: u32) {
    let home = *robot.current();
    let hips = |deg: f32| home.add(Joint::RHipRoll, deg).add(Joint::LHipRoll, deg);
    let open = |pose: Pose| {
        pose.add(Joint::RGripper, JOINT_TEST_DEG)
            .add(Joint::LGripper, JOINT_TEST_DEG)
    };
    let (left, right) = (hips(-JOINT_TEST_DEG), hips(JOINT_TEST_DEG));
    for _ in 0..times {
        robot.move_to(&left, 1000);
        robot.pause(300);
        robot.move_to(&open(left), 500);
        robot.pause(300);
        robot.move_to(&open(right), 1000);
        robot.pause(300);
        robot.move_to(&right, 500);
        robot.pause(300);
    }
    robot.move_to(&home, 600);
}

/// Like `sway_and_grip` with the arms too: lean left, raise both arms, open both claws, lean
/// right, close the claws, lower the arms, `times` times. Only one pair of servos (hips,
/// shoulder pitches or grippers) moves at a time, because the shared 5 V supply can't start
/// more than about two servos at once.
pub fn sway_arms_and_grip<H: Hardware>(robot: &mut Robot<H>, times: u32) {
    let home = *robot.current();
    let pair = |a: Joint, b: Joint| move |pose: Pose, deg: f32| pose.add(a, deg).add(b, deg);
    let hips = pair(Joint::RHipRoll, Joint::LHipRoll);
    let arms_up = |pose| pair(Joint::RShoulderPitch, Joint::LShoulderPitch)(pose, JOINT_TEST_DEG);
    let open = |pose| pair(Joint::RGripper, Joint::LGripper)(pose, JOINT_TEST_DEG);
    let (left, right) = (hips(home, -JOINT_TEST_DEG), hips(home, JOINT_TEST_DEG));
    for _ in 0..times {
        for (pose, ms) in [
            (left, 1000),
            (arms_up(left), 600),
            (open(arms_up(left)), 500),
            (open(arms_up(right)), 1000),
            (arms_up(right), 500),
            (right, 600),
        ] {
            robot.move_to(&pose, ms);
            robot.pause(300);
        }
    }
    robot.move_to(&home, 600);
}

/// Moves to `first`, then back and forth between `second` and `first` so that `second` is
/// reached `times` times, then returns to the starting pose.
fn alternate<H: Hardware>(robot: &mut Robot<H>, first: &Pose, second: &Pose, times: u32) {
    let home = *robot.current();
    robot.move_to(first, 600);
    robot.pause(500);
    for i in 0..times {
        if i > 0 {
            robot.move_to(first, 1000);
            robot.pause(500);
        }
        robot.move_to(second, 1000);
        robot.pause(500);
    }
    robot.move_to(&home, 600);
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
    use crate::robot_16_dof::motion::test_support::MockHw;
    use crate::robot_16_dof::servo::{CALIBRATION, NEUTRAL_DEG};


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
    fn joint_test_moves_only_that_joint_positive_and_returns_home() {
        let mut r = robot();
        joint_test(&mut r, &[Joint::RElbow], JOINT_TEST_DEG, 3);
        let poses = &r.hardware_mut().poses;
        for pose in poses {
            for j in Joint::ALL.iter().filter(|j| **j != Joint::RElbow) {
                assert_eq!(pose.get(*j), NEUTRAL_DEG, "{j:?} moved");
            }
            assert!(pose.get(Joint::RElbow) >= NEUTRAL_DEG);
        }
        let tops = poses
            .windows(2)
            .filter(|w| {
                w[0].get(Joint::RElbow) < NEUTRAL_DEG + JOINT_TEST_DEG
                    && (w[1].get(Joint::RElbow) - (NEUTRAL_DEG + JOINT_TEST_DEG)).abs() < 1e-3
            })
            .count();
        assert_eq!(tops, 3);
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn joint_test_moves_listed_joints_together() {
        let mut r = robot();
        joint_test(&mut r, &[Joint::RHipRoll, Joint::LHipRoll], JOINT_TEST_DEG, 1);
        for pose in &r.hardware_mut().poses {
            assert_eq!(pose.get(Joint::RHipRoll), pose.get(Joint::LHipRoll));
        }
        let top = r.hardware_mut().poses.iter().map(|p| p.get(Joint::LHipRoll)).fold(0.0, f32::max);
        assert!((top - (NEUTRAL_DEG + JOINT_TEST_DEG)).abs() < 1e-3);
    }

    #[test]
    fn sway_goes_to_both_sides_times_times_and_returns_home() {
        let mut r = robot();
        sway(&mut r, &[Joint::RHipRoll, Joint::LHipRoll], -JOINT_TEST_DEG, 3);
        let hip: Vec<f32> = r.hardware_mut().poses.iter().map(|p| p.get(Joint::RHipRoll)).collect();
        let left = NEUTRAL_DEG - JOINT_TEST_DEG;
        let right = NEUTRAL_DEG + JOINT_TEST_DEG;
        // Count arrivals at each side (the value right after reaching it, ignoring repeats).
        let arrivals = |target: f32| {
            hip.windows(2)
                .filter(|w| (w[1] - target).abs() < 1e-3 && (w[0] - target).abs() >= 1e-3)
                .count()
        };
        assert_eq!(arrivals(left), 3);
        assert_eq!(arrivals(right), 3);
        assert!(hip.iter().position(|&d| (d - left).abs() < 1e-3)
            < hip.iter().position(|&d| (d - right).abs() < 1e-3));
        for pose in &r.hardware_mut().poses {
            assert_eq!(pose.get(Joint::RHipRoll), pose.get(Joint::LHipRoll));
        }
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn sway_and_grip_never_moves_hips_and_claws_at_once() {
        let mut r = robot();
        sway_and_grip(&mut r, 3);
        let poses = &r.hardware_mut().poses;
        for w in poses.windows(2) {
            let hips_move = w[0].get(Joint::RHipRoll) != w[1].get(Joint::RHipRoll);
            let claws_move = w[0].get(Joint::RGripper) != w[1].get(Joint::RGripper);
            assert!(!(hips_move && claws_move));
        }
        // Claws open only while leaning left, and open three times.
        let open = NEUTRAL_DEG + JOINT_TEST_DEG;
        let opened: Vec<_> = poses
            .windows(2)
            .filter(|w| (w[1].get(Joint::LGripper) - open).abs() < 1e-3
                && (w[0].get(Joint::LGripper) - open).abs() >= 1e-3)
            .collect();
        assert_eq!(opened.len(), 3);
        for w in opened {
            assert!((w[1].get(Joint::LHipRoll) - (NEUTRAL_DEG - JOINT_TEST_DEG)).abs() < 1e-3);
        }
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn full_range_test_reaches_both_limits_of_only_that_joint() {
        let mut r = robot();
        full_range_test(&mut r, &[Joint::RShoulderPitch], 1);
        let cal = &CALIBRATION[Joint::RShoulderPitch.index()];
        let poses = &r.hardware_mut().poses;
        let deg: Vec<f32> = poses.iter().map(|p| p.get(Joint::RShoulderPitch)).collect();
        assert!(deg.iter().any(|&d| (d - cal.max_deg).abs() < 1e-3));
        assert!(deg.iter().any(|&d| (d - cal.min_deg).abs() < 1e-3));
        for p in poses {
            assert_eq!(p.get(Joint::LShoulderPitch), NEUTRAL_DEG);
        }
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn sway_arms_and_grip_moves_one_pair_at_a_time() {
        let mut r = robot();
        sway_arms_and_grip(&mut r, 3);
        let pairs = [
            Joint::RHipRoll,
            Joint::RShoulderPitch,
            Joint::RGripper,
        ];
        for w in r.hardware_mut().poses.windows(2) {
            let moving = pairs.iter().filter(|&&j| w[0].get(j) != w[1].get(j)).count();
            assert!(moving <= 1, "{moving} pairs moved in one step");
        }
        let poses = &r.hardware_mut().poses;
        let up = NEUTRAL_DEG + JOINT_TEST_DEG;
        assert!(poses.iter().any(|p| (p.get(Joint::LShoulderPitch) - up).abs() < 1e-3));
        assert!(poses.iter().any(|p| (p.get(Joint::LGripper) - up).abs() < 1e-3));
        assert_eq!(*r.current(), Pose::neutral());
    }

    #[test]
    fn joint_test_swing_fits_every_joints_limits() {
        for cal in CALIBRATION.iter() {
            assert!(NEUTRAL_DEG + JOINT_TEST_DEG <= cal.max_deg, "channel {}", cal.channel);
            assert!(NEUTRAL_DEG - JOINT_TEST_DEG >= cal.min_deg, "channel {}", cal.channel);
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
