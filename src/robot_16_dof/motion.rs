//! Poses and smooth movement between them.
//!
//! Hardware-independent: all output goes through the `Hardware` trait, so the moves can
//! be run against a mock on a PC.

use crate::robot_16_dof::servo::{Joint, NEUTRAL_DEG, NUM_JOINTS};

/// Time between servo updates while moving.
pub const STEP_MS: u32 = 20;

/// One angle (logical degrees, 90 = neutral) per joint, indexed by `Joint::index()`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose(pub [f32; NUM_JOINTS]);

impl Pose {
    pub const fn neutral() -> Self {
        Pose([NEUTRAL_DEG; NUM_JOINTS])
    }

    /// Sets a joint to `offset_deg` away from neutral.
    pub fn with(mut self, joint: Joint, offset_deg: f32) -> Self {
        self.0[joint.index()] = NEUTRAL_DEG + offset_deg;
        self
    }

    /// Moves a joint by `delta_deg` from wherever it currently is in this pose.
    pub fn add(mut self, joint: Joint, delta_deg: f32) -> Self {
        self.0[joint.index()] += delta_deg;
        self
    }

    #[allow(dead_code)] // used by the tests; kept for writing moves
    pub fn get(&self, joint: Joint) -> f32 {
        self.0[joint.index()]
    }

    pub fn lerp(&self, other: &Pose, t: f32) -> Pose {
        let mut out = [0.0f32; NUM_JOINTS];
        for i in 0..NUM_JOINTS {
            out[i] = self.0[i] + (other.0[i] - self.0[i]) * t;
        }
        Pose(out)
    }
}

/// What the motion engine needs from the real (or mock) robot.
pub trait Hardware {
    fn write_pose(&mut self, pose: &Pose);
    fn delay_ms(&mut self, ms: u32);
    /// Checked before every step; returning true stops the current move (see `Robot`).
    fn should_abort(&mut self) -> bool {
        false
    }
}

/// Ease in and out: starts and ends slowly so the servos and gears are not jerked.
fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Tracks the current pose and moves smoothly between poses.
///
/// Once `Hardware::should_abort` returns true, the robot stops where it is, and every later
/// `move_to` and `pause` returns straight away until `clear_abort`, so a long routine made of
/// many moves ends within one step.
pub struct Robot<H: Hardware> {
    hw: H,
    current: Pose,
    aborted: bool,
}

impl<H: Hardware> Robot<H> {
    pub fn new(hw: H) -> Self {
        Self {
            hw,
            current: Pose::neutral(),
            aborted: false,
        }
    }

    pub fn current(&self) -> &Pose {
        &self.current
    }

    pub fn hardware_mut(&mut self) -> &mut H {
        &mut self.hw
    }

    /// True once a move has been stopped by `Hardware::should_abort`.
    pub fn aborted(&self) -> bool {
        self.aborted
    }

    pub fn clear_abort(&mut self) {
        self.aborted = false;
    }

    /// Checks for an abort request, remembering it.
    fn check_abort(&mut self) -> bool {
        if !self.aborted && self.hw.should_abort() {
            self.aborted = true;
        }
        self.aborted
    }

    /// Jumps straight to a pose with no interpolation. Used once at startup, because the
    /// servos' real positions are unknown until they are first commanded.
    pub fn snap_to(&mut self, pose: &Pose) {
        self.hw.write_pose(pose);
        self.current = *pose;
    }

    /// Moves smoothly from the current pose to `target` over `duration_ms`.
    /// Stops early, holding the last written frame, if an abort is requested.
    pub fn move_to(&mut self, target: &Pose, duration_ms: u32) {
        let steps = (duration_ms / STEP_MS).max(1);
        let start = self.current;
        for i in 1..=steps {
            if self.check_abort() {
                return;
            }
            let t = smoothstep(i as f32 / steps as f32);
            let frame = start.lerp(target, t);
            self.hw.write_pose(&frame);
            self.current = frame;
            self.hw.delay_ms(STEP_MS);
        }
        self.current = *target;
    }

    /// Waits in `STEP_MS` slices so an abort request ends the wait early.
    pub fn pause(&mut self, ms: u32) {
        let mut left = ms;
        while left > 0 && !self.check_abort() {
            let slice = left.min(STEP_MS);
            self.hw.delay_ms(slice);
            left -= slice;
        }
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::*;

    /// Records every pose written and the total time spent in delays. With
    /// `abort_after_writes` set, asks for an abort once that many poses have been written.
    #[derive(Default)]
    pub struct MockHw {
        pub poses: Vec<Pose>,
        pub elapsed_ms: u64,
        pub abort_after_writes: Option<usize>,
    }

    impl Hardware for MockHw {
        fn write_pose(&mut self, pose: &Pose) {
            self.poses.push(*pose);
        }
        fn delay_ms(&mut self, ms: u32) {
            self.elapsed_ms += ms as u64;
        }
        fn should_abort(&mut self) -> bool {
            self.abort_after_writes.is_some_and(|n| self.poses.len() >= n)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::MockHw;
    use super::*;

    #[test]
    fn smoothstep_endpoints_and_midpoint() {
        assert_eq!(smoothstep(0.0), 0.0);
        assert_eq!(smoothstep(1.0), 1.0);
        assert!((smoothstep(0.5) - 0.5).abs() < 1e-6);
        assert_eq!(smoothstep(-3.0), 0.0);
        assert_eq!(smoothstep(7.0), 1.0);
    }

    #[test]
    fn move_to_ends_exactly_on_target_and_takes_the_requested_time() {
        let mut robot = Robot::new(MockHw::default());
        let target = Pose::neutral().with(Joint::RGripper, 40.0);
        robot.move_to(&target, 400);
        assert_eq!(robot.hardware_mut().poses.len(), 20);
        assert_eq!(*robot.hardware_mut().poses.last().unwrap(), target);
        assert_eq!(robot.hardware_mut().elapsed_ms, 400);
        assert_eq!(*robot.current(), target);
    }

    #[test]
    fn motion_is_monotonic_with_no_overshoot() {
        let mut robot = Robot::new(MockHw::default());
        let target = Pose::neutral().with(Joint::RKneeUpper, 30.0);
        robot.move_to(&target, 600);
        let values: Vec<f32> = robot
            .hardware_mut()
            .poses
            .iter()
            .map(|p| p.get(Joint::RKneeUpper))
            .collect();
        for w in values.windows(2) {
            assert!(w[1] >= w[0]);
        }
        assert!(values.iter().all(|v| *v >= 90.0 && *v <= 120.0));
    }

    #[test]
    fn very_short_move_still_writes_at_least_once() {
        let mut robot = Robot::new(MockHw::default());
        let target = Pose::neutral().with(Joint::LGripper, 10.0);
        robot.move_to(&target, 1);
        assert_eq!(robot.hardware_mut().poses.len(), 1);
        assert_eq!(*robot.hardware_mut().poses.last().unwrap(), target);
    }

    #[test]
    fn abort_stops_mid_move_and_skips_later_moves_and_pauses() {
        let mut robot = Robot::new(MockHw {
            abort_after_writes: Some(5),
            ..Default::default()
        });
        let target = Pose::neutral().with(Joint::RGripper, 40.0);
        robot.move_to(&target, 400);
        assert!(robot.aborted());
        assert_eq!(robot.hardware_mut().poses.len(), 5);
        // The current pose is the last frame written, partway to the target.
        let held = *robot.hardware_mut().poses.last().unwrap();
        assert_eq!(*robot.current(), held);
        assert!(held.get(Joint::RGripper) > 90.0 && held.get(Joint::RGripper) < 130.0);

        let elapsed = robot.hardware_mut().elapsed_ms;
        robot.pause(1000);
        robot.move_to(&Pose::neutral(), 400);
        assert_eq!(robot.hardware_mut().poses.len(), 5);
        assert_eq!(robot.hardware_mut().elapsed_ms, elapsed);
    }

    #[test]
    fn clear_abort_lets_the_robot_move_again() {
        let mut robot = Robot::new(MockHw {
            abort_after_writes: Some(1),
            ..Default::default()
        });
        robot.move_to(&Pose::neutral().with(Joint::RElbow, 20.0), 200);
        assert!(robot.aborted());
        robot.hardware_mut().abort_after_writes = None;
        robot.clear_abort();
        robot.move_to(&Pose::neutral(), 200);
        assert!(!robot.aborted());
        assert_eq!(*robot.current(), Pose::neutral());
    }

    #[test]
    fn pause_takes_the_full_time() {
        let mut robot = Robot::new(MockHw::default());
        robot.pause(1234);
        assert_eq!(robot.hardware_mut().elapsed_ms, 1234);
    }

    #[test]
    fn add_is_relative_and_with_is_absolute() {
        let p = Pose::neutral().with(Joint::RKneeUpper, 20.0).add(Joint::RKneeUpper, 5.0);
        assert_eq!(p.get(Joint::RKneeUpper), 115.0);
    }
}
