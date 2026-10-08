//! Poses and smooth movement between them.
//!
//! Hardware-independent: all output goes through the `Hardware` trait, so the moves can
//! be run against a mock on a PC.

use crate::servo::{Joint, NEUTRAL_DEG, NUM_JOINTS};

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
}

/// Ease in and out: starts and ends slowly so the servos and gears are not jerked.
fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub struct Robot<H: Hardware> {
    hw: H,
    current: Pose,
}

impl<H: Hardware> Robot<H> {
    pub fn new(hw: H) -> Self {
        Self {
            hw,
            current: Pose::neutral(),
        }
    }

    pub fn current(&self) -> &Pose {
        &self.current
    }

    pub fn hardware_mut(&mut self) -> &mut H {
        &mut self.hw
    }

    /// Jumps straight to a pose with no interpolation. Used once at startup, because the
    /// servos' real positions are unknown until they are first commanded.
    pub fn snap_to(&mut self, pose: &Pose) {
        self.hw.write_pose(pose);
        self.current = *pose;
    }

    /// Moves smoothly from the current pose to `target` over `duration_ms`.
    pub fn move_to(&mut self, target: &Pose, duration_ms: u32) {
        let steps = (duration_ms / STEP_MS).max(1);
        let start = self.current;
        for i in 1..=steps {
            let t = smoothstep(i as f32 / steps as f32);
            let frame = start.lerp(target, t);
            self.hw.write_pose(&frame);
            self.hw.delay_ms(STEP_MS);
        }
        self.current = *target;
    }

    pub fn pause(&mut self, ms: u32) {
        self.hw.delay_ms(ms);
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::*;

    /// Records every pose written and the total time spent in delays.
    #[derive(Default)]
    pub struct MockHw {
        pub poses: Vec<Pose>,
        pub elapsed_ms: u64,
    }

    impl Hardware for MockHw {
        fn write_pose(&mut self, pose: &Pose) {
            self.poses.push(*pose);
        }
        fn delay_ms(&mut self, ms: u32) {
            self.elapsed_ms += ms as u64;
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
    fn add_is_relative_and_with_is_absolute() {
        let p = Pose::neutral().with(Joint::RKneeUpper, 20.0).add(Joint::RKneeUpper, 5.0);
        assert_eq!(p.get(Joint::RKneeUpper), 115.0);
    }
}
