//! The JSON the device publishes on `<prefix>/pose`, and how often.
//!
//! While the robot moves it writes a frame every `STEP_MS` (20 ms); publishing every one
//! would flood the broker, so `Throttle` passes one frame per `POSE_INTERVAL_MS`. The final
//! pose of each command is published separately, so a listener always ends on where the
//! robot actually stopped.

use crate::robot_16_dof::motion::{Pose, STEP_MS};
use crate::robot_16_dof::servo::{clamp_to_limits, Joint};

/// Time between pose messages while moving (10 Hz).
pub const POSE_INTERVAL_MS: u32 = 100;

const FRAMES_PER_MESSAGE: u32 = POSE_INTERVAL_MS / STEP_MS;

/// E.g. `{"r_ankle_roll":90.0,"r_knee_lower":75.5,...}`: one key per joint in `Joint::ALL`
/// order, logical degrees to 0.1, clamped to the calibrated limits as the servo receives them.
pub fn json(pose: &Pose) -> String {
    let fields: Vec<String> = Joint::ALL
        .iter()
        .map(|&j| format!("\"{}\":{:.1}", j.name(), clamp_to_limits(j, pose.get(j))))
        .collect();
    format!("{{{}}}", fields.join(","))
}

/// Passes one servo frame in every `POSE_INTERVAL_MS / STEP_MS`.
#[derive(Debug, Default)]
pub struct Throttle {
    frames: u32,
}

impl Throttle {
    /// Call once per written frame; true when this frame should be published.
    pub fn tick(&mut self) -> bool {
        self.frames += 1;
        if self.frames >= FRAMES_PER_MESSAGE {
            self.frames = 0;
            true
        } else {
            false
        }
    }

    /// Starts counting afresh, e.g. after the final pose of a command was published.
    pub fn reset(&mut self) {
        self.frames = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::remote::json::{parse, Value};

    #[test]
    fn json_has_every_joint_in_order() {
        let text = json(&Pose::neutral().with(Joint::RHipRoll, 10.25));
        let Value::Obj(fields) = parse(&text).unwrap() else { panic!("{text}") };
        let names: Vec<&str> = fields.iter().map(|(k, _)| k.as_str()).collect();
        let expected: Vec<&str> = Joint::ALL.iter().map(|j| j.name()).collect();
        assert_eq!(names, expected);
        assert!(text.contains("\"r_hip_roll\":100.2") || text.contains("\"r_hip_roll\":100.3"));
        assert!(text.contains("\"l_ankle_roll\":90.0"));
    }

    #[test]
    fn json_clamps_to_limits() {
        let text = json(&Pose::neutral().with(Joint::RElbow, 500.0));
        let Value::Obj(fields) = parse(&text).unwrap() else { panic!("{text}") };
        let (_, Value::Num(deg)) = &fields[Joint::RElbow.index()] else { panic!("{text}") };
        let limit = clamp_to_limits(Joint::RElbow, 500.0) as f64;
        assert!((deg - limit).abs() < 0.06, "{deg} vs {limit}");
    }

    #[test]
    fn throttle_passes_one_frame_per_interval() {
        let mut t = Throttle::default();
        let passed: Vec<bool> = (0..2 * FRAMES_PER_MESSAGE).map(|_| t.tick()).collect();
        assert_eq!(passed.iter().filter(|&&p| p).count(), 2);
        assert!(passed[FRAMES_PER_MESSAGE as usize - 1]);
        t.tick();
        t.reset();
        assert_eq!((1..FRAMES_PER_MESSAGE).filter(|_| t.tick()).count(), 0);
        assert!(t.tick());
    }
}
