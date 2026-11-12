//! Turns an MQTT topic and payload into a `Command`.
//!
//! Topics, under the configured prefix (e.g. "robot"):
//!
//! | Topic                 | Payload                                                        |
//! |-----------------------|----------------------------------------------------------------|
//! | `<prefix>/servo/<j>`  | `120` or `{"angle":120,"ms":500}`                               |
//! | `<prefix>/servos`     | `{"r_hip_roll":100,"l_hip_roll":80,"ms":600}`                   |
//! | `<prefix>/led`        | `off`, `auto`, `rainbow`, a colour name, `#rrggbb` or `{"r":..,"g":..,"b":..}` |
//! | `<prefix>/cmd`        | an action word, e.g. `walk`, or `{"cmd":"walk","steps":4}`      |
//!
//! `<j>` and the `servos` keys are joint names (`r_hip_roll`) or kit servo numbers (`s4`,
//! `4`). Angles are logical degrees (90 = neutral); they're clamped to the joint's limits
//! when the move runs.

use super::json::{self, Value};
use crate::rgb_led::color::Rgb;
use crate::rgb_led::indicator::LedMode;
use crate::robot_16_dof::moves::{Arm, JOINT_TEST_DEG};
use crate::robot_16_dof::servo::Joint;

/// Subtopics the device subscribes to (under the prefix).
pub const SUBSCRIBE_TOPICS: [&str; 4] = ["servo/+", "servos", "led", "cmd"];
/// Subtopic the device publishes its status on. Not under any subscribed topic.
pub const STATUS_TOPIC: &str = "status";

/// Longer payloads are rejected without parsing.
pub const MAX_PAYLOAD_BYTES: usize = 512;

pub const DEFAULT_MOVE_MS: u32 = 500;
pub const MIN_MOVE_MS: u32 = 100;
pub const MAX_MOVE_MS: u32 = 10_000;
pub const DEFAULT_WALK_STEPS: u32 = 4;
pub const MAX_WALK_STEPS: u32 = 20;
pub const DEFAULT_TIMES: u32 = 3;
pub const MAX_TIMES: u32 = 10;
/// Largest swing a `joint_test` may ask for, either way.
pub const MAX_JOINT_TEST_DEG: f32 = 60.0;
/// Brightness of the named LED colours. `#rrggbb` and `{"r","g","b"}` are sent as given.
pub const NAMED_COLOUR_BRIGHTNESS: u8 = 64;

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    /// Something that moves the robot. Rejected while another one is running.
    Robot(RobotCommand),
    /// Changes the LED. Always applied at once.
    Led(LedMode),
    /// Stops the running robot command where it is.
    Stop,
}

#[derive(Clone, Debug, PartialEq)]
pub enum RobotCommand {
    /// Moves the listed joints to absolute angles, two servos at a time.
    Move { targets: Vec<(Joint, f32)>, ms: u32 },
    Sway { times: u32 },
    /// Hips sway left and right, claws opening and closing.
    SwayGrip { times: u32 },
    /// Like `SwayGrip` with the arms raised and lowered too.
    SwayArms { times: u32 },
    /// Raises and lowers one or both arms.
    Arms { arm: Arm, times: u32 },
    Walk { steps: u32 },
    StandUp,
    StandBy,
    Dance { times: u32 },
    /// Cuts the servo pulses so the robot goes limp.
    Release,
    SelfTest,
    JointTest { joints: Vec<Joint>, deg: f32, times: u32 },
    FullRange { joints: Vec<Joint>, times: u32 },
}

impl RobotCommand {
    /// Name used in status messages.
    pub fn name(&self) -> &'static str {
        match self {
            RobotCommand::Move { targets, .. } if targets.len() == 1 => "servo",
            RobotCommand::Move { .. } => "servos",
            RobotCommand::Sway { .. } => "sway",
            RobotCommand::SwayGrip { .. } => "sway_grip",
            RobotCommand::SwayArms { .. } => "sway_arms",
            RobotCommand::Arms { .. } => "arms",
            RobotCommand::Walk { .. } => "walk",
            RobotCommand::StandUp => "stand_up",
            RobotCommand::StandBy => "stand_by",
            RobotCommand::Dance { .. } => "dance",
            RobotCommand::Release => "release",
            RobotCommand::SelfTest => "self_test",
            RobotCommand::JointTest { .. } => "joint_test",
            RobotCommand::FullRange { .. } => "full_range",
        }
    }
}

/// `None` if `topic` isn't under `prefix/`; otherwise the parsed command or why it was
/// rejected.
pub fn parse(prefix: &str, topic: &str, payload: &[u8]) -> Option<Result<Command, String>> {
    let sub = topic.strip_prefix(prefix)?.strip_prefix('/')?;
    Some(parse_sub(sub, payload))
}

fn parse_sub(sub: &str, payload: &[u8]) -> Result<Command, String> {
    if payload.len() > MAX_PAYLOAD_BYTES {
        return Err(format!("payload longer than {MAX_PAYLOAD_BYTES} bytes"));
    }
    let text = std::str::from_utf8(payload)
        .map_err(|_| "payload is not UTF-8".to_string())?
        .trim();
    if let Some(joint) = sub.strip_prefix("servo/") {
        return parse_servo(joint, text);
    }
    match sub {
        "servos" => parse_servos(text),
        "led" => parse_led(text).map(Command::Led),
        "cmd" => parse_cmd(text),
        _ => Err(format!("unknown topic '{sub}'")),
    }
}

fn joint(name: &str) -> Result<Joint, String> {
    Joint::parse(name).ok_or_else(|| format!("unknown joint '{name}'"))
}

fn angle(v: &Value, what: &str) -> Result<f32, String> {
    v.as_f64()
        .map(|n| n as f32)
        .ok_or_else(|| format!("{what} must be a number of degrees"))
}

/// A whole number in `lo..=hi`.
fn int_in(v: &Value, key: &str, lo: u32, hi: u32) -> Result<u32, String> {
    match v.as_f64() {
        Some(n) if n.fract() == 0.0 && n >= lo as f64 && n <= hi as f64 => Ok(n as u32),
        _ => Err(format!("'{key}' must be a whole number from {lo} to {hi}")),
    }
}

fn move_ms(v: &Value) -> Result<u32, String> {
    int_in(v, "ms", MIN_MOVE_MS, MAX_MOVE_MS)
}

/// The fields of a JSON object, rejecting any key not in `allowed`.
fn object(text: &str, allowed: &[&str]) -> Result<Vec<(String, Value)>, String> {
    let Value::Obj(fields) = json::parse(text)? else {
        return Err("expected a JSON object".into());
    };
    if let Some((key, _)) = fields.iter().find(|(k, _)| !allowed.contains(&k.as_str())) {
        return Err(format!("unexpected key '{key}'"));
    }
    Ok(fields)
}

fn field<'a>(fields: &'a [(String, Value)], key: &str) -> Option<&'a Value> {
    fields.iter().find(|(k, _)| k == key).map(|(_, v)| v)
}

fn parse_servo(name: &str, text: &str) -> Result<Command, String> {
    let joint = joint(name)?;
    let (deg, ms) = if text.starts_with('{') {
        let fields = object(text, &["angle", "ms"])?;
        let deg = angle(field(&fields, "angle").ok_or("missing 'angle'")?, "'angle'")?;
        let ms = field(&fields, "ms").map(move_ms).transpose()?;
        (deg, ms.unwrap_or(DEFAULT_MOVE_MS))
    } else {
        (angle(&json::parse(text)?, "payload")?, DEFAULT_MOVE_MS)
    };
    Ok(Command::Robot(RobotCommand::Move { targets: vec![(joint, deg)], ms }))
}

fn parse_servos(text: &str) -> Result<Command, String> {
    let Value::Obj(fields) = json::parse(text)? else {
        return Err("expected a JSON object of joint: angle".into());
    };
    let mut targets: Vec<(Joint, f32)> = Vec::new();
    let mut ms = DEFAULT_MOVE_MS;
    for (key, value) in &fields {
        if key == "ms" {
            ms = move_ms(value)?;
            continue;
        }
        let j = joint(key)?;
        if targets.iter().any(|(t, _)| *t == j) {
            return Err(format!("joint '{}' given twice", j.name()));
        }
        targets.push((j, angle(value, key)?));
    }
    if targets.is_empty() {
        return Err("no joints given".into());
    }
    Ok(Command::Robot(RobotCommand::Move { targets, ms }))
}

fn parse_led(text: &str) -> Result<LedMode, String> {
    if text.starts_with('{') {
        let fields = object(text, &["r", "g", "b"])?;
        let channel = |key: &str| {
            field(&fields, key)
                .ok_or_else(|| format!("missing '{key}'"))
                .and_then(|v| int_in(v, key, 0, 255))
                .map(|n| n as u8)
        };
        return Ok(LedMode::Solid((channel("r")?, channel("g")?, channel("b")?)));
    }
    let word = text.trim_matches('"').to_ascii_lowercase();
    if let Some(hex) = word.strip_prefix('#') {
        return parse_hex(hex).map(LedMode::Solid);
    }
    let b = NAMED_COLOUR_BRIGHTNESS;
    Ok(match word.as_str() {
        "auto" => LedMode::Auto,
        "off" => LedMode::Off,
        "rainbow" => LedMode::Rainbow,
        "red" => LedMode::Solid((b, 0, 0)),
        "green" => LedMode::Solid((0, b, 0)),
        "blue" => LedMode::Solid((0, 0, b)),
        "yellow" => LedMode::Solid((b, b * 3 / 4, 0)),
        "cyan" => LedMode::Solid((0, b, b)),
        "magenta" | "purple" => LedMode::Solid((b, 0, b)),
        "orange" => LedMode::Solid((b, b / 3, 0)),
        "white" => LedMode::Solid((b, b, b)),
        _ => return Err(format!("unknown LED setting '{word}'")),
    })
}

fn parse_hex(hex: &str) -> Result<Rgb, String> {
    let bad = || format!("bad colour '#{hex}', expected #rrggbb");
    if hex.len() != 6 || !hex.is_ascii() {
        return Err(bad());
    }
    let byte = |i: usize| u8::from_str_radix(&hex[i..i + 2], 16).map_err(|_| bad());
    Ok((byte(0)?, byte(2)?, byte(4)?))
}

fn parse_cmd(text: &str) -> Result<Command, String> {
    let (name, fields) = if text.starts_with('{') {
        let fields = object(text, &["cmd", "steps", "times", "joints", "deg", "arm"])?;
        let name = field(&fields, "cmd")
            .and_then(Value::as_str)
            .ok_or("missing 'cmd'")?
            .to_string();
        (name, fields)
    } else {
        (text.trim_matches('"').to_string(), Vec::new())
    };
    // "stand_by", "standby" and "stand-by" all mean the same.
    let key: String = name
        .to_ascii_lowercase()
        .chars()
        .filter(|c| *c != '_' && *c != '-')
        .collect();

    let allowed: &[&str] = match key.as_str() {
        "walk" => &["cmd", "steps"],
        "sway" | "swaygrip" | "swayarms" | "dance" => &["cmd", "times"],
        "arms" => &["cmd", "arm", "times"],
        "jointtest" => &["cmd", "joints", "deg", "times"],
        "fullrange" => &["cmd", "joints", "times"],
        _ => &["cmd"],
    };
    if let Some((k, _)) = fields.iter().find(|(k, _)| !allowed.contains(&k.as_str())) {
        return Err(format!("'{k}' doesn't apply to '{name}'"));
    }
    let times = || {
        field(&fields, "times")
            .map(|v| int_in(v, "times", 1, MAX_TIMES))
            .unwrap_or(Ok(DEFAULT_TIMES))
    };
    let joints = || -> Result<Vec<Joint>, String> {
        let Some(Value::Arr(items)) = field(&fields, "joints") else {
            return Err("'joints' must be a list of joint names".into());
        };
        if items.is_empty() {
            return Err("'joints' is empty".into());
        }
        items
            .iter()
            .map(|v| v.as_str().ok_or("'joints' must be a list of joint names".to_string()))
            .map(|name| name.and_then(joint))
            .collect()
    };

    let robot = match key.as_str() {
        "stop" => return Ok(Command::Stop),
        "rainbow" => return Ok(Command::Led(LedMode::Rainbow)),
        "sway" => RobotCommand::Sway { times: times()? },
        "swaygrip" => RobotCommand::SwayGrip { times: times()? },
        "swayarms" => RobotCommand::SwayArms { times: times()? },
        "arms" => {
            let arm = match field(&fields, "arm").map(|v| v.as_str()) {
                None | Some(Some("both")) => Arm::Both,
                Some(Some("left")) => Arm::Left,
                Some(Some("right")) => Arm::Right,
                _ => return Err("'arm' must be \"left\", \"right\" or \"both\"".into()),
            };
            RobotCommand::Arms { arm, times: times()? }
        }
        "walk" => RobotCommand::Walk {
            steps: field(&fields, "steps")
                .map(|v| int_in(v, "steps", 1, MAX_WALK_STEPS))
                .unwrap_or(Ok(DEFAULT_WALK_STEPS))?,
        },
        "standup" => RobotCommand::StandUp,
        "standby" => RobotCommand::StandBy,
        "dance" => RobotCommand::Dance { times: times()? },
        "release" => RobotCommand::Release,
        "selftest" => RobotCommand::SelfTest,
        "jointtest" => {
            let deg = match field(&fields, "deg") {
                Some(v) => angle(v, "'deg'")?,
                None => JOINT_TEST_DEG,
            };
            if deg.abs() > MAX_JOINT_TEST_DEG {
                return Err(format!("'deg' must be within +-{MAX_JOINT_TEST_DEG}"));
            }
            RobotCommand::JointTest { joints: joints()?, deg, times: times()? }
        }
        "fullrange" => RobotCommand::FullRange { joints: joints()?, times: times()? },
        _ => return Err(format!("unknown command '{name}'")),
    };
    Ok(Command::Robot(robot))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(topic: &str, payload: &str) -> Result<Command, String> {
        parse("robot", topic, payload.as_bytes()).expect("topic under the prefix")
    }

    fn robot(topic: &str, payload: &str) -> RobotCommand {
        match p(topic, payload) {
            Ok(Command::Robot(r)) => r,
            other => panic!("{topic} {payload}: {other:?}"),
        }
    }

    #[test]
    fn ignores_topics_outside_the_prefix() {
        assert_eq!(parse("robot", "other/cmd", b"walk"), None);
        assert_eq!(parse("robot", "robotx/cmd", b"walk"), None);
        assert!(p("robot/status", "x").is_err());
    }

    #[test]
    fn single_servo_by_name_or_number() {
        assert_eq!(
            robot("robot/servo/r_hip_roll", "100"),
            RobotCommand::Move { targets: vec![(Joint::RHipRoll, 100.0)], ms: DEFAULT_MOVE_MS }
        );
        assert_eq!(
            robot("robot/servo/s25", r#"{"angle": 60.5, "ms": 1500}"#),
            RobotCommand::Move { targets: vec![(Joint::LGripper, 60.5)], ms: 1500 }
        );
        assert_eq!(robot("robot/servo/4", "90").name(), "servo");
    }

    #[test]
    fn single_servo_rejects_bad_input() {
        for (topic, payload) in [
            ("robot/servo/knee", "90"),
            ("robot/servo/s4", "ninety"),
            ("robot/servo/s4", ""),
            ("robot/servo/s4", r#"{"ms": 500}"#),
            ("robot/servo/s4", r#"{"angle": 90, "ms": 5}"#),
            ("robot/servo/s4", r#"{"angle": 90, "speed": 5}"#),
        ] {
            assert!(p(topic, payload).is_err(), "{topic} {payload}");
        }
    }

    #[test]
    fn several_servos_keep_their_order() {
        assert_eq!(
            robot("robot/servos", r#"{"r_hip_roll": 100, "s29": 80, "ms": 600}"#),
            RobotCommand::Move {
                targets: vec![(Joint::RHipRoll, 100.0), (Joint::LHipRoll, 80.0)],
                ms: 600
            }
        );
    }

    #[test]
    fn several_servos_reject_bad_input() {
        for payload in [
            "{}",
            r#"{"ms": 600}"#,
            r#"{"s4": 90, "r_hip_roll": 80}"#,
            r#"{"s4": "up"}"#,
            r#"{"nose": 90}"#,
            "[1, 2]",
        ] {
            assert!(p("robot/servos", payload).is_err(), "{payload}");
        }
    }

    #[test]
    fn led_settings() {
        let led = |payload| match p("robot/led", payload) {
            Ok(Command::Led(m)) => m,
            other => panic!("{payload}: {other:?}"),
        };
        assert_eq!(led("off"), LedMode::Off);
        assert_eq!(led("Auto"), LedMode::Auto);
        assert_eq!(led("rainbow"), LedMode::Rainbow);
        assert_eq!(led("#FF8000"), LedMode::Solid((255, 128, 0)));
        assert_eq!(led(r#"{"r": 1, "g": 2, "b": 3}"#), LedMode::Solid((1, 2, 3)));
        assert_eq!(led("red"), LedMode::Solid((NAMED_COLOUR_BRIGHTNESS, 0, 0)));
        for bad in ["#12345", "#gg0000", "pink", r#"{"r": 256, "g": 0, "b": 0}"#, r#"{"r": 1}"#] {
            assert!(p("robot/led", bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn action_words_with_defaults() {
        assert_eq!(robot("robot/cmd", "sway"), RobotCommand::Sway { times: DEFAULT_TIMES });
        assert_eq!(robot("robot/cmd", "walk"), RobotCommand::Walk { steps: DEFAULT_WALK_STEPS });
        assert_eq!(robot("robot/cmd", "standup"), RobotCommand::StandUp);
        assert_eq!(robot("robot/cmd", "stand_up"), RobotCommand::StandUp);
        assert_eq!(robot("robot/cmd", "stand_by"), RobotCommand::StandBy);
        assert_eq!(robot("robot/cmd", "STANDBY"), RobotCommand::StandBy);
        assert_eq!(robot("robot/cmd", "dance"), RobotCommand::Dance { times: DEFAULT_TIMES });
        assert_eq!(robot("robot/cmd", "release"), RobotCommand::Release);
        assert_eq!(robot("robot/cmd", "sway_grip"), RobotCommand::SwayGrip { times: DEFAULT_TIMES });
        assert_eq!(robot("robot/cmd", "sway_arms"), RobotCommand::SwayArms { times: DEFAULT_TIMES });
        assert_eq!(
            robot("robot/cmd", r#"{"cmd":"arms","times":2}"#),
            RobotCommand::Arms { arm: Arm::Both, times: 2 }
        );
        assert_eq!(
            robot("robot/cmd", r#"{"cmd":"arms","arm":"left"}"#),
            RobotCommand::Arms { arm: Arm::Left, times: DEFAULT_TIMES }
        );
        assert!(p("robot/cmd", r#"{"cmd":"arms","arm":"middle"}"#).is_err());
        assert_eq!(robot("robot/cmd", "self_test"), RobotCommand::SelfTest);
        assert_eq!(p("robot/cmd", "stop"), Ok(Command::Stop));
        assert_eq!(p("robot/cmd", "rainbow"), Ok(Command::Led(LedMode::Rainbow)));
        assert!(p("robot/cmd", "fly").is_err());
    }

    #[test]
    fn action_objects_with_parameters() {
        assert_eq!(
            robot("robot/cmd", r#"{"cmd": "walk", "steps": 6}"#),
            RobotCommand::Walk { steps: 6 }
        );
        assert_eq!(
            robot("robot/cmd", r#"{"cmd": "dance", "times": 1}"#),
            RobotCommand::Dance { times: 1 }
        );
        assert_eq!(
            robot("robot/cmd", r#"{"cmd": "joint_test", "joints": ["s4", "l_hip_roll"], "deg": -20}"#),
            RobotCommand::JointTest {
                joints: vec![Joint::RHipRoll, Joint::LHipRoll],
                deg: -20.0,
                times: DEFAULT_TIMES
            }
        );
        assert_eq!(
            robot("robot/cmd", r#"{"cmd": "full_range", "joints": ["r_elbow"], "times": 1}"#),
            RobotCommand::FullRange { joints: vec![Joint::RElbow], times: 1 }
        );
    }

    #[test]
    fn action_objects_reject_bad_parameters() {
        for payload in [
            r#"{"steps": 4}"#,
            r#"{"cmd": "walk", "steps": 0}"#,
            r#"{"cmd": "walk", "steps": 2.5}"#,
            r#"{"cmd": "walk", "steps": 99}"#,
            r#"{"cmd": "walk", "times": 2}"#,
            r#"{"cmd": "dance", "times": 11}"#,
            r#"{"cmd": "joint_test"}"#,
            r#"{"cmd": "joint_test", "joints": []}"#,
            r#"{"cmd": "joint_test", "joints": ["s4"], "deg": 90}"#,
            r#"{"cmd": "full_range", "joints": ["s99"]}"#,
            r#"{"cmd": "full_range", "joints": "s4"}"#,
        ] {
            assert!(p("robot/cmd", payload).is_err(), "{payload}");
        }
    }

    #[test]
    fn rejects_oversized_or_binary_payloads() {
        let long = "1".repeat(MAX_PAYLOAD_BYTES + 1);
        assert!(p("robot/servo/s4", &long).is_err());
        assert!(parse("robot", "robot/cmd", &[0xff, 0xfe]).unwrap().is_err());
    }
}
