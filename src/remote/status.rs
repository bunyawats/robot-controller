//! The JSON the device publishes on `<prefix>/status`.

use super::json::quote;

/// What the robot is doing, as reported in every status message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// "idle", "busy", "standby", or "pca9685_not_found" when the servo board didn't answer
    /// at boot (robot commands are then rejected).
    pub state: &'static str,
    /// The running robot command, or the one a "done"/"stopped" message is about.
    pub action: Option<&'static str>,
}

impl Snapshot {
    pub const IDLE: Snapshot = Snapshot { state: "idle", action: None };
    pub const NO_SERVOS: Snapshot = Snapshot { state: "pca9685_not_found", action: None };
}

/// E.g. `{"state":"busy","action":"walk","reason":"started","error":null}`.
///
/// `reason` says why the message was sent (connected, heartbeat, started, done, stopped,
/// stopping, led, rejected_busy, rejected_invalid, ...); `error` explains a rejection.
pub fn json(snapshot: Snapshot, reason: &str, error: Option<&str>) -> String {
    let opt = |s: Option<&str>| s.map_or("null".to_string(), quote);
    format!(
        r#"{{"state":{},"action":{},"reason":{},"error":{}}}"#,
        quote(snapshot.state),
        opt(snapshot.action),
        quote(reason),
        opt(error)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::remote::json::{parse, Value};

    #[test]
    fn formats_valid_json() {
        let busy = Snapshot { state: "busy", action: Some("walk") };
        assert_eq!(
            json(busy, "started", None),
            r#"{"state":"busy","action":"walk","reason":"started","error":null}"#
        );
        let text = json(Snapshot::IDLE, "rejected_invalid", Some(r#"unknown joint "x""#));
        let Value::Obj(fields) = parse(&text).unwrap() else { panic!("{text}") };
        assert_eq!(fields[1], ("action".into(), Value::Null));
        assert_eq!(fields[3], ("error".into(), Value::Str(r#"unknown joint "x""#.into())));
    }
}
