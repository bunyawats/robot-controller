//! What the status LED shows, worked out from messages about the robot's state.
//!
//! Pure logic: the ESP side only feeds it `LedMsg`s and writes `colour()` to the LED.
//!
//! Priority, highest first: a servo write fault (red, stays red), a colour set over MQTT
//! (`LedMode::Off`/`Solid`/`Rainbow`, until `LedMode::Auto`), then the automatic status
//! colour: red when the servo board wasn't found at boot, yellow while busy, blue while
//! booting or not connected to the broker, dim blue in stand-by, green when idle.

use super::color::{self, Rgb};

/// Status colours. Kept dim: the LED is very bright at full power.
pub const BOOTING: Rgb = (0, 0, 32);
pub const DISCONNECTED: Rgb = (0, 0, 32);
pub const IDLE: Rgb = (0, 32, 0);
pub const BUSY: Rgb = (32, 24, 0);
pub const STANDBY: Rgb = (0, 0, 8);
pub const FAULT: Rgb = (48, 0, 0);
pub const NO_SERVOS: Rgb = FAULT;

/// Peak brightness of the rainbow fade.
pub const RAINBOW_BRIGHTNESS: u8 = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Activity {
    Booting,
    Idle,
    Busy,
    Standby,
    /// The PCA9685 didn't answer at boot, so only MQTT and the LED work. Unlike `Fault`, a
    /// colour set over MQTT still shows, so the LED can be tested without the servo board.
    NoServos,
}

/// What the LED was told to show over MQTT.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LedMode {
    /// Show the robot's status colour.
    Auto,
    Off,
    Solid(Rgb),
    /// Fade around the colour wheel.
    Rainbow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LedMsg {
    Activity(Activity),
    /// Whether the MQTT connection (with subscriptions) is up.
    Connected(bool),
    Mode(LedMode),
    /// A servo write failed. Turns the LED red for good.
    Fault,
}

#[derive(Debug)]
pub struct Indicator {
    activity: Activity,
    connected: bool,
    mode: LedMode,
    fault: bool,
    hue: u32,
}

impl Default for Indicator {
    fn default() -> Self {
        Self::new()
    }
}

impl Indicator {
    pub const fn new() -> Self {
        Self {
            activity: Activity::Booting,
            connected: false,
            mode: LedMode::Auto,
            fault: false,
            hue: 0,
        }
    }

    pub fn apply(&mut self, msg: LedMsg) {
        match msg {
            LedMsg::Activity(a) => self.activity = a,
            LedMsg::Connected(c) => self.connected = c,
            LedMsg::Mode(m) => self.mode = m,
            LedMsg::Fault => self.fault = true,
        }
    }

    /// True while the colour changes over time, so `advance` needs calling.
    pub fn animating(&self) -> bool {
        !self.fault && self.mode == LedMode::Rainbow
    }

    /// One degree further around the rainbow.
    pub fn advance(&mut self) {
        self.hue = (self.hue + 1) % 360;
    }

    pub fn colour(&self) -> Rgb {
        if self.fault {
            return FAULT;
        }
        match self.mode {
            LedMode::Off => color::OFF,
            LedMode::Solid(c) => c,
            LedMode::Rainbow => color::rainbow(self.hue, RAINBOW_BRIGHTNESS),
            LedMode::Auto => match self.activity {
                Activity::NoServos => NO_SERVOS,
                Activity::Busy => BUSY,
                Activity::Booting => BOOTING,
                _ if !self.connected => DISCONNECTED,
                Activity::Idle => IDLE,
                Activity::Standby => STANDBY,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn connected_idle() -> Indicator {
        let mut i = Indicator::new();
        i.apply(LedMsg::Activity(Activity::Idle));
        i.apply(LedMsg::Connected(true));
        i
    }

    #[test]
    fn boots_blue_then_green_when_idle_and_connected() {
        let mut i = Indicator::new();
        assert_eq!(i.colour(), BOOTING);
        i.apply(LedMsg::Activity(Activity::Idle));
        assert_eq!(i.colour(), DISCONNECTED);
        i.apply(LedMsg::Connected(true));
        assert_eq!(i.colour(), IDLE);
    }

    #[test]
    fn yellow_while_busy_even_when_disconnected() {
        let mut i = connected_idle();
        i.apply(LedMsg::Activity(Activity::Busy));
        assert_eq!(i.colour(), BUSY);
        i.apply(LedMsg::Connected(false));
        assert_eq!(i.colour(), BUSY);
        i.apply(LedMsg::Activity(Activity::Idle));
        assert_eq!(i.colour(), DISCONNECTED);
    }

    #[test]
    fn standby_is_dim_blue() {
        let mut i = connected_idle();
        i.apply(LedMsg::Activity(Activity::Standby));
        assert_eq!(i.colour(), STANDBY);
    }

    #[test]
    fn user_colour_overrides_status_until_auto() {
        let mut i = connected_idle();
        i.apply(LedMsg::Mode(LedMode::Solid((1, 2, 3))));
        i.apply(LedMsg::Activity(Activity::Busy));
        assert_eq!(i.colour(), (1, 2, 3));
        i.apply(LedMsg::Mode(LedMode::Off));
        assert_eq!(i.colour(), color::OFF);
        i.apply(LedMsg::Mode(LedMode::Auto));
        assert_eq!(i.colour(), BUSY);
    }

    #[test]
    fn no_servos_is_red_but_mqtt_colours_still_show() {
        let mut i = Indicator::new();
        i.apply(LedMsg::Activity(Activity::NoServos));
        assert_eq!(i.colour(), NO_SERVOS);
        i.apply(LedMsg::Connected(true));
        assert_eq!(i.colour(), NO_SERVOS);
        i.apply(LedMsg::Mode(LedMode::Solid((0, 9, 0))));
        assert_eq!(i.colour(), (0, 9, 0));
        i.apply(LedMsg::Mode(LedMode::Auto));
        assert_eq!(i.colour(), NO_SERVOS);
    }

    #[test]
    fn rainbow_animates_and_fault_wins_over_everything() {
        let mut i = connected_idle();
        i.apply(LedMsg::Mode(LedMode::Rainbow));
        assert!(i.animating());
        let first = i.colour();
        for _ in 0..60 {
            i.advance();
        }
        assert_ne!(i.colour(), first);
        i.apply(LedMsg::Fault);
        assert!(!i.animating());
        assert_eq!(i.colour(), FAULT);
        i.apply(LedMsg::Mode(LedMode::Auto));
        assert_eq!(i.colour(), FAULT);
    }
}
