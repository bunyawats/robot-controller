//! The ESP32-S3 board's 3-colour (WS2812) LED.
//!
//! `color` and `indicator` are pure logic with no ESP dependencies, so they also build and
//! run their unit tests on the host.

pub mod color;
pub mod indicator;

#[cfg(target_os = "espidf")]
pub mod ws2812;
