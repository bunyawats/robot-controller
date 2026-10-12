//! The ESP32-S3 board's 3-colour (WS2812) LED.
//!
//! `color` is pure logic with no ESP dependencies, so it also builds and runs its unit tests
//! on the host.

pub mod color;

#[cfg(target_os = "espidf")]
pub mod ws2812;
