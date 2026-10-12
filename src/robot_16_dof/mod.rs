//! The 16-servo biped, from joint calibration up to the walk.
//!
//! Everything except `esp32` is pure logic with no ESP dependencies, so it also builds and
//! runs its unit tests on the host.

pub mod motion;
pub mod moves;
pub mod pca9685;
pub mod servo;

#[cfg(target_os = "espidf")]
pub mod esp32;
