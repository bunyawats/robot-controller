//! Remote control over MQTT.
//!
//! Everything except `mqtt` is pure logic with no ESP dependencies, so it also builds and
//! runs its unit tests on the host.

pub mod command;
pub mod json;
pub mod pose;
pub mod status;

#[cfg(target_os = "espidf")]
pub mod mqtt;
