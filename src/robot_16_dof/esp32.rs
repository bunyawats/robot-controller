//! ESP32-S3 glue for the robot: the PCA9685 on I2C, and the `Hardware` that drives it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use esp_idf_svc::hal::delay::{FreeRtos, TickType, BLOCK};
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use esp_idf_svc::hal::i2c::{I2c, I2cConfig, I2cDriver};
use esp_idf_svc::hal::units::Hertz;

use super::motion::{Hardware, Pose};
use super::pca9685::{self, Pca9685};
use super::servo;

/// I2C bus speed. 100 kHz rather than 400 kHz because servo current spikes on a shared
/// 5 V supply make the PCA9685 miss writes; the slower bus is less sensitive to that noise.
const I2C_HZ: u32 = 100_000;

/// I2C (with a delay) for the PCA9685 driver, on top of the ESP-IDF I2C driver.
pub struct I2cBus(I2cDriver<'static>);

impl pca9685::Transport for I2cBus {
    type Error = esp_idf_svc::sys::EspError;

    /// Retries a failed write once, since servo noise makes the odd write fail.
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0
            .write(addr, bytes, BLOCK)
            .or_else(|_| self.0.write(addr, bytes, BLOCK))
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

/// Opens the I2C bus, logs a scan of it and sets the PCA9685 up for 50 Hz servo pulses.
pub fn connect_pca9685(
    i2c: impl I2c + 'static,
    sda: impl InputPin + OutputPin + 'static,
    scl: impl InputPin + OutputPin + 'static,
) -> anyhow::Result<Pca9685<I2cBus>> {
    let mut i2c = I2cDriver::new(i2c, sda, scl, &I2cConfig::new().baudrate(Hertz(I2C_HZ)))?;
    log_i2c_scan(&mut i2c);

    let mut pca = Pca9685::new(I2cBus(i2c), pca9685::DEFAULT_ADDR);
    pca.init_50hz()
        .map_err(|e| anyhow::anyhow!("PCA9685 not responding at 0x40: {e:?}"))?;
    Ok(pca)
}

/// Logs every I2C address that answers (the PCA9685 should be at 0x40), so a wiring or
/// power fault shows up as "no devices" rather than only as a failed PCA9685 init.
fn log_i2c_scan(i2c: &mut I2cDriver<'static>) {
    let timeout = TickType::new_millis(10).ticks();
    let found: Vec<String> = (0x08u8..=0x77)
        .filter(|&addr| i2c.read(addr, &mut [0u8; 1], timeout).is_ok())
        .map(|addr| format!("0x{addr:02X}"))
        .collect();
    if found.is_empty() {
        log::warn!("I2C scan: no devices (check SDA, SCL and the PCA9685's 3.3 V and GND)");
    } else {
        log::info!("I2C scan: found {}", found.join(", "));
    }
}

/// Told about the first failed servo write, e.g. to turn a status light red.
pub trait FaultSignal {
    fn fault(&mut self);
}

/// Told about every frame written to the servos, e.g. to publish the pose.
pub trait PoseSink {
    fn frame(&mut self, pose: &Pose);
}

/// The real robot: poses become PCA9685 tick values through the calibration table.
pub struct Servos<F: FaultSignal, P: PoseSink> {
    pub pca: Pca9685<I2cBus>,
    pub fault_signal: F,
    pub pose_sink: P,
    /// Set by the first failed write, which is the only one passed to `fault_signal`.
    pub write_failed: bool,
    /// Set from another thread to stop the current move (see `motion::Robot`).
    abort: Arc<AtomicBool>,
}

impl<F: FaultSignal, P: PoseSink> Servos<F, P> {
    pub fn new(pca: Pca9685<I2cBus>, fault_signal: F, pose_sink: P, abort: Arc<AtomicBool>) -> Self {
        Self {
            pca,
            fault_signal,
            pose_sink,
            write_failed: false,
            abort,
        }
    }
}

impl<F: FaultSignal, P: PoseSink> Hardware for Servos<F, P> {
    fn write_pose(&mut self, pose: &Pose) {
        self.pose_sink.frame(pose);
        let ticks = servo::pose_to_ticks(&pose.0, &servo::CALIBRATION);
        if let Err(e) = self.pca.set_all_ticks(&ticks) {
            log::error!("I2C write to PCA9685 failed: {e:?}");
            if !self.write_failed {
                self.write_failed = true;
                self.fault_signal.fault();
            }
        }
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }

    fn should_abort(&mut self) -> bool {
        self.abort.load(Ordering::Relaxed)
    }
}
