//! Drives a WS2812 RGB LED through the ESP32's RMT peripheral.

use std::time::Duration;

use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::rmt::config::{TransmitConfig, TxChannelConfig};
use esp_idf_svc::hal::rmt::encoder::{BytesEncoder, BytesEncoderConfig};
use esp_idf_svc::hal::rmt::{PinState, Symbol, TxChannelDriver};
use esp_idf_svc::hal::units::Hertz;

use super::color::Rgb;

/// RMT clock: 10 MHz, so one tick is 0.1 us.
const RMT_HZ: Hertz = Hertz(10_000_000);

/// One WS2812 LED (on this board GPIO48).
pub struct Ws2812 {
    tx: TxChannelDriver<'static>,
    encoder: BytesEncoder,
}

impl Ws2812 {
    pub fn new(pin: impl OutputPin + 'static) -> anyhow::Result<Self> {
        let tx = TxChannelDriver::new(
            pin,
            &TxChannelConfig {
                resolution: RMT_HZ,
                ..Default::default()
            },
        )?;
        // WS2812 bit timings from its datasheet, sent most significant bit first.
        let bit = |high_ns, low_ns| {
            Symbol::new_with(
                RMT_HZ,
                PinState::High,
                Duration::from_nanos(high_ns),
                PinState::Low,
                Duration::from_nanos(low_ns),
            )
        };
        let encoder = BytesEncoder::with_config(&BytesEncoderConfig {
            bit0: bit(350, 800)?,
            bit1: bit(700, 600)?,
            msb_first: true,
            ..Default::default()
        })?;
        Ok(Self { tx, encoder })
    }

    /// Only logs on failure: a broken status light shouldn't stop anything else.
    pub fn set(&mut self, (r, g, b): Rgb) {
        // WS2812 takes the bytes in green, red, blue order.
        let sent = self
            .tx
            .send_and_wait(&mut self.encoder, &[g, r, b], &TransmitConfig::default());
        if let Err(e) = sent {
            log::warn!("RGB LED write failed: {e:?}");
        }
    }
}
