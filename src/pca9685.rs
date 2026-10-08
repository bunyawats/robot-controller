//! Minimal PCA9685 16-channel PWM driver.
//!
//! Written against a tiny `Transport` trait instead of a specific I2C crate, so it does
//! not depend on any HAL version and can be unit-tested with a mock on a PC.

pub const DEFAULT_ADDR: u8 = 0x40;
pub const NUM_CHANNELS: usize = 16;

const MODE1: u8 = 0x00;
const LED0_ON_L: u8 = 0x06;
const ALL_LED_OFF_H: u8 = 0xFD;
const PRESCALE: u8 = 0xFE;

/// Prescale value for about 50 Hz with the internal 25 MHz oscillator.
const PRESCALE_50HZ: u8 = 121;

/// An I2C write plus a delay: everything the driver needs from the hardware.
pub trait Transport {
    type Error: core::fmt::Debug;
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error>;
    fn delay_ms(&mut self, ms: u32);
}

pub struct Pca9685<T: Transport> {
    bus: T,
    addr: u8,
}

impl<T: Transport> Pca9685<T> {
    pub fn new(bus: T, addr: u8) -> Self {
        Self { bus, addr }
    }

    /// Sets the output frequency to ~50 Hz and wakes the chip with register auto-increment.
    pub fn init_50hz(&mut self) -> Result<(), T::Error> {
        self.bus.write(self.addr, &[MODE1, 0x10])?; // sleep, required to change prescale
        self.bus.write(self.addr, &[PRESCALE, PRESCALE_50HZ])?;
        self.bus.write(self.addr, &[MODE1, 0x20])?; // wake, auto-increment on
        self.bus.delay_ms(5); // oscillator needs 500 us to settle
        self.bus.write(self.addr, &[MODE1, 0xA0])?; // restart, auto-increment on
        Ok(())
    }

    /// Writes all 16 channels in one I2C transaction. `ticks[ch]` is the "off" tick
    /// (0-4095); every pulse starts at tick 0 of the period.
    pub fn set_all_ticks(&mut self, ticks: &[u16; NUM_CHANNELS]) -> Result<(), T::Error> {
        let mut buf = [0u8; 1 + NUM_CHANNELS * 4];
        buf[0] = LED0_ON_L;
        for (ch, &t) in ticks.iter().enumerate() {
            let t = t.min(4095); // 4096 would set the "full on" bit
            let o = 1 + ch * 4;
            buf[o] = 0; // ON_L
            buf[o + 1] = 0; // ON_H
            buf[o + 2] = (t & 0xFF) as u8; // OFF_L
            buf[o + 3] = (t >> 8) as u8; // OFF_H
        }
        self.bus.write(self.addr, &buf)
    }

    /// Switches every output fully off. Servos go limp (the robot will not hold its pose).
    pub fn all_off(&mut self) -> Result<(), T::Error> {
        self.bus.write(self.addr, &[ALL_LED_OFF_H, 0x10])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Mock {
        writes: Vec<(u8, Vec<u8>)>,
        delays: Vec<u32>,
    }

    impl Transport for Mock {
        type Error = ();
        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), ()> {
            self.writes.push((addr, bytes.to_vec()));
            Ok(())
        }
        fn delay_ms(&mut self, ms: u32) {
            self.delays.push(ms);
        }
    }

    #[test]
    fn init_sequence_sets_prescale_while_asleep() {
        let mut pca = Pca9685::new(Mock::default(), DEFAULT_ADDR);
        pca.init_50hz().unwrap();
        let w = &pca.bus.writes;
        assert_eq!(w[0], (0x40, vec![0x00, 0x10]));
        assert_eq!(w[1], (0x40, vec![0xFE, 121]));
        assert_eq!(w[2], (0x40, vec![0x00, 0x20]));
        assert_eq!(w[3], (0x40, vec![0x00, 0xA0]));
        assert_eq!(pca.bus.delays, vec![5]);
    }

    #[test]
    fn set_all_ticks_is_one_burst_with_correct_layout() {
        let mut pca = Pca9685::new(Mock::default(), DEFAULT_ADDR);
        let mut ticks = [0u16; NUM_CHANNELS];
        ticks[0] = 297;
        ticks[15] = 0x1ABC; // above 4095, must be clamped
        pca.set_all_ticks(&ticks).unwrap();

        assert_eq!(pca.bus.writes.len(), 1);
        let (addr, buf) = &pca.bus.writes[0];
        assert_eq!(*addr, 0x40);
        assert_eq!(buf.len(), 65);
        assert_eq!(buf[0], 0x06);
        // channel 0: ON=0, OFF=297 = 0x0129
        assert_eq!(&buf[1..5], &[0, 0, 0x29, 0x01]);
        // channel 15 clamped to 4095 = 0x0FFF
        assert_eq!(&buf[61..65], &[0, 0, 0xFF, 0x0F]);
    }

    #[test]
    fn all_off_sets_full_off_bit() {
        let mut pca = Pca9685::new(Mock::default(), DEFAULT_ADDR);
        pca.all_off().unwrap();
        assert_eq!(pca.bus.writes[0], (0x40, vec![0xFD, 0x10]));
    }
}
