/// Abstraction of a rotary encoder controlled by an
/// [Adafruit I2C rotary encoder breakout](https://learn.adafruit.com/adafruit-i2c-qt-rotary-encoder).
/// The breakout has a microcontroller that uses Adafruit's Seesaw library to
/// handle reading the encoder and communicating over I2C.
/// This implementation is designed to be polled regularly by calling update()
/// so that the encoder's delta can be added to the value.
use embedded_hal::i2c::I2c;

use crate::{common::{time::{Delay, Duration}, ControllerResult}, reason};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

const STATUS_REGISTER_BASE: u8 = 0x00;
const STATUS_REGISTER_RESET: u8 = 0x7F;
const RESET_DATA: u8 = 0xFF;
const RESET_WAIT: Duration = Duration::millis(500);

const ENCODER_REGISTER_BASE: u8 = 0x11;
const ENCODER_DELTA_REGISTER: u8 = 0x40;

// Microseconds of delay required between the write and read to the device.
// This is required because the device accepts a write asking it to fill certain
// data registers, then it accepts the read to those registers, but it takes a
// short time to write to those registers before they have valid data.
const READ_DELAY: Duration = Duration::micros(100);

pub struct RotaryEncoder<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
    value: i32,
    min: i32,
    max: i32,
}

impl<I2C, E, D> RotaryEncoder<I2C, D>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
    D: Delay,
{
    pub fn new(
        i2c: I2C,
        address: u8,
        delay: D,
        min: i32,
        max: i32,
        initial_value: i32
    ) -> RotaryEncoder<I2C, D> {
        Self {
            i2c,
            address,
            delay,
            value: initial_value,
            min,
            max,
        }
    }

    /// Initializes the peripheral, resetting it to a good state.
    pub fn initialize(&mut self) -> ControllerResult<()> {
        // Reset the device
        self.write(&[STATUS_REGISTER_BASE, STATUS_REGISTER_RESET, RESET_DATA])?;
        self.delay.delay(RESET_WAIT);
        Ok(())
    }

    /// Returns the cumulative value of the rotary encoder's deltas.
    pub fn value(&self) -> i32 {
        self.value
    }

    /// Forces the value of this encoder to be `value`.
    pub fn force_set_value(&mut self, value: i32) {
        self.value = value;
    }

    /// Forces the value of this encoder to be its min value.
    pub fn set_to_min(&mut self) {
        self.force_set_value(self.min);
    }

    /// Returns the minimum value this encoder's value can take.
    pub fn min(&self) -> i32 {
        self.min
    }

    /// Returns the maximum value this encoder's value can take.
    pub fn max(&self) -> i32 {
        self.max
    }

    fn set_value(&mut self, delta: i32) -> bool {
        // Change based on how fast the encoder is rotating
        let amount = match delta.abs() {
            0..=1 => delta,
            2..=3 => delta * 2,
            _ => delta * 3,
        };
        
        let prev_value = self.value;
        
        // Subtract the amount so clockwise rotation results in an increase
        self.value = (self.value - amount).clamp(self.min, self.max);
        self.value != prev_value
    }
    
    /// Updates the value based on how much the encoder has turned since the
    /// last call to this method.
    pub fn update(&mut self) -> ControllerResult<bool> {
        let mut data = [0u8; 4];
        self.read(ENCODER_REGISTER_BASE, ENCODER_DELTA_REGISTER, &mut data)?;

        // The data is a big-endian signed integer
        let delta = (data[0] as i32) << 24 | (data[1] as i32) << 16 | (data[2] as i32) << 8 | (data[3] as i32);

        Ok(self.set_value(delta))
    }

    fn write(&mut self, data: &[u8]) -> ControllerResult<()> {
        reason!(
            self.i2c.write(self.address, data),
            "Error during i2c.write in RotaryEncoder"
        )
    }

    fn read(&mut self, base_reg: u8, reg: u8, data: &mut [u8]) -> ControllerResult<()> {
        self.write(&[base_reg, reg])?;
        self.delay.delay(READ_DELAY);
        
        reason!(
            self.i2c.read(self.address, data),
            "Error during i2c.read in RotaryEncoder"
        )
    }

    /// Consumes `self` and returns the `i2c` that it owned so it can be reused.
    #[cfg(test)]
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{mock_time::MockDelay, mock_i2c::*};

    #[test]
    fn test_new() {
        let address = 0x36;
        let min = -100;
        let max = 100;
        let initial = 50;
        let rotary = RotaryEncoder::new(MockI2c::new(), address, MockDelay, min, max, initial);
    
        assert_eq!(address, rotary.address);
        assert_eq!(min, rotary.min());
        assert_eq!(max, rotary.max());
        assert_eq!(initial, rotary.value());
    }

    #[test]
    fn test_force_set_value() {
        let mut rotary = RotaryEncoder::new(MockI2c::new(), 0x36, MockDelay, 0, 100, 0);

        rotary.force_set_value(42);
        assert_eq!(42, rotary.value())
    }

    #[test]
    fn test_set_value_increments_clockwise() {
        let mut rotary = RotaryEncoder::new(MockI2c::new(), 0x36, MockDelay, 0, 100, 50);

        let result = rotary.set_value(-1);
        assert!(result);
        assert_eq!(51, rotary.value())
    }

    #[test]
    fn test_set_value_decrements_counterclockwise() {
        let mut rotary = RotaryEncoder::new(MockI2c::new(), 0x36, MockDelay, 0, 100, 50);

        let result = rotary.set_value(1);
        assert!(result);
        assert_eq!(49, rotary.value())
    }

    #[test]
    fn test_set_value_limits_to_min() {
        let mut rotary = RotaryEncoder::new(MockI2c::new(), 0x36, MockDelay, 0, 100, 0);

        let result = rotary.set_value(1);
        assert!(!result);
        assert_eq!(0, rotary.value())
    }

    #[test]
    fn test_set_value_limits_to_max() {
        let mut rotary = RotaryEncoder::new(MockI2c::new(), 0x36, MockDelay, 0, 100, 100);

        let result = rotary.set_value(-1);
        assert!(!result);
        assert_eq!(100, rotary.value())
    }
}