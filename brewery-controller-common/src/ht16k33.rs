//! The HT16K33 is an LED driver IC that communicates over I2C. It supports 16x8
//! multiplexed LEDs and can perform input key scanning, although that feature
//! is not implemented here.
//! 
//! It works by having 16 bytes of display memory that correspond to the LEDs,
//! each LED having 1 bit determining if it is on or off. It then handles the
//! output on the 16 row and 8 common pins that should be connected to the
//! multiplexed LEDs.
//!
//! The HT16K33 will hold LEDs according to how the display memory is set so
//! that it does not need to be continually refreshed by the controller.
//! 
//! Note that not all HT16K33 packages support all 16 rows, some have less pins
//! and can only output to some of the rows.
//! 
//! Which LEDs correspond to which row+common pairs depends on the multiplexed
//! LED circuit, which is external to this chip and left to the user to decide.
//! 
//! [HT16K33 datasheet](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf)
//! (The HT16K33 is actually discontinued as of 2022, but its successor HT16K33A
//! looks to be identical. Only the HT16K33 has been tested with this code.)

use embedded_hal::i2c::I2c;

use crate::{common::ControllerResult, reason};

/// The number of addressable rows supported by the device.
pub const N_ROWS: usize = 16;
/// The number of addressable commons supported by the device.
pub const N_COMS: usize = 8;
/// The number of bytes in the display data used by the device.
pub const DISPLAY_DATA_LENGTH: usize = N_COMS * 2;

const DISPLAY_DATA_START_REGISTER: u8 = 0;
const SYSTEM_SETUP_REGISTER: u8 = 0b0010_0000;
const SYSTEM_SETUP_ENABLE_OSCILLATOR: u8 = 0b0000_0001;
const DISPLAY_REGISTER: u8 = 0b1000_0000;
const DISPLAY_ON: u8 = 0b0000_0001;
const DISPLAY_OFF: u8 = 0b0000_0000;
const DIMMING_REGISTER: u8 = 0b1110_0000;

/// Driver for the HT16K33.
pub struct HT16K33<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> HT16K33<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Creates a new driver that will communicate using `i2c` as the I2C bus
    /// to the device at `address`. Valid addresses for the HT16K33 are 7 bits
    /// long and start with 1110 (0x70 - 0x77).
    /// This function does not communicate with the device.
    pub fn new(i2c: I2C, address: u8) -> Self {
        assert!(address & 0b1111_1000 == 0b0111_0000);

        Self {
            i2c,
            address,
        }
    }

    /// Turns on the devices's oscillator, which is the first step in enabling it.
    pub fn initialize(&mut self) -> ControllerResult<()> {
        self.write_one(SYSTEM_SETUP_REGISTER | SYSTEM_SETUP_ENABLE_OSCILLATOR)
    }

    /// Sets the dimming on the device. The HT16K33 supports blinking modes with
    /// this setting, but this always sets blinking to off.
    /// `dimming` is 0-15. Only the lowest 4 bits will be used.
    /// `dimming` = 0 is the lowest level, but is not off
    pub fn set_dimming(&mut self, dimming: u8) -> ControllerResult<()> {
        self.write_one(DIMMING_REGISTER | (dimming & 0b1111)) 
    }

    /// Causes the device to enable/disable displaying LEDs if they are set
    /// in its display data.
    pub fn set_display_on(&mut self, on: bool) -> ControllerResult<()> {
        if on {
            self.write_one(DISPLAY_REGISTER | DISPLAY_ON)
        } else {
            self.write_one(DISPLAY_REGISTER | DISPLAY_OFF)
        }
    }

    /// Sets the device's display data, which is how each LED is turned on or off.
    /// 
    /// Each pair of bytes in the data correspond to a common output. e.g. bytes
    /// 0 and 1 are for COM0, bytes 2 and 4 are for COM1, etc.
    /// 
    /// The bits in each pair of bytes correspond to the 16 row outputs, where
    /// ROW0 is the LSB of the first byte, ROW 7 is the MSB of the first byte,
    /// ROW8 is the LSB of the second byte, and ROW 15 is the MSB of the second.
    /// 
    /// So by setting byte 0 to 0b0001_0001, the LEDs connected to (ROW0, COM0)
    /// and (ROW4, COM0) will be turned on.
    pub fn set_display_data(&mut self, data: &[u8; DISPLAY_DATA_LENGTH])  -> ControllerResult<()> {
        let mut bytes = [0u8; DISPLAY_DATA_LENGTH + 1];
        bytes[0] = DISPLAY_DATA_START_REGISTER;

        for i in 1..=N_ROWS {
            bytes[i] = data[i - 1];
        }

        reason!(
            self.i2c.write(self.address, &bytes),
            "Error during i2c.write in HT16K33::set_display_data"
        )
    }

    /// Helper to write a single byte to the device
    fn write_one(&mut self, byte: u8) -> ControllerResult<()> {
        reason!(
            self.i2c.write(self.address, &[byte]),
            "Error during i2c.write in HT16K33::write_one"
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
    use crate::mocks::mock_i2c::*;

    #[test]
    fn test_new() {
        let address = 0x70;
        let device = HT16K33::new(MockI2c::new(), address);

        assert_eq!(device.address, address);
    }

    #[test]
    #[should_panic]
    fn test_new_bad_address() {
        HT16K33::new(MockI2c::new(), 0x00);
    }

    #[test]
    fn test_initialize_enables_oscillator() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // System setup register: 0010XXXS
        // S = 1 to enable the oscillator
        i2c.expect_write(address, &[0b0010_0001]);

        let mut device = HT16K33::new(i2c, address);
        device.initialize().unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_dimming() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Dimming register: 1110PPPP
        // P = 0-15 (10 = 0b1010)
        i2c.expect_write(address, &[0b1110_1010]);
        
        let mut device = HT16K33::new(i2c, address);
        device.set_dimming(10).unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_dimming_ignores_extra_bits() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        i2c.expect_write(address, &[0b1110_0001]);
        
        // Only the lowest 4 bits should be used
        let mut device = HT16K33::new(i2c, address);
        device.set_dimming(17).unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_display_on_true_turns_on() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Display setup register: 1000XBBD
        // BB = blinking (0 is off)
        // D = 1 is display on
        i2c.expect_write(address, &[0b1000_0001]);
       
        let mut device = HT16K33::new(i2c, address);
        device.set_display_on(true).unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_display_on_false_turns_off() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Display setup register: 1000XBBD
        // BB = blinking (0 is off)
        // D = 0 is display off
        i2c.expect_write(address, &[0b1000_0000]);

        let mut device = HT16K33::new(i2c, address);
        device.set_display_on(false).unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_display_data() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        let mut data = [0u8; DISPLAY_DATA_LENGTH];
        let mut expected_data = [0u8; DISPLAY_DATA_LENGTH + 1];

        // Make up some data
        for i in 0..DISPLAY_DATA_LENGTH {
            data[i] = (i + 1) as u8;
            // The actual write starts with 0x00, so offset by 1
            expected_data[i + 1] = data[i];
        }

        // Data should be written through untouched, except there is an extra
        // first bytes, which is 0x00 to indicate writing to the first display
        // data register.
        i2c.expect_write(address, &expected_data);
 
        let mut device = HT16K33::new(i2c, address);
        device.set_display_data(&data).unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

}