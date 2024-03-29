//! The MAX31865 is a resistance-to-digital converter IC optimized for platinum
//! RTDs (resistance temperature detectors). It can handle RTDs with 2-, 3-, or
//! 4-wire interfaces and compensate for the resistance of the leads.
//! 
//! It communicates over SPI. It has a configuration register, two RTD data
//! registers, and several fault configuration and status registers (which are
//! unused here). All registers are 8 bits.
//! 
//! It can operate in one of two conversion modes: auto and 1-shot.
//! 
//! Automatic conversion mode will convert the RTD resistance and write to the
//! RTD data registers then signal on its RDY pin. When the data registers are
//! read it deactivate the RDY signal, and repeat the conversion process. If the
//! data is read with no delay after the RDY signal conversions will happen at
//! 60Hz.
//! 
//! 1-shot conversion mode only converts when the configuration register is
//! written to with the 1-shot bit set.
//!
//! The data in the RTD registers (with both combined) is a 15-bit integer that
//! represents the ratio between the RTD's resistance and a reference resistance
//! that is also connected to the MAX31865. The integer equals:
//! (RTD_ohms / ref_ohms) * 2^15
//! 
//! The resistance of the RTD varies with temperature in a consistent way such
//! that it can be described with a set of equations. The equations for
//! temperatures 32°F and above are simplest
//! 
//! Limitations of this implementation (done for simplicity):
//! * It is hardcoded to perform auto conversions.
//! * It is hardcoded to support a 3-wire PT1000 RTD.
//! * Temperatures are only accurate 32°F and above.
//! 
//! [MAX31865 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max31865.pdf)

use embedded_hal::spi::SpiBus;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{common::{time::{Delay, Duration}, ControllerResult}, reason};

// The highest bit indicates read vs write
const CMD_READ: u8 = 0b0000_0000;
const CMD_WRITE: u8 = 0b1000_0000;
const CONFIG_REGISTER: u8 = 0x00;
const RTD_MSB_REGISTER: u8 = 0x01;
const CMD_WRITE_CONFIG: u8 = CMD_WRITE | CONFIG_REGISTER;
const CMD_READ_RTD_MSB: u8 = CMD_READ | RTD_MSB_REGISTER;
// The base configuration enables 3-wire RTD (as opposed to 2- or 4-wire)
// and it uses a 60Hz filter.
const BASE_CONFIG: u8 = 0b0001_0000;
const ENABLE_VBIAS: u8 = 0b1000_0000;
const CLEAR_FAULT: u8 = 0b0000_0010;
const ENABLE_AUTO_CONVERSION: u8 = 0b0100_0000;

// The time in milliseconds between re-enabling VBias and when conversions can
// be made. 10ms is the maximum assuming the maximum (10k) reference resistor is
// used. For smaller resistors this will be less (e.g. 5ms for a 4.3k resistor).
const VBIAS_SETTLE_TIME: Duration = Duration::millis(10);

// The resistance of the RTD at the freezing point of water.
// For a PT1000 device this is 1000.
const RTD_OHMS_AT_FREEZING: f32 = 1000f32;

/// Driver for the MAX31865
pub struct Max31865<SPI: SpiBus, D> {
    spi: SPI,
    delay: D,
    reference_resistance_ohms: u32,
}

impl<SPI: SpiBus, D: Delay> Max31865<SPI, D> {
    pub fn new(spi: SPI, delay: D, reference_resistance_ohms: u32) -> Self {
        Self {
            spi,
            delay,
            reference_resistance_ohms,
       }
    }

    /// Initializes the driver with a default configuration an automatic
    /// conversions disable. Any exisiting faults are cleared. The delay is
    /// used to wait for the device to become ready after it is enabled.
    pub fn initialize(&mut self) -> ControllerResult<()> {
        // Enable VBias, but don't start conversions
        const CONFIG: u8 = BASE_CONFIG | ENABLE_VBIAS | CLEAR_FAULT;
        self.transfer(&mut [CMD_WRITE_CONFIG, CONFIG])?;
    
        // Need to wait after enabling VBias before starting any conversions
        self.delay.delay(VBIAS_SETTLE_TIME);
        Ok(())
    }

    /// Enables automatic conversions. After this is called the device will
    /// signal on its RDY pin when data is ready.
    pub fn start_auto_conversions(&mut self) -> ControllerResult<()> {
        const CONFIG: u8 = BASE_CONFIG | ENABLE_VBIAS | ENABLE_AUTO_CONVERSION;
        self.transfer(&mut [CMD_WRITE_CONFIG, CONFIG])
    }

    /// Reads the conversion data from the device and converts it to a
    /// temperature in °F. Only accurate for temperatures 32°F and above.
    pub fn read_temperature(&mut self) -> ControllerResult<f32> {
        // data[0] is the command to read starting at the RTD_MSB register.
        // The remaining 2 bytes are to receive the data at that register and
        // the one following it, RTD_LSB.
        let data = &mut [CMD_READ_RTD_MSB, 0x00, 0x00];
        self.transfer(data)?;

        // The lowest bit of the LSB is a fault flag
        if data[2] & 0x01 == 0x01 {
            // Ignore faults. In practice they are spurious and caused by
            // touching the RTD to a piece of metal at a different ground
            // voltage. Even tiny voltage differences like 0.1 volts are enough.
            warn!("Fault bit set on RTD read");
            // Reset the fault flag
            const CONFIG: u8 = BASE_CONFIG | ENABLE_VBIAS | ENABLE_AUTO_CONVERSION | CLEAR_FAULT;
            self.transfer(&mut [CMD_WRITE_CONFIG, CONFIG])?;
        }

        // Combine the RTD_MSB and RTD_LSB into a single value
        let raw_reading = (data[1] as u16) << 8 | data[2] as u16;
        let temp = self.convert_rtd_to_temp(raw_reading);
        Ok(temp)
    }

    fn transfer(&mut self, data: &mut [u8]) -> ControllerResult<()> {
        reason!(
            self.spi.transfer_in_place(data),
            "Error during spi.transfer_in_place in MAX31865"
        )
    }

    /// Converts the raw 2-byte reading of the device's two RTD registers to
    /// the temperature in degrees Fahrenheit.
    /// This is only accurate for temperatures above the freezing point of
    /// water. The math for lower temperatures is slightly more complicated and
    /// isn't needed for brewing beer (if you have frozen beer, especially while
    /// you're doing the mash or boil, you're doing something very strange).
    fn convert_rtd_to_temp(&self, raw_reading: u16) -> f32 {
        // The raw reading's least significant bit indicates a fault if it is 1.
        // The rest of the reading represents the ratio of the RTD's resistance
        // to the reference resistor, except the ratio is scaled to the range
        // [0, 32768) (i.e. 15 bits).
        let ratio_numerator = (raw_reading >> 1) as f32;
        let resistance = (ratio_numerator * self.reference_resistance_ohms as f32) / 32768f32;
        
        // Callendar-Van Dusen equation for temperatures above the freezing
        // point of water. Constants A and B are taken from IEC 751, which
        // are "standard" PT-RTD values, although technically these should be
        // experimetally determined for each RTD probe. But the equipment for
        // that is out of reach for the average person.
        // Using f32 results in an error of 0.0002°F at 212°F compared to
        // arbitrary precision computation (as performed in python). That is
        // much smaller than the precision of the Max31865, so we don't care.
        const RTD_A: f32 = 3.90830e-3;
        const RTD_B: f32 = -5.77500e-7;
        const Z1: f32 = -RTD_A;
        const Z2: f32 = RTD_A * RTD_A - (4f32 * RTD_B);
        const Z3: f32 = (4f32 * RTD_B) / RTD_OHMS_AT_FREEZING;
        const Z4: f32 = 2f32 * RTD_B;

        let mut temp = Z2 + (Z3 * resistance as f32);
        temp = (libm::sqrtf(temp) + Z1) / Z4;

        // Convert °C to °F
        temp * 1.8f32 + 32f32
    }

    /// Consumes `self` and returns the `spi` that it owned so it can be reused.
    #[cfg(test)]
    pub fn destroy(self) -> SPI {
        self.spi
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{mock_spi::*, mock_time::MockDelay};

    #[test]
    fn test_new() {
        let device = Max31865::new(MockSpi::new(), MockDelay, 1000);

        assert_eq!(device.reference_resistance_ohms, 1000);
    }

    #[test]
    fn test_initialize() {
        let mut spi = MockSpi::new();
        // Write to config: VBias enabled, 3-wire, clear fault
        spi.expect_transfer_in_place(&[0x80, 0b1001_0010], &[0, 0]);

        let mut device = Max31865::new(spi, MockDelay, 1000);
        device.initialize().unwrap();
    
        let spi = device.destroy();
        spi.verify();
    }

    #[test]
    fn test_start_auto_conversions() {
        let mut spi = MockSpi::new();
        // Write to config: VBias enabled, auto mode, 3-wire
        spi.expect_transfer_in_place(&[0x80, 0b1101_0000], &[0, 0]);

        let mut device = Max31865::new(spi, MockDelay, 1000);
        device.start_auto_conversions().unwrap();
    
        let spi = device.destroy();
        spi.verify();
    }

    #[test]
    fn test_read_temperature() {
        let mut spi = MockSpi::new();
        // Read RTD registers and return a value indicating an RTD/ref ratio of
        // 0.25, which for a reference of 4000 ohms means the RTD is 1000 ohms,
        // which means for a PT1000 RTD that the temperature is 32°F.
        spi.expect_transfer_in_place(&[0x01, 0, 0], &[0, 0b0100_0000, 0b0000_0000]);

        let mut device = Max31865::new(spi, MockDelay, 4000);
        let temperature = device.read_temperature().unwrap();
    
        assert_eq!(temperature, 32.0);
    
        let spi = device.destroy();
        spi.verify();
    }
}