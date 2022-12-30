//! The MCP23008 is an IO expander IC that communicates over I2C. It supports
//! 8 GPIO pins that can be individually configured as inputs or outputs. Inputs
//! can have a weak internal pull-up resistor configured. Output states are
//! latched. The device has an interrupt pin that can be monitored separate from
//! the I2C bus. Each input pin can be configured as an interrupt trigger.
//!
//! [MCP23008 Datasheet](https://cdn-shop.adafruit.com/datasheets/MCP23008.pdf)
use embedded_hal::i2c::I2c;

// Whether a pin is input or output. 0 = output, 1 = input.
const IODIR_REGISTER: u8 = 0x00;
// All pins default to inputs
const IODIR_DEFAULT: u8 = 0b1111_1111;
// All input pins default to normal polarity (i.e. are not auto-inverted)
const IPOL_DEFAULT: u8 = 0b0000_0000;
const GPINTEN_DEFAULT: u8 = 0b0000_0000;
const DEFVAL_DEFAULT: u8 = 0b0000_0000;
const INTCON_DEFAULT: u8 = 0b0000_0000;
// bits 7-6: noop
// bit 5 = 0: Address pointer auto increments
// bit 4 = 0: SDA slew rate disabled
// bit 3: noop
// bit 2 = 0: Interrupt pin is active driver (as opposed to open-drain)
// bit 1 = 0: Interrupt pin is active-high
// bit 0: noop
const IOCON_DEFAULT: u8 = 0b0000_0000;
const GPPU_DEFAULT: u8 = 0b0000_0000;
const INTF_REGISTER: u8 = 0x07;
const OLAT_DEFAULT: u8 = 0b0000_0000;

#[derive(Copy, Clone, Debug)]
pub enum Mcp23008Error<I2C> {
    I2c(I2C),
}

impl<I2C> From<I2C> for Mcp23008Error<I2C> {
    fn from(value: I2C) -> Self {
        Self::I2c(value)
    }
}

/// The available GPIO pins on the device.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Pin {
    GPIO0 = 0b0000_0001,
    GPIO1 = 0b0000_0010,
    GPIO2 = 0b0000_0100,
    GPIO3 = 0b0000_1000,
    GPIO4 = 0b0001_0000,
    GPIO5 = 0b0010_0000,
    GPIO6 = 0b0100_0000,
    GPIO7 = 0b1000_0000,
}

impl Pin {
    pub const fn all() -> [Pin; 8] {
        use Pin::*;
        [GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7]
    }
}

/// Data read from the MCP23008 device.
#[derive(Copy, Clone, Debug)]
pub struct Mcp23008Data {
    pub(crate) interrupts: u8,
    pub(crate) interrupt_capture: u8,
    pub(crate) gpio: u8,
}

impl Mcp23008Data {
    /// Returns whether `pin` triggered an interrupt since the last read.
    pub fn is_interrupted(&self, pin: Pin) -> bool {
        self.interrupts & (pin as u8) != 0
    }

    /// Returns whether `pin` was high when it triggered the first interrupt
    /// since the last read. If the pin did not trigger that interrupt, then
    /// the data returned here has no meaning.
    pub fn is_capture_high(&self, pin: Pin) -> bool {
        self.interrupt_capture & (pin as u8) != 0
    }

    /// Returns whether `pin` was high when the device was read.
    pub fn is_high(&self, pin: Pin) -> bool {
        self.gpio & (pin as u8) != 0
    }
}

#[derive(Copy, Clone, Debug)]
pub enum InterruptTrigger {
    /// Triggers when the pin changes to its opposite state.
    AnyEdge,
    /// Triggers as long as the pin is in a low state.
    Low,
    /// Triggers as long as the pin is in a high state.
    High,
}

/// Configuration for the MCP23008. Corresponds directly to its config registers.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Mcp23008Config {
    /// IO direction register. Whether a pin is input (1) or output (0).
    iodir: u8,
    /// Interrupt-on-change register. Whether a pin is an interrupt trigger.
    gpinten: u8,
    /// Default compare register. The default comparison values for any pin with
    /// INTCON = 1.
    defval: u8,
    /// Interrupt control register. Whether a pin interrupts on any change (0)
    /// or a change from a default value (1).
    intcon: u8,
    /// GPIO pull-up resistor configuration register. Whether the pin has a
    /// 100 kiloohm pull-up resistor enabled (1) or not (0).
    gppu: u8,
    /// Output latch register. The values for any output pins. These can be
    /// changed later, setting it here is just setting initial values.
    olat: u8,
}

impl Mcp23008Config {
    /// Sets `pin` to be an input with an optional `pullup` resistor.
    pub fn set_input_pin(&mut self, pin: Pin, pullup: bool) {
        let pin = pin as u8;
        // 1 = input
        self.iodir |= pin;
    
        // 1 = enable pullup, 0 = disable
        if pullup {
            self.gppu |= pin;
        } else {
            self.gppu &= !pin;
        }
    }

    /// Sets `pin` to be an output with its `initial` state.
    pub fn set_output_pin(&mut self, pin: Pin, initial: bool) {
        let pin = pin as u8;
        // 0 = output
        self.iodir &= !pin;
        // 0 = disable interrupts
        self.gpinten &= !pin;
        // 0 = disable pullup
        self.gppu &= !pin;
    
        // Set the initial output value
        if initial {
            self.olat |= pin;
        } else {
            self.olat &= !pin;
        }
    }

    /// Sets `pin` to be an input pin that also interrupts on `trigger`. An
    /// optional `pullup` resistor can be used.
    pub fn set_interrupt_input_pin(&mut self, pin: Pin, pullup: bool, trigger: InterruptTrigger) {
        self.set_input_pin(pin, pullup);

        let pin = pin as u8;
        self.gpinten |= pin;
        
        match trigger {
            InterruptTrigger::AnyEdge => {
                // defval is a dont care, but set to 0 for consistency
                self.defval &= !pin;
                // Set bit to 0: interrupt any time the value changes
                self.intcon &= !pin;
            },
            InterruptTrigger::Low => {
                // Set to 1 so that we interrupt when the level is low
                self.defval |= pin;
                // Set bit to 1: interrupt if the level is different than defval
                self.intcon |= pin;
            },
            InterruptTrigger::High => {
                // Set to 0 so that we interrupt when the level is high
                self.defval &= !pin;
                // Set bit to 1: interrupt if the level is different than defval
                self.intcon |= pin;
            },
        };
    }

}

pub trait Mcp23008ReadWrite {
    type Error: core::fmt::Debug;

    /// Initializes the device with the default configuration.
    fn initialize(&mut self) -> Result<(), Mcp23008Error<Self::Error>>;

    /// Returns the current configuration.
    fn configuration(&self) -> Mcp23008Config;

    /// Sets the given configuration as current, including writing it to the device.
    fn set_configuration(&mut self, config: Mcp23008Config) -> Result<(), Mcp23008Error<Self::Error>>;

    // Reads from the device, including which pins triggered interrupts, the
    // captured interrupting pin's state during the interrupt, and the GPIO
    // states at the time of the read.
    fn read(&mut self) -> Result<Mcp23008Data, Mcp23008Error<Self::Error>>;
}

pub struct Mcp23008<I2C> {
    i2c: I2C,
    address: u8,
    config: Mcp23008Config,
}

impl<I2C, E> Mcp23008<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new driver that will communicate using `i2c` as the I2C bus
    /// to a MCP23008 device at `address`. Valid addresses for the MCP23008 are
    /// 0x20 - 0x27.
    pub fn new(i2c: I2C, address: u8) -> Self {
        assert!(address & 0b1111_1000 == 0b0010_0000);

        Self {
            i2c,
            address,
            config: Mcp23008Config {
                iodir: IODIR_DEFAULT,
                gpinten: GPINTEN_DEFAULT,
                defval: DEFVAL_DEFAULT,
                intcon: INTCON_DEFAULT,
                gppu: GPPU_DEFAULT,
                olat: OLAT_DEFAULT,
            },
        }
    }

    /// Consumes `self` and returns the `i2c` that it owned so it can be reused.
    #[cfg(test)]
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

impl<I2C, E> Mcp23008ReadWrite for Mcp23008<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    type Error = E;

    fn initialize(&mut self) -> Result<(), Mcp23008Error<Self::Error>> {
        // Configure the device to its power-on-reset defaults
        self.set_configuration(self.config)?;
        Ok(())
    }

    fn configuration(&self) -> Mcp23008Config {
        self.config
    }

    fn set_configuration(&mut self, config: Mcp23008Config) -> Result<(), Mcp23008Error<Self::Error>> {
        // The write starts at the IODIR register and continues through all
        // registers. The device automatically increments the target register
        // after each byte is written.
        // This writes to a few read only registers, but the device ignores those.
        self.i2c.write(self.address, &[
            IODIR_REGISTER,
            config.iodir,
            IPOL_DEFAULT,
            config.gpinten,
            config.defval,
            config.intcon,
            IOCON_DEFAULT,
            config.gppu,
            0, // INTF, read only
            0, // INTCAP, read only
            0, // GPIO, read only
            config.olat,
        ])?;

        self.config = config;
    
        Ok(())
    }

    fn read(&mut self) -> Result<Mcp23008Data, Mcp23008Error<Self::Error>> {
        // Start the read at the INTF register and read 3 bytes. The first byte
        // will be the INTF register (interrupt triggered flags), the second
        // will be the next register (INTCAP, the captured gpios at the time of
        // the first interrupt), and the third will be the next register (GPIO,
        // the states of the gpios at the time this read happens).
        let data = &mut [0u8; 3];
        self.i2c.write_read(self.address, &[INTF_REGISTER], data)?;

        Ok(Mcp23008Data {
            interrupts: data[0],
            interrupt_capture: data[1],
            gpio: data[2],
        })
    }

}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::mock_i2c::*;

    #[test]
    fn test_new() {
        let address = 0x20;
        let i2c = MockI2c::new();

        let device = Mcp23008::new(i2c, address);

        assert_eq!(device.address, address);
    }

    #[test]
    #[should_panic]
    fn test_new_bad_address() {
        Mcp23008::new(MockI2c::new(), 0x00);
    }

    #[test]
    fn test_initialize() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        // All data should be 0 except for the IODIR (0x00) register is all 1s
        // and the IOCON (0x05) register has bit 1 set to 1. Note the indices
        // offset by one because the first byte of data is the address of the
        // first register to write to (0x00).
        let mut expected_data = [0u8; 12];
        expected_data[1] = 0b1111_1111;
        expected_data[6] = 0b0000_0000;
        i2c.expect_write(address, &expected_data);
        let mut device = Mcp23008::new(i2c, address);
        
        device.initialize().unwrap();
    
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_configuration_pullup() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        let mut expected_data = [0u8; 12];
        expected_data[1] = IODIR_DEFAULT;
        expected_data[6] = IOCON_DEFAULT;
        // pullup is enabled on gpio0
        expected_data[7] = 0b0000_0001;
        i2c.expect_write(address, &expected_data);
 
        let mut device = Mcp23008::new(i2c, address);
        let mut config = device.configuration();
        config.set_input_pin(Pin::GPIO0, true);
        device.set_configuration(config).unwrap();
 
        assert_eq!(config, device.configuration());
 
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_configuration_pullup_all() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        let mut expected_data = [0u8; 12];
        expected_data[1] = IODIR_DEFAULT;
        expected_data[6] = IOCON_DEFAULT;
        // pullup is enabled on all pins
        expected_data[7] = 0b1111_1111;
        i2c.expect_write(address, &expected_data);
 
        let mut device = Mcp23008::new(i2c, address);
        let mut config = device.configuration();
        
        for pin in Pin::all() {
            config.set_input_pin(pin, true);
        }

        device.set_configuration(config).unwrap();
        
 
        assert_eq!(config, device.configuration());
 
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_configuration_interrupt_anyedge() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        let mut expected_data = [0u8; 12];
        expected_data[1] = IODIR_DEFAULT;
        // Interrupt is enabled on gpio0
        expected_data[3] = 0b0000_0001;
        expected_data[6] = IOCON_DEFAULT;
        i2c.expect_write(address, &expected_data);
 
        let mut device = Mcp23008::new(i2c, address);
        let mut config = device.configuration();
        
        config.set_interrupt_input_pin(Pin::GPIO0, false, InterruptTrigger::AnyEdge);
        device.set_configuration(config).unwrap();
 
        assert_eq!(config, device.configuration());
 
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_configuration_interrupt_low() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        let mut expected_data = [0u8; 12];
        expected_data[1] = IODIR_DEFAULT;
        // Interrupt is enabled on gpio0
        expected_data[3] = 0b0000_0001;
        // Interrupt when pin is different from 1 (i.e. on low level)
        expected_data[4] = 0b0000_0001;
        // Interrupt by comparing to the DEFVAL register
        expected_data[5] = 0b0000_0001;
        expected_data[6] = IOCON_DEFAULT;
        i2c.expect_write(address, &expected_data);
 
        let mut device = Mcp23008::new(i2c, address);
        let mut config = device.configuration();
        
        config.set_interrupt_input_pin(Pin::GPIO0, false, InterruptTrigger::Low);
        device.set_configuration(config).unwrap();
 
        assert_eq!(config, device.configuration());
 
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_configuration_interrupt_high() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        let mut expected_data = [0u8; 12];
        expected_data[1] = IODIR_DEFAULT;
        // Interrupt is enabled on gpio0
        expected_data[3] = 0b0000_0001;
        // Interrupt when pin is different from 0 (i.e. on high level)
        expected_data[4] = 0b0000_0000;
        // Interrupt by comparing to the DEFVAL register
        expected_data[5] = 0b0000_0001;
        expected_data[6] = IOCON_DEFAULT;
        i2c.expect_write(address, &expected_data);
 
        let mut device = Mcp23008::new(i2c, address);
        let mut config = device.configuration();
        
        config.set_interrupt_input_pin(Pin::GPIO0, false, InterruptTrigger::High);
        device.set_configuration(config).unwrap();
 
        assert_eq!(config, device.configuration());
 
        let i2c = device.destroy();
        i2c.verify();
    }

    #[test]
    fn test_read() {
        let address = 0x20;
        let mut i2c = MockI2c::new();
        i2c.expect_write_read(address, &[0x07], &[0x12, 0x34, 0x56]);
 
        let mut device = Mcp23008::new(i2c, address);
        let data = device.read().unwrap();
 
        assert_eq!(data.interrupts, 0x12);
        assert_eq!(data.interrupt_capture, 0x34);
        assert_eq!(data.gpio, 0x56);
 
        let i2c = device.destroy();
        i2c.verify();
    }


}