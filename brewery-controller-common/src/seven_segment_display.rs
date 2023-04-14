use core::cmp::min;

use embedded_hal::i2c::I2c;
use crate::{ht16k33::{HT16K33, self}, common::ControllerResult};

// Data required to show a negative sign on the 7-segment display
const NEGATIVE_SIGN_DATA: u8 = 0b0100_0000;
const MAX_DIGITS: usize = 4;
// Default to maximum brightness
const DEFAULT_DIMMING: u8 = 15;

/// Display driver for the [Adafruit 7-segment HT16K33 Backpack](https://learn.adafruit.com/adafruit-led-backpack/0-dot-56-seven-segment-backpack).
/// 
/// It uses the [KW4-56NCXX 4-digit 7-segment LED display](https://cdn-shop.adafruit.com/datasheets/812datasheet.pdf),
/// which is multiplexed with common cathode.
/// 
/// Each digit (and colon) on the display maps to a common on the HT16K33. The
/// same LED segment across all digits maps to the same row (i.e. the top LED of
/// every digit is ROW0).
pub struct SevenSegmentDisplay<I2C> {
    device: HT16K33<I2C>,
    max_decimal_places: usize,
}

impl<I2C, E> SevenSegmentDisplay<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Creates a new driver that will communicate using `i2c` as the I2C bus
    /// to a HT16K33 device at `address`. Valid addresses for the HT16K33 are
    /// 0x70 - 0x77.
    ///
    /// When `display_digits` is called, it will show up to `max_decimal_places`
    /// decimal digits, rounding to the nearest displayable digit. Less decimal
    /// digits will be shown if more digits need to be dedicated to the integer.
    pub fn new(i2c: I2C, address: u8, max_decimal_places: usize) -> Self {
        Self {
            device: HT16K33::new(i2c, address),
            max_decimal_places,
        }
    }

    /// Consumes `self` and returns the `i2c` that it owned so it can be reused.
    #[cfg(test)]
    pub fn destroy(self) -> I2C {
        self.device.destroy()
    }

    /// Initializes the display, but does not turn on the LEDs.
    pub fn initialize(&mut self) -> ControllerResult<()> {
        self.device.initialize()?;
        self.set_dimming(DEFAULT_DIMMING)?;
        // Clear the display since it will show whatever was last stored
        self.device.set_display_data(&mut [0u8; ht16k33::DISPLAY_DATA_LENGTH])?;
        self.set_display_on(true)?;
        Ok(())
    }

    /// Sets how dim the display should be, where `dimming` is 0-15. 0 being
    /// the most dim but still on.
    pub fn set_dimming(&mut self, dimming: u8) -> ControllerResult<()> {
        self.device.set_dimming(dimming)?;
        Ok(())
    }

    /// Enables/disables displaying the LEDs that are set to be on.
    pub fn set_display_on(&mut self, on: bool) -> ControllerResult<()> {
        self.device.set_display_on(on)?;
        Ok(())
    }

    /// Displays `str` as characters adapted for a 7-segment display.
    /// Only digits, dash, and ASCII letters are supported. Letters K, M, Q, V,
    /// W, X, and Z are not supported. Any unsupported characters will show as
    /// blanks. Case of letters is ignored. If more characters are supplied than
    /// the display can show, any extra characters will be ignored.
    pub fn display_str(&mut self, str: &str) -> ControllerResult<()> {
        let display_data = &mut [0u8; ht16k33::DISPLAY_DATA_LENGTH];
        let mut position = 0;

        for char in str.chars() {
            let row = position_to_row(position);
            display_data[row] = char_to_data(char);
            position += 1;
        
            if position >= MAX_DIGITS {
                break;
            }
        }
    
        self.device.set_display_data(display_data)?;
        Ok(())
    }

    /// Displays `n` as digits on the display. This handles showing the decimal
    /// point and negative sign. It will prioritize showing the negative sign
    /// and all integer digits over decimal digits, which will be rounded and
    /// truncated to fit. If `n` is a number with more integer digits than will
    /// fit on the display, display behavior is undefined but no error will occur.
    pub fn display_digits(&mut self, n: f32) -> ControllerResult<()> {
        // The negative sign takes up a digit
        let max_digits = if n < 0f32 {
            MAX_DIGITS - 1
        } else {
            MAX_DIGITS
        };

        // max_number represents the maximum number the display can show. So for
        // MAX_DIGITS=4, max_number would be 9999 for a non-negative n.
        let max_number = 10u32.pow(max_digits as u32) - 1;

        let mut decimal_places = min(self.max_decimal_places, max_digits);
        let mut scale = 10u32.pow(decimal_places as u32);

        // Round n to the current decimal places and see if it fits within the
        // digits we have to work with. If it doesn't fit, try rounding to the
        // next higher decimal place and see if that works, etc.
        let digits = loop {
            // Round with decimal_places precision and discard the decimal point
            let d = libm::roundf(libm::fabsf(n) * scale as f32) as u32;

            // Stop if the number fits within the digits we have
            if d <= max_number || decimal_places == 0 {
                break d;
            }

            scale /= 10;
            decimal_places -= 1;
        };

        let display_data = digits_to_display_data(digits, n < 0f32, decimal_places);
        self.device.set_display_data(&display_data)?;
        Ok(())
    }
}

/// Converts `digits` to the display data bytes required by the HT16K33.
/// required to display those digits, plus an optional `negative` sign and
/// a decimal point such that digits has that many `decimal_places`.
fn digits_to_display_data(
    digits: u32,
    negative: bool,
    decimal_places: usize,
) -> [u8; ht16k33::DISPLAY_DATA_LENGTH] {
    let mut curr_digits = digits;
    let mut last_blank_row = MAX_DIGITS - 1;
    let mut display_data = [0u8; ht16k33::DISPLAY_DATA_LENGTH];

    for i in 0..MAX_DIGITS {
        // Iterate the display digits from right to left
        let row = position_to_row(MAX_DIGITS - i - 1);
        let is_ones = decimal_places == i;
        let digit = (curr_digits % 10) as u8;
        curr_digits /= 10;

        // Show a blank if we've passed the first digit of the number,
        // unless this is 1's digit or decimal digits.
        // Always show non-zero digits. Only show zero if the current
        // position is at the decimal point (e.g. "0.") or after, or if
        // there will be other non-zero digits to the left of this one.
        if digit != 0 || decimal_places >= i || curr_digits != 0  {
            // Show the decimal point if this is the ones position but
            // not the rightmost digit.
            let decimal = is_ones && i != 0;
            display_data[row] = digit_to_data(digit, decimal);
        } else {
            last_blank_row = row;
        }
    }

    // Show the negative sign (if needed) at the last seen blank row
    if negative {
        display_data[last_blank_row] = NEGATIVE_SIGN_DATA;
    }

    display_data
}

/// Converts a `position` on the display to the HT16K33 row that it is connected
/// to. `position` is 0-based with 0 being the leftmost digit on the display.
fn position_to_row(position: usize) -> usize {
    // The rows are connected such that even rows are connected to the first
    // digits. Except row 4 is connected to the colon in the middle of the
    // display, so we skip that one.
    match position {
        0 => 0,
        1 => 2,
        2 => 6,
        _ => 8,
    }
}

/// Converts `digit` to the corresponding 7-segment bits. If `decimal` is true,
/// the digit's trailing decimal point will also be turned on.
/// The bits are (with 0 as LSB):
/// ```text
///   —0— 
///  |   |
///  5   1
///  |   |
///   —6—
///  |   |
///  4   2
///  |   |
///   —3—  7.
/// ```
fn digit_to_data(digit: u8, decimal: bool) -> u8 {
    const LOOKUP: [u8; 10] = [
        0b0011_1111,
        0b0000_0110,
        0b0101_1011,
        0b0100_1111,
        0b0110_0110,
        0b0110_1101,
        0b0111_1101,
        0b0000_0111,
        0b0111_1111,
        0b0110_1111,
    ];

    let mut digit_data = LOOKUP[(digit % 10) as usize];
    
    if decimal {
        digit_data |= 0b1000_0000;
    }

    digit_data
}

fn char_to_data(char: char) -> u8 {
    match char.to_ascii_uppercase() {
        '0'..='9' => {
            if let Some(d) = char.to_digit(10) {
                digit_to_data(d as u8, false)
            } else {
                0
            }
        },
        'A' => 0b0111_0111,
        'B' => 0b0111_1100,
        'C' => 0b0011_1001,
        'D' => 0b0101_1110,
        'E' => 0b0111_1001,
        'F' => 0b0111_0001,
        'G' => 0b0011_1101,
        'H' => 0b0111_0110,
        'I' => 0b0011_0000,
        'J' => 0b0001_1110,
        'L' => 0b0011_1000,
        'N' => 0b0101_0100,
        'O' => 0b0011_1111,
        'P' => 0b0111_0011,
        'R' => 0b0101_0000,
        'S' => 0b0110_1101,
        'T' => 0b0111_1000,
        'U' => 0b0011_1110,
        'Y' => 0b0110_1110,
        '-' => NEGATIVE_SIGN_DATA,
        _ => 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::mock_i2c::*;

    fn expect_display(i2c: &mut MockI2c, address: u8, d0: u8, d1: u8, d2: u8, d3:u8)
    {
        let mut bytes = [0u8; ht16k33::DISPLAY_DATA_LENGTH + 1];
        // Offset by 1 because the first byte is a 0x00 address
        bytes[1] = d0;
        bytes[3] = d1;
        bytes[7] = d2;
        bytes[9] = d3;
        i2c.expect_write(address, &bytes);
    }

    #[test]
    fn test_new() {
        let address = 0x70;
        let i2c = MockI2c::new();
        let max_decimal_places = 3;

        let display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);

        assert_eq!(display.max_decimal_places, max_decimal_places);
    }

    #[test]
    fn test_initialize() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        i2c.expect_write(address, &[0b0010_0001]);
        i2c.expect_write(address, &[0b1110_1111]);
        expect_display(&mut i2c, address, 0, 0, 0, 0);
        i2c.expect_write(address, &[0b1000_0001]);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.initialize().unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_dimming() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        i2c.expect_write(address, &[0b1110_1010]);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        let dimming = 10;
        display.set_dimming(dimming).unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_set_display_on() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        i2c.expect_write(address, &[0b1000_0001]);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.set_display_on(true).unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_str() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        expect_display(&mut i2c, address, 0b0111_0111, 0b0111_1100, 0b0011_1001, 0b0101_1110);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_str("ABCD").unwrap();

        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_str_ignores_case() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        expect_display(&mut i2c, address, 0b0111_0111, 0b0111_1100, 0b0011_1001, 0b0101_1110);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_str("abcd").unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_str_shows_digits() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        expect_display(&mut i2c, address, 0b0011_1111, 0b0000_0110, 0b0100_0000, 0b0111_1111);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_str("01-8").unwrap();

        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_no_rounding() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        expect_display(&mut i2c, address, 0b1100_1111, 0b0000_0110, 0b0110_0110, 0b0000_0110);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(3.141).unwrap();

        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_rounding_up() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        expect_display(&mut i2c, address, 0b1100_1111, 0b0000_0110, 0b0110_0110, 0b0101_1011);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(3.1415).unwrap();

        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_rounding_down() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        expect_display(&mut i2c, address, 0b1100_1111, 0b0000_0110, 0b0110_0110, 0b0000_0110);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(3.1414).unwrap();
          
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_less_than_max_digits() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Digits should be right aligned
        expect_display(&mut i2c, address, 0, 0, 0b1100_1111, 0b0000_0110);

        let max_decimal_places = 1;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(3.1).unwrap();

        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_zero_pads_decimal_places() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Adds 0s to the right to fill out max decimal places
        expect_display(&mut i2c, address, 0b1100_1111, 0b0000_0110, 0b0011_1111, 0b0011_1111);

        let max_decimal_places = 3;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(3.1).unwrap();

        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_has_leading_zero_for_decimal_only() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // If the integer number is 0, the number before the decimal point is 0
        expect_display(&mut i2c, address, 0, 0b1011_1111, 0b0100_1111, 0b0000_0110);

        let max_decimal_places = 2;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(0.31).unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_prioritizes_integer_over_decimals() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Should show "1234", sacrificing decimals to show the whole integer.
        // Also, The last digit does not show the decimal point since there are
        // no decimal numbers to display.
        expect_display(&mut i2c, address, 0b0000_0110, 0b0101_1011, 0b0100_1111, 0b0110_1101);

        let max_decimal_places = 4;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(1234.5678).unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }

    #[test]
    fn test_display_digits_prioritizes_negative_over_decimals() {
        let address = 0x70;
        let mut i2c = MockI2c::new();
        // Should show "-3.14", sacrificing decimals to show the "-" plus integer
        expect_display(&mut i2c, address, 0b0100_0000, 0b1100_1111, 0b0000_0110, 0b0110_0110);
        
        let max_decimal_places = 4;
        let mut display = SevenSegmentDisplay::new(i2c, address, max_decimal_places);
        
        display.display_digits(-3.1415).unwrap();
    
        let i2c = display.destroy();
        i2c.verify();
    }
}