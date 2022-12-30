use crate::common::time::{Instant, self};
use crate::mcp23008::{Mcp23008Data, Pin};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

pub struct RotaryEncoder {
    pin_a: Pin,
    pin_b: Pin,
    value: i32,
    min: i32,
    max: i32,
    prev_next: u8,
    store: u8,
    last_updated_at: Instant,
}

impl RotaryEncoder {
    pub fn new(
        pin_a: Pin,
        pin_b: Pin,
        min: i32,
        max: i32,
        initial_value: i32
    ) -> RotaryEncoder {
        Self {
            pin_a,
            pin_b,
            value: initial_value,
            min,
            max,
            prev_next: 0,
            store: 0,
            last_updated_at: time::ZERO_INSTANT,
        }
    }

    pub fn value(&self) -> i32 {
        self.value
    }

    pub fn force_set_value(&mut self, value: i32) {
        self.value = value;
    }

    pub fn min(&self) -> i32 {
        self.min
    }

    pub fn max(&self) -> i32 {
        self.max
    }

    fn set_value(&mut self, time: Instant, clockwise: bool) -> bool {
        // Logarithmic value change based on how fast the encoder is rotating.
        // The values are experimentally determined according to usability.
        let rate = match (time - self.last_updated_at).to_millis() {
            0..=50 => 4,
            51..=100 => 2,
            _ => 1,
        };
        
        let prev_value = self.value;
        
        if clockwise {
            self.value = (self.value + rate).min(self.max);
        } else {
            self.value = (self.value - rate).max(self.min);
        }

        self.last_updated_at = time;
        self.value != prev_value    
    }
    
    pub fn update(&mut self, interrupt: Mcp23008Data, time: Instant) -> bool {
        if interrupt.is_interrupted(self.pin_a) || interrupt.is_interrupted(self.pin_b) {
            self.prev_next = (self.prev_next << 2) & 0b1111;
            
            if interrupt.is_high(self.pin_a) {
                self.prev_next |= 0b10;
            }
            if interrupt.is_high(self.pin_b) {
                self.prev_next |= 0b01;
            }

            match self.prev_next {
                  // Valid states are those where 1 bit has changed.
                  // On a CPU with a dedicated hamming weight instruction we
                  // may be able to optimize this, but the ESP32-C3 doesn't.
                0b0001 | 0b0010 | 0b0100 | 0b0111 | 0b1000 | 0b1011 | 0b1101 | 0b1110 => {
                    // store has two state changes: the previous state change in
                    // the higher 4 bits and the current state change in the
                    // lowest 4 bits.
                    self.store = (self.store << 4) | self.prev_next;

                    // Require 3 consecutive valid states that show A and B are
                    // out of phase.
                    match self.store {
                        0b0001_0111 => {
                            return self.set_value(time, false);
                        }
                        0b0010_1011 => {
                            return self.set_value(time, true);
                        }
                        _ => {}
                    }
                },
                _ => {}
            }
        }
    
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mocks::mock_time::MockTime, common::time::RealTimeClock};

    const PIN_A: Pin = Pin::GPIO0;
    const PIN_B: Pin = Pin::GPIO1;

    #[test]
    fn test_new() {
        let min = -100;
        let max = 100;
        let initial = 50;
        let rotary = RotaryEncoder::new(PIN_A, PIN_B, min, max, initial);
    
        assert_eq!(PIN_A, rotary.pin_a);
        assert_eq!(PIN_B, rotary.pin_b);
        assert_eq!(min, rotary.min());
        assert_eq!(max, rotary.max());
        assert_eq!(initial, rotary.value());
    }

    #[test]
    fn test_force_set_value() {
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 0);

        rotary.force_set_value(42);
        assert_eq!(42, rotary.value())
    }

    #[test]
    fn test_set_value_increments_clockwise() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 50);

        let result = rotary.set_value(time.now(), true);
        assert!(result);
        assert_eq!(51, rotary.value())
    }

    #[test]
    fn test_set_value_decrements_counterclockwise() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 50);

        let result = rotary.set_value(time.now(), false);
        assert!(result);
        assert_eq!(49, rotary.value())
    }

    #[test]
    fn test_set_value_limits_to_min() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 0);

        let result = rotary.set_value(time.now(), false);
        assert!(!result);
        assert_eq!(0, rotary.value())
    }

    #[test]
    fn test_set_value_limits_to_max() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 100);

        let result = rotary.set_value(time.now(), true);
        assert!(!result);
        assert_eq!(100, rotary.value())
    }

    #[test]
    fn test_set_value_increases_rate_clockwise() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 50);

        // The first time increments by 1
        rotary.set_value(time.now(), true);
        assert_eq!(51, rotary.value());
        
        // The second time should increment by more than 1 since the time delta
        // is very small.
        rotary.set_value(time.now(), true);
        assert!(rotary.value() > 52);
    }

    #[test]
    fn test_set_value_increases_rate_counterclockwise() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 50);

        // The first time decrements by 1
        rotary.set_value(time.now(), false);
        assert_eq!(49, rotary.value());
        
        // The second time should decrement by more than 1 since the time delta
        // is very small.
        rotary.set_value(time.now(), false);
        assert!(rotary.value() < 48);
    }

    fn mock_interrupt(states: u8) -> Mcp23008Data {
        mock_interrupt_full(0b11, 0b00, states)
    }

    fn mock_interrupt_full(interrupts: u8, capture: u8, gpio: u8) -> Mcp23008Data {
        let pin_a_int_mask = if interrupts & 0b10 != 0 { PIN_A as u8} else { 0 };
        let pin_b_int_mask = if interrupts & 0b01 != 0 { PIN_B as u8} else { 0 };
        let pin_a_cap_mask = if capture & 0b10 != 0 { PIN_A as u8} else { 0 };
        let pin_b_cap_mask = if capture & 0b01 != 0 { PIN_B as u8} else { 0 };
        let pin_a_gpio_mask = if gpio & 0b10 != 0 { PIN_A as u8} else { 0 };
        let pin_b_gpio_mask = if gpio & 0b01 != 0 { PIN_B as u8} else { 0 };

        Mcp23008Data {
            interrupts: pin_a_int_mask | pin_b_int_mask,
            interrupt_capture: pin_a_cap_mask | pin_b_cap_mask,
            gpio: pin_a_gpio_mask | pin_b_gpio_mask,
        }
    }

    fn test_update(states: &[u8], diffs: &[i32]) {
        let mut time = MockTime::new();
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 50);

        assert_eq!(states.len(), diffs.len());

        for (&state, &diff) in states.iter().zip(diffs) {
            time.increment_millis(1000);
            let interrupt = mock_interrupt(state);
            let prev = rotary.value();

            let result = rotary.update(interrupt, time.now());
        
            assert_eq!((diff != 0), result);
            assert_eq!(prev + diff, rotary.value());
        }
    }

    #[test]
    fn test_update_noop_for_no_interrupts() {
        let mut time = MockTime::new();
        time.increment_millis(1000);
        let mut rotary = RotaryEncoder::new(PIN_A, PIN_B, 0, 100, 50);

        let interrupt = mock_interrupt_full(0b00, 0b00, 0b00);
        let result = rotary.update(interrupt, time.now());
        assert!(!result);
        assert_eq!(50, rotary.value());
    
        // Change one bit, which would otherwise cause an update
        let interrupt = mock_interrupt_full(0b00, 0b01, 0b01);
        let result = rotary.update(interrupt, time.now());
        assert!(!result);
        assert_eq!(50, rotary.value());
    }

    #[test]
    fn test_update_increments_clockwise() {
        let states = [0b00, 0b10, 0b11];
        let diffs  = [   0,    0,    1];
        test_update(&states, &diffs);
    }

    #[test]
    fn test_update_decrements_clockwise() {
        let states = [0b00, 0b01, 0b11];
        let diffs  = [   0,    0,    -1];
        test_update(&states, &diffs);
    }

    #[test]
    fn test_update_ignores_invalid_states() {
        // Going from 10 to 01 is invalid because both bits change
        let states = [0b00, 0b10, 0b01, 0b00, 0b10, 0b11];
        let diffs  = [   0,    0,    0,    0,    0,    1];
        test_update(&states, &diffs);
    }
}