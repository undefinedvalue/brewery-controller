use core::convert::Infallible;

use embedded_hal::digital::OutputPin;

use crate::common::time::{Instant, RealTimeClock, self, Duration};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

// Power percentage is converted to pulse-width modulation such that each
// percentage point equals 1/60th of a second (US AC line frequency is 60Hz and
// the SSR being used is of zero-crossing type).
const PERCENT_PERIOD: Duration = Duration::nanos(16666666);
// 100 PERCENT_PERIODs
const UPDATE_PERIOD: Duration = Duration::nanos(PERCENT_PERIOD.to_nanos() * 100);

pub struct SolidStateRelay<P, R> {
    control_pin: P,
    rtc: R,
    power_percent: u32,
    prev_toggle_time: Instant,
    next_toggle_time: Instant,
    curr_state: bool,
}

impl<P, R> SolidStateRelay<P, R>
where
    P: OutputPin<Error = Infallible>,
    R: RealTimeClock,
{
    pub fn new(control_pin: P, rtc: R) -> Self {
        Self {
            control_pin,
            rtc,
            power_percent: 0,
            prev_toggle_time: time::ZERO_INSTANT,
            next_toggle_time: time::ZERO_INSTANT,
            curr_state: false,
        }
    }

    pub fn initialize(&mut self) {
        self.set_low();
        self.prev_toggle_time = self.rtc.now();
    }

    /// Turns off the SSR and destroys self
    pub fn turn_off(mut self) {
        self.set_low();
    }

    fn update_next_toggle_time(&mut self) {
        let high_time = self.power_percent * PERCENT_PERIOD;
       
        if self.curr_state {
            // The pin is currently high, so keep it high for the time required
            // by the new power_percent.
            self.next_toggle_time = self.prev_toggle_time + high_time;
        } else {
            // The pin is currently low, so keep it low for the time required by
            // the new power_percent.
            self.next_toggle_time = self.prev_toggle_time + (UPDATE_PERIOD - high_time);
        }
    }

    pub fn power_percent(&self) -> u32 {
        self.power_percent
    }

    pub fn set_power_percent(&mut self, power_percent: u32) {
        self.power_percent = power_percent;
        self.update_next_toggle_time();
    }

    pub fn update(&mut self) {
        let now = self.rtc.now();
        
        if now >= self.next_toggle_time {
            // Toggle the pin
            if self.curr_state {
                self.set_low();
            } else {
                self.set_high();
            }
        
            self.prev_toggle_time = now;
            self.update_next_toggle_time();
        }
    }

    fn set_low(&mut self) {
        // Ignore errors since it is Infallible
        self.control_pin.set_low().unwrap_or_default();
        self.curr_state = false;
    }

    fn set_high(&mut self) {
        // Ignore errors since it is Infallible
        self.control_pin.set_high().unwrap_or_default();
        self.curr_state = true;
    }
}