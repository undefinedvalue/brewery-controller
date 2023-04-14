//! A Proportional-Integral-Derivative controller.

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::circular_buffer::CircularBuffer;

// This must be odd
const DERIVATIVE_WINDOW: usize = 9;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PidConfig {
    /// Units are "% controller output" / "process variable unit"
    /// Higher values increase the output of the P, I, and D terms.
    /// This should be set to a negative value if a positive control output
    /// is expected to cause a decrease in the process variable.
    pub controller_gain: f32,

    /// Units are in seconds. Higher values decrease the output of the I term.
    /// Set to 0.0 to disable the I term.
    pub integral_time: f32,

    /// Units are in seconds. Higher values increase the output of the D term.
    /// Set to 0.0 to disable the D term.
    pub derivative_time: f32,
}

pub struct Pid {
    config: PidConfig,
    enabled: bool,
    setpoint: f32,
    integral: f32,
    past_pvs: CircularBuffer<DERIVATIVE_WINDOW>,
}

impl PidConfig {
    fn proportional_gain(&self) -> f32 {
        self.controller_gain
    }

    fn integral_gain(&self) -> f32 {
        if self.integral_time == 0.0 {
            0.0
        } else {
            self.controller_gain / self.integral_time
        }
    }

    fn derivative_gain(&self) -> f32 {
        self.controller_gain * self.derivative_time
    }
}

impl Pid {
    pub fn new(initial_process_variable: f32, config: PidConfig) -> Self {
        Self {
            config,
            enabled: false,
            setpoint: 0.0,
            integral: 0.0,
            past_pvs: CircularBuffer::new(initial_process_variable),
        }
    }

    /// Returns whether this controller has been enabled. Default is false.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Enables this controller and sets the initial setpoint.
    pub fn enable(&mut self, initial_setpoint: f32) {
        self.set_setpoint(initial_setpoint);
        self.enabled = true;
    }

    /// Disables this controller. Until reenabled, `control` will return 0.0.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Returns the current setpoint.
    pub fn setpoint(&self) -> f32 {
        self.setpoint
    }

    /// Sets the current setpoint. If it is different than the old one, the
    /// integral is reset.
    pub fn set_setpoint(&mut self, setpoint: f32) {
        if setpoint != self.setpoint {
            self.integral = 0.0;
        }
        self.setpoint = setpoint;
    }

    /// Returns the current configuration.
    pub fn config(&self) -> PidConfig {
        self.config
    }

    /// Sets a new configuration.
    pub fn set_config(&mut self, config: PidConfig) {
        self.config = config;
    }

    /// Returns the control output as a ratio between 0.0 and 1.0 inclusive.
    /// `process_variable` is the most recent measured value and the PID
    /// controller will return outputs that should cause it to converge to
    /// the setpoint.
    pub fn control(&mut self, process_variable: f32) -> f32 {
        self.past_pvs.add(process_variable);

        if !self.enabled {
            return 0.0;
        }

        let error = self.setpoint - process_variable;

        // Proportional term
        let p_out = self.config.proportional_gain() * error;
        
        // Integral term
        let i_out = if self.integral == 0.0 && (p_out < -1.0 || p_out > 1.0) {
            // Only accumulate error if we are in a relatively controllable
            // range of the setpoint. Otherwise the integral grows too large.
            0.0
        } else {
            self.integral += error;
            self.config.integral_gain() * self.integral
        };
        
        // Derivative term
        // Negative because we are using the derivative of the process variable
        // instead of the the error (they are equal but opposite).
        let d_out = self.config.derivative_gain() * -self.derivative(); 
      
        (p_out + i_out + d_out).clamp(0.0, 1.0)
    }

    // Returns the derivative of self.past_pvs
    fn derivative(&self) -> f32 {
        // Compute the Savitzky–Golay filter for a 1st derivative quadratic
        // polynomial with window size DERIVATIVE_WINDOW.
        let mut sum = 0.0;
        for i in 0..DERIVATIVE_WINDOW {
            // Coefficients are centered on the middle datapoint.
            // e.g. for window=9: -4, -3, -2, -1, 0, 1, 2, 3, 4
            let c = i as f32 - (DERIVATIVE_WINDOW / 2) as f32;
            // Start with the oldest point
            sum += c * self.past_pvs.pastget(DERIVATIVE_WINDOW - 1 - i);
        }
    
        const N: f32 = DERIVATIVE_WINDOW as f32;
        const NORM: f32 = N * ((N * N) - 1.0) / 12.0;
        sum / NORM
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KC: f32 = 1.9;
    const TI: f32 = 3.5;
    const TD: f32 = 2.0;

    const CONFIG: PidConfig = PidConfig {
        controller_gain: KC,
        integral_time: TI,
        derivative_time: TD,
    };

    #[test]
    fn test_new() {
        let pid = Pid::new(0.0, CONFIG);

        assert_eq!(pid.config.proportional_gain(), KC);
        assert_eq!(pid.config.integral_gain(), KC / TI);
        assert_eq!(pid.config.derivative_gain(), KC * TD);
        assert!(!pid.enabled());
    }

    #[test]
    fn test_enable() {
        let mut pid = Pid::new(0.0, CONFIG);
        pid.enable(1.2);
        
        assert!(pid.enabled());
        assert_eq!(pid.setpoint(), 1.2);
    }

    #[test]
    fn test_disable() {
        let mut pid = Pid::new(0.0, CONFIG);
        pid.disable();
        
        assert!(!pid.enabled());
    }

    #[test]
    fn test_control_when_not_enabled_is_zero() {
        let mut pid = Pid::new(0.0, CONFIG);

        assert_eq!(pid.control(1.0), 0.0);
    }

    #[test]
    fn test_control_no_error_is_zero() {
        let pv = 100.0;
        let mut pid = Pid::new(pv, CONFIG);

        // Initial setpoint equals PV, so no error
        pid.enable(pv);
        
        // If PV is the same as SP, there is no error and the control output
        // should be 0.
        for _ in 0..10 {
            assert_eq!(pid.control(pv), 0.0);
        }
    }

    #[test]
    fn test_control_with_small_pos_error() {
        // Disable the derivative term for simpler math
        let mut config = CONFIG;
        config.derivative_time = 0.0;

        let pv1 = 10.0;
        let pv2 = 9.99;
        let mut pid = Pid::new(pv1, config);

        pid.enable(pv1);
        
        let error = pv1 - pv2;
        let p = KC * error;
        let i = (KC / TI) * error;
        assert_eq!(pid.control(pv2), p + i);
    }

    #[test]
    fn test_control_with_neg_error_is_zero() {
        let pv1 = 10.0;
        let pv2 = 10.01;
        let mut pid = Pid::new(pv1, CONFIG);

        pid.enable(pv1);
        
        // Should return 0 because the raw output is negative but the output is
        // clamped to [0, 1].
        assert_eq!(pid.control(pv2), 0.0);
    }

    #[test]
    fn test_control_with_large_pos_error_is_one() {
        let pv1 = 10.0;
        let pv2 = 11.0;
        let mut pid = Pid::new(pv1, CONFIG);
        
        pid.enable(pv1);
        
        // Should return 1 because the raw output is greater than 1 but the
        // output is clamped to [0, 1].
        assert_eq!(pid.control(pv2), 0.0);
    }

    #[test]
    fn test_control_accumulates_error_in_integral() {
        // Disable the derivative term for simpler math
        let mut config = CONFIG;
        config.derivative_time = 0.0;

        let pv1 = 10.0;
        let pv2 = 9.99;
        let mut pid = Pid::new(pv1, config);

        pid.enable(pv1);
        
        let error = pv1 - pv2;
        let mut e_sum = 0.0;

        for _ in 0..10 {
            let p = KC * error;
            e_sum += error;
            let i = (KC / TI) * e_sum;

            assert_eq!(pid.control(pv2), p + i);
        }
    }

    #[test]
    fn test_derivative_pos() {
        let mut pid = Pid::new(0.0, CONFIG);
    
        for i in 0..DERIVATIVE_WINDOW {
            pid.control(i as f32);
        }
    
        assert_eq!(pid.derivative(), 1.0);
    }

    #[test]
    fn test_derivative_neg() {
        let mut pid = Pid::new(0.0, CONFIG);
    
        for i in 0..DERIVATIVE_WINDOW {
            pid.control(-(i as f32));
        }
    
        assert_eq!(pid.derivative(), -1.0);
    }
}