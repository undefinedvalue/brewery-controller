//! Temperature statistics helper for aggregating and processing high frequency
//! temperature data.

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::circular_buffer::CircularBuffer;

#[derive(Copy, Clone, Debug)]
pub struct Calibration {
    pub actual: f32,
    pub expected: f32,
}

#[derive(Copy, Clone, Debug)]
struct ComputedCalibration {
    actual: f32,
    expected: f32,
    slope: f32,
}

pub struct TemperatureStats<const N_SAMPLES: usize, const N_CALIBRATIONS: usize> {
    samples: CircularBuffer<N_SAMPLES>,
    calibration_data: [ComputedCalibration; N_CALIBRATIONS],
}

impl<const N_SAMPLES: usize, const N_CALIBRATIONS: usize> TemperatureStats<N_SAMPLES, N_CALIBRATIONS> {
    pub fn new(initial_temperature: f32, calibration_data: &[Calibration; N_CALIBRATIONS]) -> Self {
        // Precompute the slopes of the calibrations. Each data point stores
        // the slope of the linear interpolation between it and the previous point.
        let def = ComputedCalibration { actual: 0.0, expected: 0.0, slope: 0.0};
        let mut computed_calibrations = [def; N_CALIBRATIONS];

        if N_CALIBRATIONS > 1 {
            for i in 1..N_CALIBRATIONS {
                let prev = calibration_data[i - 1];
                let curr = calibration_data[i];

                computed_calibrations[i] = ComputedCalibration {
                    actual: curr.actual,
                    expected: curr.expected,
                    slope: (curr.expected - prev.expected) / (curr.actual - prev.actual),
                }
            }

            // The slope of the first calibration is the same as the second
            computed_calibrations[0] = ComputedCalibration {
                actual: calibration_data[0].actual,
                expected: calibration_data[0].expected,
                slope: computed_calibrations[1].slope,
            };
        }

        Self {
            samples: CircularBuffer::new(initial_temperature),
            calibration_data: computed_calibrations,
        }
    }

    /// Returns the current aggregated temperature, or None if not enough
    /// samples have been added yet.
    pub fn temperature(&self) -> f32 {
        let avg = self.samples.avg();

        if N_CALIBRATIONS == 0 {
            avg
        } else {
            self.apply_calibration(avg)
        }
    }

    /// Adds a temperature data sample to be aggregated.
    pub fn add_sample(&mut self, temperature: f32) {
        self.samples.add(temperature);
    }

    /// Applies calibration to the raw temperature by performing linear
    /// interpolation across the calibration data points:
    fn apply_calibration(&self, raw_temp: f32) -> f32 {
        // 1. Find the last calibration point the raw_temp is less than
        let mut cal_pt = self.calibration_data[N_CALIBRATIONS - 1];

        for i in (0..self.calibration_data.len()).rev() {
            if raw_temp <= self.calibration_data[i].actual {
                cal_pt = self.calibration_data[i];
            } else {
                break;
            }
        }

        // 2. Interpolate between those two points at the raw_temp
        cal_pt.expected + ((raw_temp - cal_pt.actual) * cal_pt.slope)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const N_SAMPLES: usize = 16;
    const INIT_TEMP: f32 = 70.0;
    const EMPTY_CALIBRATION: [Calibration; 0] = [];
    const CALIBRATION_DATA: [Calibration; 4] = [
        Calibration { actual: 68.0, expected: 70.0 },
        Calibration { actual: 103.0, expected: 100.0 },
        Calibration { actual: 150.0, expected: 150.0 },
        Calibration { actual: 210.0, expected: 212.0 },
    ];

    type Stats = TemperatureStats::<N_SAMPLES, 4>;
    type RawStats = TemperatureStats::<N_SAMPLES, 0>;

    #[test]
    fn test_new() {
        let stats = RawStats::new(INIT_TEMP, &EMPTY_CALIBRATION);

        assert_eq!(stats.temperature(), INIT_TEMP);
    }

    #[test]
    fn test_temperature_computes_average() {
        // No calibration to simplify test
        let mut stats = RawStats::new(INIT_TEMP, &EMPTY_CALIBRATION);

        for i in 0..N_SAMPLES {
            stats.add_sample(i as f32);
        }

        // The average of 0 + 1 + ... + 15 is the same as 15/2 = 7.5
        assert_eq!(stats.temperature(), 7.5);
    }

    #[test]
    fn test_add_sample_calibration_less_than_all() {
        // A data point less than all calibration points should use the same
        // interpolation slope as if it were between the first two points.
        test_calibration(47.0, 52.0);
    }

    #[test]
    fn test_add_sample_calibration_greater_than_all() {
        // A data point greater than all calibration points should use the same
        // interpolation slope as if it were between the last two points.
        test_calibration(225.0, 227.5);
    }

    #[test]
    fn test_add_sample_calibration() {
        test_calibration(75.0, 76.0);
        test_calibration(145.3, 145.0);
        test_calibration(171.0, 171.7);
    }

    #[test]
    fn test_add_sample_calibration_at_calibration_points() {
        // Ensure temperatures at the calibration points work
        for c in CALIBRATION_DATA {
            test_calibration(c.actual, c.expected);
        }
    }

    fn test_calibration(value: f32, expected: f32) {
        let mut stats = Stats::new(INIT_TEMP, &CALIBRATION_DATA);

        for _ in 0..N_SAMPLES {
            stats.add_sample(value);
        }

        let mut a = stats.temperature();
        let mut b = expected;

        if a > b {
            (a, b) = (b, a);
        }

        // Ensure values are very close to account for floating point imprecision
        assert!(1.0 - (a / b) < 0.000001, "value: {}, expected: {}, actual: {}", value, expected, stats.temperature());
    }

}