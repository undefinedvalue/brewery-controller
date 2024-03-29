use crate::common::time::{Duration, Instant, RealTimeClock, self, Delay};

pub struct MockTime {
    now: Instant,
}

impl MockTime {
    pub fn new() -> Self {
        Self { now: time::ZERO_INSTANT }
    }

    pub fn increment_millis(&mut self, millis: u64) {
        self.now += Duration::millis(millis);
    }
}

impl RealTimeClock for MockTime {
    fn now(&self) -> Instant {
        self.now
    }
}

pub struct MockDelay;
impl Delay for MockDelay {
    fn delay(&self, _: Duration) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        assert_eq!(time::ZERO_INSTANT, MockTime::new().now());
    }

    #[test]
    fn test_increment_millis() {
        let mut time = MockTime::new();
        time.increment_millis(1000);

        assert_eq!(time::ZERO_INSTANT + Duration::millis(1000), time.now());
    }
}