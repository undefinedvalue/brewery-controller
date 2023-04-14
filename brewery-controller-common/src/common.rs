#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

/// Maps all errors from the expression to a ControllerError, but logs the given
/// reason and some debug information first.
#[macro_export]
macro_rules! reason {
    ($has_result:expr, $reason:literal) => {
        $has_result.map_err(|e| {
            log::error!($reason);
            log::error!("Error at line {} in {}: {:?}", line!(), file!(), e);
            crate::common::ControllerError
        })
    }
}

#[derive(Debug)]
pub struct ControllerError;
pub type ControllerResult<T> = core::result::Result<T, ControllerError>;

pub mod time {
    // 16 MHz is the value of esp32c3_hal::systimer::SystemTimer::TICKS_PER_SECOND
    // A more generic library would have to use const generics every time an instant
    // was needed, which ends up being super annoying. Just hardcode it here.
    const HERTZ: u32 = 16_000_000;
    pub type Instant = fugit::TimerInstantU64<HERTZ>;
    pub type Duration = fugit::TimerDurationU64<HERTZ>;

    pub const ZERO_INSTANT: Instant = Instant::from_ticks(0);

    pub trait RealTimeClock {
        fn now(&self) -> Instant;
    }

    pub trait Delay {
        fn delay(&self, duration: Duration);
    }
}