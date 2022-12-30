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
}
