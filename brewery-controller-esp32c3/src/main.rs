#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(never_type)]

#[cfg(feature = "server")]
use embassy_executor::Executor;
#[cfg(feature = "server")]
use esp32c3_hal::{embassy, Rng};
#[cfg(feature = "server")]
mod server;

use esp32c3_hal::{
    clock::{ClockControl, Clocks, CpuClock},
    dma::{DmaPriority, ChannelTx, ChannelRx},
    gdma::{Gdma, Channel0TxImpl, Channel0RxImpl, SuitablePeripheral0, Channel0},
    gpio::{DriveStrength, GpioPin, Input, PullUp, Bank0GpioRegisterAccess, SingleCoreInteruptStatusRegisterAccessBank0, InputOutputAnalogPinType, Gpio4Signals, InputOutputPinType, Gpio5Signals, Output, PushPull, Gpio19Signals},
    peripherals::{Peripherals, I2C0, SPI2},
    prelude::*,
    spi::{dma::{WithDmaSpi2, SpiDma}, Spi, SpiMode, FullDuplexMode},
    systimer::SystemTimer,
    timer::TimerGroup,
    Rtc,
    IO, peripheral::Peripheral
};
//use esp_backtrace as _;
use panic_halt as _;
use shared_bus::{BusManagerSimple, I2cProxy, NullMutex};
use static_cell::StaticCell;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use brewery_controller_common::{
    common::{ControllerError, ControllerResult},
    common::time::{Instant, RealTimeClock, Duration},
    max31865::Max31865,
    pid::Pid,
    pid::PidConfig,
    rotary_encoder::RotaryEncoder,
    seven_segment_display::SevenSegmentDisplay,
    solid_state_relay::SolidStateRelay,
    temperature_stats::{Calibration, TemperatureStats},
};

mod esp_logger;

const TEMP_MAX_DEC_PLACES: usize = 1;
const ROTARY_DISPLAY_MAX_DEC_PLACES: usize = 0;
const TARGET_TEMP_ROTARY_ADDR: u8 = 0x36;
const TEMP_DISP_ADDR: u8 = 0x72;
const TARGET_TEMP_DISP_ADDR: u8 = 0x70;
const POWER_PERCENT_ROTARY_ADDR: u8 = 0x37;
const POWER_DISP_ADDR: u8 = 0x71;

const TARGET_TEMP_MIN: i32 = 50;
const TARGET_TEMP_MAX: i32 = 212;
const TARGET_TEMP_DEFAULT: i32 = TARGET_TEMP_MIN;

const POWER_PERCENT_MIN: i32 = 0;
const POWER_PERCENT_MAX: i32 = 100;
const POWER_PERCENT_DEFAULT: i32 = POWER_PERCENT_MIN;

const MAX31865_REFERENCE_RESISTANCE_OHMS: u32 = 4300;

const DEVICE_UPDATE_INTERVAL: Duration = Duration::millis(100);
const PID_UPDATE_INTERVAL: Duration = Duration::millis(1000);
#[cfg(feature = "server")]
const SERVER_UPDATE_INTERVAL: Duration = Duration::millis(1000);

const INITIAL_PID_CONFIG: PidConfig = PidConfig {
    controller_gain: 0.5,
    integral_time: 540.0,
    derivative_time: 14.0,
};

// Data is generated at 60Hz, so 60 samples represents a 1 second window of data
const N_TEMP_SAMPLES: usize = 60;
#[cfg(feature = "calibration")]
const RTD_CALIBRATION: [Calibration; 0] = [];
#[cfg(not(feature = "calibration"))]
const RTD_CALIBRATION: [Calibration; 7] = [
    Calibration { actual: 53.9, expected: 53.0 },
    Calibration { actual: 72.4, expected: 72.0 },
    Calibration { actual: 99.3, expected: 99.3 },
    Calibration { actual: 119.5, expected: 120.0 },
    Calibration { actual: 149.4, expected: 150.0 },
    Calibration { actual: 166.8, expected: 167.8 },
    Calibration { actual: 210.6, expected: 211.5 },
];
const N_CALIBRATIONS: usize = RTD_CALIBRATION.len();

#[cfg(feature = "server")]
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

static CLOCKS: StaticCell<Clocks> = StaticCell::new();
static I2C_BUS: StaticCell<BusManagerSimple<esp32c3_hal::i2c::I2C<I2C0>>> = StaticCell::new();
static SPI_TX_DESCRIPTORS: StaticCell<[u32; 24]> = StaticCell::new();
static SPI_RX_DESCRIPTORS: StaticCell<[u32; 24]> = StaticCell::new();

#[allow(non_snake_case)]
#[entry]
fn main() -> ! {
    esp_logger::init_logger(log::LevelFilter::Info);
    info!("Controller initializing...");

    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = CLOCKS.init_with(|| {
        ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze()
    });
    let delay = Delay { clocks };

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = esp32c3_hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio0,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let i2c_bus = I2C_BUS.init_with(|| BusManagerSimple::new(i2c));

    info!("MAX31865 SPI temperature device initializing...");
    let temperature_device = {
        // Use SPI with DMA because that seems to be the only SPI implementation
        // that actually works.
        let gdma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
        let gdma_channel = gdma.channel0;

        let spi = Spi::new(
            peripherals.SPI2,
            io.pins.gpio6,  // clk
            io.pins.gpio7,  // mosi
            io.pins.gpio2,  // miso
            io.pins.gpio10, // cs
            5u32.MHz(),
            SpiMode::Mode3,
            &mut system.peripheral_clock_control,
            &clocks,
        )
        .with_dma(gdma_channel.configure(
            false,
            SPI_TX_DESCRIPTORS.init_with(|| [0u32; 24]),
            SPI_RX_DESCRIPTORS.init_with(|| [0u32; 24]),
            DmaPriority::Priority0,
        ));

        Max31865::new(spi, delay, MAX31865_REFERENCE_RESISTANCE_OHMS)
    };

    let temperature_display = SevenSegmentDisplay::new(
        i2c_bus.acquire_i2c(),
        TEMP_DISP_ADDR,
        TEMP_MAX_DEC_PLACES
    );

    let target_temp_rotary = RotaryEncoder::new(
        i2c_bus.acquire_i2c(),
        TARGET_TEMP_ROTARY_ADDR,
        delay,
        TARGET_TEMP_MIN,
        TARGET_TEMP_MAX,
        TARGET_TEMP_DEFAULT,
    );
    let target_temp_display = SevenSegmentDisplay::new(
        i2c_bus.acquire_i2c(),
        TARGET_TEMP_DISP_ADDR,
        ROTARY_DISPLAY_MAX_DEC_PLACES,
    );

    let power_percent_rotary = RotaryEncoder::new(
        i2c_bus.acquire_i2c(),
        POWER_PERCENT_ROTARY_ADDR,
        delay,
        POWER_PERCENT_MIN,
        POWER_PERCENT_MAX,
        POWER_PERCENT_DEFAULT,
    );
    let power_percent_display = SevenSegmentDisplay::new(
        i2c_bus.acquire_i2c(),
        POWER_DISP_ADDR,
        ROTARY_DISPLAY_MAX_DEC_PLACES,
    );

    let temperature_ready = io.pins.gpio4.into_pull_up_input();

    // GPIO 18 and 19 default to using USB signals, ignoring any output we set.
    // https://github.com/esp-rs/esp-hal/issues/279
    peripherals.USB_DEVICE.conf0.modify(|_,w| w.usb_pad_enable().clear_bit());

    let mut target_temp_ssr_pin = io.pins.gpio5.into_push_pull_output();
    let mut power_percent_ssr_pin = io.pins.gpio19.into_push_pull_output();
    target_temp_ssr_pin.set_drive_strength(DriveStrength::I5mA);
    power_percent_ssr_pin.set_drive_strength(DriveStrength::I5mA);

    let target_temp_ssr = SolidStateRelay::new(target_temp_ssr_pin, Time);
    let power_percent_ssr = SolidStateRelay::new(power_percent_ssr_pin, Time);

    let mut devices = Devices {
        temperature_ready,
        temperature_device,
        temperature_display,
        target_temp_rotary,
        target_temp_display,
        target_temp_ssr,
        power_percent_rotary,
        power_percent_display,
        power_percent_ssr,
    };

    // Delay to allow connected devices to turn on
    //delay.delay(Duration::millis(500));

    info!("Initializing devices...");
    devices.initialize().unwrap();

    #[cfg(feature = "server")]
    {
        let mut rng = Rng::new(peripherals.RNG);
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;
        esp_wifi::init_heap();

        esp_wifi::initialize(
            SystemTimer::new(peripherals.SYSTIMER).alarm0,
            rng,
            system.radio_clock_control,
            &clocks,
        )
        .unwrap();


        embassy::init(&clocks, timer_group0.timer0);

        let executor = EXECUTOR.init_with(Executor::new);
        executor.run(|spawner| {
            spawner.must_spawn(server::run_server_task(
                spawner,
                seed,
                peripherals.RADIO,
            ));
            spawner.must_spawn(run_devices_task(devices));
        });
    }

    #[cfg(not(feature = "server"))]
    {
        run_devices(devices);
    }
}

#[cfg(feature = "server")]
#[embassy_executor::task]
async fn run_devices_task(mut devices: Devices<'static>) -> ! {
    // Execute the main run loop. It only returns if there is an error.
    let run_result = devices.run().await;

    devices.turn_off_error();
    panic!("Fatal error in run: {:?}", run_result.err());
}

#[cfg(not(feature = "server"))]
fn run_devices(mut devices: Devices<'_>) -> ! {
    // Execute the main run loop. It only returns if there is an error.
    let run_result = embassy_futures::block_on(devices.run());

    // Show an error code on the displays and panic
    devices.turn_off_error();
    panic!("Fatal error in run: {:?}", run_result.err());
}

type I2C<'a> = I2cProxy<'a, NullMutex<esp32c3_hal::i2c::I2C<'a, <I2C0 as Peripheral>::P>>>;
type Pin<D, T, S, const N: u8> = GpioPin<D, Bank0GpioRegisterAccess, SingleCoreInteruptStatusRegisterAccessBank0, T, S, N>;
type Pin4<D> = Pin<D, InputOutputAnalogPinType, Gpio4Signals, 4>;
type Pin5<D> = Pin<D, InputOutputAnalogPinType, Gpio5Signals, 5>;
type Pin19<D> = Pin<D, InputOutputPinType, Gpio19Signals, 19>;

#[derive(PartialEq)]
enum UpdateState {
    TargetTempRotary,
    TargetTempDisplay,
    PowerPercentRotary,
    PowerPercentDisplay,
    TemperatureDisplay,
}

struct Devices<'a> {
    temperature_ready: Pin4<Input<PullUp>>,
    temperature_device: Max31865<SpiDma<'a, SPI2, ChannelTx<'a, Channel0TxImpl, Channel0>, ChannelRx<'a, Channel0RxImpl, Channel0>, SuitablePeripheral0, FullDuplexMode>, Delay>,
    temperature_display: SevenSegmentDisplay<I2C<'a>>,
    target_temp_rotary: RotaryEncoder<I2C<'a>, Delay>,
    target_temp_display: SevenSegmentDisplay<I2C<'a>>,
    target_temp_ssr: SolidStateRelay<Pin5<Output<PushPull>>, Time>,
    power_percent_rotary: RotaryEncoder<I2C<'a>, Delay>,
    power_percent_display: SevenSegmentDisplay<I2C<'a>>,
    power_percent_ssr: SolidStateRelay<Pin19<Output<PushPull>>, Time>,
}

impl Devices<'_> {
    fn initialize(&mut self) -> ControllerResult<()> {
        self.target_temp_ssr.initialize();
        self.power_percent_ssr.initialize();

        self.target_temp_rotary.initialize().unwrap();
        self.power_percent_rotary.initialize().unwrap();

        self.temperature_device.initialize()?;
        self.temperature_device.start_auto_conversions()?;

        // Read the data in order to reset DRDY. The data is stale from some
        // other session, so just discard it.
        self.temperature_device.read_temperature()?;

        // Initialize and turn on the displays
        self.temperature_display.initialize()?;
        self.target_temp_display.initialize()?;
        self.power_percent_display.initialize()?;

        Self::show_value(
            &mut self.target_temp_rotary,
            &mut self.target_temp_display
        )?;
        Self::show_value(
            &mut self.power_percent_rotary,
            &mut self.power_percent_display,
        )?;

        Ok(())
    }

    async fn run(&mut self) -> Result<!, ControllerError> {
        // Fetch the initial temperature for initializing the PID and stats
        let initial_temp = loop {
            if let Ok(true) = embedded_hal::digital::InputPin::is_low(&self.temperature_ready) {
                break self.temperature_device.read_temperature()?;
            }
        };

        let mut pid = Pid::new(initial_temp, INITIAL_PID_CONFIG);
        let mut stats = TemperatureStats::<N_TEMP_SAMPLES, N_CALIBRATIONS>::new(initial_temp, &RTD_CALIBRATION);

        info!("Running!");

        // Stagger the updates of the devices, PID, and server
        const INITIAL_UPDATE_STATE: UpdateState = UpdateState::TargetTempRotary;
        let mut next_update = Time.now();
        let mut update_state = INITIAL_UPDATE_STATE;
        let mut next_pid_update = next_update + DEVICE_UPDATE_INTERVAL / 2;
        #[cfg(feature = "server")]
        let mut next_server_update = next_pid_update + DEVICE_UPDATE_INTERVAL / 4;

        loop {
            // Always update the SSRs since they need to maintain a PWM signal
            self.target_temp_ssr.update();
            self.power_percent_ssr.update();

            // Always check the temperature_ready interrupt pin and record a new
            // temperature sample if it is set.
            if let Ok(true) = embedded_hal::digital::InputPin::is_low(&self.temperature_ready) {
                stats.add_sample(self.temperature_device.read_temperature()?);
            }

            // Every so often, yield to the executor to allow the server task to
            // run. Otherwise it won't ever run since this task never awaits.
            #[cfg(feature = "server")]
            embassy_futures::yield_now().await;

            let now = Time.now();

            // Only update one device per loop iteration in order to minimize
            // the time taken per iteration.
            if now >= next_update {
                update_state = match update_state {
                    UpdateState::TargetTempRotary => {
                        if let Ok(true) = self.target_temp_rotary.update() {
                            UpdateState::TargetTempDisplay
                        } else {
                            UpdateState::PowerPercentRotary
                        }
                    },
                    UpdateState::TargetTempDisplay => {
                        Self::show_value(&mut self.target_temp_rotary, &mut self.target_temp_display)?;
                        UpdateState::PowerPercentRotary
                    },
                    UpdateState::PowerPercentRotary => {
                        if let Ok(true) = self.power_percent_rotary.update() {
                            let value = self.power_percent_rotary.value();
                            self.power_percent_ssr.set_power_percent(value as u32);
                            UpdateState::PowerPercentDisplay
                        } else {
                            UpdateState::TemperatureDisplay
                        }
                    },
                    UpdateState::PowerPercentDisplay => {
                        Self::show_value(&mut self.power_percent_rotary, &mut self.power_percent_display)?;
                        UpdateState::TemperatureDisplay
                    }
                    UpdateState::TemperatureDisplay => {
                        self.temperature_display.display_digits(stats.temperature())?;
                        INITIAL_UPDATE_STATE
                    },

                };
            
                if update_state == INITIAL_UPDATE_STATE {
                    next_update += DEVICE_UPDATE_INTERVAL;
                }
            }

            // Update the PID and use its output to set the target temp SSR
            if now >= next_pid_update {
                next_pid_update += PID_UPDATE_INTERVAL;

                #[cfg(feature = "server")]
                {
                    let pid_config = critical_section::with(|cs| {
                        server::PID_CONFIG.borrow(cs).get()
                    });
                    
                    pid.set_config(pid_config);
                }

                // Update the PID controller and target temp SSR
                let value = self.target_temp_rotary.value();
                pid.set_setpoint(value as f32);

                if value <= self.target_temp_rotary.min() {
                    pid.disable();
                } else if !pid.enabled() {
                    pid.enable(value as f32);
                }
            
                // Send the sample to the PID controller to determine the
                // appropriate power for the target temperature SSR.
                // The output will be 0 if it is not enabled.
                let pid_output = pid.control(stats.temperature());
                let power_percent = libm::roundf(pid_output * 100.0) as u32;
            
                if power_percent != self.target_temp_ssr.power_percent() {
                    self.target_temp_ssr.set_power_percent(power_percent);
                }
            }

            // Periodically send data to the server
            #[cfg(feature = "server")]
            if now >= next_server_update {
                next_server_update += SERVER_UPDATE_INTERVAL;

                let data = server::DisplayData {
                    time: now.duration_since_epoch().to_millis(),
                    temperature: stats.temperature(),
                    setpoint: pid.setpoint(),
                    power1: self.target_temp_ssr.power_percent(),
                    power2: self.power_percent_ssr.power_percent(),
                    pid: pid.config(),
                };

                server::DATA_SIGNAL.signal(data);
            }
        }
    }

    fn show_value(
        rotary: &mut RotaryEncoder<I2C, Delay>,
        display: &mut SevenSegmentDisplay<I2C>,
    ) -> ControllerResult<()> {
        let value = rotary.value();

        if value <= rotary.min() {
            display.display_str("OFF")
        } else {
            display.display_digits(value as f32)
        }
    }

    fn turn_off_error(mut self) {
        self.target_temp_ssr.turn_off();
        self.power_percent_ssr.turn_off();

        // Ignore errors while displaying. Hopefully one of the displays works.
        self.temperature_display
            .display_str("Err")
            .unwrap_or_default();

        self.target_temp_rotary.set_to_min();
        self.power_percent_rotary.set_to_min();
        self.target_temp_display
            .display_str("Err")
            .unwrap_or_default();
        self.power_percent_display
            .display_str("Err")
            .unwrap_or_default();
    }
}

struct Time;

impl RealTimeClock for Time {
    fn now(&self) -> Instant {
        Instant::from_ticks(SystemTimer::now())
    }
}

#[derive(Copy, Clone)]
struct Delay {
    clocks: &'static Clocks<'static>
}

impl Delay {
    fn delay(&self, duration: Duration) {
        let mut delay = esp32c3_hal::Delay::new(self.clocks);

        let micros = duration.to_micros();
        let ms = micros / 1000;
        let us = micros % 1000;
        
        if ms > 0 {
            embedded_hal::delay::DelayUs::delay_ms(&mut delay, ms as u32).unwrap_or_default();
        }

        if us > 0 {
            embedded_hal::delay::DelayUs::delay_us(&mut delay, us as u32).unwrap_or_default();
        }
    }
}

impl brewery_controller_common::common::time::Delay for Delay {
    fn delay(&self, duration: Duration) {
        Delay::delay(&self, duration);
    }
}