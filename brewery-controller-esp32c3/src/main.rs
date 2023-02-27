#![no_std]
#![no_main]

use core::convert::Infallible;

use embedded_hal::{i2c::I2c, digital::{OutputPin, InputPin}};
use esp32c3_hal::{
    clock::Clocks,
    clock::ClockControl,
    dma::DmaPriority,
    gdma::Gdma,
    i2c::I2C,
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    spi::{dma::WithDmaSpi2, Spi, SpiMode},
    systimer::SystemTimer,
    timer::TimerGroup,
    Delay, Rtc, IO,
    gpio::DriveStrength,
};
use esp_backtrace as _;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use shared_bus::BusManagerSimple;

use brewery_controller_common::{
    common::time::{Instant, RealTimeClock},
    max31865::{Max31865, Max31865ReadWrite, Max31865Error},
    mcp23008::{Mcp23008, Mcp23008ReadWrite, Pin, InterruptTrigger, Mcp23008Error},
    rotary_encoder::RotaryEncoder,
    seven_segment_display::{SevenSegmentDisplay, SevenSegmentDisplayWrite, DisplayError},
    solid_state_relay::SolidStateRelay,
};

mod esp_logger;

const TEMP_MAX_DEC_PLACES: usize = 1;
const ROTARY_DISPLAY_MAX_DEC_PLACES: usize = 0;
const TEMP_DISP_ADDR: u8 = 0x72;
const TARGET_TEMP_DISP_ADDR: u8 = 0x70;
const POWER_DISP_ADDR: u8 = 0x71;
const MCP23008_ADDR: u8 = 0x20;

const TARGET_TEMP_MIN: i32 = 70;
const TARGET_TEMP_MAX: i32 = 212;
const TARGET_TEMP_DEFAULT: i32 = TARGET_TEMP_MIN;

const POWER_PERCENT_MIN: i32 = 0;
const POWER_PERCENT_MAX: i32 = 100;
const POWER_PERCENT_DEFAULT: i32 = POWER_PERCENT_MIN;

const MAX31865_REFERENCE_RESISTANCE_OHMS: u32 = 4300;

#[allow(non_snake_case)]
#[riscv_rt::entry]
fn main() -> ! {
    esp_logger::init_logger(log::LevelFilter::Trace);

    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

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

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio0,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let i2c_bus = BusManagerSimple::new(i2c);

    let mut io_expander = new_io_expander(i2c_bus.acquire_i2c(), MCP23008_ADDR);
    io_expander.initialize().unwrap();

    let mut ioexp_config = io_expander.configuration();
    ioexp_config.set_interrupt_input_pin(Pin::GPIO0, true, InterruptTrigger::AnyEdge);
    ioexp_config.set_interrupt_input_pin(Pin::GPIO1, true, InterruptTrigger::AnyEdge);
    ioexp_config.set_interrupt_input_pin(Pin::GPIO2, true, InterruptTrigger::AnyEdge);
    ioexp_config.set_interrupt_input_pin(Pin::GPIO3, true, InterruptTrigger::AnyEdge);
    ioexp_config.set_interrupt_input_pin(Pin::GPIO4, true, InterruptTrigger::AnyEdge);
    ioexp_config.set_interrupt_input_pin(Pin::GPIO5, true, InterruptTrigger::AnyEdge);
    io_expander.set_configuration(ioexp_config).unwrap();


    let mut tx_descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let temperature_device = {
        // Use SPI with DMA because that seems to be the only SPI implementation
        // that actually works.
        let gdma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
        let gdma_channel = gdma.channel0;

        let spi = Spi::new(
            peripherals.SPI2,
            io.pins.gpio6, // clk
            io.pins.gpio7, // mosi
            io.pins.gpio2, // miso
            io.pins.gpio10, // cs
            5u32.MHz(),
            SpiMode::Mode3,
            &mut system.peripheral_clock_control,
            &clocks,
        )
        .with_dma(gdma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));

        Max31865::new(spi, MAX31865_REFERENCE_RESISTANCE_OHMS)
    };

    let temperature_display = new_display(
        i2c_bus.acquire_i2c(),
        TEMP_DISP_ADDR,
        TEMP_MAX_DEC_PLACES
    );

    let target_temp_rotary = RotaryEncoder::new(
        Pin::GPIO3,
        Pin::GPIO4,
        TARGET_TEMP_MIN,
        TARGET_TEMP_MAX,
        TARGET_TEMP_DEFAULT,
    );
    let target_temp_display = new_display(
        i2c_bus.acquire_i2c(),
        TARGET_TEMP_DISP_ADDR,
        ROTARY_DISPLAY_MAX_DEC_PLACES
    );

    let power_percent_rotary = RotaryEncoder::new(
        Pin::GPIO0,
        Pin::GPIO1,
        POWER_PERCENT_MIN,
        POWER_PERCENT_MAX,
        POWER_PERCENT_DEFAULT,
    );
    let power_percent_display = new_display(
        i2c_bus.acquire_i2c(),
        POWER_DISP_ADDR,
        ROTARY_DISPLAY_MAX_DEC_PLACES
    );

    let temperature_ready = io.pins.gpio4.into_pull_up_input();
    let io_expander_ready = io.pins.gpio3.into_pull_up_input();
    
    let mut target_temp_ssr_pin = io.pins.gpio5.into_push_pull_output();
    let mut power_percent_ssr_pin = io.pins.gpio9.into_push_pull_output();
    target_temp_ssr_pin.set_drive_strength(DriveStrength::I5mA);
    power_percent_ssr_pin.set_drive_strength(DriveStrength::I5mA);
  
    let target_temp_ssr = SolidStateRelay::new(target_temp_ssr_pin, Time);
    let power_percent_ssr = SolidStateRelay::new(power_percent_ssr_pin, Time);
  
    interrupt::enable(pac::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    unsafe { riscv::interrupt::enable(); }

    let mut devices = Devices {
        temperature_ready,
        io_expander_ready,
        temperature_device,
        temperature_display,
        target_temp_rotary,
        target_temp_display,
        target_temp_ssr,
        power_percent_rotary,
        power_percent_display,
        power_percent_ssr,
        io_expander,
        run: 0,
    };

    // Delay to allow connected devices to turn on
    Delay::new(&clocks).delay_ms(500u32);

    devices.initialize(&clocks).unwrap();

    // Execute the main run loop
    let run_result = devices.run();

    // We aren't processing interrupts anymore, disable them
    unsafe { riscv::interrupt::disable(); }

    match run_result {
        Ok(_) => {
            // Just turn off the displays and do nothing
            devices.turn_off_normal();
            info!("Turned off");
            loop {};
        },
        Err(err) => {
            // Show an error code on the displays and panic
            devices.turn_off_error();
            panic!("Fatal error in run: {:?}", err);
        },
    }
}

#[derive(Copy, Clone, Debug)]
enum DeviceError<SPI, I2CA, I2CB> {
    Temperature(Max31865Error<SPI>),
    Display(DisplayError<I2CA>),
    IoExpander(Mcp23008Error<I2CB>),
    DoubleRun,
}

impl<SPI, I2CA, I2CB> From<Max31865Error<SPI>> for DeviceError<SPI, I2CA, I2CB>
{
    fn from(value: Max31865Error<SPI>) -> DeviceError<SPI, I2CA, I2CB> {
        Self::Temperature(value)
    }
}

impl<SPI, I2CA, I2CB> From<DisplayError<I2CA>> for DeviceError<SPI, I2CA, I2CB>
{
    fn from(value: DisplayError<I2CA>) -> DeviceError<SPI, I2CA, I2CB> {
        Self::Display(value)
    }
}

impl<SPI, I2CA, I2CB> From<Mcp23008Error<I2CB>> for DeviceError<SPI, I2CA, I2CB>
{
    fn from(value: Mcp23008Error<I2CB>) -> DeviceError<SPI, I2CA, I2CB> {
        Self::IoExpander(value)
    }
}

impl<SPI, I2CA, I2CB> From<Infallible> for DeviceError<SPI, I2CA, I2CB>
{
    fn from(_: Infallible) -> DeviceError<SPI, I2CA, I2CB> {
        unreachable!()
    }
}

struct Devices<TR, IR, T, D, I, S1, S2>
where
    TR: InputPin,
    IR: InputPin,
    T: Max31865ReadWrite,
    D: SevenSegmentDisplayWrite,
    I: Mcp23008ReadWrite,
    S1: OutputPin,
    S2: OutputPin,
{
    temperature_ready: TR,
    io_expander_ready: IR,
    temperature_device: T,
    temperature_display: D,
    target_temp_rotary: RotaryEncoder,
    target_temp_display: D,
    target_temp_ssr: SolidStateRelay<S1, Time>,
    power_percent_rotary: RotaryEncoder,
    power_percent_display: D,
    power_percent_ssr: SolidStateRelay<S2, Time>,
    io_expander: I,
    run: u32,
}

impl<TR, IR, T, D, I, S1, S2, TErr, DErr, IErr> Devices<TR, IR, T, D, I, S1, S2>
where
    TR: InputPin<Error = Infallible>,
    IR: InputPin<Error = Infallible>,
    T: Max31865ReadWrite<Error = TErr>,
    D: SevenSegmentDisplayWrite<Error = DErr>,
    I: Mcp23008ReadWrite<Error = IErr>,
    S1: OutputPin<Error = Infallible>,
    S2: OutputPin<Error = Infallible>,
{
    fn initialize(&mut self, clocks: &Clocks) -> Result<(), DeviceError<TErr, DErr, IErr>> {
        self.target_temp_ssr.initialize();
        self.power_percent_ssr.initialize();

        self.temperature_device.initialize(&mut Delay::new(clocks))?;
        self.temperature_device.start_auto_conversions()?;
        
        // Read the data in order to reset DRDY. The data is stale from some
        // other session, so just discard it.
        self.temperature_device.read_temperature()?;
        
        // Same for the IO expander, to reset its interrupt pin
        self.io_expander.read()?;
        
        // Initialize and turn on the displays
        self.temperature_display.initialize()?;
        self.target_temp_display.initialize()?;
        self.power_percent_display.initialize()?;

        Self::show_value(&mut self.target_temp_rotary, &mut self.target_temp_display)?;
        Self::show_value(&mut self.power_percent_rotary, &mut self.power_percent_display)?;

        Ok(())
    } 

    fn run(&mut self) -> Result<(), DeviceError<TErr, DErr, IErr>> {
        // Once stopped, do not allow a restart
        if self.run != 0 {
            return Err(DeviceError::DoubleRun);
        }

        self.run += 1;

        let mut stats = TemperatureStats::new();

        while self.run == 1 {
            self.target_temp_ssr.update();
            self.power_percent_ssr.update();

            if self.io_expander_ready.is_low()? {
                let data = self.io_expander.read()?;
                let now = Time.now();
                
                if self.target_temp_rotary.update(data, now) {
                    Self::show_value(&mut self.target_temp_rotary, &mut self.target_temp_display)?;
                    //self.target_temp_ssr.set_power_percent(self.target_temp_rotary.value() as u32);
                }
            
                if self.power_percent_rotary.update(data, now) {
                    Self::show_value(&mut self.power_percent_rotary, &mut self.power_percent_display)?;
                    self.power_percent_ssr.set_power_percent(self.power_percent_rotary.value() as u32);
                }
            }
            
            if self.temperature_ready.is_low()? {
                let temp = self.temperature_device.read_temperature()?;

                if stats.add_sample(temp) {
                    // Enough samples were gathered to update the temperature
                    let temp = stats.temperature();
                    self.temperature_display.display_digits(temp)?;
                }
            }
        }
    
        Ok(())
    }

    fn show_value(rotary: &mut RotaryEncoder, display: &mut D) -> Result<(), DisplayError<D::Error>> {
        let value = rotary.value();

        if value <= rotary.min() {
            display.display_str("OFF")
        } else {
            display.display_digits(value as f32)
        }
    }

    #[allow(dead_code)]
    fn stop(&mut self) {
        self.run += 1;
    }

    fn turn_off_normal(mut self) {
        self.target_temp_ssr.turn_off();
        self.power_percent_ssr.turn_off();

        // Ignore errors while turning the display off
        self.temperature_display.set_display_on(false).unwrap_or_default();
        
        Self::set_to_min(&mut self.power_percent_rotary);
        Self::set_to_min(&mut self.target_temp_rotary);
        self.target_temp_display.set_display_on(false).unwrap_or_default();
        self.power_percent_display.set_display_on(false).unwrap_or_default();
    }

    fn turn_off_error(mut self) {
        self.target_temp_ssr.turn_off();
        self.power_percent_ssr.turn_off();

        // Ignore errors while displaying. Hopefully one of the displays works.
        self.temperature_display.display_str("Err").unwrap_or_default();
        
        Self::set_to_min(&mut self.power_percent_rotary);
        Self::set_to_min(&mut self.target_temp_rotary);
        self.target_temp_display.display_str("Err").unwrap_or_default();
        self.power_percent_display.display_str("Err").unwrap_or_default();
    }

    fn set_to_min(rotary: &mut RotaryEncoder) {
        rotary.force_set_value(rotary.min());
    }
}

fn new_display<I2C, E>(i2c: I2C, address: u8, max_decimal_places: usize) -> impl SevenSegmentDisplayWrite
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    SevenSegmentDisplay::new(i2c, address, max_decimal_places)
}

fn new_io_expander<I2C, E>(i2c: I2C, address: u8) -> impl Mcp23008ReadWrite
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    Mcp23008::new(i2c, address)
}

struct Time;

impl RealTimeClock for Time {
    fn now(&self) -> Instant {
        Instant::from_ticks(SystemTimer::now())
    }
}

struct TemperatureStats {
    // Data is generated at 60Hz, so 60 samples represents one second of data
    samples: [f32; 16],
    sample_idx: usize,
    temperature: f32,
}

impl TemperatureStats {
    fn new() -> Self {
        Self {
            samples: [0f32; 16],
            sample_idx: 0,
            temperature: 0f32,
        }
    }

    fn temperature(&self) -> f32 {
        self.temperature
    }

    fn add_sample(&mut self, temperature: f32) -> bool {
        self.samples[self.sample_idx] = temperature;
        self.sample_idx = (self.sample_idx + 1) % self.samples.len();

        if self.sample_idx == 0 {
            let mut sum = 0f32;

            for i in 0..self.samples.len() {
                sum += self.samples[i];
            }

            let prev_temp = self.temperature;
            self.temperature = sum / self.samples.len() as f32;
        
            self.temperature != prev_temp
        } else {
            false
        }
    }
}
