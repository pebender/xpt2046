#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println as _;

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Delay, Timer};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::Rate,
    Blocking,
};
use mipidsi::{models::ILI9341Rgb565, options::ColorOrder, options::Orientation};
use static_cell::StaticCell;

#[cfg(feature = "log")]
use esp_println::logger::init_logger_from_env;

#[cfg(feature = "defmt")]
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

#[cfg(feature = "log")]
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

// The embassy-executor main sets up shared hardware resources, resulting in
// structs that represent the hardware. main shares references to these structs
// with embassy-executor tasks that make use of these hardware resources.
// Because main spawns these tasks, the Rust compiler cannot infer when the
// tasks will compete. As a result, it cannot infer whether or not the structs
// representing the hardware will last the lifetime of the spawned task. To
// solve this, main makes these structs static using StaticCell. For those not
// familiar with StaticCell, it allows you to statically allocate data that
// requires runtime initialization. This macro takes the data's type (time) and
// value (v) and returns a static reference to the data.
macro_rules! make_static {
    ($time:ty,$v:expr) => {{
        static STATIC_CELL: StaticCell<$time> = StaticCell::new();
        STATIC_CELL.init(($v))
    }};
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    #[cfg(feature = "log")]
    init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    let spi = peripherals.SPI2;
    let spi_sclk = peripherals.GPIO6;
    let spi_mosi = peripherals.GPIO7;
    let spi_miso = peripherals.GPIO2;
    let lcd_cs = peripherals.GPIO16;
    let lcd_dc = peripherals.GPIO22;
    let lcd_reset = peripherals.GPIO21;
    let lcd_backlight = peripherals.GPIO20;
    let touch_cs = peripherals.GPIO17;
    let touch_irq = peripherals.GPIO18;

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    _ = spawner;

    // Set up the shared SPI bus.
    let spi = Spi::new(spi, Config::default())
        .unwrap()
        .with_sck(spi_sclk)
        .with_miso(spi_miso)
        .with_mosi(spi_mosi);
    let spi_bus = Mutex::<CriticalSectionRawMutex, _>::new(RefCell::new(spi));
    let spi_bus =
        &*make_static!(Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>, spi_bus);

    // Set up the LCD SPI device.
    let lcd_cs = Output::new(lcd_cs, Level::High, OutputConfig::default());
    let lcd_spi_device = SpiDeviceWithConfig::new(
        spi_bus,
        lcd_cs,
        Config::default()
            .with_mode(Mode::_0)
            .with_frequency(Rate::from_mhz(40)),
    );

    // Set up the mipidsi display interface
    let lcd_dc = Output::new(lcd_dc, Level::Low, OutputConfig::default());
    let mut lcd_buffer = [0_u8; 512];
    let lcd_interface =
        mipidsi::interface::SpiInterface::new(lcd_spi_device, lcd_dc, &mut lcd_buffer);

    // Set up the mipidsi display
    let mut lcd_backlight = Output::new(lcd_backlight, Level::Low, OutputConfig::default());
    let lcd_reset = Output::new(lcd_reset, Level::Low, OutputConfig::default());
    let mut delay = Delay;
    let mut lcd = mipidsi::Builder::new(ILI9341Rgb565, lcd_interface)
        .display_size(240, 320)
        .orientation(Orientation::new().flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .reset_pin(lcd_reset)
        .init(&mut delay)
        .unwrap();
    lcd.clear(Rgb565::BLACK).unwrap();
    lcd_backlight.set_high();

    // Set up the touch SPI device.
    let touch_cs = Output::new(touch_cs, Level::High, OutputConfig::default());
    let touch_spi_device = SpiDeviceWithConfig::new(
        spi_bus,
        touch_cs,
        Config::default()
            .with_mode(Mode::_0)
            .with_frequency(Rate::from_mhz(2)),
    );

    let mut touch_irq = Input::new(touch_irq, InputConfig::default().with_pull(Pull::Up));

    let mut touch = xpt2046::Xpt2046::new(
        touch_spi_device,
        &xpt2046::calibration::estimate_calibration_data(
            xpt2046::calibration::Transform::new(false, false, true),
            xpt2046::Size::new(240, 320),
        ),
    );

    loop {
        let calibration_data =
            xpt2046::calibration::run_calibration(&mut touch, &mut touch_irq, &mut lcd, &mut delay);
        match calibration_data {
            Ok(v) => {
                defmt::println!("{:?}", v);
                touch.set_calibration_data(&v);
            }
            Err(e) => defmt::println!("{:?}", e),
        }
        Timer::after_secs(5).await;
    }
}
