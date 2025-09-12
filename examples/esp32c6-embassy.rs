#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![no_std]
#![no_main]

use xpt2046::{calibration::*, driver::*};

use esp_backtrace as _;
use esp_println as _;

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Delay, Timer};
use embedded_graphics::prelude::*;
use embedded_graphics::{geometry, pixelcolor::Rgb565, primitives};
use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::Rate,
    Blocking,
};
use mipidsi;
use static_cell::StaticCell;

#[cfg(feature = "defmt")]
use defmt::Format;

#[cfg(feature = "defmt")]
#[allow(unused_imports)]
use defmt::{debug, error, info, println, trace, warn};

#[cfg(feature = "log")]
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

#[cfg(feature = "log")]
#[allow(unused_imports)]
use esp_println::println;

use esp_hal::peripherals::{
    GPIO16 as GPIO_DISPLAY_CS, GPIO17 as GPIO_TOUCH_CS, GPIO18 as GPIO_TOUCH_IRQ,
    GPIO2 as GPIO_SPI_MISO, GPIO20 as GPIO_DISPLAY_BACKLIGHT, GPIO21 as GPIO_DISPLAY_RESET,
    GPIO22 as GPIO_DISPLAY_DC, GPIO6 as GPIO_SPI_SCLK, GPIO7 as GPIO_SPI_MOSI, SPI2 as SPI_BUS,
};

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
struct PeripheralMap<'a> {
    pub spi: SPI_BUS<'a>,
    pub spi_sclk: GPIO_SPI_SCLK<'a>,
    pub spi_mosi: GPIO_SPI_MOSI<'a>,
    pub spi_miso: GPIO_SPI_MISO<'a>,
    pub display_cs: GPIO_DISPLAY_CS<'a>,
    pub display_dc: GPIO_DISPLAY_DC<'a>,
    pub display_reset: GPIO_DISPLAY_RESET<'a>,
    pub display_backlight: GPIO_DISPLAY_BACKLIGHT<'a>,
    pub touch_cs: GPIO_TOUCH_CS<'a>,
    pub touch_irq: GPIO_TOUCH_IRQ<'a>,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
enum DisplayCommand {
    Clear,
    TouchPoint(geometry::Point),
}

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
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    let peripheral_map = PeripheralMap {
        spi: peripherals.SPI2,
        spi_sclk: peripherals.GPIO6,
        spi_mosi: peripherals.GPIO7,
        spi_miso: peripherals.GPIO2,
        display_cs: peripherals.GPIO16,
        display_dc: peripherals.GPIO22,
        display_reset: peripherals.GPIO21,
        display_backlight: peripherals.GPIO20,
        touch_cs: peripherals.GPIO17,
        touch_irq: peripherals.GPIO18,
    };

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    // Set up the shared SPI bus.
    let spi = Spi::new(peripheral_map.spi, Config::default())
        .unwrap()
        .with_sck(peripheral_map.spi_sclk)
        .with_miso(peripheral_map.spi_miso)
        .with_mosi(peripheral_map.spi_mosi);
    let spi_bus = Mutex::<CriticalSectionRawMutex, _>::new(RefCell::new(spi));
    let spi_bus =
        &*make_static!(Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>, spi_bus);

    // Set up the channel for sending commands to the display task.
    let display_command_channel = Channel::<CriticalSectionRawMutex, DisplayCommand, 16>::new();
    let display_command_channel = &*make_static!(Channel<CriticalSectionRawMutex, DisplayCommand, 16>, display_command_channel);

    // Create a channel receiver to give to the display task.
    let display_command_receiver = display_command_channel.receiver();
    let display_command_receiver = &*make_static!(Receiver<CriticalSectionRawMutex, DisplayCommand, 16>, display_command_receiver);

    // Create a channel sender to give to the touch task.
    let display_command_sender = display_command_channel.sender();
    let display_command_sender =
        &*make_static!(Sender<CriticalSectionRawMutex, DisplayCommand, 16>, display_command_sender);

    spawner.must_spawn(display_task(
        spi_bus,
        peripheral_map.display_cs,
        peripheral_map.display_dc,
        peripheral_map.display_reset,
        peripheral_map.display_backlight,
        display_command_receiver,
    ));

    spawner.must_spawn(touch_task(
        spi_bus,
        peripheral_map.touch_cs,
        peripheral_map.touch_irq,
        display_command_sender,
    ));

    display_command_sender.send(DisplayCommand::Clear).await;

    loop {
        Timer::after_secs(60).await;
    }
}

#[embassy_executor::task]
async fn touch_task(
    spi_bus: &'static Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>,
    touch_cs: GPIO_TOUCH_CS<'static>,
    touch_irq: GPIO_TOUCH_IRQ<'static>,
    display_command_sender: &'static Sender<'static, CriticalSectionRawMutex, DisplayCommand, 16>,
) -> ! {
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

    let calibration_data = estimate_calibration_data(
        RelativeOrientation::new(false, true, false),
        Size::new(240, 320),
    );
    let mut touch = Xpt2046::new(touch_spi_device, &calibration_data);

    touch.init().unwrap();
    touch.clear_touch();
    loop {
        touch_irq.wait_for_low().await;
        while touch_irq.is_low() {
            touch.run(&mut touch_irq).unwrap();
            if touch.is_touched() {
                let point = touch.get_touch_point();

                if point.x >= 0 && point.x < 240 && point.y >= 0 && point.y < 320 {
                    display_command_sender
                        .send(DisplayCommand::TouchPoint(point))
                        .await;
                }
            }
            Timer::after_micros(500).await;
        }
        touch.clear_touch();
    }
}

#[embassy_executor::task]
async fn display_task(
    spi_bus: &'static Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>,
    display_cs: GPIO_DISPLAY_CS<'static>,
    display_dc: GPIO_DISPLAY_DC<'static>,
    display_reset: GPIO_DISPLAY_RESET<'static>,
    display_backlight: GPIO_DISPLAY_BACKLIGHT<'static>,
    display_command_receiver: &'static Receiver<
        'static,
        CriticalSectionRawMutex,
        DisplayCommand,
        16,
    >,
) -> ! {
    // Set up the LCD SPI device.
    let display_cs = Output::new(display_cs, Level::High, OutputConfig::default());
    let display_spi_device = SpiDeviceWithConfig::new(
        spi_bus,
        display_cs,
        Config::default()
            .with_mode(Mode::_0)
            .with_frequency(Rate::from_mhz(40)),
    );

    // Set up the mipidsi display interface
    let display_dc = Output::new(display_dc, Level::Low, OutputConfig::default());
    let mut display_buffer = [0_u8; 512];
    let display_interface =
        mipidsi::interface::SpiInterface::new(display_spi_device, display_dc, &mut display_buffer);

    // Set up the mipidsi display
    let mut display_backlight = Output::new(display_backlight, Level::Low, OutputConfig::default());
    let display_reset = Output::new(display_reset, Level::Low, OutputConfig::default());
    let mut delay = Delay;
    let mut display = mipidsi::Builder::new(mipidsi::models::ILI9341Rgb565, display_interface)
        .display_size(240, 320)
        .orientation(mipidsi::options::Orientation::new().flip_horizontal())
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .reset_pin(display_reset)
        .init(&mut delay)
        .unwrap();
    display.clear(Rgb565::CSS_BLACK).unwrap();
    display_backlight.set_high();

    let dot = primitives::Rectangle::new(geometry::Point::zero(), geometry::Size::new_equal(1))
        .into_styled(primitives::PrimitiveStyle::with_fill(Rgb565::CSS_WHITE));

    loop {
        match display_command_receiver.receive().await {
            DisplayCommand::Clear => display.clear(Rgb565::CSS_BLACK).unwrap(),
            DisplayCommand::TouchPoint(point) => {
                dot.translate(point).draw(&mut display).unwrap();
            }
        }
    }
}
