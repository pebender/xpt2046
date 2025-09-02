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

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Delay, Timer};

use embedded_graphics::prelude::*;

#[cfg(feature = "defmt")]
use defmt::Format;

#[cfg(feature = "log")]
use esp_println::logger::init_logger_from_env;

#[cfg(feature = "defmt")]
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

#[cfg(feature = "log")]
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use esp_hal::peripherals::{
    GPIO16 as GPIO_LCD_CS, GPIO17 as GPIO_TOUCH_CS, GPIO18 as GPIO_TOUCH_IRQ,
    GPIO2 as GPIO_SPI_MISO, GPIO20 as GPIO_LCD_BACKLIGHT, GPIO21 as GPIO_LCD_RESET,
    GPIO22 as GPIO_LCD_DC, GPIO6 as GPIO_SPI_SCLK, GPIO7 as GPIO_SPI_MOSI, SPI2 as SPI_BUS,
};

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
struct PeripheralMap<'a> {
    pub spi: SPI_BUS<'a>,
    pub spi_sclk: GPIO_SPI_SCLK<'a>,
    pub spi_mosi: GPIO_SPI_MOSI<'a>,
    pub spi_miso: GPIO_SPI_MISO<'a>,
    pub lcd_cs: GPIO_LCD_CS<'a>,
    pub lcd_dc: GPIO_LCD_DC<'a>,
    pub lcd_reset: GPIO_LCD_RESET<'a>,
    pub lcd_backlight: GPIO_LCD_BACKLIGHT<'a>,
    pub touch_cs: GPIO_TOUCH_CS<'a>,
    pub touch_irq: GPIO_TOUCH_IRQ<'a>,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
enum LcdCommand {
    Clear,
    TouchPoint(embedded_graphics::geometry::Point),
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
        static STATIC_CELL: static_cell::StaticCell<$time> = static_cell::StaticCell::new();
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

    let peripheral_map = PeripheralMap {
        spi: peripherals.SPI2,
        spi_sclk: peripherals.GPIO6,
        spi_mosi: peripherals.GPIO7,
        spi_miso: peripherals.GPIO2,
        lcd_cs: peripherals.GPIO16,
        lcd_dc: peripherals.GPIO22,
        lcd_reset: peripherals.GPIO21,
        lcd_backlight: peripherals.GPIO20,
        touch_cs: peripherals.GPIO17,
        touch_irq: peripherals.GPIO18,
    };

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    // Set up the shared SPI bus.
    let spi_config = esp_hal::spi::master::Config::default();
    let spi = esp_hal::spi::master::Spi::new(peripheral_map.spi, spi_config)
        .unwrap()
        .with_sck(peripheral_map.spi_sclk)
        .with_miso(peripheral_map.spi_miso)
        .with_mosi(peripheral_map.spi_mosi);
    let spi_bus =
        blocking_mutex::Mutex::<CriticalSectionRawMutex, _>::new(core::cell::RefCell::new(spi));
    let spi_bus = &*make_static!(blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::RefCell<esp_hal::spi::master::Spi<'static, esp_hal::Blocking>>>, spi_bus);

    // Set up the channel for sending commands to the lcd task.
    let lcd_command_channel = Channel::<CriticalSectionRawMutex, LcdCommand, 16>::new();
    let lcd_command_channel =
        &*make_static!(Channel<CriticalSectionRawMutex, LcdCommand, 16>, lcd_command_channel);

    // Create a channel receiver to give to the lcd task.
    let lcd_command_receiver = lcd_command_channel.receiver();
    let lcd_command_receiver =
        &*make_static!(Receiver<CriticalSectionRawMutex, LcdCommand, 16>, lcd_command_receiver);

    // Create a channel sender to give to the touch task.
    let lcd_command_sender = lcd_command_channel.sender();
    let lcd_command_sender =
        &*make_static!(Sender<CriticalSectionRawMutex, LcdCommand, 16>, lcd_command_sender);

    spawner.must_spawn(lcd_task(
        spi_bus,
        peripheral_map.lcd_cs,
        peripheral_map.lcd_dc,
        peripheral_map.lcd_reset,
        peripheral_map.lcd_backlight,
        lcd_command_receiver,
    ));

    spawner.must_spawn(touch_task(
        spi_bus,
        peripheral_map.touch_cs,
        peripheral_map.touch_irq,
        lcd_command_sender,
    ));

    lcd_command_sender.send(LcdCommand::Clear).await;

    loop {
        Timer::after_secs(60).await;
    }
}

#[embassy_executor::task]
async fn touch_task(
    spi_bus: &'static blocking_mutex::Mutex<
        CriticalSectionRawMutex,
        core::cell::RefCell<esp_hal::spi::master::Spi<'static, esp_hal::Blocking>>,
    >,
    touch_cs: GPIO_TOUCH_CS<'static>,
    touch_irq: GPIO_TOUCH_IRQ<'static>,
    lcd_command_sender: &'static Sender<'static, CriticalSectionRawMutex, LcdCommand, 16>,
) -> ! {
    // Set up the touch SPI device.
    let touch_cs = esp_hal::gpio::Output::new(
        touch_cs,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    let touch_spi_config = esp_hal::spi::master::Config::default()
        .with_mode(esp_hal::spi::Mode::_0)
        .with_frequency(esp_hal::time::Rate::from_mhz(2));
    let touch_spi_device = SpiDeviceWithConfig::new(spi_bus, touch_cs, touch_spi_config);

    let mut touch_irq = esp_hal::gpio::Input::new(
        touch_irq,
        esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    let mut touch = xpt2046::Xpt2046::new(touch_spi_device, xpt2046::Orientation::Portrait);

    touch.init(&mut touch_irq).unwrap();
    touch.clear_touch();
    loop {
        touch_irq.wait_for_low().await;
        while touch_irq.is_low() {
            touch.run(&mut touch_irq).unwrap();
            if touch.is_touched() {
                let point = touch.get_touch_point();
                if point.x >= 0 && point.x < 240 && point.y >= 0 && point.y < 320 {
                    lcd_command_sender.send(LcdCommand::TouchPoint(point)).await;
                }
            }
            Timer::after_micros(500).await;
        }
        touch.clear_touch();
    }
}

#[embassy_executor::task]
async fn lcd_task(
    spi_bus: &'static blocking_mutex::Mutex<
        CriticalSectionRawMutex,
        core::cell::RefCell<esp_hal::spi::master::Spi<'static, esp_hal::Blocking>>,
    >,
    lcd_cs: GPIO_LCD_CS<'static>,
    lcd_dc: GPIO_LCD_DC<'static>,
    lcd_reset: GPIO_LCD_RESET<'static>,
    lcd_backlight: GPIO_LCD_BACKLIGHT<'static>,
    lcd_command_receiver: &'static Receiver<'static, CriticalSectionRawMutex, LcdCommand, 16>,
) -> ! {
    // Set up the LCD SPI device.
    let lcd_cs = esp_hal::gpio::Output::new(
        lcd_cs,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    let lcd_spi_config = esp_hal::spi::master::Config::default()
        .with_mode(esp_hal::spi::Mode::_0)
        .with_frequency(esp_hal::time::Rate::from_mhz(40));
    let lcd_spi_device = SpiDeviceWithConfig::new(spi_bus, lcd_cs, lcd_spi_config);

    let lcd_dc = esp_hal::gpio::Output::new(
        lcd_dc,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let lcd_reset = esp_hal::gpio::Output::new(
        lcd_reset,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut lcd_backlight = esp_hal::gpio::Output::new(
        lcd_backlight,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut lcd_buffer = [0_u8; 512];
    let lcd_interface =
        mipidsi::interface::SpiInterface::new(lcd_spi_device, lcd_dc, &mut lcd_buffer);
    let mut delay = Delay;
    let mut lcd = mipidsi::Builder::new(mipidsi::models::ILI9341Rgb565, lcd_interface)
        .display_size(240, 320)
        .orientation(mipidsi::options::Orientation::new())
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .reset_pin(lcd_reset)
        .init(&mut delay)
        .unwrap();
    lcd_backlight.set_high();

    let dot = embedded_graphics::primitives::Rectangle::new(
        embedded_graphics::geometry::Point::zero(),
        embedded_graphics::geometry::Size::new_equal(1),
    )
    .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
        embedded_graphics::pixelcolor::Rgb565::CSS_WHITE,
    ));

    loop {
        match lcd_command_receiver.receive().await {
            LcdCommand::Clear => lcd
                .clear(embedded_graphics::pixelcolor::Rgb565::CSS_BLACK)
                .unwrap(),
            LcdCommand::TouchPoint(p) => dot.translate(p).draw(&mut lcd).unwrap(),
        }
    }
}
