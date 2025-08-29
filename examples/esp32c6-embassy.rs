#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;
use esp_println as _;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::Delay;

use embedded_graphics::prelude::*;

use esp_hal::peripherals::{
    GPIO16 as GPIO_LCD_CS, GPIO17 as GPIO_TOUCH_CS, GPIO18 as GPIO_TOUCH_IRQ,
    GPIO2 as GPIO_SPI_MISO, GPIO20 as GPIO_LCD_BACKLIGHT, GPIO21 as GPIO_LCD_RESET,
    GPIO22 as GPIO_LCD_DC, GPIO6 as GPIO_SPI_SCLK, GPIO7 as GPIO_SPI_MOSI, SPI2 as SPI_BUS,
};

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

pub struct MyIrq<'a>(esp_hal::gpio::Input<'a>);

impl<'a> xpt2046::Xpt2046Exti for MyIrq<'a> {
    type Exti = bool;
    fn clear_interrupt(&mut self) {}

    fn disable_interrupt(&mut self, _exti: &mut Self::Exti) {}

    fn enable_interrupt(&mut self, _exti: &mut Self::Exti) {}

    fn is_high(&self) -> bool {
        self.0.is_high()
    }

    fn is_low(&self) -> bool {
        self.0.is_low()
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
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

    // Set up the LCD SPI device.
    let lcd_cs = esp_hal::gpio::Output::new(
        peripheral_map.lcd_cs,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    let lcd_spi_config = esp_hal::spi::master::Config::default()
        .with_mode(esp_hal::spi::Mode::_0)
        .with_frequency(esp_hal::time::Rate::from_mhz(40));
    let lcd_spi_device = SpiDeviceWithConfig::new(&spi_bus, lcd_cs, lcd_spi_config);

    let lcd_dc = esp_hal::gpio::Output::new(
        peripheral_map.lcd_dc,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let lcd_reset = esp_hal::gpio::Output::new(
        peripheral_map.lcd_reset,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut lcd_backlight = esp_hal::gpio::Output::new(
        peripheral_map.lcd_backlight,
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

    // Set up the touch SPI device.
    let touch_cs = esp_hal::gpio::Output::new(
        peripheral_map.touch_cs,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    let touch_spi_config = esp_hal::spi::master::Config::default()
        .with_mode(esp_hal::spi::Mode::_0)
        .with_frequency(esp_hal::time::Rate::from_mhz(2));
    let touch_spi_device = SpiDeviceWithConfig::new(&spi_bus, touch_cs, touch_spi_config);

    let touch_irq: esp_hal::gpio::Input<'_> = esp_hal::gpio::Input::new(
        peripheral_map.touch_irq,
        esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    let mut touch = xpt2046::Xpt2046::new(
        touch_spi_device,
        MyIrq(touch_irq),
        xpt2046::Orientation::Portrait,
    );

    let _ = spawner;

    lcd.clear(embedded_graphics::pixelcolor::Rgb565::CSS_BLACK)
        .unwrap();
    let mut delay = Delay;
    touch.init(&mut delay).unwrap();
    touch.clear_touch();
    let dot = embedded_graphics::primitives::Rectangle::new(
        embedded_graphics::geometry::Point::zero(),
        embedded_graphics::geometry::Size::new_equal(1),
    )
    .into_styled(embedded_graphics::primitives::PrimitiveStyle::with_fill(
        embedded_graphics::pixelcolor::Rgb565::CSS_WHITE,
    ));
    let mut exti = true;
    loop {
        touch.irq.0.wait_for_low().await;
        while touch.irq.0.is_low() {
            touch.run(&mut exti).unwrap();
            if touch.is_touched() {
                let point = touch.get_touch_point();
                if point.x >= 0 && point.x < 240 && point.y >= 0 && point.y < 320 {
                    dot.translate(point).draw(&mut lcd).unwrap();
                }
            }
            touch.clear_touch();
        }
    }
}
