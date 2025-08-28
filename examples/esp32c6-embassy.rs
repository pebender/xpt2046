#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;
use esp_println as _;
use esp_println::println;

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};

use esp_hal::peripherals::{
    GPIO17 as GPIO_TOUCH_CS, GPIO18 as GPIO_TOUCH_IRQ, GPIO2 as GPIO_SPI_MISO,
    GPIO6 as GPIO_SPI_SCLK, GPIO7 as GPIO_SPI_MOSI, SPI2 as SPI_BUS,
};

struct PeripheralMap<'a> {
    pub spi: SPI_BUS<'a>,
    pub spi_sclk: GPIO_SPI_SCLK<'a>,
    pub spi_mosi: GPIO_SPI_MOSI<'a>,
    pub spi_miso: GPIO_SPI_MISO<'a>,
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
        touch_cs: peripherals.GPIO17,
        touch_irq: peripherals.GPIO18,
    };

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    // Set up the shared SPI bus.
    let spi_config =
        esp_hal::spi::master::Config::default().with_frequency(esp_hal::time::Rate::from_mhz(1));
    let spi = esp_hal::spi::master::Spi::new(peripheral_map.spi, spi_config)
        .unwrap()
        .with_sck(peripheral_map.spi_sclk)
        .with_miso(peripheral_map.spi_miso)
        .with_mosi(peripheral_map.spi_mosi);
    let touch_cs = esp_hal::gpio::Output::new(
        peripheral_map.touch_cs,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    let touch_irq: esp_hal::gpio::Input<'_> = esp_hal::gpio::Input::new(
        peripheral_map.touch_irq,
        esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up),
    );

    let mut touch = xpt2046::Xpt2046::new(
        spi,
        touch_cs,
        MyIrq(touch_irq),
        xpt2046::Orientation::PortraitFlipped,
    );

    let _ = spawner;

    let mut delay = Delay;
    touch.init(&mut delay).unwrap();
    touch.clear_touch();
    loop {
        touch.irq.0.wait_for_low().await;
        while touch.irq.0.is_low() {
            let mut exti = true;
            touch.run(&mut exti).unwrap();
            if touch.is_touched() {
                let point = touch.get_touch_point();
                println!("x: {}, y: {}", point.x, point.y);
                touch.clear_touch();
                if touch.irq.0.is_low() {
                    Timer::after(Duration::from_millis(100)).await;
                }
            }
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}
