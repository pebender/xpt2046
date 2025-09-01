#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![no_main]
#![no_std]

esp_bootloader_esp_idf::esp_app_desc!();

#[rtic::app(device = esp32c6, dispatchers=[FROM_CPU_INTR0, FROM_CPU_INTR1])]
mod app {
    use esp_backtrace as _;
    use esp_hal::delay::Delay;
    use esp_hal::gpio::{Event, Input, InputConfig, Level, Output, OutputConfig, Pull};
    use esp_hal::{spi::master::Spi, Blocking};
    use xpt2046::{self, Xpt2046};

    type TouchSpi<'a> =
        embedded_hal_bus::spi::ExclusiveDevice<Spi<'a, Blocking>, Output<'a>, Delay>;
    #[shared]
    struct Shared {
        touch_drv: Xpt2046<TouchSpi<'static>>,
        touch_irq: Input<'static>,
    }
    #[local]
    struct Local {
        delay: Delay,
    }

    // do nothing in init
    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let device_peripherals = esp_hal::init(esp_hal::Config::default());

        let delay = Delay::new();

        let spi = device_peripherals.SPI2;
        let spi_sclk = device_peripherals.GPIO6;
        let spi_miso = device_peripherals.GPIO2;
        let spi_mosi = device_peripherals.GPIO7;
        let touch_cs = device_peripherals.GPIO17;
        let touch_irq = device_peripherals.GPIO18;

        // Set up SPI bus.
        let touch_spi_config = esp_hal::spi::master::Config::default()
            .with_mode(esp_hal::spi::Mode::_0)
            .with_frequency(esp_hal::time::Rate::from_mhz(1));
        let spi = Spi::new(spi, touch_spi_config)
            .unwrap()
            .with_sck(spi_sclk)
            .with_miso(spi_miso)
            .with_mosi(spi_mosi);
        let spi_bus = spi;

        // Set up touch SPI device.
        let touch_cs = Output::new(touch_cs, Level::High, OutputConfig::default());
        let touch_spi_device =
            embedded_hal_bus::spi::ExclusiveDevice::new(spi_bus, touch_cs, Delay::new()).unwrap();

        // Set up touch PENIRQ, including interrupt handler.
        let mut touch_irq = Input::new(touch_irq, InputConfig::default().with_pull(Pull::Up));
        touch_irq.listen(Event::FallingEdge);

        // Set up touch device.
        let mut touch_drv = Xpt2046::new(touch_spi_device, xpt2046::Orientation::PortraitFlipped);
        touch_drv.init(&mut touch_irq).unwrap();
        touch_drv.clear_touch();

        (
            Shared {
                touch_drv,
                touch_irq,
            },
            Local { delay },
        )
    }

    #[idle(local = [delay], shared = [touch_drv, touch_irq])]
    fn idle(ctx: idle::Context) -> ! {
        let mut touch_drv = ctx.shared.touch_drv;
        let mut touch_irq = ctx.shared.touch_irq;
        let delay = ctx.local.delay;
        loop {
            touch_irq.lock(|irq| {
                if irq.is_low() {
                    touch_drv.lock(|drv| {
                        match drv.run(irq) {
                            Ok(_) => {}
                            Err(e) => defmt::error!("{:?}", e),
                        }
                        if drv.is_touched() {
                            #[cfg(feature = "defmt")]
                            {
                                let p = drv.get_touch_point();
                                defmt::println!("x:{} y:{}", p.x, p.y);
                            }
                        }
                    });
                }
                if irq.is_high() {
                    irq.clear_interrupt();
                }
            });
            delay.delay_millis(1u32);
        }
    }

    /*
    #[task(binds=GPIO, shared=[touch_irq], priority = 3)]
    fn gpio_handler(ctx: gpio_handler::Context) {
        let mut touch_irq = ctx.shared.touch_irq;
        touch_irq.lock(|irq| {
            irq.clear_interrupt();
        });
        defmt::println!("touch_irq");
    }
    */
}
