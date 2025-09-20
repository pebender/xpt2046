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
    use xpt2046::{calibration::*, driver::*};

    use esp_backtrace as _;
    use esp_hal::delay::Delay;
    use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
    use esp_hal::{spi::master::Spi, Blocking};

    #[cfg(feature = "defmt")]
    #[allow(unused_imports)]
    use defmt::{debug, error, info, trace, warn};

    #[cfg(feature = "log")]
    #[allow(unused_imports)]
    use log::{debug, error, info, trace, warn};

    type TouchSpi<'a> =
        embedded_hal_bus::spi::ExclusiveDevice<Spi<'a, Blocking>, Output<'a>, Delay>;
    type TouchIrq<'a> = Input<'static>;
    #[shared]
    struct Shared {
        touch_drv: Xpt2046<TouchSpi<'static>, TouchIrq<'static>>,
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

        // Set up touch device.
        let touch_irq = Input::new(touch_irq, InputConfig::default().with_pull(Pull::Up));
        let calibration_data = estimate_calibration_data(
            RelativeOrientation::new(false, true, false),
            Size::new(240, 320),
        );
        let mut touch_drv = Xpt2046::new(touch_spi_device, touch_irq, &calibration_data);
        touch_drv.init().unwrap();
        touch_drv.clear_touch();

        (Shared { touch_drv }, Local { delay })
    }

    #[idle(local = [delay], shared = [touch_drv])]
    fn idle(ctx: idle::Context) -> ! {
        let mut touch_drv = ctx.shared.touch_drv;
        let delay = ctx.local.delay;
        loop {
            touch_drv.lock(|drv| match drv.penirq_is_active() {
                Ok(irq) => {
                    if irq {
                        match drv.run() {
                            Ok(_) => {}
                            Err(e) => error!("{:?}", e),
                        }
                        if drv.is_touched() {
                            let p = drv.get_touch_point();
                            info!("x:{} y:{}", p.x, p.y);
                        }
                    }
                }
                Err(e) => error!("{:?}", e),
            });
            delay.delay_millis(1u32);
        }
    }
}
