#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use xpt2046::{calibration::*, driver::*};

    use stm32f4xx_hal::{
        gpio::{Input, Output, Pin},
        pac::{SPI1, TIM1},
        prelude::*,
        spi::{Mode, Phase, Polarity, Spi},
        timer::Delay,
    };

    #[cfg(feature = "defmt")]
    #[allow(unused_imports)]
    use defmt::{debug, error, info, trace, warn};

    #[cfg(feature = "log")]
    #[allow(unused_imports)]
    use log::{debug, error, info, trace, warn};

    type TouchSpi = embedded_hal_bus::spi::ExclusiveDevice<
        Spi<SPI1>,
        Pin<'A', 4, Output>,
        embedded_hal_bus::spi::NoDelay,
    >;
    type TouchIrq = Pin<'A', 2, Input>;
    #[shared]
    struct Shared {
        touch_drv: Xpt2046<TouchSpi, TouchIrq>,
    }

    #[local]
    struct Local {
        delay: Delay<TIM1, 1000000>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let device_peripherals = ctx.device;

        let mut rcc = device_peripherals
            .RCC
            .freeze(stm32f4xx_hal::rcc::Config::hse(25.MHz()).sysclk(100.MHz()));

        let delay = device_peripherals.TIM1.delay_us(&mut rcc);

        // Pins in GPIO Port A.
        let gpioa = device_peripherals.GPIOA.split(&mut rcc);

        let spi = device_peripherals.SPI1;
        let spi_sclk = gpioa.pa5;
        let spi_miso = gpioa.pa6;
        let spi_mosi = gpioa.pa7;
        let touch_cs = gpioa.pa4;
        let touch_irq = gpioa.pa2;

        // Set up SPI bus.
        let spi_mosi = spi_mosi.internal_pull_up(true);
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = Spi::new(
            spi,
            (Some(spi_sclk), Some(spi_miso), Some(spi_mosi)),
            spi_mode,
            2.MHz(),
            &mut rcc,
        );
        let spi_bus = spi;

        // Set up touch SPI device.
        let touch_cs = touch_cs.into_push_pull_output();
        let touch_spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(
            spi_bus,
            touch_cs,
            embedded_hal_bus::spi::NoDelay,
        )
        .unwrap();

        // Set up touch driver.
        let touch_irq = touch_irq.into_pull_up_input();
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
            touch_drv.lock(|drv| {
                if drv.penirq_is_active().unwrap() {
                    drv.run().unwrap();
                }
                if drv.is_touched() {
                    let p = drv.get_touch_point();
                    info!("x:{} y:{}", p.x, p.y);
                }
            });
            delay.delay_ms(1u32);
        }
    }
}
