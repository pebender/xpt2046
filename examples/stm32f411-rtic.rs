#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::{
        gpio::{Edge, Input, Output, Pin},
        pac::{EXTI, SPI1, TIM1},
        prelude::*,
        spi::{Mode, Phase, Polarity, Spi},
        timer::Delay,
    };
    use xpt2046::{self, Xpt2046};

    type TouchSpi = embedded_hal_bus::spi::ExclusiveDevice<
        Spi<SPI1>,
        Pin<'A', 4, Output>,
        embedded_hal_bus::spi::NoDelay,
    >;
    #[shared]
    struct Shared {
        touch_drv: Xpt2046<TouchSpi>,
        touch_irq: Pin<'A', 2, Input>,
        touch_exti: EXTI,
    }

    #[local]
    struct Local {
        delay: Delay<TIM1, 1000000>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let device_peripherals = ctx.device;

        let rcc = device_peripherals.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let delay = device_peripherals.TIM1.delay_us(&clocks);

        // Pins in GPIO Port A.
        let gpioa = device_peripherals.GPIOA.split();

        let spi = device_peripherals.SPI1;
        let spi_sclk = gpioa.pa5;
        let spi_miso = gpioa.pa6;
        let spi_mosi = gpioa.pa7;
        let touch_cs = gpioa.pa4;
        let touch_irq = gpioa.pa2;
        let touch_exti = device_peripherals.EXTI;

        // Set up SPI bus.
        let spi_sclk = spi_sclk.into_alternate();
        let spi_miso = spi_miso.into_alternate();
        let spi_mosi = spi_mosi.into_alternate().internal_pull_up(true);
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = Spi::new(
            spi,
            (spi_sclk, spi_miso, spi_mosi),
            spi_mode,
            2.MHz(),
            &clocks,
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

        // Set up touch PENIRQ, including interrupt handler.
        let mut touch_irq = touch_irq.into_pull_up_input();
        let mut touch_exti = touch_exti;
        let mut syscfg = device_peripherals.SYSCFG.constrain();
        touch_irq.make_interrupt_source(&mut syscfg);
        touch_irq.trigger_on_edge(&mut touch_exti, Edge::Falling);
        touch_irq.enable_interrupt(&mut touch_exti);

        // Set up touch driver.
        let mut touch_drv = Xpt2046::new(
            touch_spi_device,
            &xpt2046::calibration::estimate_calibration(false, false, true, 240, 320),
        );
        touch_drv.init(&mut touch_irq).unwrap();
        touch_drv.clear_touch();

        (
            Shared {
                touch_drv,
                touch_irq,
                touch_exti,
            },
            Local { delay },
        )
    }

    #[idle(local = [delay], shared = [touch_drv, touch_irq, touch_exti])]
    fn idle(ctx: idle::Context) -> ! {
        let mut touch_drv = ctx.shared.touch_drv;
        let mut touch_irq = ctx.shared.touch_irq;
        let mut touch_exti = ctx.shared.touch_exti;
        let delay = ctx.local.delay;

        loop {
            touch_drv.lock(|drv| {
                touch_irq.lock(|irq| {
                    drv.run(irq).unwrap();
                    if drv.is_touched() {
                        touch_exti.lock(|exti| {
                            irq.clear_interrupt_pending_bit();
                            irq.enable_interrupt(exti);
                        });
                    }
                });
                if drv.is_touched() {
                    #[cfg(feature = "defmt")]
                    {
                        let p = drv.get_touch_point();
                        defmt::println!("x:{} y:{}", p.x, p.y);
                    }
                }
            });
            delay.delay_ms(1u32);
        }
    }

    #[task(binds = EXTI2, shared = [touch_drv, touch_irq, touch_exti])]
    fn exti2(ctx: exti2::Context) {
        let touch_drv = ctx.shared.touch_drv;
        let touch_irq = ctx.shared.touch_irq;
        let touch_exti = ctx.shared.touch_exti;
        (touch_drv, touch_irq, touch_exti).lock(|drv, irq, exti| {
            irq.disable_interrupt(exti);
            irq.clear_interrupt_pending_bit();
            drv.clear_touch();
        })
    }
}
