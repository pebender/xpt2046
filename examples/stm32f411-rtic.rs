#![no_main]
#![no_std]

use panic_semihosting as _;

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
        xpt_drv: Xpt2046<TouchSpi>,
        touch_irq: Pin<'A', 2, Input>,
        exti: EXTI,
    }

    #[local]
    struct Local {
        delay: Delay<TIM1, 1000000>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = ctx.device;
        let _cp = ctx.core;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let _gpiob = dp.GPIOB.split();
        let _gpioc = dp.GPIOC.split();

        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let mut delay = dp.TIM1.delay_us(&clocks);

        // Touch interface
        let mut touch_irq = gpioa.pa2.into_pull_up_input();
        let mut syscfg = dp.SYSCFG.constrain();
        touch_irq.make_interrupt_source(&mut syscfg);
        touch_irq.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        touch_irq.enable_interrupt(&mut dp.EXTI);
        let touch_cs = gpioa.pa4.into_push_pull_output();
        let touch_clk = gpioa.pa5.into_alternate();
        let touch_mosi = gpioa.pa7.into_alternate().internal_pull_up(true);
        let touch_miso = gpioa.pa6.into_alternate();
        let spi = Spi::new(
            dp.SPI1,
            (touch_clk, touch_miso, touch_mosi),
            mode,
            2.MHz(),
            &clocks,
        );
        let spi_bus = spi;
        let touch_spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(
            spi_bus,
            touch_cs,
            embedded_hal_bus::spi::NoDelay,
        )
        .unwrap();
        let mut xpt_drv = Xpt2046::new(touch_spi_device, xpt2046::Orientation::PortraitFlipped);
        xpt_drv.init(&mut touch_irq, &mut delay).unwrap();

        (
            Shared {
                xpt_drv,
                touch_irq,
                exti: dp.EXTI,
            },
            Local { delay },
            init::Monotonics(),
        )
    }

    #[idle(local = [delay], shared = [xpt_drv, touch_irq, exti])]
    fn idle(ctx: idle::Context) -> ! {
        let mut xpt_drv = ctx.shared.xpt_drv;
        let mut touch_irq = ctx.shared.touch_irq;
        let mut exti = ctx.shared.exti;
        let delay = ctx.local.delay;

        loop {
            xpt_drv.lock(|xpt| {
                touch_irq.lock(|irq| {
                    xpt.run(irq).unwrap();
                    if xpt.is_touched() {
                        exti.lock(|e| {
                            irq.clear_interrupt_pending_bit();
                            irq.enable_interrupt(e);
                        });
                    }
                });
                if xpt.is_touched() {
                    #[cfg(feature = "with_defmt")]
                    {
                        let p = xpt.get_touch_point();
                        defmt::println!("x:{} y:{}", p.x, p.y);
                    }
                }
            });
            delay.delay_ms(1u32);
        }
    }

    #[task(binds = EXTI2, local = [], shared = [xpt_drv, touch_irq, exti])]
    fn exti2(ctx: exti2::Context) {
        let xpt_drv = ctx.shared.xpt_drv;
        let touch_irq = ctx.shared.touch_irq;
        let exti = ctx.shared.exti;
        (xpt_drv, touch_irq, exti).lock(|xpt, irq, e| {
            irq.disable_interrupt(e);
            irq.clear_interrupt_pending_bit();
            xpt.clear_touch();
        })
    }
}
