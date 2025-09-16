//! Demonstrates using the Xpt2046 driver running on an ESP32-C6 using the
//! [esp-hal](https://docs.esp-rs.org/index.html) and the [Embassy
//! framework](https://embassy.dev).
//!
//! The example uses a 240x320 TFT LCD touchscreen with an ILI Technology's
//! ILI9341 display panel controller. However, the example uses the
//! [mipidsi](https://docs.rs/mipidsi/latest/mipidsi/) driver, so as long as
//! your touchscreen's display is supported by the mipidsi driver and connects
//! over SPI, you should be able to make the example work by updating the
//! `DISPLAY_*` and `TOUCH_*` constants with values to values appropriate for
//! your touchscreen.
#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![no_std]
#![no_main]

/// Change these constants to values appropriate for your touchscreen.
const DISPLAY_MODEL: mipidsi::models::ILI9341Rgb565 = mipidsi::models::ILI9341Rgb565;
const DISPLAY_SIZE: Size = Size::new(240, 320);
const DISPLAY_ORIENTATION: mipidsi::options::Orientation =
    mipidsi::options::Orientation::new().flip_horizontal();
const DISPLAY_COLOR_ORDER: mipidsi::options::ColorOrder = mipidsi::options::ColorOrder::Bgr;

const TOUCH_ORIENTATION: RelativeOrientation = RelativeOrientation::new(false, true, false);

use esp_backtrace as _;
use esp_println as _;

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    channel::Channel,
    signal::Signal,
};
use embassy_time::{Delay, Timer};
use embedded_graphics::geometry::Size;
use esp_hal::{
    gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::Rate,
    Blocking,
};
use mipidsi;
use static_cell::StaticCell;
use xpt2046::{
    calibration::{estimate_calibration_data, RelativeOrientation},
    driver::Xpt2046,
};

#[cfg(feature = "defmt")]
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

#[cfg(feature = "log")]
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

macro_rules! make_static {
    ($t:ty,$v:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.init(($v))
    }};
}

// Required since espflash 4.x.
esp_bootloader_esp_idf::esp_app_desc!();

/// main
///
/// This function does five things
/// - initializes esp-hal,
/// - initializes Embassy,
/// - creates the shared SPI bus that will be used by the touch input task and
///   the display output task,
/// - creates channels and signals for communicating between main, the touch
///   input task and the display output task,
/// - starts touch input task and the display output task,
///
/// I use main to do four things:
/// - initialize the HAL,
/// - initialize Embassy,
/// - create/configure resources that are shared between tasks, and
/// - spawn tasks. Everything else is left to the tasks.
///
/// One reason I do this is two restrictions the Embassy executor puts the task
/// functions that it spawns. First, the Embassy executor requires task
/// functions have function parameters with static lifetimes. This is how the
/// Embassy executor guarantees that the spawned task functions won't outlive
/// the task function's parameters. Second, the Embassy executor requires task
/// function not have generic parameters. This is one way Embassy executor
/// ensures that it knows how much memory it needs. Unfortunately, this means
/// task function parameters cannot be traits. And tasks can call functions that
/// have generic parameters.
///
/// Variables with static lifetimes give me flashbacks to FORTRAN common blocks
/// and C/C++ global variables. I know they are not the same. However, they all
/// are too easily abused when you can't be bothered to reason through variable
/// scope, variable ownership and program flow.
///
/// Requiring function parameters to have concrete types means that the
/// functions that make use of hardware are tied to a specific HAL or device
/// driver rather than being more reusable by leveraging traits provided by
/// embedded-hal and embedded-graphics.
///
/// Since task functions can call functions with have functions parameters that
/// have non-static lifetimes and have generic parameters, I believe it is best
/// to move from main to the individual tasks as soon as possible.
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger_from_env();

    // This should be independent of ESP32 variant.
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    // These depend on the ESP32 variant. I chose these values for the ESP-C6
    // because
    //
    // - SPI2 is the only SPI peripheral available to the user,
    // - GPIO6, GPIO7, GPIO2 and GPIO16 have dedicated IO_MUX pins for SPI2's
    //   SCLK, MOSI, MISO and CS0 respectively,
    // - GPIO17 is SPI2's CS1 signal on the IO_MUX.
    //
    // Signals using IO_MUX direct input/output have lower delay than signals
    // not using IO_MUX direct input/output, and signals using the IO_MUX for
    // routing have lower delay than signals using the GPIO_MUX for routing.
    // However, I have no idea whether esp-hal takes IO_MUX into account when
    // configuring SPI or always uses GPIO_MUX. So, I do not know whether
    // selecting GPIOs that can take advantage of IO_MUX matters. In addition,
    // the ESP32-C6's SPI2 has a maximum clock frequency of 80MHz and it can be
    // run at 80MHz when the signals are routed using the GPIO_MUX. So, it
    // doesn't matter for the ESP32-C6. However, for some ESP32 variants, using
    // the GPIO_MUX rather than the IO_MUX does lower the maximum support SPI
    // clock frequency.
    let spi = peripherals.SPI2;
    let spi_sclk = peripherals.GPIO6;
    let spi_mosi = peripherals.GPIO7;
    let spi_miso = peripherals.GPIO2;
    let display_cs = peripherals.GPIO16;
    let display_dc = peripherals.GPIO22;
    let display_reset = peripherals.GPIO21;
    let display_backlight = peripherals.GPIO20;
    let touch_cs = peripherals.GPIO17;
    let touch_irq = peripherals.GPIO18;

    // The depends on the ESP32 variant.
    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // Set up the shared SPI bus.
    let spi = Spi::new(spi, Config::default())
        .unwrap()
        .with_sck(spi_sclk)
        .with_miso(spi_miso)
        .with_mosi(spi_mosi);
    let spi_bus = Mutex::<CriticalSectionRawMutex, _>::new(RefCell::new(spi));
    let spi_bus =
        &*make_static!(Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>, spi_bus);

    // Create a signal for sending touch events from the touch task to the display task.
    let touch_events = Signal::<CriticalSectionRawMutex, touch::TouchEvent>::new();
    let touch_events = &*make_static!(
        Signal::<CriticalSectionRawMutex, touch::TouchEvent>,
        touch_events
    );

    // Set up the channel for sending commands to the touch task.
    let touch_commands = Channel::<CriticalSectionRawMutex, touch::TouchCommand, 8>::new();
    let touch_commands =
        &*make_static!(Channel<CriticalSectionRawMutex, touch::TouchCommand, 8>, touch_commands);

    // Set up the channel for sending commands to the display task.
    let display_commands = Channel::<CriticalSectionRawMutex, display::DisplayCommand, 8>::new();
    let display_commands = &*make_static!(Channel<CriticalSectionRawMutex, display::DisplayCommand, 8>, display_commands);

    // Spawn touch task.
    spawner.must_spawn(touch_task(
        spi_bus,
        touch_cs.into(),
        touch_irq.into(),
        touch_commands,
        touch_events,
    ));

    // Spawn display task.
    spawner.must_spawn(display_task(
        spi_bus,
        display_cs.into(),
        display_dc.into(),
        display_reset.into(),
        display_backlight.into(),
        display_commands,
        touch_commands,
        touch_events,
    ));

    let display_commands_sender = display_commands.sender();

    display_commands_sender
        .send(display::DisplayCommand::Clear)
        .await;

    display_commands_sender
        .send(display::DisplayCommand::Calibrate)
        .await;
    loop {
        display_commands_sender
            .send(display::DisplayCommand::Clear)
            .await;
        Timer::after_secs(60).await;
    }
}

#[embassy_executor::task]
async fn touch_task(
    spi_bus: &'static Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>,
    touch_cs: AnyPin<'static>,
    touch_irq: AnyPin<'static>,
    touch_commands: &'static Channel<CriticalSectionRawMutex, touch::TouchCommand, 8>,
    touch_events: &'static Signal<CriticalSectionRawMutex, touch::TouchEvent>,
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

    // Set up touch driver.
    let mut touch_irq = Input::new(touch_irq, InputConfig::default().with_pull(Pull::Up));
    let calibration_data = estimate_calibration_data(TOUCH_ORIENTATION, DISPLAY_SIZE);
    let mut touch = Xpt2046::new(touch_spi_device, &calibration_data);

    // Signal used to stop the touch runner.
    let stop = Signal::<CriticalSectionRawMutex, bool>::new();

    loop {
        match touch::touch_runner(
            &mut touch,
            &mut touch_irq,
            touch_commands,
            touch_events,
            &stop,
        )
        .await
        {
            Ok(_) => {}
            Err(e) => error!("{:?}", e),
        };
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn display_task(
    spi_bus: &'static Mutex<CriticalSectionRawMutex, RefCell<Spi<'static, Blocking>>>,
    display_cs: AnyPin<'static>,
    display_dc: AnyPin<'static>,
    display_reset: AnyPin<'static>,
    display_backlight: AnyPin<'static>,
    display_commands: &'static Channel<CriticalSectionRawMutex, display::DisplayCommand, 8>,
    touch_commands: &'static Channel<CriticalSectionRawMutex, touch::TouchCommand, 8>,
    touch_events: &'static Signal<CriticalSectionRawMutex, touch::TouchEvent>,
) -> ! {
    // Set up the display SPI device.
    let display_cs = Output::new(display_cs, Level::High, OutputConfig::default());
    let display_spi_device = SpiDeviceWithConfig::new(
        spi_bus,
        display_cs,
        Config::default()
            .with_mode(Mode::_0)
            .with_frequency(Rate::from_mhz(80)),
    );

    // Set up the mipidsi display interface.
    let display_dc = Output::new(display_dc, Level::Low, OutputConfig::default());
    let mut display_buffer = [0_u8; 512];
    let display_interface =
        mipidsi::interface::SpiInterface::new(display_spi_device, display_dc, &mut display_buffer);

    // Set up the mipidsi display.
    let mut display_backlight = Output::new(display_backlight, Level::Low, OutputConfig::default());
    let display_reset = Output::new(display_reset, Level::Low, OutputConfig::default());
    let mut delay = Delay;
    let mut display = mipidsi::Builder::new(DISPLAY_MODEL, display_interface)
        .display_size(DISPLAY_SIZE.width as u16, DISPLAY_SIZE.height as u16)
        .orientation(DISPLAY_ORIENTATION)
        .color_order(DISPLAY_COLOR_ORDER)
        .reset_pin(display_reset)
        .init(&mut delay)
        .unwrap();

    display_backlight.set_high();

    // Signal used to stop the display runner.
    let stop = Signal::<CriticalSectionRawMutex, bool>::new();

    loop {
        match display::display_runner(
            &mut display,
            display_commands,
            touch_commands,
            touch_events,
            &stop,
        )
        .await
        {
            Ok(_) => {}
            Err(e) => error!("{:?}", e),
        };
        Timer::after_secs(1).await;
    }
}

pub(self) mod touch {
    use super::DISPLAY_SIZE;
    use core::fmt::Debug;
    use embassy_futures::select::{select3, Either3};
    use embassy_sync::{blocking_mutex::raw::RawMutex, channel::Channel, signal::Signal};
    use embassy_time::Timer;
    use embedded_hal::{digital::InputPin, spi::SpiDevice};
    use embedded_hal_async::digital::Wait;
    use xpt2046::driver::{CalibrationData, Error, Point, Xpt2046};

    #[cfg(feature = "defmt")]
    #[allow(unused_imports)]
    use defmt::{debug, error, info, trace, warn};

    #[cfg(feature = "log")]
    #[allow(unused_imports)]
    use log::{debug, error, info, trace, warn};

    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug, Clone, Copy)]
    pub enum TouchEvent {
        /// The first touch point sent since a touch was detected.
        ///
        /// The tuple is (calibrated_touch_point, raw_touch_point)
        Down((Point, Option<Point>)),
        /// The touch point has changed since the previously sent touch point.
        ///
        /// The tuple is (calibrated_touch_point, raw_touch_point)
        Move((Point, Option<Point>)),
        /// Touch is no longer detected.
        Up,
    }

    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum TouchEventMode {
        /// Don't send a new touch event when the new touch point is outside the
        /// display or the new touch point is the same as same as the touch
        /// point in the most recent touch event sent. And don't include the raw
        /// touch point.
        Filtered,
        /// Send a touch event for every new touch point. And include the raw
        /// touch point.
        Raw,
    }

    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum TouchCommand {
        /// Update the calibration data being used by the XPT2046 driver.
        UpdateCalibration(CalibrationData),
        /// Update the touch event mode (see [`TouchEventMode`]).
        UpdateTouchEventMode(TouchEventMode),
    }

    /// Runs the touch task.
    ///
    /// It waits for the XPT2046 to detect that the touch panel has been touched
    /// (irq is low). Once this has been detected it calls [`Xpt2046::run()`]
    /// every 500 microseconds until the XPT2046 detects that the touch panel is
    /// no longer being touched (irq is high). After each call to
    /// [`Xpt2046::run()`], if [`Xpt2046::is_touched()`] indicates that the
    /// touch point is reliable, then it may send a touch event
    /// ([`TouchEvent`]).
    ///
    /// If it is in filtered touch event mode ([`TouchEventMode::Filtered`]),
    /// then it will send a down touch event ([`TouchEvent::Down`]) if the touch
    /// point is the first one since the XPT2046 detected a touch, and it will
    /// send a move touch event ([`TouchEvent::Move`]) if the touch point is
    /// different from the previous touch point. If it is in raw touch event
    /// mode ([`TouchEventMode::Raw`]), then it sends a down touch event if the
    /// touch point is the first one since the XPT2046 detected a touch,
    /// otherwise it will send a move touch event. When the XPT2046 no longer
    /// detects a touch, an up touch event ([`TouchEvent::Up`]) is sent.
    ///
    /// When it is in filtered touch event mode, touch events contain the
    /// calibrated touch point but not the raw touch point. When it is in raw
    /// mode, touch events contain both the calibrated and raw touch points.
    pub async fn touch_runner<'a, Spi, SpiError, Irq, IrqError, M, const N: usize>(
        driver: &mut Xpt2046<Spi>,
        irq: &mut Irq,
        commands: &'static Channel<M, TouchCommand, N>,
        events: &'a Signal<M, TouchEvent>,
        stop: &'a Signal<M, bool>,
    ) -> Result<(), Error<SpiError, IrqError>>
    where
        Spi: SpiDevice<Error = SpiError>,
        SpiError: embedded_hal::spi::Error,
        Irq: InputPin<Error = IrqError> + Wait,
        IrqError: embedded_hal::digital::Error,
        M: RawMutex,
    {
        let commands_receiver = commands.receiver();

        driver.init().map_err(|e| Error::Spi(e))?;
        driver.clear_touch();
        let mut event_mode = TouchEventMode::Filtered;
        let mut event = TouchEvent::Up;
        loop {
            match select3(irq.wait_for_low(), commands_receiver.receive(), stop.wait()).await {
                Either3::First(r) => r.map_err(|e| Error::Irq(e))?,
                Either3::Second(command) => match command {
                    TouchCommand::UpdateCalibration(calibration_data) => {
                        driver.set_calibration_data(&calibration_data);
                    }
                    TouchCommand::UpdateTouchEventMode(mode) => event_mode = mode,
                },
                Either3::Third(_b) => return Ok(()),
            }
            while irq.is_low().map_err(|e| Error::Irq(e))? {
                driver.run(irq)?;
                if driver.is_touched() {
                    let point = driver.get_touch_point();
                    match event_mode {
                        TouchEventMode::Filtered => {
                            if point.x >= 0
                                && point.x < DISPLAY_SIZE.width as i32
                                && point.y >= 0
                                && point.y < DISPLAY_SIZE.height as i32
                            {
                                match event {
                                    TouchEvent::Up => {
                                        event = TouchEvent::Down((point, None));
                                        events.signal(event);
                                    }
                                    TouchEvent::Down((p, _)) | TouchEvent::Move((p, _)) => {
                                        if point != p {
                                            event = TouchEvent::Move((point, None));
                                            events.signal(event);
                                        }
                                    }
                                }
                            }
                        }
                        TouchEventMode::Raw => {
                            let point_raw = driver.get_touch_point_raw();
                            match event {
                                TouchEvent::Up => {
                                    event = TouchEvent::Down((point, Some(point_raw)));
                                    events.signal(event);
                                }
                                TouchEvent::Down(_) | TouchEvent::Move(_) => {
                                    event = TouchEvent::Move((point, Some(point_raw)));
                                    events.signal(event);
                                }
                            }
                        }
                    }
                }
                match select3(
                    Timer::after_micros(500),
                    commands_receiver.receive(),
                    stop.wait(),
                )
                .await
                {
                    Either3::First(_) => {}
                    Either3::Second(command) => match command {
                        TouchCommand::UpdateCalibration(calibration_data) => {
                            driver.set_calibration_data(&calibration_data);
                        }
                        TouchCommand::UpdateTouchEventMode(mode) => event_mode = mode,
                    },
                    Either3::Third(_b) => return Ok(()),
                }
            }
            driver.clear_touch();
            event = TouchEvent::Up;
            events.signal(event);
        }
    }
}

pub(self) mod display {
    use crate::touch::TouchEventMode;

    use super::touch::{TouchCommand, TouchEvent};
    use super::DISPLAY_SIZE;
    use core::fmt::Debug;
    use embassy_futures::select::{select, select3, Either, Either3};
    use embassy_sync::{blocking_mutex::raw::RawMutex, channel::Channel, signal::Signal};
    use embassy_time::Timer;
    use embedded_graphics::{
        geometry::Size,
        prelude::*,
        primitives::{Line, PrimitiveStyle, Rectangle},
    };
    use xpt2046::calibration::{
        calculate_calibration_data, generate_display_calibration_points, CalibrationPoints,
    };

    #[cfg(feature = "defmt")]
    #[allow(unused_imports)]
    use defmt::{debug, error, info, trace, warn};

    #[cfg(feature = "log")]
    #[allow(unused_imports)]
    use log::{debug, error, info, trace, warn};

    /// Commands that can be sent to the display task.
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum DisplayCommand {
        /// Commands the display task to clear the display.
        Clear,
        /// Commands the display task to run touch calibration.
        Calibrate,
    }

    /// Runs the display task.
    ///
    /// By default, the display task lets the user scribble on the display.
    /// However, it can be commanded to clear the display or to run touch
    /// calibration.
    pub async fn display_runner<'a, DT, M, const N: usize>(
        display: &mut DT,
        commands: &'a Channel<M, DisplayCommand, N>,
        touch_commands: &'static Channel<M, TouchCommand, N>,
        touch_events: &'a Signal<M, TouchEvent>,
        stop: &'a Signal<M, bool>,
    ) -> Result<(), DT::Error>
    where
        DT: DrawTarget<Color: RgbColor>,
        M: RawMutex,
    {
        let commands_receiver = commands.receiver();
        let touch_commands_sender = touch_commands.sender();

        display.clear(DT::Color::BLACK)?;
        loop {
            match select3(
                commands_receiver.receive(),
                touch_events.wait(),
                stop.wait(),
            )
            .await
            {
                Either3::First(display_command) => match display_command {
                    DisplayCommand::Clear => display.clear(DT::Color::BLACK)?,
                    DisplayCommand::Calibrate => {
                        match calibrate(display, touch_commands, touch_events, stop).await {
                            Ok(_) => {}
                            Err(e) => {
                                touch_commands_sender
                                    .send(TouchCommand::UpdateTouchEventMode(
                                        TouchEventMode::Filtered,
                                    ))
                                    .await;
                                return Err(e);
                            }
                        };
                    }
                },
                Either3::Second(touch_event) => match touch_event {
                    TouchEvent::Down((touch_point, _)) | TouchEvent::Move((touch_point, _)) => {
                        Rectangle::new(touch_point, Size::new_equal(1))
                            .into_styled(PrimitiveStyle::with_fill(DT::Color::WHITE))
                            .draw(display)?
                    }
                    TouchEvent::Up => {}
                },
                Either3::Third(_b) => return Ok(()),
            }
        }
    }

    /// Runs the calibration routine and updates the calibration.
    ///
    /// It enables raw touch event mode so that it is sent the raw touch points.
    /// It creates the three display calibration points by calling
    /// [`xpt2046::calibration::generate_display_calibration_points()`]. For
    /// each display calibration point, it collects a corresponding touch
    /// calibration point by
    ///
    /// - drawing a crosshair on the display,
    /// - waiting for a down touch event,
    /// - waiting 500ms to give the user time to be make sure the pointer is on
    ///   the center of the crosshair, and
    /// - saving the next move touch event as the touch calibration point.
    ///
    /// After it has collected the three touch calibration points, it calculates
    /// the calibration data. If an error occurs in the calculation of the
    /// calibration data, then it repeats touch calibration point collection.
    /// Once it has valid calibration data, it sends it to the touch driver.
    /// Finally, it re-enables filtered touch event mode and returns.
    ///
    /// If the process exits prematurely for eny reason, it re-enables filtered
    /// touch event mode and returns.
    async fn calibrate<'a, DT, M, const N: usize>(
        display: &mut DT,
        touch_commands: &'static Channel<M, TouchCommand, N>,
        touch_events: &'a Signal<M, TouchEvent>,
        stop: &'a Signal<M, bool>,
    ) -> Result<(), DT::Error>
    where
        DT: DrawTarget<Color: RgbColor>,
        M: RawMutex,
    {
        let touch_commands_sender = touch_commands.sender();
        touch_commands_sender
            .send(TouchCommand::UpdateTouchEventMode(TouchEventMode::Raw))
            .await;
        let result: Result<(), DT::Error> = {
            let display_cp: CalibrationPoints = generate_display_calibration_points(DISPLAY_SIZE);
            let mut cp: [(Point, Option<Point>); 3] = [
                (display_cp.a, None),
                (display_cp.b, None),
                (display_cp.c, None),
            ];
            let mut calibrated = false;
            while !calibrated {
                for (display_point, touch_point) in cp.iter_mut() {
                    display.clear(DT::Color::BLACK)?;
                    draw_calibration_point(display, &display_point)?;
                    *touch_point = None;
                    while touch_point.is_none() {
                        match select(stop.wait(), touch_events.wait()).await {
                            Either::First(_b) => return Ok(()),
                            Either::Second(event) => match event {
                                TouchEvent::Down((_, p)) => *touch_point = p,
                                _ => {}
                            },
                        }
                    }
                    match select(Timer::after_millis(500), stop.wait()).await {
                        Either::First(_) => {}
                        Either::Second(_b) => return Ok(()),
                    }
                    *touch_point = None;
                    while touch_point.is_none() {
                        match select(stop.wait(), touch_events.wait()).await {
                            Either::First(_b) => return Ok(()),
                            Either::Second(event) => match event {
                                TouchEvent::Move((_, p)) => *touch_point = p,
                                _ => {}
                            },
                        }
                    }
                }
                display.clear(DT::Color::BLACK)?;
                if cp[0].1.is_some() && cp[1].1.is_some() && cp[2].1.is_some() {
                    let touch_cp = CalibrationPoints {
                        a: cp[0].1.unwrap(),
                        b: cp[1].1.unwrap(),
                        c: cp[2].1.unwrap(),
                    };
                    match calculate_calibration_data(&display_cp, &touch_cp) {
                        Ok(calibration_data) => {
                            touch_commands_sender
                                .send(TouchCommand::UpdateCalibration(calibration_data))
                                .await;
                            calibrated = true;
                        }
                        Err(_) => continue,
                    }
                }
            }
            Ok(())
        };
        touch_commands_sender
            .send(TouchCommand::UpdateTouchEventMode(TouchEventMode::Filtered))
            .await;
        result
    }

    /// Draws a crosshair on the display centered at the location `point`.
    fn draw_calibration_point<DT>(display: &mut DT, point: &Point) -> Result<(), DT::Error>
    where
        DT: DrawTarget<Color: RgbColor>,
    {
        Line::new(
            Point::new(point.x - 4, point.y),
            Point::new(point.x + 4, point.y),
        )
        .into_styled(PrimitiveStyle::with_stroke(DT::Color::WHITE, 1))
        .draw(display)?;
        Line::new(
            Point::new(point.x, point.y - 4),
            Point::new(point.x, point.y + 4),
        )
        .into_styled(PrimitiveStyle::with_stroke(DT::Color::WHITE, 1))
        .draw(display)?;

        Ok(())
    }
}
