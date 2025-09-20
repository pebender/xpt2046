//! Functions for running the touch screen calibration procedure.
//!
//! The two requirements on the touch screen are that its touch panel controller
//! is the XPT2046, and that the software driver for its display panel
//! controller implements the
//! [embedded-graphics-core's](https://crates.io/crates/embedded-graphics-core)
//! [DrawTarget](https://docs.rs/embedded-graphics-core/latest/embedded_graphics_core/draw_target/trait.DrawTarget.html)
//! trait with a Color type implementing the
//! [RgbColor](https://docs.rs/embedded-graphics-core/latest/embedded_graphics_core/pixelcolor/trait.RgbColor.html)
//! trait and an Error type implementing the Debug trait.

use super::{
    calibration::{
        calculate_calibration_data, generate_display_calibration_points, CalibrationError,
        CalibrationPoints,
    },
    driver::{CalibrationData, Error, Point, Xpt2046},
};
use core::fmt::Debug;
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::RgbColor,
    primitives::{Line, Primitive, PrimitiveStyle},
    Drawable,
};
use embedded_hal::{delay::DelayNs, digital::InputPin, spi::SpiDevice};

/// The error returned when an error occurs in [`run_calibration()`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum CalibrationRunError<SpiError, IrqError, DTError>
where
    SpiError: embedded_hal::spi::Error,
    IrqError: embedded_hal::digital::Error,
    DTError: Debug,
{
    /// An error occurred in the Xpt2046 touch panel driver.
    Xpt2046(Error<SpiError, IrqError>),
    /// An error occurred in the display panel driver.
    DrawTarget(DTError),
    /// An error occurred in the calibration calculation.
    Calibration(CalibrationError),
}

/// Runs the touch screen calibration procedure.
pub fn run_calibration<Spi, SpiError, Irq, IrqError, DT, DTError, DELAY>(
    touch: &mut Xpt2046<Spi, Irq>,
    draw_target: &mut DT,
    delay: &mut DELAY,
) -> Result<CalibrationData, CalibrationRunError<SpiError, IrqError, DTError>>
where
    Spi: SpiDevice<u8, Error = SpiError>,
    SpiError: embedded_hal::spi::Error,
    Irq: InputPin<Error = IrqError>,
    IrqError: embedded_hal::digital::Error,
    DT: DrawTarget<Color: RgbColor, Error = DTError>,
    DTError: Debug,
    DELAY: DelayNs,
{
    let display_cp = generate_display_calibration_points(draw_target.bounding_box().size);

    let mut touch_cp = CalibrationPoints {
        a: Point::zero(),
        b: Point::zero(),
        c: Point::zero(),
    };

    // Measure touch point for calibration point a.
    draw_target
        .clear(DT::Color::BLACK)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    draw_calibration_point(draw_target, &display_cp.a)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    touch.clear_touch();
    while !touch.is_touched() {
        touch.run().map_err(|e| CalibrationRunError::Xpt2046(e))?;
        delay.delay_us(500);
    }
    touch_cp.a = touch.get_touch_point_raw();
    let _ = draw_target.clear(DT::Color::BLACK);
    while touch
        .penirq_is_active()
        .map_err(|e| CalibrationRunError::Xpt2046(e))?
    {
        delay.delay_ms(100);
    }
    delay.delay_ms(200);

    // Measure touch point for calibration point b.
    draw_target
        .clear(DT::Color::BLACK)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    draw_calibration_point(draw_target, &display_cp.b)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    touch.clear_touch();
    while !touch.is_touched() {
        touch.run().map_err(|e| CalibrationRunError::Xpt2046(e))?;
        delay.delay_us(500);
    }
    touch_cp.b = touch.get_touch_point_raw();
    draw_target
        .clear(DT::Color::BLACK)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    while touch
        .penirq_is_active()
        .map_err(|e| CalibrationRunError::Xpt2046(e))?
    {
        delay.delay_ms(100);
    }
    delay.delay_ms(200);

    // Measure touch point for calibration point c.
    draw_target
        .clear(DT::Color::BLACK)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    draw_calibration_point(draw_target, &display_cp.c)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    touch.clear_touch();
    while !touch.is_touched() {
        touch.run().map_err(|e| CalibrationRunError::Xpt2046(e))?;
        delay.delay_us(500);
    }
    touch_cp.c = touch.get_touch_point_raw();
    let _ = draw_target.clear(DT::Color::BLACK);
    while touch
        .penirq_is_active()
        .map_err(|e| CalibrationRunError::Xpt2046(e))?
    {
        delay.delay_ms(100);
    }
    delay.delay_ms(200);

    let calibration_data = calculate_calibration_data(&display_cp, &touch_cp);

    #[cfg(feature = "defmt")]
    {
        defmt::debug!("displayed (LCD) calibration points: {:?}", display_cp);
        defmt::debug!("measured (touch) calibration points: {:?}", touch_cp);
        defmt::debug!("calculated calibration data: {:?}", calibration_data);
    }

    calibration_data.map_err(|e| CalibrationRunError::Calibration(e))
}

/// Draws a calibration point on the touch screen.
fn draw_calibration_point<DT>(draw_target: &mut DT, point: &Point) -> Result<(), DT::Error>
where
    DT: DrawTarget<Color: RgbColor>,
{
    Line::new(
        Point::new(point.x - 4, point.y),
        Point::new(point.x + 4, point.y),
    )
    .into_styled(PrimitiveStyle::with_stroke(DT::Color::WHITE, 1))
    .draw(draw_target)?;
    Line::new(
        Point::new(point.x, point.y - 4),
        Point::new(point.x, point.y + 4),
    )
    .into_styled(PrimitiveStyle::with_stroke(DT::Color::WHITE, 1))
    .draw(draw_target)?;

    Ok(())
}
