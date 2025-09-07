//! This module contains functions for calculating the X and Y measurement
//! calibration data ([`super::CalibrationData`]) as well as a function for
//! running the calibration procedure.
//!
//! The calibration uses the three-point calibration algorithm from "SLYT277:
//! Calibration in Touch-Screen Systems" by Texas Instruments
//! (<https://www.ti.com/lit/an/slyt277/slyt277.pdf>). The algorithm assumes the
//! display panel and touch panel may be misaligned such that the touch panel
//! may be translated, rotated, and scaled relative to the display panel.//! The
//! algorithm uses three known display panel points along with the three
//! measured touch panel points corresponding to the the three known display
//! panel points.

use super::CalibrationData;
use super::{Error, Xpt2046};
use core::fmt::Debug;
pub use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::RgbColor,
    primitives::{Line, Primitive, PrimitiveStyle},
    Drawable,
};
use embedded_hal::{delay::DelayNs, digital::InputPin, spi::SpiDevice};

#[cfg(feature = "defmt")]
use defmt::Format;

/// The error returned when an error occurs in [`run_calibration()`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub enum CalibrationRunError<SpiError, IrqError, DTError> {
    Xpt2046(Error<SpiError, IrqError>),
    DrawTarget(DTError),
    Calibration(CalibrationError),
}

/// The error returned when an error occurs in [`calculate_calibration_data()`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub enum CalibrationError {
    All,
    AlphaX,
    BetaX,
    DeltaX,
    AlphaY,
    BetaY,
    DeltaY,
}

/// The three calibration points for either the display panel or the touch
/// panel.
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub struct CalibrationPoints {
    pub a: Point,
    pub b: Point,
    pub c: Point,
}

/// The transform performed on the XPT2046's X and Y axes so that it has the
/// same orientation as the display's X and Y axes.
///
/// It is used in by [`estimate_calibration_data()`] in conjunction with the
/// display size to create an estimate of [`super::CalibrationData`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub struct Transform {
    /// Swap the X and Y axes.
    pub swap_xy: bool,
    /// Mirror about the X axis. mirror_x is applied after swap_xy;
    pub mirror_x: bool,
    /// Mirror about the Y axis. mirror_y is applied after swap_xy;
    pub mirror_y: bool,
}

impl Transform {
    pub fn new(swap_xy: bool, mirror_x: bool, mirror_y: bool) -> Self {
        Transform {
            swap_xy,
            mirror_x,
            mirror_y,
        }
    }
}

/// This function provides a course estimate of the calibration data.
///
/// The function makes only a coarse estimate. It assumes the only differences
/// between the display panel and touch panel are their understanding of the
/// orientation and scale of the X and Y axes. It assumes the display panel and
/// touch panel are exactly the same size, are perfectly aligned and use the
/// full range of the X and Y axes values.
///
/// The intention of the providing estimated calibration data it to make it
/// possible to initiate the calibration process using a device's touch display
/// before the touch display has been calibrated. Is is expected that after the
/// calibration process has been completed successfully, the device will store
/// and use the calibration data from the successful calibration process.
pub fn estimate_calibration_data(transform: Transform, display_size: Size) -> CalibrationData {
    const TOUCH_SIZE: f32 = 4096.0;

    let mut calibration_data = CalibrationData {
        alpha_x: display_size.width as f32 / TOUCH_SIZE,
        beta_x: 0.0,
        delta_x: 0.0,
        alpha_y: 0.0,
        beta_y: display_size.height as f32 / TOUCH_SIZE,
        delta_y: 0.0,
    };

    if transform.swap_xy {
        let swap_x = calibration_data.alpha_x;
        calibration_data.alpha_x = calibration_data.beta_x;
        calibration_data.beta_x = swap_x;
        let swap_y = calibration_data.alpha_y;
        calibration_data.alpha_y = calibration_data.beta_y;
        calibration_data.beta_y = swap_y;
    }
    if transform.mirror_x {
        calibration_data.alpha_y *= -1.0;
        calibration_data.beta_y *= -1.0;
        calibration_data.delta_y *= -1.0;
        calibration_data.delta_y += display_size.height as f32;
    }
    if transform.mirror_y {
        calibration_data.alpha_x *= -1.0;
        calibration_data.beta_x *= -1.0;
        calibration_data.delta_x *= -1.0;
        calibration_data.delta_x += display_size.width as f32;
    }

    defmt::info!("calibration estimate: {:?}", calibration_data);

    calibration_data
}

/// This function generates the three display calibration points.
///
/// The points depend on the size (in pixels) of the display panel. The points
/// are chosen as shown below. They are chosen to provide reasonable coverage of
/// the display panel. They are chosen such that their is a very good chance
/// they will result in three linearly independent equations as required by
/// three-point calibration algorithm.
///
/// ```rust
/// CalibrationPoints {
///   a: Point {x: 1 * display_size.width / 4, 1 * y: display_size.heigh / 4 },
///   b: Point {x: 2 * display_size.width / 4, 3 * y: display_size.heigh / 4 },
///   c: Point {x: 3 * display_size.width / 4, 2 * y: display_size.heigh / 4 },
/// }
/// ```
pub fn generate_display_calibration_points(display_size: Size) -> CalibrationPoints {
    let x = display_size.width as i32 / 4;
    let y = display_size.height as i32 / 4;
    CalibrationPoints {
        a: Point::new(1 * x, 1 * y),
        b: Point::new(2 * x, 3 * y),
        c: Point::new(3 * x, 2 * y),
    }
}

/// This function calculates the [`super::CalibrationData`] from the display
/// panel calibration points and corresponding measured touch panel calibration
/// points.
///
/// uses the three-point calibration algorithm from "SLYT277:
/// Calibration in Touch-Screen Systems" by Texas Instruments
/// <https://www.ti.com/lit/an/slyt277/slyt277.pdf>.
pub fn calculate_calibration_data(
    display_cp: &CalibrationPoints,
    touch_cp: &CalibrationPoints,
) -> Result<CalibrationData, CalibrationError> {
    let det = (touch_cp.a.x - touch_cp.c.x) * (touch_cp.b.y - touch_cp.c.y)
        - (touch_cp.b.x - touch_cp.c.x) * (touch_cp.a.y - touch_cp.c.y);

    if det == 0 {
        return Err(CalibrationError::All);
    }

    let det_alpha_x = (display_cp.a.x - display_cp.c.x) * (touch_cp.b.y - touch_cp.c.y)
        - (display_cp.b.x - display_cp.c.x) * (touch_cp.a.y - touch_cp.c.y);
    let det_beta_x = (touch_cp.a.x - touch_cp.c.x) * (display_cp.b.x - display_cp.c.x)
        - (touch_cp.b.x - touch_cp.c.x) * (display_cp.a.x - display_cp.c.x);
    let det_delta_x = (display_cp.a.x)
        * (touch_cp.b.x * touch_cp.c.y - touch_cp.c.x * touch_cp.b.y)
        - (display_cp.b.x) * (touch_cp.a.x * touch_cp.c.y - touch_cp.c.x * touch_cp.a.y)
        + (display_cp.c.x) * (touch_cp.a.x * touch_cp.b.y - touch_cp.b.x * touch_cp.a.y);
    let det_alpha_y = (display_cp.a.y - display_cp.c.y) * (touch_cp.b.y - touch_cp.c.y)
        - (display_cp.b.y - display_cp.c.y) * (touch_cp.a.y - touch_cp.c.y);
    let det_beta_y = (touch_cp.a.x - touch_cp.c.x) * (display_cp.b.y - display_cp.c.y)
        - (touch_cp.b.x - touch_cp.c.x) * (display_cp.a.y - display_cp.c.y);
    let det_delta_y = (display_cp.a.y)
        * (touch_cp.b.x * touch_cp.c.y - touch_cp.c.x * touch_cp.b.y)
        - (display_cp.b.y) * (touch_cp.a.x * touch_cp.c.y - touch_cp.c.x * touch_cp.a.y)
        + (display_cp.c.y) * (touch_cp.a.x * touch_cp.b.y - touch_cp.b.x * touch_cp.a.y);

    let det = det as f32;
    let det_alpha_x = det_alpha_x as f32;
    let det_beta_x = det_beta_x as f32;
    let det_delta_x = det_delta_x as f32;
    let det_alpha_y = det_alpha_y as f32;
    let det_beta_y = det_beta_y as f32;
    let det_delta_y = det_delta_y as f32;

    let alpha_x = det_alpha_x / det;
    if !alpha_x.is_normal() {
        return Err(CalibrationError::AlphaX);
    }
    let beta_x = det_beta_x / det;
    if !beta_x.is_normal() {
        return Err(CalibrationError::BetaX);
    }
    let delta_x = det_delta_x / det;
    if !delta_x.is_normal() {
        return Err(CalibrationError::DeltaX);
    }
    let alpha_y = det_alpha_y / det;
    if !alpha_y.is_normal() {
        return Err(CalibrationError::AlphaY);
    }
    let beta_y = det_beta_y / det;
    if !beta_y.is_normal() {
        return Err(CalibrationError::BetaY);
    }
    let delta_y = det_delta_y / det;
    if !delta_y.is_normal() {
        return Err(CalibrationError::DeltaY);
    }

    Ok(CalibrationData {
        alpha_x,
        beta_x,
        delta_x,
        alpha_y,
        beta_y,
        delta_y,
    })
}

pub fn run_calibration<SPI, SPIError, IRQ, IRQError, DT, DTError, DELAY>(
    touch: &mut Xpt2046<SPI>,
    irq: &mut IRQ,
    draw_target: &mut DT,
    delay: &mut DELAY,
) -> Result<CalibrationData, CalibrationRunError<SPIError, IRQError, DTError>>
where
    SPI: SpiDevice<u8, Error = SPIError>,
    SPIError: Debug,
    IRQ: InputPin<Error = IRQError>,
    IRQError: Debug,
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
        touch
            .run(irq)
            .map_err(|e| CalibrationRunError::Xpt2046(e))?;
        delay.delay_us(500);
    }
    touch_cp.a = touch.get_touch_point_raw();
    let _ = draw_target.clear(DT::Color::BLACK);
    while irq
        .is_low()
        .map_err(|e| CalibrationRunError::Xpt2046(Error::Irq(e)))?
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
        touch
            .run(irq)
            .map_err(|e| CalibrationRunError::Xpt2046(e))?;
        delay.delay_us(500);
    }
    touch_cp.b = touch.get_touch_point_raw();
    draw_target
        .clear(DT::Color::BLACK)
        .map_err(|e| CalibrationRunError::DrawTarget(e))?;
    while irq
        .is_low()
        .map_err(|e| CalibrationRunError::Xpt2046(Error::Irq(e)))?
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
        touch
            .run(irq)
            .map_err(|e| CalibrationRunError::Xpt2046(e))?;
        delay.delay_us(500);
    }
    touch_cp.c = touch.get_touch_point_raw();
    let _ = draw_target.clear(DT::Color::BLACK);
    while irq
        .is_low()
        .map_err(|e| CalibrationRunError::Xpt2046(Error::Irq(e)))?
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

fn draw_calibration_point<DT>(draw_target: &mut DT, point: &Point) -> Result<(), DT::Error>
where
    DT: DrawTarget<Color: RgbColor>,
{
    draw_target.clear(DT::Color::BLACK)?;

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
