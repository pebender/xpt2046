//! Functions for calculating the touch panel X and Y measurement calibration
//! data.
//!
//! The calibration uses the three-point calibration algorithm from "SLYT277:
//! Calibration in Touch-Screen Systems" by Texas Instruments
//! (<https://www.ti.com/lit/an/slyt277/slyt277.pdf>). The algorithm assumes the
//! display panel and touch panel may be misaligned such that the touch panel
//! may be translated, rotated, and scaled relative to the display panel. The
//! algorithm uses three known display panel points along with three measured
//! touch panel points corresponding to the three known display panel points to
//! calculate the coefficients of an affine transformation from the touch panel
//! points to the display panel points.
//!
//! While these functions were created to provide calibration data for the
//! Xpt2046 driver, they are not specific to the Xpt2046 driver.

use super::driver::{Point, Size};
use core::fmt::Debug;
#[cfg(feature = "defmt")]
use defmt::Format;

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

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone, Copy)]
pub struct CalibrationData {
    pub alpha_x: f32,
    pub beta_x: f32,
    pub delta_x: f32,
    pub alpha_y: f32,
    pub beta_y: f32,
    pub delta_y: f32,
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

/// The orientation difference between the XPT2046's X and Y axes the
/// display's X and Y axes.
///
/// It is used by [`estimate_calibration_data()`] in conjunction with the
/// display size to create an estimate of [`CalibrationData`].
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub struct RelativeOrientation {
    /// Swap the X and Y axes.
    pub swap_xy: bool,
    /// Mirror about the X axis. mirror_x is applied after swap_xy;
    pub mirror_x: bool,
    /// Mirror about the Y axis. mirror_y is applied after swap_xy;
    pub mirror_y: bool,
}

impl RelativeOrientation {
    pub fn new(swap_xy: bool, mirror_x: bool, mirror_y: bool) -> Self {
        RelativeOrientation {
            swap_xy,
            mirror_x,
            mirror_y,
        }
    }
}

/// Provides a coarse estimate of the calibration data.
///
/// The function makes only a coarse estimate. It assumes the only differences
/// between the display panel and touch panel are their understanding of the
/// orientation and scale of the X and Y axes. It assumes the display panel and
/// touch panel are exactly the same size, are perfectly aligned and use the
/// full range of the X and Y axes values.
///
/// The intention of providing estimated calibration data it to make it
/// possible to initiate the calibration process using a device's touch display
/// before the touch display has been calibrated. Is is expected that after the
/// calibration process has been completed successfully, the device will store
/// and use the calibration data from the successful calibration process.
///
/// `orientation` is relative to the orientation presented to upper layer
/// software by the display driver. In practice, this means it is relative to a
/// display orientation where the (0,0) coordinate is the upper left corner of
/// the display, the X axis is the horizontal axis of the display with values
/// increasing in the rightward direction, and the Y axis in the vertical axis
/// of the display with values increasing in the downward direction.
///
/// The `display_size.width` is along the X axis and the `display_size.height`
/// is along the Y axis. In practice, `display_size.width` and
/// `display_size.height` are in pixels.
pub fn estimate_calibration_data(
    orientation: RelativeOrientation,
    display_size: Size,
) -> CalibrationData {
    const TOUCH_SIZE: f32 = 4096.0;

    let mut calibration_data = CalibrationData {
        alpha_x: display_size.width as f32 / TOUCH_SIZE,
        beta_x: 0.0,
        delta_x: 0.0,
        alpha_y: 0.0,
        beta_y: display_size.height as f32 / TOUCH_SIZE,
        delta_y: 0.0,
    };

    if orientation.swap_xy {
        let swap_x = calibration_data.alpha_x;
        calibration_data.alpha_x = calibration_data.beta_x;
        calibration_data.beta_x = swap_x;
        let swap_y = calibration_data.alpha_y;
        calibration_data.alpha_y = calibration_data.beta_y;
        calibration_data.beta_y = swap_y;
    }
    if orientation.mirror_x {
        calibration_data.alpha_y *= -1.0;
        calibration_data.beta_y *= -1.0;
        calibration_data.delta_y *= -1.0;
        calibration_data.delta_y += display_size.height as f32;
    }
    if orientation.mirror_y {
        calibration_data.alpha_x *= -1.0;
        calibration_data.beta_x *= -1.0;
        calibration_data.delta_x *= -1.0;
        calibration_data.delta_x += display_size.width as f32;
    }

    calibration_data
}

/// Generates the three display panel calibration points.
///
/// The points depend on the display_size (in pixels) of the display panel. The
/// points are chosen as shown below. They are chosen to provide reasonable
/// coverage of the display panel. They are chosen such that their is a very
/// good chance they will result in three linearly independent equations as
/// required by three-point calibration algorithm.
///
/// ```rust
/// CalibrationPoints {
///   a: Point {x: 1 * display_size.width / 4, 1 * y: display_size.height / 4 },
///   b: Point {x: 2 * display_size.width / 4, 3 * y: display_size.height / 4 },
///   c: Point {x: 3 * display_size.width / 4, 2 * y: display_size.height / 4 },
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

/// Calculates the calibration data used by the [`crate::driver::Xpt2046`] driver.
/// It uses the display panel calibration points along with their corresponding
/// measured touch panel calibration points to calculate the coefficients of an
/// affine transformation that will convert the X and Y measurements provided by
/// the XPT2046 controller into the corresponding pixel positions of the
/// display.
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

/// Applies calibration data to the touch point.
pub fn apply_calibration_data(touch_point: &Point, calibration_data: &CalibrationData) -> Point {
    let x = touch_point.x as f32;
    let y = touch_point.y as f32;
    let x = calibration_data.alpha_x * x + calibration_data.beta_x * y + calibration_data.delta_x;
    let y = calibration_data.alpha_y * x + calibration_data.beta_y * y + calibration_data.delta_y;
    let x = x as i32;
    let y = y as i32;
    Point::new(x, y)
}
