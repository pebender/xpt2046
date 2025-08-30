use super::CalibrationData;
use crate::error::CalibrationError;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::{Rgb565, RgbColor},
    primitives::{Line, Primitive, PrimitiveStyle},
    Drawable,
};

#[cfg(feature = "defmt")]
use defmt::Format;

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, Clone)]
pub struct CalibrationPoint {
    pub a: Point,
    pub b: Point,
    pub c: Point,
}

impl CalibrationPoint {
    pub fn delta(&self) -> i32 {
        (self.a[0] - self.c[0]) * (self.b[1] - self.c[1])
            - (self.b[0] - self.c[0]) * (self.a[1] - self.c[1])
    }
}

/*
 *  TODO:
 *  Find a way to hide this behind a feature
 */
pub(crate) fn calibration_draw_point<DT: DrawTarget<Color = Rgb565>>(dt: &mut DT, p: &Point) {
    let _ = Line::new(Point::new(p.x - 4, p.y), Point::new(p.x + 4, p.y))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
    let _ = Line::new(Point::new(p.x, p.y - 4), Point::new(p.x, p.y + 4))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
}

pub(crate) fn calculate_calibration(
    old_cp: &CalibrationPoint,
    new_cp: &CalibrationPoint,
) -> Result<CalibrationData, CalibrationError> {
    /*
     * We need delta calculated from the new points
     */
    let delta = new_cp.delta() as f32;

    /*
     * 3 point triangulation
     */
    let alpha_x = ((old_cp.a[0] - old_cp.c[0]) * (new_cp.b[1] - new_cp.c[1])
        - (old_cp.b[0] - old_cp.c[0]) * (new_cp.a[1] - new_cp.c[1])) as f32
        / delta;
    /*
     * We might end up reading garabge from SPI
     * so we must test for the weird floats
     */
    if !alpha_x.is_normal() {
        return Err(CalibrationError::Alpha);
    }
    let beta_x = ((new_cp.a[0] - new_cp.c[0]) * (old_cp.b[0] - old_cp.c[0])
        - (new_cp.b[0] - new_cp.c[0]) * (old_cp.a[0] - old_cp.c[0])) as f32
        / delta;
    if !beta_x.is_normal() {
        return Err(CalibrationError::Beta);
    }
    let delta_x = ((old_cp.a[0]) * (new_cp.b[0] * new_cp.c[1] - new_cp.c[0] * new_cp.b[1])
        - (old_cp.b[0]) * (new_cp.a[0] * new_cp.c[1] - new_cp.c[0] * new_cp.a[1])
        + (old_cp.c[0]) * (new_cp.a[0] * new_cp.b[1] - new_cp.b[0] * new_cp.a[1]))
        as f32
        / delta;
    if !delta_x.is_normal() {
        return Err(CalibrationError::Delta);
    }
    let alpha_y = ((old_cp.a[1] - old_cp.c[1]) * (new_cp.b[1] - new_cp.c[1])
        - (old_cp.b[1] - old_cp.c[1]) * (new_cp.a[1] - new_cp.c[1])) as f32
        / delta;
    if !alpha_y.is_normal() {
        return Err(CalibrationError::Alpha);
    }
    let beta_y = ((new_cp.a[0] - new_cp.c[0]) * (old_cp.b[1] - old_cp.c[1])
        - (new_cp.b[0] - new_cp.c[0]) * (old_cp.a[1] - old_cp.c[1])) as f32
        / delta;
    if !beta_y.is_normal() {
        return Err(CalibrationError::Beta);
    }
    let delta_y = ((old_cp.a[1]) * (new_cp.b[0] * new_cp.c[1] - new_cp.c[0] * new_cp.b[1])
        - (old_cp.b[1]) * (new_cp.a[0] * new_cp.c[1] - new_cp.c[0] * new_cp.a[1])
        + (old_cp.c[1]) * (new_cp.a[0] * new_cp.b[1] - new_cp.b[0] * new_cp.a[1]))
        as f32
        / delta;
    if !delta_y.is_normal() {
        return Err(CalibrationError::Delta);
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
