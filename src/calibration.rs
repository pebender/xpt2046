use super::CalibrationData;
use super::Xpt2046;
use crate::error::{CalibrationError, Error};
use core::fmt::Debug;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::{Rgb565, RgbColor},
    primitives::{Line, Primitive, PrimitiveStyle},
    Drawable,
};
use embedded_hal::{delay::DelayNs, digital::InputPin, spi::SpiDevice};

#[cfg(feature = "defmt")]
use defmt::Format;

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
struct CalibrationPoints {
    pub a: Point,
    pub b: Point,
    pub c: Point,
}

/// This function uses the three-point calibration algorithm from "SLYT277:
/// Calibration in Touch-Screen Systems" by Texas Instruments
/// <https://www.ti.com/lit/an/slyt277/slyt277.pdf>.
pub(crate) fn calibrate<SPI, SPIError, IRQ, IRQError, DT, DELAY>(
    touch: &mut Xpt2046<SPI>,
    irq: &mut IRQ,
    dt: &mut DT,
    delay: &mut DELAY,
) -> Result<CalibrationData, Error<SPIError, IRQError>>
where
    SPI: SpiDevice<u8, Error = SPIError>,
    SPIError: Debug,
    IRQ: InputPin<Error = IRQError>,
    IRQError: Debug,
    DT: DrawTarget<Color = Rgb565>,
    DELAY: DelayNs,
{
    let lcd_cp = create_calibration_point(dt);

    let mut touch_cp = CalibrationPoints {
        a: Point::zero(),
        b: Point::zero(),
        c: Point::zero(),
    };

    // Measure touch point for calibration point a.
    let _ = dt.clear(Rgb565::BLACK);
    draw_calibration_point(dt, &lcd_cp.a);
    touch.clear_touch();
    while !touch.is_touched() {
        touch.run(irq)?;
        delay.delay_us(500);
    }
    touch_cp.a = touch.get_touch_point_raw();
    let _ = dt.clear(Rgb565::BLACK);
    while irq.is_low().map_err(|e| Error::Irq(e))? {
        delay.delay_ms(100);
    }

    delay.delay_ms(200);

    // Measure touch point for calibration point b.
    let _ = dt.clear(Rgb565::BLACK);
    draw_calibration_point(dt, &lcd_cp.b);
    touch.clear_touch();
    while !touch.is_touched() {
        touch.run(irq)?;
        delay.delay_us(500);
    }
    touch_cp.b = touch.get_touch_point_raw();
    let _ = dt.clear(Rgb565::BLACK);
    while irq.is_low().map_err(|e| Error::Irq(e))? {
        delay.delay_ms(100);
    }

    delay.delay_ms(200);

    // Measure touch point for calibration point c.
    let _ = dt.clear(Rgb565::BLACK);
    draw_calibration_point(dt, &lcd_cp.c);
    touch.clear_touch();
    while !touch.is_touched() {
        touch.run(irq)?;
        delay.delay_us(500);
    }
    touch_cp.c = touch.get_touch_point_raw();
    let _ = dt.clear(Rgb565::BLACK);
    while irq.is_low().map_err(|e| Error::Irq(e))? {
        delay.delay_ms(100);
    }
    delay.delay_ms(200);

    let calibration_data = calculate_calibration(&lcd_cp, &touch_cp);

    #[cfg(feature = "defmt")]
    {
        defmt::debug!("displayed (LCD) calibration points: {:?}", lcd_cp);
        defmt::debug!("measured (touch) calibration points: {:?}", touch_cp);
        defmt::debug!("calculated calibration data: {:?}", calibration_data);
    }

    calibration_data.map_err(|e| Error::Calibration(e))
}

// This function creates three independent calibration points are created based
// on the position and size of the draw target.
//
// +-------+-------+-------+-------+
// |       |       |       |       |
// +-------a-------+-------+-------+
// |       |       |       |       |
// +-------+-------+-------c-------+
// |       |       |       |       |
// +-------+-------b-------+-------+
// |       |       |       |       |
// +-------+-------+-------+-------+
//
fn create_calibration_point<DT: DrawTarget<Color = Rgb565>>(dt: &DT) -> CalibrationPoints {
    let lcd_size = dt.bounding_box();
    let x = lcd_size.size.width as i32 / 4;
    let y = lcd_size.size.height as i32 / 4;
    CalibrationPoints {
        a: lcd_size.top_left + Point::new(1 * x, 1 * y),
        b: lcd_size.top_left + Point::new(2 * x, 3 * y),
        c: lcd_size.top_left + Point::new(3 * x, 2 * y),
    }
}

fn draw_calibration_point<DT: DrawTarget<Color = Rgb565>>(dt: &mut DT, p: &Point) {
    let _ = Line::new(Point::new(p.x - 4, p.y), Point::new(p.x + 4, p.y))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
    let _ = Line::new(Point::new(p.x, p.y - 4), Point::new(p.x, p.y + 4))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
}

fn calculate_calibration(
    lcd_cp: &CalibrationPoints,
    touch_cp: &CalibrationPoints,
) -> Result<CalibrationData, CalibrationError> {
    let det = (touch_cp.a.x - touch_cp.c.x) * (touch_cp.b.y - touch_cp.c.y)
        - (touch_cp.b.x - touch_cp.c.x) * (touch_cp.a.y - touch_cp.c.y);

    if det == 0 {
        return Err(CalibrationError::All);
    }

    let det_alpha_x = (lcd_cp.a.x - lcd_cp.c.x) * (touch_cp.b.y - touch_cp.c.y)
        - (lcd_cp.b.x - lcd_cp.c.x) * (touch_cp.a.y - touch_cp.c.y);
    let det_beta_x = (touch_cp.a.x - touch_cp.c.x) * (lcd_cp.b.x - lcd_cp.c.x)
        - (touch_cp.b.x - touch_cp.c.x) * (lcd_cp.a.x - lcd_cp.c.x);
    let det_delta_x = (lcd_cp.a.x) * (touch_cp.b.x * touch_cp.c.y - touch_cp.c.x * touch_cp.b.y)
        - (lcd_cp.b.x) * (touch_cp.a.x * touch_cp.c.y - touch_cp.c.x * touch_cp.a.y)
        + (lcd_cp.c.x) * (touch_cp.a.x * touch_cp.b.y - touch_cp.b.x * touch_cp.a.y);
    let det_alpha_y = (lcd_cp.a.y - lcd_cp.c.y) * (touch_cp.b.y - touch_cp.c.y)
        - (lcd_cp.b.y - lcd_cp.c.y) * (touch_cp.a.y - touch_cp.c.y);
    let det_beta_y = (touch_cp.a.x - touch_cp.c.x) * (lcd_cp.b.y - lcd_cp.c.y)
        - (touch_cp.b.x - touch_cp.c.x) * (lcd_cp.a.y - lcd_cp.c.y);
    let det_delta_y = (lcd_cp.a.y) * (touch_cp.b.x * touch_cp.c.y - touch_cp.c.x * touch_cp.b.y)
        - (lcd_cp.b.y) * (touch_cp.a.x * touch_cp.c.y - touch_cp.c.x * touch_cp.a.y)
        + (lcd_cp.c.y) * (touch_cp.a.x * touch_cp.b.y - touch_cp.b.x * touch_cp.a.y);

    let det = det as f32;
    let det_alpha_x = det_alpha_x as f32;
    let det_beta_x = det_beta_x as f32;
    let det_delta_x = det_delta_x as f32;
    let det_alpha_y = det_alpha_y as f32;
    let det_beta_y = det_beta_y as f32;
    let det_delta_y = det_delta_y as f32;

    let alpha_x = det_alpha_x / det;
    if !alpha_x.is_normal() {
        return Err(CalibrationError::Alpha);
    }
    let beta_x = det_beta_x / det;
    if !beta_x.is_normal() {
        return Err(CalibrationError::Beta);
    }
    let delta_x = det_delta_x / det;
    if !delta_x.is_normal() {
        return Err(CalibrationError::Delta);
    }
    let alpha_y = det_alpha_y / det;
    if !alpha_y.is_normal() {
        return Err(CalibrationError::Alpha);
    }
    let beta_y = det_beta_y / det;
    if !beta_y.is_normal() {
        return Err(CalibrationError::Beta);
    }
    let delta_y = det_delta_y / det;
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
