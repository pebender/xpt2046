#![doc(html_root_url = "https://docs.rs/xpt2046")]
#![doc(issue_tracker_base_url = "https://github.com/VersBinarii/xpt2046/issues/")]
#![deny(
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    unused_variables,
    unreachable_code,
    unused_comparisons,
    unused_must_use
)]
#![no_std]

//! A platform agnostic Rust driver for XPT2046 touch controller, based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!

use crate::calibration::{calculate_calibration, calibration_draw_point};
pub use crate::{calibration::CalibrationPoint, error::Error};
use core::{fmt::Debug, ops::RemAssign};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::{Rgb565, RgbColor},
};
use embedded_hal::{delay::DelayNs, digital::InputPin, spi::SpiDevice};

#[cfg(feature = "defmt")]
use defmt::Format;

pub mod calibration;
pub mod error;

// For information on the operation of the XPT2046, refer to the XPT2046 data
// sheet <https://www.snapeda.com/parts/XPT2046/Xptek/datasheet/>. The format of
// the XPT2046 Control Byte is shown in Table 6. Per Table 6 and Table 7, it
// consists of 1 start bit, 3 channel select bits, 1 mode bit, 1
// single-ended/differential select bit, and 2 power down mode select bits.
//
// The values of READ_X_CONTROL_BYTE and READ_Y_CONTROL_BYTE have been
// selected
// - to enable the XPT2046 the most accurate measurements of the X and Y
//   positions,
// - to enable repeated, uninterrupted sequential measurements of the X and Y
//   positions, and
// - to enable the PENIRQ signal.
//
// The start bit is set to '1' to signal the start of the byte over the SPI's
// output serial line (MOSI). Per Table 5, a channel select value of '001' tells
// the XPT2046 to measure the X-channel whereas a channel select value of '101'
// tells the XPT2046 to measure the Y-channel. A single-ended/differential
// select value of '0' tells the XPT2046 to make the more accurate differential
// measurement. A mode select value of '0' tells the XPT2046 ADC to produce the
// higher resolution 12-bit sample as send it over the SPI's input serial line
// (MISO). Per Table 8, a power down value of '00' tells the XPT2046 ADC power
// down but keep the PENIRQ active and be ready to make the next measurement
// after the current measurement is complete.
//
// Figure 14 shows that a new measurement can be made every 16 clock cycles on
// the byte oriented SPI interface. In addition, it shows that it takes an
// additional 5 clock cycles beyond the 16 clock cycles to receive the remaining
// 5 bits of final measurement. Therefore, an X,Y measurement can be made with 5
// bytes.
//
// The start of a measurement is tied to the start of the Control Byte that
// initiated the measurement. By shifting the position of the Control Byte in
// the SPI transmit bytes, the position of the measurement in SPI receive bytes
// can be changed. Because fo the timing between the start of the Control Byte
// and the end of the measurement, if the Control Byte is aligned with the SPI
// transmit bytes, then the measurement in the SPI receive bytes will be
// misaligned by 3 bits. Essentially, it will be as if the measurement was 15
// bits rather than 12 bits where the low 3 bits are always 0. So, we would
// either need to divide the measurement by 2^3 or lose 3 bits of headroom.

// However, delaying the Control Byte by 3 bits in the SPI transmit bytes
// changes the alignment of the measurement in the SPI receive bytes so that the
// measurement is aligned with the SPI receive bytes, eliminating the need to
// divide the  measurement by 2^3 or lose 3 bits of headroom. So, that is what
// we do.
const READ_X_CONTROL_BYTE: u8 = 0b1_001_0_0_00;
const READ_Y_CONTROL_BYTE: u8 = 0b1_101_0_0_00;

const READ_X_SPI_TX_BUF: [u8; 2] = ((READ_X_CONTROL_BYTE as u16) << 5).to_be_bytes();
const READ_Y_SPI_TX_BUF: [u8; 2] = ((READ_Y_CONTROL_BYTE as u16) << 5).to_be_bytes();

const READ_XY_SPI_BUF_LEN: usize = 5;

const READ_XY_SPI_TX_BUF: [u8; READ_XY_SPI_BUF_LEN] = [
    READ_X_SPI_TX_BUF[0],
    READ_X_SPI_TX_BUF[1],
    READ_Y_SPI_TX_BUF[0],
    READ_Y_SPI_TX_BUF[1],
    0,
];

const MAX_SAMPLES: usize = 128;

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub struct CalibrationData {
    pub alpha_x: f32,
    pub beta_x: f32,
    pub delta_x: f32,
    pub alpha_y: f32,
    pub beta_y: f32,
    pub delta_y: f32,
}

/// Orientation of the touch screen
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub enum Orientation {
    Portrait,
    PortraitFlipped,
    Landscape,
    LandscapeFlipped,
}

impl Orientation {
    /// Default location for the test touch point
    /// Those depend on whether the touch screen operates in
    /// Portrait or Landscape position
    pub fn calibration_point(&self) -> CalibrationPoint {
        match self {
            Orientation::Portrait | Orientation::PortraitFlipped => CalibrationPoint {
                a: Point::new(10, 10),
                b: Point::new(80, 210),
                c: Point::new(200, 170),
            },
            Orientation::Landscape | Orientation::LandscapeFlipped => CalibrationPoint {
                a: Point::new(20, 25),
                b: Point::new(160, 220),
                c: Point::new(300, 110),
            },
        }
    }

    /// Default calibration values used for calculating the touch points
    /// Those depend on whether the touch screen operates in
    /// Portrait or Landscape position
    pub fn calibration_data(&self) -> CalibrationData {
        match self {
            Orientation::Portrait => CalibrationData {
                alpha_x: -0.0009337,
                beta_x: -0.0636839,
                delta_x: 250.342,
                alpha_y: -0.0889775,
                beta_y: -0.00118110,
                delta_y: 356.538,
            },
            Orientation::PortraitFlipped => CalibrationData {
                alpha_x: 0.0006100,
                beta_x: 0.0647828,
                delta_x: -13.634,
                alpha_y: 0.0890609,
                beta_y: 0.0001381,
                delta_y: -35.73,
            },
            Orientation::Landscape => CalibrationData {
                alpha_x: -0.0885542,
                beta_x: 0.0016532,
                delta_x: 349.800,
                alpha_y: 0.0007309,
                beta_y: 0.06543699,
                delta_y: -15.290,
            },
            Orientation::LandscapeFlipped => CalibrationData {
                alpha_x: 0.0902216,
                beta_x: 0.0006510,
                delta_x: -38.657,
                alpha_y: -0.0010005,
                beta_y: -0.0667030,
                delta_y: 258.08,
            },
        }
    }
}

/// Current state of the driver
#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, PartialEq)]
pub enum TouchScreenState {
    /// Driver waits for touch
    IDLE,
    /// Driver debounces the touch
    PRESAMPLING,
    /// Confirmed touch
    TOUCHED,
    /// Touch released
    RELEASED,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug, PartialEq)]
pub enum TouchScreenOperationMode {
    /// Normal touch reading
    NORMAL,
    /// Manual calibration mode
    CALIBRATION,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub struct TouchSamples {
    /// All the touch samples
    samples: [Point; MAX_SAMPLES],
    /// current number of captured samples
    counter: usize,
}

impl Default for TouchSamples {
    fn default() -> Self {
        Self {
            counter: 0,
            samples: [Point::default(); MAX_SAMPLES],
        }
    }
}

impl TouchSamples {
    pub fn average(&self) -> Point {
        let mut x = 0;
        let mut y = 0;

        for point in self.samples {
            x += point.x;
            y += point.y;
        }
        x /= MAX_SAMPLES as i32;
        y /= MAX_SAMPLES as i32;
        Point::new(x, y)
    }
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub struct Xpt2046<SPI> {
    /// THe SPI device interface
    spi: SPI,
    /// Current driver state
    screen_state: TouchScreenState,
    /// Buffer for the touch data samples
    ts: TouchSamples,
    calibration_data: CalibrationData,
    operation_mode: TouchScreenOperationMode,
    /// Location of the touch points used for
    /// performing manual calibration
    calibration_point: CalibrationPoint,
}

impl<SPI> Xpt2046<SPI>
where
    SPI: SpiDevice<u8>,
{
    pub fn new(spi: SPI, orientation: Orientation) -> Self {
        Self {
            spi,
            screen_state: TouchScreenState::IDLE,
            ts: TouchSamples::default(),
            calibration_data: orientation.calibration_data(),
            operation_mode: TouchScreenOperationMode::NORMAL,
            calibration_point: orientation.calibration_point(),
        }
    }
}

impl<SPI, SPIError> Xpt2046<SPI>
where
    SPI: SpiDevice<u8, Error = SPIError>,
    SPIError: Debug,
{
    /// Read raw X,Y values.
    fn read_xy(&mut self) -> Result<Point, SPIError> {
        let mut read_xy_spi_rx_buf = [0; READ_XY_SPI_BUF_LEN];

        self.spi
            .transfer(&mut read_xy_spi_rx_buf, &READ_XY_SPI_TX_BUF)?;

        let x: i32 = u16::from_be_bytes([read_xy_spi_rx_buf[1], read_xy_spi_rx_buf[2]]) as i32;
        let y: i32 = u16::from_be_bytes([read_xy_spi_rx_buf[3], read_xy_spi_rx_buf[4]]) as i32;

        Ok(Point::new(x, y))
    }

    /// Read the calibrated point of touch from XPT2046
    fn read_touch_point(&mut self) -> Result<Point, SPIError> {
        let raw_point = self.read_xy()?;

        let (x, y) = match self.operation_mode {
            TouchScreenOperationMode::NORMAL => {
                let x = self.calibration_data.alpha_x * raw_point.x as f32
                    + self.calibration_data.beta_x * raw_point.y as f32
                    + self.calibration_data.delta_x;
                let y = self.calibration_data.alpha_y * raw_point.x as f32
                    + self.calibration_data.beta_y * raw_point.y as f32
                    + self.calibration_data.delta_y;
                (x as i32, y as i32)
            }
            TouchScreenOperationMode::CALIBRATION => {
                /*
                 * We're running calibration so just return raw
                 * point measurements without compensation
                 */
                (raw_point.x, raw_point.y)
            }
        };
        Ok(Point::new(x, y))
    }

    /// Get the actual touch point
    pub fn get_touch_point(&self) -> Point {
        self.ts.average()
    }

    /// Check if the display is currently touched
    pub fn is_touched(&self) -> bool {
        self.screen_state == TouchScreenState::TOUCHED
    }

    /// Sometimes the TOUCHED state needs to be cleared
    pub fn clear_touch(&mut self) {
        self.screen_state = TouchScreenState::PRESAMPLING;
    }

    /// Reset the driver and preload tx buffer with register data. The unused
    /// parameter _irq is included aso that IRQError type parameter will be
    /// accepted, allowing the init, run and calibrate functions to return the
    /// same error type.
    pub fn init<IRQ, IRQError>(&mut self, _irq: &mut IRQ) -> Result<(), Error<SPIError, IRQError>>
    where
        IRQ: InputPin<Error = IRQError>,
        IRQError: Debug,
    {
        // Make a throwaway X,Y measurement to make sure that the Power Down Select Mode
        // bits are in the correct state to enable PENIRQ.
        _ = self.read_xy().map_err(|e| Error::Spi(e))?;

        Ok(())
    }

    /// Continually runs and and collects the touch data from xpt2046.
    /// You should drive this either in some main loop or dedicated timer
    /// interrupt
    pub fn run<IRQ, IRQError>(&mut self, irq: &mut IRQ) -> Result<(), Error<SPIError, IRQError>>
    where
        IRQ: InputPin<Error = IRQError>,
        IRQError: Debug,
    {
        match self.screen_state {
            TouchScreenState::IDLE => {
                if self.operation_mode == TouchScreenOperationMode::CALIBRATION
                    && irq.is_low().map_err(|e| Error::Irq(e))?
                {
                    self.screen_state = TouchScreenState::PRESAMPLING;
                }
            }
            TouchScreenState::PRESAMPLING => {
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.screen_state = TouchScreenState::RELEASED
                }
                let point_sample = self.read_touch_point().map_err(|e| Error::Spi(e))?;
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                if self.ts.counter + 1 == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let point_sample = self.read_touch_point().map_err(|e| Error::Spi(e))?;
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                /*
                 * Wrap around the counter if the screen
                 * is touched for longer time
                 */
                self.ts.counter.rem_assign(MAX_SAMPLES - 1);
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.screen_state = TouchScreenState::RELEASED
                }
            }
            TouchScreenState::RELEASED => {
                self.screen_state = TouchScreenState::IDLE;
                self.ts.counter = 0;
            }
        }
        Ok(())
    }

    /// Collects the reading for 3 sample points and
    /// calculates a set of calibration data. The default calibration data seem
    /// to work ok but if for some reason touch screen needs to be recalibrated
    /// then look no further.
    /// This should be run after init() method.
    pub fn calibrate<IRQ, IRQError, DT, DELAY>(
        &mut self,
        irq: &mut IRQ,
        dt: &mut DT,
        delay: &mut DELAY,
    ) -> Result<(), Error<SPIError, IRQError>>
    where
        IRQ: InputPin<Error = IRQError>,
        IRQError: Debug,
        DT: DrawTarget<Color = Rgb565>,
        DELAY: DelayNs,
    {
        let mut calibration_count = 0;
        let mut retry = 3;
        let mut new_a = Point::zero();
        let mut new_b = Point::zero();
        let mut new_c = Point::zero();
        let old_cp = self.calibration_point.clone();
        // Prepare the screen for points
        let _ = dt.clear(Rgb565::BLACK);

        // Set correct state to fetch raw data from touch controller
        self.operation_mode = TouchScreenOperationMode::CALIBRATION;
        while calibration_count < 4 {
            // We must run our state machine to capture user input
            self.run(irq)?;
            match calibration_count {
                0 => {
                    calibration_draw_point(dt, &old_cp.a);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_a = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                1 => {
                    calibration_draw_point(dt, &old_cp.b);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_b = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }
                2 => {
                    calibration_draw_point(dt, &old_cp.c);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_c = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                3 => {
                    // Create new calibration point from the captured samples
                    self.calibration_point = CalibrationPoint {
                        a: new_a,
                        b: new_b,
                        c: new_c,
                    };
                    // and then re-caculate calibration
                    match calculate_calibration(&old_cp, &self.calibration_point) {
                        Ok(new_calibration_data) => {
                            self.calibration_data = new_calibration_data;
                            calibration_count += 1;
                        }
                        Err(e) => {
                            // We have problem calculating new values
                            if retry == 0 {
                                return Err(Error::Calibration(e));
                            }
                            /*
                             * If out calculation failed lets retry
                             */
                            retry -= 1;
                            calibration_count = 0;

                            let _ = dt.clear(Rgb565::BLACK);
                        }
                    }
                }
                _ => {}
            }
        }

        let _ = dt.clear(Rgb565::WHITE);
        self.operation_mode = TouchScreenOperationMode::NORMAL;

        Ok(())
    }
}
