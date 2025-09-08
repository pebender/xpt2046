//! The Xpt2046 driver.

use core::fmt::Debug;
#[cfg(feature = "defmt")]
use defmt::Format;
pub use embedded_graphics::geometry::{Point, Size};
use embedded_hal::{digital::InputPin, spi::SpiDevice};

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
//
// However, delaying the Control Byte by 3 bits in the SPI transmit bytes
// changes the alignment of the measurement in the SPI receive bytes so that the
// measurement is aligned with the SPI receive bytes, eliminating the need to
// divide the measurement by 2^3 or lose 3 bits of headroom. So, that is what we
// do.
const READ_X_CONTROL_BYTE: u8 = 0b1_101_0_0_00;
const READ_Y_CONTROL_BYTE: u8 = 0b1_001_0_0_00;

const READ_X_SPI_TX_BUF: [u8; 2] = ((READ_X_CONTROL_BYTE as u16) << 5).to_be_bytes();
const READ_Y_SPI_TX_BUF: [u8; 2] = ((READ_Y_CONTROL_BYTE as u16) << 5).to_be_bytes();

const READ_POSITION_SPI_BUF_LEN: usize = 5;

const READ_POSITION_SPI_TX_BUF: [u8; READ_POSITION_SPI_BUF_LEN] = [
    READ_X_SPI_TX_BUF[0],
    READ_X_SPI_TX_BUF[1],
    READ_Y_SPI_TX_BUF[0],
    READ_Y_SPI_TX_BUF[1],
    0,
];

const MAX_SAMPLES: usize = 128;

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub enum Error<SpiError, IrqError> {
    /// SPI error
    Spi(SpiError),
    /// IRQ error
    Irq(IrqError),
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
pub struct Xpt2046<Spi> {
    /// THe SPI device interface
    spi: Spi,
    /// Current driver state
    screen_state: TouchScreenState,
    /// Buffer for the touch measurement samples
    ts: TouchSamples,
    /// Calibration data for transforming touch measurements into display pixel positions.
    calibration_data: CalibrationData,
}

impl<Spi> Xpt2046<Spi>
where
    Spi: SpiDevice<u8>,
{
    pub fn new(spi: Spi, calibration_data: &CalibrationData) -> Self {
        Self {
            spi,
            screen_state: TouchScreenState::IDLE,
            ts: TouchSamples::default(),
            calibration_data: *calibration_data,
        }
    }

    pub fn set_calibration_data(&mut self, calibration_data: &CalibrationData) {
        self.calibration_data = *calibration_data;
    }
}

impl<Spi, SpiError> Xpt2046<Spi>
where
    Spi: SpiDevice<u8, Error = SpiError>,
    SpiError: Debug,
{
    /// Read position.
    fn read_position(&mut self) -> Result<Point, SpiError> {
        let mut read_position_spi_rx_buf = [0; READ_POSITION_SPI_BUF_LEN];

        self.spi
            .transfer(&mut read_position_spi_rx_buf, &READ_POSITION_SPI_TX_BUF)?;

        let x: i32 =
            u16::from_be_bytes([read_position_spi_rx_buf[1], read_position_spi_rx_buf[2]]) as i32;
        let y: i32 =
            u16::from_be_bytes([read_position_spi_rx_buf[3], read_position_spi_rx_buf[4]]) as i32;

        Ok(Point::new(x, y))
    }

    pub fn get_touch_point_raw(&self) -> Point {
        self.ts.average()
    }

    /// Get the actual touch point
    pub fn get_touch_point(&self) -> Point {
        let raw_point = self.get_touch_point_raw();

        let x = raw_point.x as f32;
        let y = raw_point.y as f32;
        let x = self.calibration_data.alpha_x * x
            + self.calibration_data.beta_x * y
            + self.calibration_data.delta_x;
        let y = self.calibration_data.alpha_y * x
            + self.calibration_data.beta_y * y
            + self.calibration_data.delta_y;
        let x = x as i32;
        let y = y as i32;
        Point::new(x, y)
    }

    /// Check if the display is currently touched
    pub fn is_touched(&self) -> bool {
        self.screen_state == TouchScreenState::TOUCHED
    }

    /// Sometimes the TOUCHED state needs to be cleared
    pub fn clear_touch(&mut self) {
        self.ts.counter = 0;
        self.screen_state = TouchScreenState::IDLE;
    }

    /// Reset the driver and preload tx buffer with register data. The unused
    /// parameter _irq is included aso that IrqError type parameter will be
    /// accepted, allowing the init, run and calibrate functions to return the
    /// same error type.
    pub fn init<Irq, IrqError>(&mut self, _irq: &mut Irq) -> Result<(), Error<SpiError, IrqError>>
    where
        Irq: InputPin<Error = IrqError>,
        IrqError: Debug,
    {
        // Make a throwaway position measurement so that the internal voltage
        // reference is disabled and PENIRQ is enabled.
        _ = self.read_position().map_err(|e| Error::Spi(e))?;

        self.ts.counter = 0;
        self.screen_state = TouchScreenState::IDLE;

        Ok(())
    }

    /// Continually runs and and collects the touch data from XPT2046.
    /// You should drive this either in some main loop or dedicated timer
    /// interrupt.
    pub fn run<Irq, IrqError>(&mut self, irq: &mut Irq) -> Result<(), Error<SpiError, IrqError>>
    where
        Irq: InputPin<Error = IrqError>,
        IrqError: Debug,
    {
        match self.screen_state {
            TouchScreenState::IDLE => {
                if irq.is_low().map_err(|e| Error::Irq(e))? {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::PRESAMPLING;
                }
            }
            TouchScreenState::PRESAMPLING => {
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.screen_state = TouchScreenState::RELEASED
                }
                let point_sample = self.read_position().map_err(|e| Error::Spi(e))?;
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                if self.ts.counter == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let point_sample = self.read_position().map_err(|e| Error::Spi(e))?;
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                self.ts.counter %= MAX_SAMPLES;
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.screen_state = TouchScreenState::RELEASED
                }
            }
            TouchScreenState::RELEASED => {
                self.ts.counter = 0;
                self.screen_state = TouchScreenState::IDLE;
            }
        }
        Ok(())
    }
}
