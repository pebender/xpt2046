//! The Xpt2046 driver.
//!
//! The driver is for the XPT2046 Resistive Touch Screen Controller connected
//! over an SPI interface.
//!
//! The primary purpose of the driver is to read and filter the X and Y touch
//! position measurements so that the touch panel connected to the XPT2046 can
//! be used as pointing device for the display panel. However, the driver
//! provides for measuring the eight values that can be measured: X-Position,
//! Y-Position, Z1-Position, Z2-Position, TEMP0, TEMP1, VBAT and AUXIN. In
//! addition, the driver provides for measure in pairs values that are often
//! paired in calculations: (X-Position, Y-Position), (Z1-Position, Z2-Position)
//! and (TEMP0, TEMP1).
//!
//! Information on the operation of the XPT2046 Touch Screen Controller can be
//! found in the XPT2046 data sheet
//! (<https://www.snapeda.com/parts/XPT2046/Xptek/datasheet/>).

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

// For information on the operation of the XPT2046 Touch Screen Controller,
// refer to the XPT2046 data sheet
// (<https://www.snapeda.com/parts/XPT2046/Xptek/datasheet/>).
//
// Basically, the XPT2046 is an ADC (analog to digital converter). What the
// XPT2046 measures and how the XPT2046 measures what it measures is controlled
// by the XPT2046 Control Byte.
//
// The format of the XPT2046 Control Byte is shown in Table 6. Per Table 6,
// Table 7 and Table 8. It consists of one start bit (S), three channel select
// bits (A2-A0), one 12-bit/8-bit ADC conversion select bit (MODE), one
// single-ended/differential select bit (SER/DER), one internal/external voltage
// reference select bit (PD1) and one PENIRQ enable/disable bit (PD0). In the
// Control Byte, S, A2-A1, MODE and SER/DER apply to the current measurement
// wheres PD1 and PD0 apply after the current measurement is complete.
//
// The S bit is set to '1' to signal the start of the Control Byte over the
// SPI's output serial line (MOSI). The A2-A0 bits tell the XPT2046 what to
// measure, where
//
// - 0b000 tells it to measure temperature 0 (TEMP0),
// - 0b001 tells it to measure the Y position,
// - 0b010 tells it to measure the battery voltage (VBAT),
// - 0b011 tells it to measure the Z1 position,
// - 0b100 tells it to measure the Z2 position,
// - 0b101 tells it to measure the X position,
// - 0b110 tells it to measure the auxiliary input (AUXIN), and
// - 0b111 tells it to measure temperature 1 (TEMP1).
//
// The MODE bit tells it whether to produce a 12-bit ADC output or a 8-bit ADC
// output, where
//
// - 0b0 tells it to produce a 12-bit ADC output, and
// - 0b1 tells it to produce an 8-bit ADC output.
//
// The SER/DER tells it whether to make a single-ended or differential
// measurement, where
//
// - 0b0 tells it to make a differential measurement, and
// - 0b1 tells it to make a single-ended measurement.
//
// The PD1 bit tells it whether to use the internal or external voltage
// reference in the future, where
//
// - 0b0 tells it to use the external voltage reference in the future, and
// - 0b1 tells it to use the internal voltage reference in the future.
//
// The PD0 bit tells it whether to enable or disable the PENIRQ in the future,
// where
//
// - 0b0 tells it to enable the PENIRQ in the future, and
// - 0b1 tells it to disable the PENIRQ in the future.
//
// All measurements can use 8-bit ADC output or 12-bit ADC output. According to
// the data sheet, not all measurements necessarily benefit from the additional
// precision.
//
// The position measurements (X, Y, Z1 and Z2) can use single-ended or
// differential. According to the data sheet, differential is more accurate. So,
// the driver makes uses differential for positions. The non-position
// measurements (TEMP0, TEMP1, VBAT and AUXIN) can use single-ended but not
// differential. So, the driver uses single-ended for non-position measurements.
//
// The non-position measurements (TEMP0, TEMP1, VBAT and AUXIN) use the internal
// voltage reference because their measurements are single-ended and the
// internal voltage reference is a known value. The position measurements do not
// use the internal voltage reference in part because their measurements are
// differential. Finally, having the internal voltage reference enabled,
// consumes power. So, the driver enables the internal voltage reference before
// making non-position measurement and disables the internal voltage reference
// after completing the non-position measurement.
//
// The driver enables PENIRQ.
//
// The values of READ_X_CONTROL_BYTE and READ_Y_CONTROL_BYTE have been
// selected
// - to enable the XPT2046 the most accurate measurements of the X and Y
//   positions,
// - to enable repeated, uninterrupted sequential measurements of the X and Y
//   positions, and
// - to enable the PENIRQ signal.
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

/// Convenience enums and functions for working with the XPT2046 Control Byte.
///
/// The XPT2046 Control Byte consists of one start bit (S), three channel select
/// bits (A2-A0), one 12-bit/8-bit ADC conversion select bit (MODE), one
/// single-ended/differential select bit (SER/DER), one internal/external
/// voltage reference select bit (PD1) and one PENIRQ enable/disable bit (PD0).
/// In a Control Byte, A2-A0, MODE and SER/DER apply to the current measurement
/// wheres PD1 and PD0 apply after the current measurement is complete.
pub mod control_byte {
    use core::fmt::Debug;
    #[cfg(feature = "defmt")]
    use defmt::Format;

    /// Selects the channel to be measured.
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug)]
    pub enum ChannelSelect {
        XPosition = 0b101,
        YPosition = 0b001,
        Z1Position = 0b011,
        Z2Position = 0b100,
        TEMP0 = 0b000,
        TEMP1 = 0b111,
        VBAT = 0b010,
        AUXIN = 0b110,
    }

    /// Selects the precision of the ADC measurement.
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug)]
    pub enum ADCModeSelect {
        Bits12 = 0b0,
        Bits8 = 0b1,
    }

    /// Selects whether the measurement uses a singled-ended or duel-ended
    /// (differential) reference.
    ///
    /// Measurements made using a duel-ended reference are more accurate than
    /// measurements using a single-ended reference. The measurement of Y, Y, Z1 or
    /// Z2 can use either singled-ended or duel-ended reference. The measurement of
    /// TEMP0, TEMP1, VBAT and AUXIN can only use a single ended reference.
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug)]
    pub enum SerDerSelect {
        Der = 0b0,
        Ser = 0b1,
    }

    /// Selects whether to enable the internal voltage reference after the current
    /// measurement.
    ///
    /// Measurements using a duel-ended (differential) reference do not use the
    /// internal voltage reference. Measurements using a single-ended reference are
    /// more accurate using the internal voltage reference than using an external
    /// voltage reference. Measurements of X, Y, Z1 or Z2 can use either the
    /// internal voltage reference or an external voltage reference. Measurements of
    /// TEMP0, TEMP1, VBAT and AUXIN can only use the internal voltage reference.
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug)]
    pub enum InternalReferenceEnable {
        Disable = 0b0,
        Enable = 0b1,
    }

    // Selects whether to enable PENIRQ.
    #[cfg_attr(feature = "defmt", derive(Format))]
    #[derive(Debug)]
    pub enum PenIrqEnable {
        Enable = 0b0,
        Disable = 0b1,
    }

    /// Builds a X2046 Control Byte.
    pub const fn build_control_byte(
        channel: ChannelSelect,
        adc_mode: ADCModeSelect,
        ser_der: SerDerSelect,
        internal_reference: InternalReferenceEnable,
        penirq: PenIrqEnable,
    ) -> u8 {
        (0b1 << 7)
            | ((channel as u8) << 4)
            | ((adc_mode as u8) << 3)
            | ((ser_der as u8) << 2)
            | ((internal_reference as u8) << 1)
            | ((penirq as u8) << 0)
    }

    /// Builds a X2046 Control Byte delayed by three bits.
    ///
    /// The control byte is delayed by three bits (three SPI bus clock cycles)
    /// so 12-bit ADC measurements are byte boundary aligned. Without the three
    /// bit delay, each returned 12-bit ADC measurement would need to be shifted
    /// by three bits to be correctly byte boundary aligned.
    pub const fn build_delayed_control_byte(
        channel: ChannelSelect,
        adc_mode: ADCModeSelect,
        ser_der: SerDerSelect,
        internal_reference: InternalReferenceEnable,
        penirq: PenIrqEnable,
    ) -> [u8; 2] {
        ((build_control_byte(channel, adc_mode, ser_der, internal_reference, penirq) as u16) << 5)
            .to_be_bytes()
    }

    impl ChannelSelect {
        /// Converts the [`ChannelSelect`] value into its preferred delayed
        /// control byte.
        ///
        /// 12-bit ADC precision is chosen for the X-Position and Y-Position
        /// measurements because the extra precision enables more precise
        /// mapping of these measurements to display pixels. 12-bit ADC
        /// precision is chosen for the other measurements because it simplifies
        /// the software.
        ///
        /// A duel-ended (differential) reference is chosen for position
        /// (X-Position, Y-Position, Z1-Position and Z2-Position) measurements
        /// because a duel-ended reference measurement is more accurate than
        /// single-ended reference measurement. A single-ended reference is
        /// chosen for the non-position (TEMP0, TEMP1, VBAT, AUXIN) measurements
        /// because they do not support duel-ended reference measurements.
        ///
        /// The PENIRQ is enabled because it allows the system to be responsive
        /// to touch input without requiring the frequent position measurements
        /// (including Z1-Position and Z2-Position) that would otherwise be
        /// needed to promptly detect touch input.
        ///
        /// For measurements using a single-ended reference, measurements using
        /// the internal voltage reference are likely to be more accurate than
        /// measurements using an external voltage reference because the value
        /// of the internal voltage reference is known and stable. However, the
        /// internal voltage reference consumes power. Therefore,
        ///
        ///
        ///
        /// The XPT2046 can measure eight different values. In practice, the
        /// position measurements (X-Position, Y-Position, Z1-Position and
        /// Z2-Position) tend to be far more common than the non-position
        /// measurements (TEMP0, TEMP1, VBAT and AUXIN). Of the position
        /// measurements, the X-Position and Y-Position measurements tend to be
        /// far more common than the Z1-Position adn Z2-Position measurements.
        /// Therefore, the driver optimizes for the X-Position and Y-Position
        /// measurements.
        ///
        /// The X-Position and Y-Position benefit from the extra precision of
        /// 12-bit ADC measurement because the extra precision enables more
        /// precise mapping of these measurements to display pixels. Independent
        /// of whether the an 12-bit or 8-bit ADC measurement is made, a single
        /// measurement transaction will occupy the SPI bus for three bytes, and
        /// a multiple measurement transaction will occupy the SPI bus for two
        /// additional two bytes for each additional measurement. It might be
        /// possible to use a higher SPI clock frequency when communicating with
        /// XPT2046 were we to make only 8-bit ADC measurements.
        ///
        ///  not all measurements benefit from the extra precision of a
        /// 12-bit ADC conversion, the most common measurements (X-Position and
        /// Y-Position)
        pub const fn into_delayed_control_byte(self) -> [u8; 2] {
            let x = match self {
                ChannelSelect::XPosition
                | ChannelSelect::YPosition
                | ChannelSelect::Z1Position
                | ChannelSelect::Z2Position => build_delayed_control_byte(
                    self,
                    ADCModeSelect::Bits12,
                    SerDerSelect::Der,
                    InternalReferenceEnable::Disable,
                    PenIrqEnable::Enable,
                ),
                ChannelSelect::TEMP0
                | ChannelSelect::TEMP1
                | ChannelSelect::VBAT
                | ChannelSelect::AUXIN => build_delayed_control_byte(
                    self,
                    ADCModeSelect::Bits12,
                    SerDerSelect::Ser,
                    InternalReferenceEnable::Enable,
                    PenIrqEnable::Enable,
                ),
            };
            x
        }
    }

    /// The delayed control byte used to enable the internal voltage reference.
    ///
    /// A channel must be selected
    pub const INTERNAL_REFERENCE_ENABLE: [u8; 2] = build_delayed_control_byte(
        ChannelSelect::TEMP0,
        ADCModeSelect::Bits8,
        SerDerSelect::Ser,
        InternalReferenceEnable::Enable,
        PenIrqEnable::Enable,
    );

    /// The delayed control byte used to disable the internal voltage reverence.
    pub const INTERNAL_REFERENCE_DISABLE: [u8; 2] = build_delayed_control_byte(
        ChannelSelect::TEMP0,
        ADCModeSelect::Bits8,
        SerDerSelect::Ser,
        InternalReferenceEnable::Disable,
        PenIrqEnable::Enable,
    );
}

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

/// The Xpt2046 driver.
///
/// The driver is for the XPT2046 Resistive Touch Screen Controller connected
/// over an SPI interface.
///
/// The primary purpose of the driver is to read and filter the X and Y touch
/// position measurements so that the touch panel connected to the XPT2046 can
/// be used as pointing device for the display panel. However, the driver
/// provides for measuring the eight values that can be measured: X-Position,
/// Y-Position, Z1-Position, Z2-Position, TEMP0, TEMP1, VBAT and AUXIN. In
/// addition, the driver provides for measuring together values that are used
/// together in calculations: (X-Position, Y-Position), (Z1-Position,
/// Z2-Position), (X-Position, Y-Position, Z1-Position, Z2-Position), and
/// (TEMP0, TEMP1).
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

impl<Spi, SpiError> Xpt2046<Spi>
where
    Spi: SpiDevice<u8, Error = SpiError>,
    SpiError: Debug,
{
    pub fn new(spi: Spi, calibration_data: &CalibrationData) -> Self {
        Self {
            spi,
            screen_state: TouchScreenState::IDLE,
            ts: TouchSamples::default(),
            calibration_data: *calibration_data,
        }
    }

    /// Reset the driver and preload tx buffer with register data.
    pub fn init(&mut self) -> Result<(), SpiError> {
        // Make a throwaway position measurement so that the internal voltage
        // reference is disabled and PENIRQ is enabled.
        _ = self.measure_xy_positions()?;

        self.ts.counter = 0;
        self.screen_state = TouchScreenState::IDLE;

        Ok(())
    }

    /// Sets the calibration data used by [`Self::get_touch_point()`]
    pub fn set_calibration_data(&mut self, calibration_data: &CalibrationData) {
        self.calibration_data = *calibration_data;
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
                let position = self.measure_xy_positions().map_err(|e| Error::Spi(e))?;
                self.ts.samples[self.ts.counter] = Point::new(position.0.into(), position.1.into());
                self.ts.counter += 1;
                if self.ts.counter == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let position = self.measure_xy_positions().map_err(|e| Error::Spi(e))?;
                self.ts.samples[self.ts.counter] = Point::new(position.0.into(), position.1.into());
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

    /// Returns the touch point in XPT2046 measurement units.
    ///
    /// This is made available for use in touch screen calibration procedures.
    pub fn get_touch_point_raw(&self) -> Point {
        self.ts.average()
    }

    /// Returns the touch point in display pixel units.
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

    /// Returns the measurement of X-Position.
    pub fn measure_x_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::XPosition.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(v0)
    }

    /// Returns the measurement of Y-Position.
    pub fn measure_y_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::YPosition.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(v0)
    }

    /// Returns the measurements of X-Position and Y-Position as the tuple
    /// (X,Y).
    ///
    /// This is by far the most common measurement made.
    pub fn measure_xy_positions(&mut self) -> Result<(u16, u16), SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::XPosition.into_delayed_control_byte();
        const M1: [u8; 2] = control_byte::ChannelSelect::YPosition.into_delayed_control_byte();
        const TX_BUF: [u8; 5] = [M0[0], M0[1], M1[0], M1[1], 0];
        let mut rx_buf = [0; 5];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        let v1 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok((v0, v1))
    }

    /// Returns the measurement of Z1-Position.
    pub fn measure_z1_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::Z1Position.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(v0)
    }

    /// Returns the measurement of Z2-Position.
    pub fn measure_z2_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::Z2Position.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let value = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(value)
    }

    /// Returns the measurements of Z1-Position and Z2-Position as the tuple
    /// (Z1,Z2).
    pub fn measure_z_positions(&mut self) -> Result<(u16, u16), SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::Z1Position.into_delayed_control_byte();
        const M1: [u8; 2] = control_byte::ChannelSelect::Z2Position.into_delayed_control_byte();
        const TX_BUF: [u8; 5] = [M0[0], M0[1], M1[0], M1[1], 0];
        let mut rx_buf = [0; 5];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        let v1 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok((v0, v1))
    }

    /// Returns the measurements of the X-Position, Y-Position, Z1-Position and
    /// Z2-Position as the tuple (X,Y,Z1,Z2).
    pub fn measure_xyz_positions(&mut self) -> Result<(u16, u16, u16, u16), SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::XPosition.into_delayed_control_byte();
        const M1: [u8; 2] = control_byte::ChannelSelect::YPosition.into_delayed_control_byte();
        const M2: [u8; 2] = control_byte::ChannelSelect::Z1Position.into_delayed_control_byte();
        const M3: [u8; 2] = control_byte::ChannelSelect::Z2Position.into_delayed_control_byte();
        const TX_BUF: [u8; 9] = [M0[0], M0[1], M1[0], M1[1], M2[0], M2[1], M3[0], M3[1], 0];
        let mut rx_buf = [0; 9];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        let v1 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        let v2 = u16::from_be_bytes([rx_buf[5], rx_buf[6]]);
        let v3 = u16::from_be_bytes([rx_buf[7], rx_buf[8]]);
        Ok((v0, v1, v2, v3))
    }

    /// Returns the measurement of TEMP0.
    pub fn measure_temp0(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::TEMP0.into_delayed_control_byte();
        const RD: [u8; 2] = control_byte::INTERNAL_REFERENCE_DISABLE;
        const TX_BUF: [u8; 7] = [RE[0], RE[1], M0[0], M0[1], RD[0], RD[1], 0];
        let mut rx_buf = [0; 7];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }

    /// Returns the measurement of TEMP1.
    pub fn measure_temp1(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::TEMP1.into_delayed_control_byte();
        const RD: [u8; 2] = control_byte::INTERNAL_REFERENCE_DISABLE;
        const TX_BUF: [u8; 7] = [RE[0], RE[1], M0[0], M0[1], RD[0], RD[1], 0];
        let mut rx_buf = [0; 7];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }

    /// Returns the measurements of TEMP0 and TEMP1 as the tuple
    /// (TEMP0,TEMP1).
    pub fn measure_temps(&mut self) -> Result<(u16, u16), SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::TEMP0.into_delayed_control_byte();
        const M1: [u8; 2] = control_byte::ChannelSelect::TEMP1.into_delayed_control_byte();
        const RD: [u8; 2] = control_byte::INTERNAL_REFERENCE_DISABLE;
        const TX_BUF: [u8; 9] = [RE[0], RE[1], M0[0], M0[1], M1[0], M1[0], RD[1], RD[1], 0];
        let mut rx_buf = [0; 9];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        let v1 = u16::from_be_bytes([rx_buf[5], rx_buf[6]]);
        Ok((v0, v1))
    }

    /// Returns the measurement of VBAT.
    pub fn measure_vbat(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::VBAT.into_delayed_control_byte();
        const RD: [u8; 2] = control_byte::INTERNAL_REFERENCE_DISABLE;
        const TX_BUF: [u8; 7] = [RE[0], RE[1], M0[0], M0[1], RD[0], RD[1], 0];
        let mut rx_buf: [u8; _] = [0; 7];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }

    /// Returns the measurement of AUXIN.
    pub fn measure_auxin(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::AUXIN.into_delayed_control_byte();
        const RD: [u8; 2] = control_byte::INTERNAL_REFERENCE_DISABLE;
        const TX_BUF: [u8; 7] = [RE[0], RE[1], M0[0], M0[1], RD[0], RD[1], 0];
        let mut rx_buf: [u8; _] = [0; 7];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }
}
