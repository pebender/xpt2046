//! The Xpt2046 touch panel driver.
//!
//! This driver is for the Shenzhen Xptek Technology's XPT2046 resistive touch
//! panel controller connected using SPI.
//!
//! This driver expects the XPT2046 is part of a touch screen assembly. The
//! assembly includes a display panel (such as a 240x320 TFT LCD display panel)
//! overlaid with a resistive touch panel. The display panel is controlled by a
//! display panel controller (such as an ILI Technology's ILI9341). The
//! resistive touch panel is controlled by the XPT2046.
//!
//! Therefore, the primary function of this driver is to enable the touch panel
//! to be used as a pointing device for the display panel. To accomplish this,
//! this driver collects (x,y) resistive touch screen position measurements from
//! the XPT2046, filters them to improve their quality, and transforms them into
//! pixel locations that map to the corresponding location on the display panel.
//!
//! However, the XPT2046 is essentially an a ADC (analog to digital converter)
//! that can be controlled to measure one of eight voltage sources using one of
//! three references and outputting one of two different precisions. So, this
//! driver provides access to measurements of each of the eight voltage sources
//! using their recommended voltage reference and the higher precision output.
//!
//! Information on the operation of the XPT2046 can be found in the XPT2046 data
//! sheet (<https://www.snapeda.com/parts/XPT2046/Xptek/datasheet/>).

use core::fmt::Debug;
use embedded_hal::{digital::InputPin, spi::SpiDevice};

/// Re-exported from
/// [embedded_graphics](https://docs.rs/embedded-graphics/latest/embedded_graphics/index.html)
/// for convenience.
pub use embedded_graphics::geometry::Point;

/// Convenience enums and functions for working with the XPT2046 control byte.
///
/// The control byte consists of one start bit (S), three channel select bits
/// (A2-A0), one 12-bit/8-bit ADC conversion select bit (MODE), one
/// single-ended/duel-ended reference select bit (SER/DER), one
/// internal/external voltage reference select bit (PD1) and one PENIRQ
/// enable/disable bit (PD0). In a control byte, A2-A0, MODE and SER/DER apply
/// to the current measurement wheres PD1 and PD0 apply after the current
/// measurement is complete.
///
/// The XPT2046 communicates using a clocked serial interface such as SPI. The
/// start bit signals the start of the control byte on the serial interface. The
/// timing of everything associated with that control byte is relative to the
/// start bit, including the timing of the measurement sent in response to the
/// control byte.
mod control_byte {
    use core::fmt::Debug;

    /// Selects the voltage source (channel) to be measured.
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum ChannelSelect {
        /// the X-Position measurement.
        XPosition = 0b101,
        /// Make the Y-Position measurement.
        YPosition = 0b001,
        /// Make the Z1-Position measurement.
        Z1Position = 0b011,
        /// Make the Z1-Position measurement.
        Z2Position = 0b100,
        /// Make the TEMP0 measurement.
        TEMP0 = 0b000,
        /// Make the TEMP1 measurement.
        TEMP1 = 0b111,
        /// Make the VBAT measurement.
        VBAT = 0b010,
        /// Make the AUXIN measurement.
        AUXIN = 0b110,
    }

    /// Selects the ADC precision for the measurement.
    #[allow(dead_code)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum ADCModeSelect {
        /// Make the measurement with 12-bits of ADC precision.
        Bits12 = 0b0,
        /// Make the measurement with 8-bits of ADC precision.
        Bits8 = 0b1,
    }

    /// Selects whether the measurement uses a singled-ended or duel-ended
    /// (differential) reference.
    ///
    /// Measurements made using a duel-ended reference are more accurate than
    /// measurements using a single-ended reference. The X-Position, Y-Position,
    /// Z1-Position or Z2-Position measurements can use either a singled-ended
    /// or a duel-ended reference. The TEMP0, TEMP1, VBAT and AUXIN measurements
    /// can only use a single-ended reference.
    #[allow(dead_code)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum SerDerSelect {
        Der = 0b0,
        Ser = 0b1,
    }

    /// Selects whether to enable the internal 2.5V reference after the current
    /// measurement.
    ///
    /// Measurements using a duel-ended (differential) reference do not use the
    /// internal voltage reference. Measurements using a single-ended reference
    /// are more accurate using the internal voltage reference than using an
    /// external voltage reference. The X-Position, Y-Position, Z1-Position or
    /// Z2-Position can use either the internal voltage reference or an external
    /// voltage reference. The TEMP0, TEMP1, VBAT and AUXIN can only use the
    /// internal voltage reference.
    #[allow(dead_code)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum InternalReferenceEnable {
        Disable = 0b0,
        Enable = 0b1,
    }

    // Selects whether to enable PENIRQ.
    #[allow(dead_code)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Debug)]
    pub enum PenIrqEnable {
        Enable = 0b0,
        Disable = 0b1,
    }

    /// Builds an XPT2046 control byte.
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

    /// Builds an XPT2046 control byte delayed by three bits.
    ///
    /// The start of the measurement sent on the SPI's MISO serial line is tied
    /// to the start of the control byte received on the SPI's MOSI serial line.
    /// Specifically, the measurement starts 9 clock cycles after the start of
    /// the control byte that initiated the measurement. For the 12-bit
    /// measurement, the LSB of the measurement will be sent 21 clock cycles
    /// after that start bit of the control byte. Therefore, if the start of the
    /// control byte is byte aligned, the measurement will not be byte aligned.
    /// For the 12-bit measurement to be byte aligned, either the transmitted
    /// control byte must be delayed by three bits (three clock cycles) or the
    /// received measurement must be right-shifted by three bits. Since the
    /// different control bytes used by the driver can be built and shifted at
    /// compile time whereas each measurement would need to be shifted at
    /// runtime, the control byte is shifted.
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
        /// A duel-ended (differential) reference is chosen for the X-Position,
        /// Y-Position, Z1-Position and Z2-Position measurements because
        /// measurements made using a duel-ended reference are more accurate
        /// than measurements using a single-ended reference. A single-ended
        /// reference is chosen for the TEMP0, TEMP1, VBAT and AUXIN
        /// measurements because a duel-ended reference is not supported for
        /// these measurements.
        ///
        /// The internal 2.5V voltage reference is disabled because it consumes
        /// power when enabled and is rarely needed. It is only needed when
        /// making a measurement using a single-ended reference. So, it is
        /// only needed when requesting a TEMP0, TEMP1, VBAT or AUXIN
        /// measurement. Therefore, instead of leaving it enabled, the driver
        /// sends a [`INTERNAL_REFERENCE_ENABLE`] control byte before sending
        /// the control byte requesting a measurement that needs the internal
        /// 2.5V voltage reference enabled.
        ///
        /// The PENIRQ is enabled because it allows the system to be responsive
        /// to touch input without requiring the frequent requests of
        /// X-Position, Y-Position (as well as Z1-Position and Z2-Position)
        /// measurements that would otherwise be needed to promptly detect touch
        /// input.
        pub const fn into_delayed_control_byte(self) -> [u8; 2] {
            match self {
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
                    InternalReferenceEnable::Disable,
                    PenIrqEnable::Enable,
                ),
            }
        }
    }

    /// The delayed control byte used to enable the internal 2.5V voltage
    /// reference.
    ///
    /// Normally, the driver disables the internal 2.5V voltage reference
    /// because it consumes power and is rarely used. When it is needed, the
    /// driver sends INTERNAL_REFERENCE_ENABLE to enable it.
    ///
    /// Since every control byte must trigger some measurement,
    /// INTERNAL_REFERENCE_ENABLE triggers a throwaway 8-bit, single-ended
    /// measurement of TEMP0.
    pub const INTERNAL_REFERENCE_ENABLE: [u8; 2] = build_delayed_control_byte(
        ChannelSelect::TEMP0,
        ADCModeSelect::Bits8,
        SerDerSelect::Ser,
        InternalReferenceEnable::Enable,
        PenIrqEnable::Enable,
    );
}

const TOUCH_SAMPLE_BUFFER_SIZE: usize = 64;

/// Error type returned by [`Xpt2046::run()`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum Error<SpiError, IrqError>
where
    SpiError: embedded_hal::spi::Error,
    IrqError: embedded_hal::digital::Error,
{
    /// SPI error
    Spi(SpiError),
    /// IRQ error
    Irq(IrqError),
}

/// The touch position calibration data.
///
/// The driver transforms a measured touch panel position into its corresponding
/// display panel position using the equations
///
/// ```rust
/// display.x = alpha_x * touch.x + beta_x * touch.y + delta_x
/// display.y = alpha_y * touch.y + beta_y * touch.y + delta_y
/// ```
///
/// [`crate::calibration::estimate_calibration_data()`] and
/// [`crate::calibration::calculate_calibration_data()`] to create the
/// calibration data. [`crate::calibration_run::run_calibration()`] can be used
/// to run a calibration routine for a touch screen and create the calibration
/// data.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
enum TouchPanelState {
    /// Waiting for a touch (waiting for PENIRQ to go low).
    IDLE,
    /// Letting the touch panel settle.
    SETTLING,
    /// Buffering touch samples for the touch sample filter.
    BUFFERING,
    /// A touch has been reliably detected and filtered.
    TOUCHED,
    /// Touch released
    RELEASED,
}

/// A circular buffer for storing and filtering touch samples.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
struct TouchSampleBuffer {
    /// A buffer of the last TOUCH_SAMPLE_BUFFER_SIZE touch samples.
    buffer: [Point; TOUCH_SAMPLE_BUFFER_SIZE],
    /// The index in the buffer where the next touch sample will be stored.
    index: usize,
}

impl Default for TouchSampleBuffer {
    fn default() -> Self {
        Self {
            buffer: [Point::zero(); TOUCH_SAMPLE_BUFFER_SIZE],
            index: 0,
        }
    }
}

impl TouchSampleBuffer {
    pub fn average(&self) -> Point {
        let mut x = 0;
        let mut y = 0;

        for point in self.buffer {
            x += point.x;
            y += point.y;
        }
        x /= TOUCH_SAMPLE_BUFFER_SIZE as i32;
        y /= TOUCH_SAMPLE_BUFFER_SIZE as i32;
        Point::new(x, y)
    }
}

/// The Xpt2046 driver.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct Xpt2046<Spi> {
    /// THe SPI device interface
    spi: Spi,
    /// Current driver state
    panel_state: TouchPanelState,
    /// Buffer for the touch measurement samples
    sample_buffer: TouchSampleBuffer,
    /// Calibration data for transforming touch measurements into display pixel positions.
    calibration_data: CalibrationData,
}

impl<Spi, SpiError> Xpt2046<Spi>
where
    Spi: SpiDevice<u8, Error = SpiError>,
    SpiError: embedded_hal::spi::Error,
{
    pub fn new(spi: Spi, calibration_data: &CalibrationData) -> Self {
        Self {
            spi,
            panel_state: TouchPanelState::IDLE,
            sample_buffer: TouchSampleBuffer::default(),
            calibration_data: *calibration_data,
        }
    }

    /// Reset the driver and preload tx buffer with register data.
    pub fn init(&mut self) -> Result<(), SpiError> {
        // Make a throwaway position measurement so that the internal voltage
        // reference is disabled and PENIRQ is enabled.
        _ = self.measure_xy_positions()?;

        self.sample_buffer.index = 0;
        self.panel_state = TouchPanelState::IDLE;

        Ok(())
    }

    /// Sets the calibration data used by [`Self::get_touch_point()`]
    ///
    /// This can be used to update the calibration data after running calibration.
    pub fn set_calibration_data(&mut self, calibration_data: &CalibrationData) {
        self.calibration_data = *calibration_data;
    }

    /// Collects the touch position measurements.
    ///
    /// This should be run continually. This can be done by calling from some
    /// main loop of some task or by calling from a timer interrupt. After
    /// PENIRQ goes high, calling can be suspended until PENIRQ goes low again.
    pub fn run<Irq, IrqError>(&mut self, irq: &mut Irq) -> Result<(), Error<SpiError, IrqError>>
    where
        Irq: InputPin<Error = IrqError>,
        IrqError: embedded_hal::digital::Error,
    {
        match self.panel_state {
            TouchPanelState::IDLE => {
                if irq.is_low().map_err(|e| Error::Irq(e))? {
                    self.sample_buffer.index = 0;
                    self.panel_state = TouchPanelState::SETTLING;
                }
            }
            TouchPanelState::SETTLING => {
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.panel_state = TouchPanelState::RELEASED
                }
                self.sample_buffer.index += 1;
                if self.sample_buffer.index == TOUCH_SAMPLE_BUFFER_SIZE {
                    self.sample_buffer.index = 0;
                    self.panel_state = TouchPanelState::BUFFERING;
                }
            }
            TouchPanelState::BUFFERING => {
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.panel_state = TouchPanelState::RELEASED
                }
                let position = self.measure_xy_positions().map_err(|e| Error::Spi(e))?;
                self.sample_buffer.buffer[self.sample_buffer.index] =
                    Point::new(position.0.into(), position.1.into());
                self.sample_buffer.index += 1;
                if self.sample_buffer.index == TOUCH_SAMPLE_BUFFER_SIZE {
                    self.sample_buffer.index = 0;
                    self.panel_state = TouchPanelState::TOUCHED;
                }
            }
            TouchPanelState::TOUCHED => {
                let position = self.measure_xy_positions().map_err(|e| Error::Spi(e))?;
                self.sample_buffer.buffer[self.sample_buffer.index] =
                    Point::new(position.0.into(), position.1.into());
                self.sample_buffer.index += 1;
                self.sample_buffer.index %= TOUCH_SAMPLE_BUFFER_SIZE;
                if irq.is_high().map_err(|e| Error::Irq(e))? {
                    self.panel_state = TouchPanelState::RELEASED
                }
            }
            TouchPanelState::RELEASED => {
                self.sample_buffer.index = 0;
                self.panel_state = TouchPanelState::IDLE;
            }
        }
        Ok(())
    }

    /// Returns the touch position in XPT2046 measurement units.
    ///
    /// This is made available for use in touch screen calibration procedures.
    pub fn get_touch_point_raw(&self) -> Point {
        self.sample_buffer.average()
    }

    /// Returns the touch position in display pixel units.
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
        self.panel_state == TouchPanelState::TOUCHED
    }

    /// Sometimes the TOUCHED state needs to be cleared
    pub fn clear_touch(&mut self) {
        self.sample_buffer.index = 0;
        self.panel_state = TouchPanelState::IDLE;
    }

    /// Returns the X-Position measurement.
    ///
    /// For this measurement, the ADC reference is a duel-ended (differential)
    /// reference, and the ADC output is 12-bits.
    pub fn measure_x_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::XPosition.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(v0)
    }

    /// Returns the Y-Position measurement.
    ///
    /// For this measurement, the ADC reference is a duel-ended (differential)
    /// reference, and the ADC output is 12-bits.
    pub fn measure_y_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::YPosition.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(v0)
    }

    /// Returns the Z1-Position measurement.
    ///
    /// For this measurement, the ADC reference is a duel-ended (differential)
    /// reference, and the ADC output is 12-bits.
    pub fn measure_z1_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::Z1Position.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(v0)
    }

    /// Returns the Z2-Position measurement.
    ///
    /// For this measurement, the ADC reference is a duel-ended (differential)
    /// reference, and the ADC output is 12-bits.
    pub fn measure_z2_position(&mut self) -> Result<u16, SpiError> {
        const M0: [u8; 2] = control_byte::ChannelSelect::Z2Position.into_delayed_control_byte();
        const TX_BUF: [u8; 3] = [M0[0], M0[1], 0];
        let mut rx_buf = [0; 3];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let value = u16::from_be_bytes([rx_buf[1], rx_buf[2]]);
        Ok(value)
    }

    /// Returns the X-Position and Y-Position measurements as the tuple
    /// (X-Position,Y-Position).
    ///
    /// For these measurements, the ADC reference is a duel-ended (differential)
    /// reference, and the ADC output is 12-bits.
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

    /// Returns the X-Position, Y-Position, Z1-Position and Z2-Position
    /// measurements as the tuple
    /// (X-Position,Y-Position,Z1-Position,Z2-Position).
    ///
    /// For these measurements, the ADC reference is a duel-ended (differential)
    /// reference, and the ADC output is 12-bits.
    ///
    /// On page 20, the XPT2046 data sheet gives an equation for using the
    /// X-Position, Z1-Position and Z2-Position measurements to calculate the
    /// value of the touch resistance when the X plate resistance is known (and
    /// a value proportional to touch resistance when X plate resistance is not
    /// known known). It purports this can be used as a measure of touch
    /// pressure.
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

    /// Returns the TEMP0 measurement.
    ///
    /// For this measurement, the ADC reference is a singled-ended reference
    /// using the internal 2.5V reference, and the ADC output is 12-bits.
    ///
    /// According to pages 18-19 of the XPT2046 data sheet, TEMP0 can be used
    /// alone or in conjunction with TEMP1 to calculate an estimate the ambient
    /// temperature. However, I have to been able to get a sensible temperature
    /// estimate using the hardware I have.
    pub fn measure_temp0(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::TEMP0.into_delayed_control_byte();
        const TX_BUF: [u8; 5] = [RE[0], RE[1], M0[0], M0[1], 0];
        let mut rx_buf = [0; 5];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }

    /// Returns the TEMP1 measurement.
    ///
    /// For this measurement, the ADC reference is a singled-ended reference
    /// using the internal 2.5V reference, and the ADC output is 12-bits.
    ///
    /// According to pages 18-19 of the XPT2046 data sheet, TEMP1 can be used in
    /// conjunction with TEMP0 to calculate an estimate the ambient temperature.
    /// However, I have to been able to get a sensible temperature estimate
    /// using the hardware I have.
    pub fn measure_temp1(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::TEMP1.into_delayed_control_byte();
        const TX_BUF: [u8; 5] = [RE[0], RE[1], M0[0], M0[1], 0];
        let mut rx_buf = [0; 5];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }

    /// Returns the TEMP0 and TEMP1 measurements as the tuple (TEMP0,TEMP1).
    ///
    /// For these measurements, the ADC reference is a singled-ended reference
    /// using the internal 2.5V reference, and the ADC output is 12-bits.
    ///
    /// According to pages 18-19 of the XPT2046 data sheet, TEMP0 and TEMP1 can
    /// be used to calculate an estimate the ambient temperature. However, I
    /// have to been able to get a sensible temperature estimate using the
    /// hardware I have.
    pub fn measure_temps(&mut self) -> Result<(u16, u16), SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::TEMP0.into_delayed_control_byte();
        const M1: [u8; 2] = control_byte::ChannelSelect::TEMP1.into_delayed_control_byte();
        const TX_BUF: [u8; 9] = [RE[0], RE[1], M0[0], M0[1], RE[0], RE[1], M1[0], M1[0], 0];
        let mut rx_buf = [0; 9];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        let v1 = u16::from_be_bytes([rx_buf[5], rx_buf[6]]);
        Ok((v0, v1))
    }

    /// Returns the VBAT measurement.
    ///
    /// For this measurement, the ADC reference is a singled-ended reference
    /// using the internal 2.5V reference, and the ADC output is 12-bits.
    pub fn measure_vbat(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::VBAT.into_delayed_control_byte();
        const TX_BUF: [u8; 5] = [RE[0], RE[1], M0[0], M0[1], 0];
        let mut rx_buf: [u8; _] = [0; 5];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }

    /// Returns the AUXIN measurement.
    ///
    /// For this measurement, the ADC reference is a singled-ended reference
    /// using the internal 2.5V reference, and the ADC output is 12-bits.
    pub fn measure_auxin(&mut self) -> Result<u16, SpiError> {
        const RE: [u8; 2] = control_byte::INTERNAL_REFERENCE_ENABLE;
        const M0: [u8; 2] = control_byte::ChannelSelect::AUXIN.into_delayed_control_byte();
        const TX_BUF: [u8; 5] = [RE[0], RE[1], M0[0], M0[1], 0];
        let mut rx_buf: [u8; _] = [0; 5];
        self.spi.transfer(&mut rx_buf, &TX_BUF)?;
        let v0 = u16::from_be_bytes([rx_buf[3], rx_buf[4]]);
        Ok(v0)
    }
}
