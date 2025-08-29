//! Error definition for the crate

#[cfg(feature = "with_defmt")]
use defmt::{write, Format, Formatter};

#[cfg_attr(feature = "with_defmt", derive(Format))]
#[derive(Debug)]
pub enum CalibrationError {
    Alpha,
    Beta,
    Delta,
}

#[derive(Debug)]
pub enum Error<SPIError, IRQError> {
    /// SPI error
    Spi(SPIError),
    /// IRQ error
    Irq(IRQError),
    /// Error when calculating new calibration values
    Calibration(CalibrationError),
}

#[cfg(feature = "with_defmt")]
impl<SPIError, IRQError> Format for Error<SPIError, IRQError> {
    fn format(&self, fmt: Formatter) {
        match self {
            Error::Spi(_) => write!(fmt, "SPI error"),
            Error::Irq(_) => write!(fmt, "IRQ error"),
            Error::Calibration(e) => write!(fmt, "Error when calculating calibration for: {}", e),
        }
    }
}
