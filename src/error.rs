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
pub enum Error<E> {
    /// SPI error
    Spi(E),
    /// Error when calculating new calibration values
    Calibration(CalibrationError),
    /// Delay error
    Delay,
}

#[cfg(feature = "with_defmt")]
impl<E> Format for Error<E> {
    fn format(&self, fmt: Formatter) {
        match self {
            Error::Spi(_) => write!(fmt, "SPI error"),
            Error::Calibration(e) => write!(fmt, "Error when calculating calibration for: {}", e),
            Error::Delay => write!(fmt, "Delay error"),
        }
    }
}
