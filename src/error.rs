//! Error definition for the crate

#[cfg(feature = "defmt")]
use defmt::Format;

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub enum CalibrationError {
    Alpha,
    Beta,
    Delta,
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Debug)]
pub enum Error<SPIError, IRQError> {
    /// SPI error
    Spi(SPIError),
    /// IRQ error
    Irq(IRQError),
    /// Error when calculating new calibration values
    Calibration(CalibrationError),
}
