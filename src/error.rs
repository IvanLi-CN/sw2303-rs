//! SW2303 driver library error types.

use core::fmt::Debug;
#[cfg(feature = "defmt")]
use defmt;

/// Represents possible errors that can occur when interacting with the SW2303 driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<I2cError: Debug> {
    /// An error occurred during an underlying I2C bus operation.
    I2c(I2cError),
    /// An attempt was made to access a reserved register address or an invalid port.
    InvalidRegisterOrPort,
    /// Invalid parameter value provided to a function.
    InvalidParameter,
    /// Device not responding or device ID mismatch.
    DeviceNotResponding,
    /// Operation timeout occurred.
    Timeout,
    /// Port configuration error.
    PortConfigError,
    /// Hub initialization failed.
    InitializationFailed,
    /// Device is in an unexpected state.
    InvalidDeviceState,
    /// Overcurrent condition detected.
    OvercurrentDetected,
}

impl<I2cError: Debug> core::fmt::Display for Error<I2cError> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::I2c(err) => write!(f, "I2C communication error: {:?}", err),
            Error::InvalidRegisterOrPort => write!(f, "Invalid register address or port number"),
            Error::InvalidParameter => write!(f, "Invalid parameter value"),
            Error::DeviceNotResponding => write!(f, "Device not responding or device ID mismatch"),
            Error::Timeout => write!(f, "Operation timeout"),
            Error::PortConfigError => write!(f, "Port configuration error"),
            Error::InitializationFailed => write!(f, "Hub initialization failed"),
            Error::InvalidDeviceState => write!(f, "Device is in an unexpected state"),
            Error::OvercurrentDetected => write!(f, "Overcurrent condition detected"),
        }
    }
}

// TODO: Implement From trait for I2cError if possible
// impl<I2cError: Debug> From<I2cError> for Error<I2cError> {
//     fn from(err: I2cError) -> Self {
//         Error::I2c(err)
//     }
// }
