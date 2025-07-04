#![no_std]
#![doc = include_str!("../README.md")]

//! # SW2303 USB PD Controller Driver
//!
//! This crate provides a driver for the SW2303 USB PD (Power Delivery) controller chip.
//! The SW2303 is a USB PD controller that supports charging management and device detection.
//!
//! ## Features
//!
//! - **Device Detection**: Monitor sink device connection status
//! - **Power Management**: USB PD charging control and monitoring
//! - **Status Monitoring**: Real-time controller and charging status information
//! - **Async/Sync Support**: Compatible with both blocking and async I2C implementations
//! - **Optional defmt logging support**
//!
//! ## Usage
//!
//! ### Synchronous Version
//!
//! ```rust,no_run
//! use sw2303::{SW2303, registers::constants::DEFAULT_ADDRESS};
//! use embedded_hal::i2c::I2c;
//!
//! # fn example<I2C: I2c>(mut i2c: I2C) -> Result<(), sw2303::error::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug
//! # {
//! // Initialize the driver
//! let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
//! sw2303.init()?;
//!
//! // Check if a sink device is connected
//! if sw2303.is_sink_device_connected()? {
//!     // Sink device connected
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ### Asynchronous Version (requires "async" feature)
//!
//! ```rust,no_run
//! # #[cfg(feature = "async")]
//! # async fn async_example() -> Result<(), sw2303::error::Error<()>> {
//! use sw2303::{SW2303, registers::constants::DEFAULT_ADDRESS};
//! use embedded_hal_async::i2c::I2c;
//!
//! // Initialize the driver
//! // let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
//! // sw2303.init().await?;
//!
//! // Check if a sink device is connected
//! // if sw2303.is_sink_device_connected().await? {
//! //     // Sink device connected
//! // }
//! # Ok(())
//! # }
//! ```
//!
//! ## Feature Flags
//!
//! - `async`: Enable asynchronous I2C support
//! - `defmt`: Enable defmt logging support

mod data_types;
pub mod error;
pub mod registers;
pub mod driver;

use crate::error::Error;
pub use data_types::*;
pub use driver::SW2303;

/// Result type for SW2303 operations
pub type Result<T, I2cError> = core::result::Result<T, Error<I2cError>>;

/// SW2303 driver version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
