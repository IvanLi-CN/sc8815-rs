#![no_std]
#![doc = include_str!("../README.md")]

//! # SC8815 Power Management IC Driver
//!
//! This crate provides a driver for the SC8815 power management IC.
//! The SC8815 is a power management IC that supports charging management and power delivery control.
//!
//! Test formatting issue - this line has bad formatting
//!
//! ## Features
//!
//! - **Power Management**: Comprehensive power delivery control and monitoring
//! - **Charging Control**: Battery charging management and status monitoring
//! - **Status Monitoring**: Real-time controller and power status information
//! - **Async/Sync Support**: Compatible with both blocking and async I2C implementations
//! - **Optional defmt logging support**
//!
//! ## Installation
//!
//! Add this to your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! sc8815 = "0.1.0"
//! # For async support
//! sc8815 = { version = "0.1.0", features = ["async"] }
//! # For defmt logging support
//! sc8815 = { version = "0.1.0", features = ["defmt"] }
//! # For both async and defmt
//! sc8815 = { version = "0.1.0", features = ["async", "defmt"] }
//! ```
//!
//! ## Usage
//!
//! ### Synchronous Version
//!
//! ```rust,no_run
//! use sc8815::{SC8815, registers::constants::DEFAULT_ADDRESS};
//! use embedded_hal::i2c::I2c;
//!
//! # fn example<I2C: I2c>(mut i2c: I2C) -> Result<(), sc8815::error::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug
//! # {
//! // Initialize the driver
//! let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);
//! sc8815.init()?;
//!
//! // Check power status
//! if sc8815.is_power_good()? {
//!     // Power is good
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ### Asynchronous Version (requires "async" feature)
//!
//! ```rust,no_run
//! # #[cfg(feature = "async")]
//! # async fn async_example() -> Result<(), sc8815::error::Error<()>> {
//! use sc8815::{SC8815, registers::constants::DEFAULT_ADDRESS};
//! use embedded_hal_async::i2c::I2c;
//!
//! // Initialize the driver
//! // let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);
//! // sc8815.init().await?;
//!
//! // Check power status
//! // if sc8815.is_power_good().await? {
//! //     // Power is good
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
pub mod driver;
pub mod error;
pub mod registers;

use crate::error::Error;
pub use data_types::*;
pub use driver::SC8815;
pub use registers::{
    BatteryStatusFlags, Ctrl0Flags, Ctrl1Flags, Ctrl2Flags, Ctrl3Flags, InputSourceStatusFlags,
    MaskFlags, RatioFlags, StatusFlags, ThermalStatusFlags, VbatSetFlags,
};

/// Result type for SC8815 operations
pub type Result<T, I2cError> = core::result::Result<T, Error<I2cError>>;

/// SC8815 driver version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
