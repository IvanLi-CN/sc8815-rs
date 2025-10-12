#![no_std]
#![doc = include_str!("../README.md")]

//! # SC8815 Power Management IC Driver
//!
//! This crate provides a driver for the SC8815 power management IC.
//! The SC8815 is a power management IC that supports charging management and power delivery control.
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
#![doc = concat!("```toml\n[dependencies]\nsc8815 = \"", env!("CARGO_PKG_VERSION"), "\"\n# For async support\nsc8815 = { version = \"", env!("CARGO_PKG_VERSION"), "\", features = [\"async\"] }\n# For defmt logging support\nsc8815 = { version = \"", env!("CARGO_PKG_VERSION"), "\", features = [\"defmt\"] }\n# For both async and defmt\nsc8815 = { version = \"", env!("CARGO_PKG_VERSION"), "\", features = [\"async\", \"defmt\"] }\n```")]
//!
//! ## Usage
//!
//! ### Basic Setup
//!
//! ```rust,ignore
//! use sc8815::{SC8815, registers::constants::DEFAULT_ADDRESS};
//! use embedded_hal::i2c::I2c;
//!
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), sc8815::error::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug
//! # {
//! // Initialize the driver
//! let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
//! sc8815.init()?;
//!
//! // Check device status
//! let status = sc8815.get_device_status()?;
//! if !status.otp_fault && !status.vbus_short_fault {
//!     // Device is ready for operation
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ### Charging Mode Configuration
//!
//! ```rust,ignore
//! use sc8815::{
//!     SC8815, DeviceConfiguration, OperatingMode, CellCount,
//!     SwitchingFrequency, DeadTime, VoltagePerCell,
//!     registers::constants::DEFAULT_ADDRESS
//! };
//! # use embedded_hal::i2c::I2c;
//! # fn charging_example<I2C: I2c>(i2c: I2C) -> Result<(), sc8815::error::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug
//! # {
//!
//! let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
//! sc8815.init()?;
//!
//! // Configure for 4S Li-ion battery charging
//! let mut config = DeviceConfiguration::default();
//! config.battery.cell_count = CellCount::Cells4S;
//! config.battery.voltage_per_cell = VoltagePerCell::Mv4200;
//! config.power.operating_mode = OperatingMode::Charging;
//! config.power.switching_frequency = SwitchingFrequency::Freq450kHz;
//! config.power.dead_time = DeadTime::Ns60;
//! config.current_limits.ibus_limit_ma = 1000; // 1A charging current
//!
//! sc8815.configure_device(&config)?;
//! sc8815.set_adc_conversion(true)?;
//!
//! // Monitor charging status
//! let measurements = sc8815.get_adc_measurements()?;
//! # Ok(())
//! # }
//! ```
//!
//! ### OTG (Power Bank) Mode Configuration
//!
//! ```rust,ignore
//! # use sc8815::{
//! #     SC8815, DeviceConfiguration, OperatingMode, CellCount,
//! #     SwitchingFrequency, DeadTime, VoltagePerCell,
//! #     registers::constants::DEFAULT_ADDRESS
//! # };
//! # use embedded_hal::i2c::I2c;
//! # fn otg_example<I2C: I2c>(i2c: I2C) -> Result<(), sc8815::error::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug
//! # {
//!
//! let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
//! sc8815.init()?;
//!
//! // Configure for OTG mode (power bank)
//! let mut config = DeviceConfiguration::default();
//! config.battery.cell_count = CellCount::Cells4S;
//! config.power.operating_mode = OperatingMode::OTG;
//! config.power.switching_frequency = SwitchingFrequency::Freq450kHz;
//! config.power.dead_time = DeadTime::Ns80; // Higher dead time for OTG
//! // NOTE: VINREG is for charging mode only and is ignored in OTG.
//! config.current_limits.ibus_limit_ma = 6000; // 6A output limit
//!
//! sc8815.configure_device(&config)?;
//! sc8815.set_otg_mode(true)?;
//! // Set OTG output voltage via internal VBUS reference
//! sc8815.set_vbus_internal_voltage(19000)?; // 19V output
//!
//! // Monitor output
//! let status = sc8815.get_device_status()?;
//! if status.usb_load_detected {
//!     // Providing power to connected device
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
//! use sc8815::{SC8815, DeviceConfiguration, registers::constants::DEFAULT_ADDRESS};
//! use embedded_hal_async::i2c::I2c;
//!
//! // Same API as synchronous version, but with .await
//! // let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
//! // sc8815.init().await?;
//! // let status = sc8815.get_device_status().await?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Important Safety Notes
//!
//! - **PSTOP Control**: Always configure the device in standby mode (PSTOP=HIGH) before enabling power blocks
//! - **Dead Time**: Critical for preventing shoot-through in power MOSFETs. Use 60-80ns for most applications
//! - **Switching Frequency**: Higher frequencies (450kHz) improve efficiency but require careful PCB layout
//! - **Current Sense Resistors**: Ensure Rs1/Rs2 values match your hardware (typically 5-10mΩ)
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
