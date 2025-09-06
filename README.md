# SC8815 Power Management IC Driver

A Rust driver for the SC8815 power management IC, designed for embedded systems.

## Features

- **No-std compatible**: Works in embedded environments without standard library
- **Async support**: Optional async/await support with `async` feature
- **Defmt logging**: Optional structured logging with `defmt` feature
- **Type-safe register access**: Strongly typed register definitions based on actual SC8815 datasheet
- **Dual-mode operation**: Support for both charging and OTG (discharging) modes
- **Comprehensive ADC support**: 10-bit ADC readings for VBUS, VBAT, IBUS, IBAT, and ADIN
- **Advanced power management**: Switching frequency control, dead time settings, PFM mode
- **Battery configuration**: Multi-cell support (1S-4S), voltage per cell settings, IR compensation
- **Current limit control**: Configurable IBUS and IBAT limits with ratio settings
- **Interrupt handling**: Comprehensive interrupt mask and status management
- **VBUS voltage control**: Programmable output voltage for discharging mode
- **Charging algorithms**: Trickle charging, EOC detection, charging termination control
- **Error handling**: Comprehensive error types for robust applications

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
sc8815 = "0.0.0"

# For async support
sc8815 = { version = "0.0.0", features = ["async"] }

# For defmt logging support
sc8815 = { version = "0.0.0", features = ["defmt"] }

# For both async and defmt
sc8815 = { version = "0.0.0", features = ["async", "defmt"] }
```

## Usage

### Basic Usage

```rust
use sc8815::{SC8815, DeviceConfiguration, OperatingMode, CellCount, VoltagePerCell, registers::constants::DEFAULT_ADDRESS};
use embedded_hal::i2c::I2c;

fn example<I2C: I2c>(mut i2c: I2C) -> Result<(), sc8815::error::Error<I2C::Error>>
where I2C::Error: core::fmt::Debug
{
    // Initialize the driver with an I2C interface
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize the controller
    sc8815.init()?;

    // Check device status
    let status = sc8815.get_device_status()?;
    if status.ac_adapter_connected {
        println!("AC adapter connected");
    }
    if status.eoc {
        println!("End of charge reached");
    }
    if status.otp_fault {
        println!("Over-temperature protection triggered");
    }

    // Configure device with custom settings
    let mut config = DeviceConfiguration::default();
    config.battery.cell_count = CellCount::Cells2S; // 2S battery
    config.battery.voltage_per_cell = VoltagePerCell::Mv4200; // 4.2V per cell
    config.current_limits.ibus_limit_ma = 2000; // 2A IBUS limit
    config.current_limits.ibat_limit_ma = 2000; // 2A IBAT limit
    config.power.operating_mode = OperatingMode::Charging;

    sc8815.configure_device(&config)?;

    // Switch to OTG (discharging) mode
    sc8815.set_otg_mode(true)?;

    // Set VBUS output voltage for discharging (5V)
    sc8815.set_vbus_internal_voltage(5000, 1)?; // 5000mV, 5x ratio

    // Read ADC measurements
    let measurements = sc8815.get_adc_measurements()?;
    println!("VBUS: {}mV", measurements.vbus_mv);
    println!("VBAT: {}mV", measurements.vbat_mv);
    println!("IBUS: {}mA", measurements.ibus_ma);
    println!("IBAT: {}mA", measurements.ibat_ma);

    Ok(())
}
```

### Async Usage

```rust
# #[cfg(feature = "async")]
# {
use sc8815::{SC8815, DeviceConfiguration, registers::constants::DEFAULT_ADDRESS};
use embedded_hal_async::i2c::I2c;

// With async feature enabled
async fn async_example<I2C: I2c>(mut i2c: I2C) -> Result<(), sc8815::error::Error<I2C::Error>>
where I2C::Error: core::fmt::Debug
{
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize asynchronously
    sc8815.init().await?;

    // Configure device asynchronously
    let config = DeviceConfiguration::default();
    sc8815.configure_device(&config).await?;

    // Check AC adapter status
    if sc8815.is_ac_adapter_connected().await? {
        println!("AC adapter connected");
    }

    // Read ADC measurements asynchronously
    let measurements = sc8815.get_adc_measurements().await?;
    println!("Battery voltage: {}mV", measurements.vbat_mv);

    // Configure interrupts
    use sc8815::MaskFlags;
    sc8815.enable_interrupts(MaskFlags::AC_OK_MASK | MaskFlags::EOC_MASK).await?;

    Ok(())
}
# }
```

## Feature Flags

- `async`: Enables async/await support for non-blocking operations
- `defmt`: Enables structured logging with defmt

## Hardware Support

This driver is designed for the SC8815 power management IC. The SC8815 is a versatile power management IC that supports:

### Core Features
- **Dual-mode operation**: Charging mode and OTG (On-The-Go) discharging mode
- **Multi-cell battery support**: 1S to 4S battery configurations
- **Programmable voltage**: 4.1V to 4.45V per cell (4.1V, 4.2V, 4.3V, 4.35V, 4.4V, 4.45V)
- **Current monitoring**: Separate IBUS and IBAT current sensing
- **10-bit ADC**: High-resolution voltage and current measurements
- **I2C interface**: 7-bit address 0x74, standard I2C protocol

### Power Management
- **Switching frequency**: 150kHz, 300kHz, or 450kHz with dithering support
- **Dead time control**: 20ns to 80ns programmable dead time
- **PFM mode**: Pulse Frequency Modulation for light load efficiency
- **VINREG control**: Input voltage regulation for charging mode

### Protection Features
- **Over-temperature protection (OTP)**: Thermal shutdown protection
- **VBUS short circuit protection**: Current foldback in discharging mode
- **Overvoltage protection**: Configurable OVP for discharging mode
- **Current limiting**: Programmable IBUS and IBAT current limits

### Hardware Requirements
- **I2C connection**: SDA, SCL, and ground connections
- **Current sense resistors**: Typically 10mΩ for IBUS and IBAT sensing
- **External components**: As specified in SC8815 datasheet

## Register Map

The driver provides comprehensive access to all SC8815 registers based on the actual datasheet:

### Configuration Registers

- **VBAT_SET (0x00)**: Battery voltage and cell configuration
- **VBUSREF_I_SET (0x01-0x02)**: Internal VBUS reference voltage setting
- **VBUSREF_E_SET (0x03-0x04)**: External VBUS reference voltage setting
- **IBUS_LIM_SET (0x05)**: IBUS current limit configuration
- **IBAT_LIM_SET (0x06)**: IBAT current limit configuration
- **VINREG_SET (0x07)**: VINREG voltage setting for charging mode
- **RATIO (0x08)**: Current and voltage sensing ratio configuration
- **CTRL0_SET (0x09)**: Dead time, frequency, VINREG ratio, OTG enable
- **CTRL1_SET (0x0A)**: OVP, trickle, termination, current selection
- **CTRL2_SET (0x0B)**: Slew rate, dithering, factory settings
- **CTRL3_SET (0x0C)**: PFM, EOC, foldback, loop, ADC, GPO, PGATE

### ADC Value Registers

- **VBUS_FB_VALUE (0x0D-0x0E)**: VBUS feedback ADC value (10-bit)
- **VBAT_FB_VALUE (0x0F-0x10)**: VBAT feedback ADC value (10-bit)
- **IBUS_VALUE (0x11-0x12)**: IBUS current ADC value (10-bit)
- **IBAT_VALUE (0x13-0x14)**: IBAT current ADC value (10-bit)
- **ADIN_VALUE (0x15-0x16)**: ADIN voltage ADC value (10-bit)

### Status and Control Registers

- **STATUS (0x17)**: Device status (EOC, OTP, VBUS_SHORT, INDET, AC_OK)
- **MASK (0x19)**: Interrupt mask configuration

## Error Handling

The driver provides comprehensive error handling with specific error types:

- `Communication`: I2C communication errors
- `InvalidRegisterOrParameter`: Invalid register access or parameter
- `InvalidParameter`: Invalid function parameters
- `DeviceNotResponding`: Device not found or not responding
- `Timeout`: Operation timeout
- `PowerConfigError`: Power configuration issues
- `InitializationFailed`: Device initialization problems
- `OvercurrentDetected`: Overcurrent protection triggered
- `OvervoltageDetected`: Overvoltage protection triggered
- `ThermalProtection`: Thermal protection triggered
- `BatteryError`: Battery related errors
- `ChargingError`: Charging related errors

## Development Status

This implementation provides comprehensive SC8815 functionality based on the actual datasheet:

### ✅ Completed Features

- **Complete register mapping**: All SC8815 registers implemented with actual addresses
- **Dual-mode operation**: Full support for charging and OTG (discharging) modes
- **ADC measurements**: 10-bit ADC readings for VBUS, VBAT, IBUS, IBAT, and ADIN
- **Battery configuration**: Multi-cell support (1S-4S) with voltage and IR compensation
- **Current limit control**: Configurable IBUS and IBAT limits with ratio calculations
- **Power management**: Switching frequency, dead time, PFM mode, VINREG control
- **VBUS voltage control**: Programmable output voltage for discharging mode
- **Charging algorithms**: Trickle charging, EOC detection, charging termination
- **Interrupt handling**: Comprehensive interrupt mask and status management
- **Protection features**: OTP, VBUS short circuit, OVP protection control
- **Power optimization**: Slew rate control, loop response, bandwidth settings
- **Comprehensive configuration**: High-level device configuration with data structures
- **Error handling**: Comprehensive error types for robust operation
- **Async support**: Full async/await support with feature flag
- **Testing**: Unit tests and integration tests with mock I2C
- **Examples**: Multiple usage examples demonstrating different features


## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
