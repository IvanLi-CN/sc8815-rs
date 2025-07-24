# SC8815 PSTOP Pin Safety Guide

## ⚠️ CRITICAL SAFETY INFORMATION

**The PSTOP pin is a critical safety control for the SC8815 power management IC. Incorrect usage can result in hardware damage!**

## PSTOP Pin Function

The PSTOP pin controls the power blocks of the SC8815:

- **PSTOP = HIGH**: Standby mode (power blocks disabled, I2C communication active)
- **PSTOP = LOW**: Active mode (power blocks enabled)

## ⚠️ Why PSTOP HIGH is Required During Configuration

### Factory Default Risk
The SC8815 ships with factory default configuration that may not match your hardware setup. Enabling power blocks (PSTOP = LOW) before proper configuration can cause:

- **Overvoltage damage** to connected devices
- **Overcurrent conditions** 
- **Thermal damage** to the SC8815 or external components
- **Battery damage** from incorrect charging parameters

### Register Configuration Requirements
According to the SC8815 datasheet, many critical registers **MUST** be configured while "PSTOP pin is high":

- Battery voltage settings (VBAT_SET register)
- Current limit ratios (IBUS_RATIO, IBAT_RATIO)
- Switching frequency and dead time
- Voltage monitoring ratios
- Loop control settings

## Correct Initialization Sequence

### 1. Power-On State
```rust
// Start with PSTOP HIGH for safety
let mut pstop = Output::new(p.PB5, Level::High, Speed::Low);
```

### 2. I2C Communication Test
```rust
// Test I2C communication while in standby mode
let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
match sc8815.read_register(Register::Status).await {
    Ok(status) => info!("I2C OK in standby: 0x{:02X}", status),
    Err(e) => {
        error!("I2C failed: {:?}", e);
        // PSTOP already HIGH - keep it that way for safety
        return; // or handle error appropriately
    }
}
```

### 3. Device Initialization
```rust
// Initialize device while in standby mode
sc8815.init().await?;
```

### 4. Complete Configuration
```rust
// Configure all device parameters while in standby
let config = DeviceConfiguration::default();
// ... configure all parameters ...
sc8815.configure_device(&config).await?;
sc8815.set_otg_mode(true).await?; // or false for charging
sc8815.set_adc_conversion(true).await?;
```

### 5. Enable Power Blocks
```rust
// ONLY after complete configuration, enable power blocks
info!("Configuration complete - enabling power blocks");
pstop.set_low(); // Enable power blocks
Timer::after(Duration::from_millis(10)).await; // Stabilization delay
```

## Error Handling and Safety

### Communication Failures
If I2C communication fails at any point during operation:

```rust
Err(e) => {
    error!("Communication failed: {:?}", e);
    
    // SAFETY: Immediately disable power blocks
    pstop.set_high(); // Enter standby mode
    
    // Handle error (blink LED, restart, etc.)
}
```

### Configuration Failures
If device configuration fails:

```rust
if let Err(e) = sc8815.configure_device(&config).await {
    error!("Configuration failed: {:?}", e);
    
    // SAFETY: Keep PSTOP HIGH (already in standby)
    // Do NOT enable power blocks with invalid configuration
    
    // Handle error appropriately
}
```

## Updated Example Code

Both charging and OTG examples have been updated to follow this safety protocol:

1. **examples/stm32g0-charging/src/main.rs**: Safe charging mode initialization
2. **examples/stm32g0-otg/src/main.rs**: Safe OTG mode initialization

## Key Safety Points

1. **Always start with PSTOP = HIGH**
2. **Complete ALL configuration before setting PSTOP = LOW**
3. **If communication fails, immediately set PSTOP = HIGH**
4. **Never enable power blocks with unverified configuration**
5. **Use the 10ms stabilization delay after enabling power blocks**

## Hardware Connections

Ensure proper hardware connections:

- PSTOP pin connected to MCU GPIO (PB5 in examples)
- 4.7kΩ pull-up resistors on I2C lines
- Proper power supply connections
- Current sense resistors properly connected

## Testing Recommendations

1. Test I2C communication first in standby mode
2. Verify all configuration registers are set correctly
3. Monitor device status after enabling power blocks
4. Implement proper error handling and recovery

This safety protocol prevents hardware damage and ensures reliable operation of the SC8815 power management system.
