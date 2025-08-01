# SC8815 STM32G0 OTG Mode Example

This example demonstrates using the SC8815 battery management IC in OTG (On-The-Go) mode with an STM32G031G8U6 microcontroller. In OTG mode, the SC8815 operates as a power bank, converting battery voltage to a regulated output voltage for powering external devices.

## Hardware Configuration

### Target MCU

- **STM32G031G8U6** - ARM Cortex-M0+ microcontroller

### Pin Configuration

- **LED**: PB8 - Status indication
- **I2C**: PB6 (SCL), PB7 (SDA) - Communication with SC8815
- **PSTOP**: PB5 - Critical safety control pin

### Battery Configuration

- **Type**: 4S Li-ion battery pack
- **Voltage per cell**: 4.2V (4200mV)
- **Total voltage**: ~16.8V nominal

### Current Sensing

- **Sense resistors**: 5mΩ (both RS1 and RS2)
- **Current limits**:
  - IBUS: 1.5A (output current limit)
  - IBAT: 2.0A (battery current limit)

### Power Settings

- **Output voltage**: 19V (VINREG)
- **Switching frequency**: 450kHz
- **Dead time**: 80ns

## Features

### OTG Mode Operation

- Converts battery voltage to regulated 19V output
- Automatic load detection and power delivery
- Current limiting for safe operation
- Real-time monitoring of voltage and current

### Safety Features

- **PSTOP Control**: Critical safety pin management
  - HIGH: Standby mode (safe for configuration)
  - LOW: Active mode (power blocks enabled)
- **Fault Detection**: Over-temperature and short-circuit protection
- **Communication Monitoring**: Automatic shutdown on I2C failures
- **Short Circuit Recovery**: Automatic VBUS short fault clearing

### Testing Features

- **OTG Toggle**: Output toggles every 10 seconds for testing
- **LED Status Indication**:
  - Fast blink: Power being delivered to load
  - Slow blink: OTG mode active but no load
  - Rapid blink: Error condition
- **Real-time Monitoring**: ADC measurements and status reporting

## Building and Running

### Prerequisites

1. Rust toolchain with `thumbv6m-none-eabi` target
2. Embassy framework dependencies
3. STM32 development tools (probe-rs, OpenOCD, etc.)

### Build Commands

```bash
# Build the example
cargo build --release

# Flash to target (using probe-rs)
cargo run --release
```

### Dependencies

- `embassy-stm32`: STM32 HAL and async runtime
- `sc8815`: SC8815 driver library with async support
- `defmt`: Logging framework
- `embassy-time`: Async timing utilities

## Usage

### Startup Sequence

1. **Initialization**: MCU starts with PSTOP HIGH (safe mode)
2. **I2C Communication**: Verify communication with SC8815
3. **Configuration**: Configure all parameters in standby mode
4. **Activation**: Set PSTOP LOW to enable power blocks
5. **Monitoring**: Continuous status monitoring and safety checks

### Expected Behavior

- LED indicates system status
- Debug output shows voltage/current measurements
- OTG output toggles every 10 seconds
- Automatic fault detection and recovery

### Connecting Loads

- Connect USB devices or other 19V loads to the output
- Maximum current: 1.5A
- System will detect load connection and provide power

## Safety Considerations

### Critical Safety Features

- **Always configure in standby mode first** (PSTOP HIGH)
- **Enable power blocks only after complete configuration** (PSTOP LOW)
- **Automatic shutdown on communication failures**
- **Current limiting to prevent overcurrent conditions**

### Error Handling

- I2C communication failures trigger immediate shutdown
- Configuration errors keep system in safe standby mode
- Fault conditions are logged and handled appropriately

## Troubleshooting

### Common Issues

1. **I2C Communication Failure**
   - Check wiring of PB6 (SCL) and PB7 (SDA)
   - Verify pull-up resistors or enable internal pull-ups
   - Check SC8815 power supply

2. **No Output Voltage**
   - Verify PSTOP is LOW (active mode)
   - Check battery voltage and connections
   - Ensure OTG mode is enabled

3. **LED Blinking Patterns**
   - Slow blink (500ms): I2C or initialization error
   - Fast blink (250ms): Configuration or OTG mode error
   - Very fast blink (100ms): Normal operation with load

### Debug Output

The example provides detailed debug output via defmt/RTT:

- Initialization status
- Configuration parameters
- Real-time measurements
- Fault conditions
- Safety state changes

## Hardware Requirements

### Minimum Hardware

- STM32G031G8U6 development board
- SC8815 evaluation board or custom PCB
- 4S Li-ion battery pack
- I2C connections between MCU and SC8815
- LED on PB8 for status indication

### Recommended Setup

- Proper PCB layout with appropriate current handling
- Adequate heat dissipation for SC8815
- Protection circuits for battery and output
- Debug probe for development and monitoring

## Related Examples

See other examples in this repository:

- `stm32g0-discharge`: Basic discharge mode operation
- Other STM32 variants with different configurations

## License

This example is part of the sc8815-rs project and follows the same license terms.
