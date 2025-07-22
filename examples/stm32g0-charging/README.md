# SC8815 STM32G031C8U6 Charging Example

This example demonstrates using the SC8815 battery management IC in charging mode with an STM32G031C8U6 microcontroller.

## Hardware Configuration

- **MCU**: STM32G031C8U6 (Cortex-M0+)
- **LED**: PB8 (Active Low)
- **I2C**: PB6 (SCL), PB7 (SDA)
- **PSTOP**: PB5 (Low = Enable, High = Disable)
- **Battery**: 4S LiFePO4 (4.45V per cell, 17.8V total)
- **Current sense resistors**: 5mΩ
- **Switching frequency**: 450kHz
- **Dead time**: 60ns

## Battery Configuration

- **Cell count**: 4S (4 cells in series)
- **Cell voltage**: 4.45V per cell (LiFePO4)
- **Total voltage**: 17.8V
- **Charging current limit**: 1A
- **VINREG voltage**: 11.5V

## LED Status Indicators

- **Solid ON**: Charging complete (End of Charge)
- **1 second blink**: Charging in progress
- **Fast blink (250ms)**: Fault condition
- **OFF**: No AC adapter connected

## Building and Flashing

### Prerequisites

1. Install Rust and cargo
2. Install probe-rs: `cargo install probe-rs --features cli`
3. Add the ARM Cortex-M target: `rustup target add thumbv6m-none-eabi`

### Build

```bash
# Debug build
cargo build

# Release build (optimized)
cargo build --release
```

### Flash to MCU

```bash
# Flash debug version
cargo run

# Flash release version
cargo run --release
```

### Monitor Output

Use probe-rs to monitor the defmt output:

```bash
probe-rs run --chip STM32G031C8U6 target/thumbv6m-none-eabi/release/sc8815_stm32g031c8u6_charging_example
```

## Expected Behavior

1. **Initialization**: LED turns on briefly to indicate successful SC8815 initialization
2. **No AC Adapter**: LED stays off, device waits for AC adapter connection
3. **AC Adapter Connected**: LED blinks every 1 second, charging begins
4. **Charging Complete**: LED stays on solid
5. **Fault Conditions**: LED blinks rapidly (250ms intervals)

## Troubleshooting

### I2C Communication Issues

- Check I2C connections (PB6=SCL, PB7=SDA)
- Verify pull-up resistors on I2C lines
- Check SC8815 power supply

### No Charging

- Verify AC adapter is connected and providing adequate voltage
- Check battery connections
- Ensure PSTOP pin (PB5) is low to enable SC8815
- Monitor defmt output for error messages

### LED Not Working

- Check LED connection to PB8
- Verify LED polarity (active low configuration)
- Check for GPIO configuration issues

## Configuration Details

The example uses the following SC8815 configuration:

- **Operating Mode**: Charging
- **Battery Type**: LiFePO4 (internal voltage setting)
- **Current Sensing**: 5mΩ resistors on both RS1 and RS2
- **Current Limits**: 1A charging current (IBAT), 1A input current (IBUS)
- **Voltage Regulation**: 11.5V VINREG
- **Protection Features**: Over-temperature protection, VBUS short circuit protection
- **Termination**: Charging termination enabled
- **Trickle Charging**: Enabled for deeply discharged batteries

## Serial Output

The example provides detailed logging via defmt, including:

- Initialization status
- I2C communication results
- Device configuration status
- Real-time ADC measurements (VBUS, VBAT, IBUS, IBAT, ADIN)
- Charging status and fault conditions
- LED control logic

Connect a debug probe to view this output for detailed system monitoring and troubleshooting.
