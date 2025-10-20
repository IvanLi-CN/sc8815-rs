# SC8815 + SW2303 FB Regulation Demo (STM32G031G8U6)

This example configures an SC8815 power-management IC and an SW2303 USB-PD controller on an STM32G031G8U6 platform. VBUS is regulated through the SC8815 FB network using an internal 1.2 V reference and external resistors, while the SW2303 negotiates charging protocols and provides telemetry.

## Hardware Overview

- **MCU**: STM32G031G8U6 (PB6 = I²C1 SCL, PB7 = I²C1 SDA, PB5 = SC8815 PSTOP, PB8 = status LED)
- **SC8815**:
  - Operating mode: OTG (discharge)
  - External FB divider: Rp = 100 kΩ, Rd = 31.6 kΩ → ≈5 V output with 1.2 V reference
  - Sense resistors: RS1 = RS2 = 5 mΩ
  - Current limits: IBUS = 3.5 A, IBAT = 2 A
  - VBUS/VBAT sensing: **internal VBAT monitoring (VBATS tied to VBAT)** with driver defaults targeting a 4S pack
  - Switching frequency: 450 kHz, dead time: 80 ns
- **SW2303**:
  - Default protocol set, monitoring of negotiated fast-charging mode
  - 12-bit ADC readings for VBUS and output current (ICH)

## Firmware Behavior

1. Hold `PSTOP` high to keep SC8815 in standby.
2. Configure SC8815 (OTG mode, FB network, ADC).
3. Set `PSTOP` low and wait 500 ms for stabilization.
4. Initialize SW2303 on the same I²C bus.
5. Every 500 ms log:
   - SC8815 VBAT / IBAT / VBUS / IBUS
   - SW2303 VBUS / IBUS (ICH)
   - Active SW2303 protocol (if any)

Any configuration failure triggers a repeating LED blink fault loop.

## Building & Running

```bash
# From this example directory
make build       # Cross-builds for thumbv6m-none-eabi (release profile)
eval "$(make -s select-probe)"  # Optional helper to choose a probe
make run         # Flash + run via probe-rs
```

Debug output is available via `defmt-rtt`. To keep the control loop stable:

- Keep `VBATS` hard-wired to `VBAT` so the SC8815 uses its internal VBAT monitor.
- Use the bundled configuration (4S battery defaults) unless you also redesign the resistor network and ADC ratios.
- Verify the external FB network (Rp/Rd) is wired exactly as shown to avoid hardware damage.
