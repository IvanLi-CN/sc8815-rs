#![no_std]
#![no_main]

// SC8815 STM32G0 OTG Mode Example
// Configured for 19V output voltage with 1.5A current limit
// This example demonstrates using SC8815 in reverse discharge (OTG) mode
// Hardware configuration:
// - LED: PB8
// - I2C: PB6 (SCL), PB7 (SDA)
// - Battery: 4S LiFePO4 (4.5V per cell)
// - Current sense resistors: 5mΩ
// - Switching frequency: 450kHz
// - Dead time: 80ns

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    i2c::{self, Config, I2c},
    peripherals::I2C1,
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use sc8815::{
    SC8815, DeviceConfiguration, OperatingMode, CellCount, SwitchingFrequency, DeadTime, VoltagePerCell,
    registers::constants::DEFAULT_ADDRESS,
};

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<I2C1>, i2c::ErrorInterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("SC8815 STM32G0 OTG Mode Example Starting");
    info!("Configuration: 4S LiFePO4, 19V output, 1.5A current limit, 450kHz switching");

    // Configure LED for status indication on PB8
    let mut led = Output::new(p.PB8, Level::High, Speed::Low);

    // Configure PSTOP pin on PB5 (Low = Enable, High = Disable)
    let mut pstop = Output::new(p.PB5, Level::Low, Speed::Low); // Start with SC8815 enabled

    // Configure I2C1 (PB6: SCL, PB7: SDA) - Changed from PB8/PB9 since PB8 is now used for LED
    let mut i2c_config = Config::default();
    // Enable internal pull-ups for I2C lines
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;

    let i2c = I2c::new(
        p.I2C1,
        p.PB6, // SCL
        p.PB7, // SDA
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Hertz(100_000), // 100kHz
        i2c_config,
    );

    info!("Starting SC8815 initialization...");
    info!("I2C address: 0x{:02X}", DEFAULT_ADDRESS);

    // Try to communicate with SC8815 at the expected address
    let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
    info!("Testing I2C communication with SC8815 at 0x{:02X}...", DEFAULT_ADDRESS);
    match sc8815.read_register(sc8815::registers::Register::Status).await {
        Ok(status) => {
            info!("I2C communication successful, status register: 0x{:02X}", status);
        }
        Err(e) => {
            error!("I2C communication failed: {:?}", e);

            // SAFETY: Immediately disable SC8815 by pulling PSTOP high
            error!("SAFETY: Disabling SC8815 due to I2C communication failure");
            pstop.set_high(); // Disable SC8815 chip for safety

            // Blink LED slowly to indicate I2C error with SC8815
            loop {
                led.toggle();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }

    info!("Initializing SC8815...");
    match sc8815.init().await {
        Ok(()) => {
            info!("SC8815 initialized successfully");
            led.set_low(); // Turn on LED to indicate success
        }
        Err(e) => {
            error!("Failed to initialize SC8815: {:?}", e);

            // SAFETY: Immediately disable SC8815 by pulling PSTOP high
            error!("SAFETY: Disabling SC8815 due to initialization failure");
            pstop.set_high(); // Disable SC8815 chip for safety

            // Blink LED slowly to indicate init error
            loop {
                led.toggle();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }

    // Configure the device for OTG (reverse discharge) mode - 19V output, 1.5A limit
    let mut config = DeviceConfiguration::default();

    // Configure for 4S LiFePO4 battery using internal voltage setting
    config.battery.cell_count = CellCount::Cells4S;
    config.battery.voltage_per_cell = VoltagePerCell::Mv4450;
    config.battery.use_internal_setting = true;

    // Configure current limits with 5mΩ sense resistors
    config.current_limits.rs1_mohm = 5;
    config.current_limits.rs2_mohm = 5;
    config.current_limits.ibus_limit_ma = 1500;
    config.current_limits.ibat_limit_ma = 2000;

    // Configure power settings
    config.power.operating_mode = OperatingMode::OTG;
    config.power.switching_frequency = SwitchingFrequency::Freq450kHz;
    config.power.dead_time = DeadTime::Ns80;
    config.power.vinreg_voltage_mv = 19000;

    // OTG mode settings
    config.trickle_charging = false;
    config.charging_termination = false;
    config.use_ibus_for_charging = false;

    // Disable short circuit foldback for startup with load
    info!("Disabling short circuit foldback for startup...");
    if let Err(e) = sc8815.set_short_foldback_disable(true).await {
        error!("Failed to disable short foldback: {:?}", e);
    } else {
        info!("Short circuit foldback disabled for startup");
    }

    info!("Configuring SC8815 for OTG mode (19V output, 1.5A limit)...");
    if let Err(e) = sc8815.configure_device(&config).await {
        error!("Failed to configure SC8815: {:?}", e);

        // SAFETY: Immediately disable SC8815 by pulling PSTOP high
        error!("SAFETY: Disabling SC8815 due to configuration failure");
        pstop.set_high(); // Disable SC8815 chip for safety

        // Blink LED rapidly to indicate configuration error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("SC8815 configured successfully for OTG mode");
    }

    // Explicitly enable OTG mode
    info!("Enabling OTG mode...");
    if let Err(e) = sc8815.set_otg_mode(true).await {
        error!("Failed to enable OTG mode: {:?}", e);

        // SAFETY: Immediately disable SC8815 by pulling PSTOP high
        error!("SAFETY: Disabling SC8815 due to OTG mode configuration failure");
        pstop.set_high(); // Disable SC8815 chip for safety

        // Blink LED rapidly to indicate OTG mode error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("OTG mode enabled successfully");
    }

    // Enable ADC conversion
    if let Err(e) = sc8815.set_adc_conversion(true).await {
        error!("Failed to start ADC conversion: {:?}", e);

        // SAFETY: Immediately disable SC8815 by pulling PSTOP high
        error!("SAFETY: Disabling SC8815 due to ADC configuration failure");
        pstop.set_high(); // Disable SC8815 chip for safety

        // Blink LED rapidly to indicate ADC error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("ADC conversion started");
    }

    info!("SC8815 configured as power bank: 19V output, 1.5A current limit");
    info!("Connect USB load to start power delivery");
    info!("OTG output will toggle every 10 seconds using PSTOP pin");

    // Variables for OTG toggle control
    let mut otg_enabled = true; // Start with OTG enabled
    let mut last_toggle_time = embassy_time::Instant::now();
    let toggle_interval = Duration::from_secs(10);

    // Main monitoring loop
    loop {
        // Check if it's time to toggle OTG state
        let current_time = embassy_time::Instant::now();
        if current_time.duration_since(last_toggle_time) >= toggle_interval {
            otg_enabled = !otg_enabled;
            last_toggle_time = current_time;

            if otg_enabled {
                info!("=== ENABLING OTG OUTPUT (PSTOP = LOW) ===");
                pstop.set_low(); // Enable SC8815 (PSTOP = Low)

                // Re-enable OTG mode after enabling the chip
                Timer::after(Duration::from_millis(100)).await; // Wait for chip to stabilize
                if let Err(e) = sc8815.set_otg_mode(true).await {
                    error!("Failed to enable OTG mode: {:?}", e);

                    // SAFETY: If we can't configure OTG mode, disable SC8815 for safety
                    error!("SAFETY: Disabling SC8815 due to OTG mode configuration failure");
                    pstop.set_high(); // Disable SC8815 chip for safety

                    // Blink LED rapidly to indicate communication error
                    loop {
                        led.toggle();
                        Timer::after(Duration::from_millis(250)).await;
                    }
                } else {
                    info!("OTG mode enabled - 19V output active");
                }
            } else {
                info!("=== DISABLING OTG OUTPUT (PSTOP = HIGH) ===");
                pstop.set_high(); // Disable SC8815 (PSTOP = High)
                info!("SC8815 disabled - No output");
            }
        }
        // Only check device status when OTG is enabled
        if otg_enabled {
            // Read basic device status
            let status = match sc8815.get_device_status().await {
            Ok(status) => {
                if status.ac_adapter_connected {
                    info!("AC adapter connected - Warning: OTG mode is active");

                    // In this example we stay in OTG mode regardless of AC adapter
                    // But we could implement charging mode switching here if needed
                } else {
                    info!("AC adapter disconnected");
                }

                // Check if USB load is detected (power is being drawn)
                if status.usb_load_detected {
                    info!("USB load detected - Providing 19V output power");

                    // Ensure OTG mode is enabled
                    if let Ok(false) = sc8815.is_otg_mode().await {
                        info!("Re-enabling OTG mode");
                        if let Err(e) = sc8815.set_otg_mode(true).await {
                            error!("Failed to enable OTG mode: {:?}", e);

                            // SAFETY: If we can't configure OTG mode, disable SC8815 for safety
                            error!("SAFETY: Disabling SC8815 due to OTG mode re-enable failure");
                            pstop.set_high(); // Disable SC8815 chip for safety

                            // Blink LED rapidly to indicate communication error
                            loop {
                                led.toggle();
                                Timer::after(Duration::from_millis(250)).await;
                            }
                        }
                    }
                } else {
                    info!("No USB load detected - OTG mode idle");
                }

                // Check for faults
                if status.otp_fault {
                    error!("Over-temperature protection fault detected!");
                }

                if status.vbus_short_fault {
                    error!("VBUS short circuit fault detected! Attempting official recovery method...");

                    // Use official recommended method to clear VBUS short fault
                    match sc8815.clear_vbus_short_fault_with_delay(|| async {
                        Timer::after(Duration::from_millis(10)).await;
                    }).await {
                        Ok(()) => info!("VBUS short fault recovery attempted"),
                        Err(e) => error!("Failed to clear VBUS short fault: {:?}", e),
                    }
                }

                status
            }
            Err(e) => {
                error!("Failed to read device status: {:?}", e);

                // SAFETY: If we can't communicate with SC8815, disable it for safety
                error!("SAFETY: Disabling SC8815 due to status read failure");
                pstop.set_high(); // Disable SC8815 chip for safety

                // Blink LED rapidly to indicate communication error
                loop {
                    led.toggle();
                    Timer::after(Duration::from_millis(250)).await;
                }
            }
        };

        // Read ADC measurements (using the configuration stored in the driver)
        match sc8815.get_adc_measurements().await {
            Ok(measurements) => {
                info!("ADC Measurements:");
                info!("  VBUS: {}mV", measurements.vbus_mv);
                info!("  VBAT: {}mV", measurements.vbat_mv);
                info!("  IBUS: {}mA", measurements.ibus_ma);
                info!("  IBAT: {}mA", measurements.ibat_ma);
                info!("  ADIN: {}mV", measurements.adin_mv);

                // Debug: Read raw IBUS registers for troubleshooting using optimized consecutive read
                if let Ok((ibus_high, ibus_low)) = sc8815.read_consecutive_registers(sc8815::registers::Register::IbusValue).await {
                    let ibus_value2 = (ibus_low >> 6) & 0x03;
                    info!("Raw IBUS registers: IBUS_VALUE={}, IBUS_VALUE2={}", ibus_high, ibus_value2);
                }
            }
            Err(e) => error!("Failed to read ADC measurements: {:?}", e),
        }



        // Check if we're actually in OTG mode
        match sc8815.is_otg_mode().await {
            Ok(is_otg) => {
                if !is_otg {
                    info!("OTG mode disabled - re-enabling");
                    if let Err(e) = sc8815.set_otg_mode(true).await {
                        error!("Failed to re-enable OTG mode: {:?}", e);

                        // SAFETY: If we can't configure OTG mode, disable SC8815 for safety
                        error!("SAFETY: Disabling SC8815 due to OTG mode re-enable failure");
                        pstop.set_high(); // Disable SC8815 chip for safety

                        // Blink LED rapidly to indicate communication error
                        loop {
                            led.toggle();
                            Timer::after(Duration::from_millis(250)).await;
                        }
                    }
                } else {
                    // LED pattern: fast blink when providing power, slow blink when idle
                    if status.usb_load_detected {
                        // Fast blink when providing power
                        led.toggle();
                        Timer::after(Duration::from_millis(100)).await;
                        led.toggle();
                        Timer::after(Duration::from_millis(100)).await;
                    } else {
                        // Slow blink when idle
                        led.toggle();
                    }
                }
            },
            Err(e) => {
                error!("Failed to check OTG mode: {:?}", e);

                // SAFETY: If we can't communicate with SC8815, disable it for safety
                error!("SAFETY: Disabling SC8815 due to OTG mode check failure");
                pstop.set_high(); // Disable SC8815 chip for safety

                // Blink LED rapidly to indicate communication error
                loop {
                    led.toggle();
                    Timer::after(Duration::from_millis(250)).await;
                }
            },
        }
        } else {
            // OTG is disabled - just blink LED slowly to show we're alive
            led.toggle();
            info!("OTG disabled - SC8815 is in standby mode");
        }

        info!("----------------------------");

        // Wait before next measurement
        Timer::after(Duration::from_millis(800)).await;
    }
}
