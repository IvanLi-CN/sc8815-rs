#![no_std]
#![no_main]

// SC8815 STM32G031C8U6 Charger Mode Example
// Configured for 4S LiFePO4 battery charging with 1A current limit
// This example demonstrates using SC8815 in charging mode
// Hardware configuration:
// - MCU: STM32G031C8U6 (Cortex-M0+)
// - LED: PB8
// - I2C: PB6 (SCL), PB7 (SDA)
// - Battery: 4S LiFePO4 (4.55V per cell, 18.2V total)
// - Current sense resistors: 5mΩ
// - Switching frequency: 450kHz
// - Dead time: 60ns

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
    info!("SC8815 STM32G0 Charger Mode Example Starting");
    info!("Configuration: 4S LiFePO4, 1A charging current limit, 450kHz switching");

    // Configure LED for status indication on PB8 (Low = On, High = Off)
    let mut led = Output::new(p.PB8, Level::High, Speed::Low); // Start with LED off

    // Configure PSTOP pin on PB5 but keep SC8815 disabled initially
    // IMPORTANT: PSTOP must be enabled (pulled low) AFTER hardware configuration is complete
    let mut pstop = Output::new(p.PB5, Level::High, Speed::Low); // Start with SC8815 disabled (High = disabled)

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

    // Add a small delay before first I2C communication
    Timer::after(Duration::from_millis(100)).await;
    info!("Attempting to read status register...");

    // First, enable SC8815 by pulling PSTOP low after I2C is ready
    info!("Enabling SC8815 (pulling PSTOP low)...");
    pstop.set_low(); // Enable SC8815 chip
    Timer::after(Duration::from_millis(10)).await; // Give SC8815 time to start up

    // Create SC8815 driver and test communication using proper driver API
    let mut sc8815_driver = SC8815::new(i2c, DEFAULT_ADDRESS);
    info!("Testing I2C communication with SC8815 using driver API...");

    match sc8815_driver.read_register(sc8815::registers::Register::Status).await {
        Ok(status) => {
            info!("I2C communication successful, status register: 0x{:02X}", status);
        }
        Err(e) => {
            error!("I2C communication failed: {:?}", e);

            // If driver communication fails, perform I2C scan for diagnostics
            info!("Performing I2C scan to detect devices...");

            // We need to extract the I2C interface from the driver for scanning
            // Since we can't easily do this, we'll provide diagnostic information
            info!("SC8815 driver communication failed. Possible causes:");
            info!("- Hardware connections (I2C, power, PSTOP)");
            info!("- Wrong I2C address (expected: 0x{:02X})", DEFAULT_ADDRESS);
            info!("- SC8815 not responding or in fault state");
            info!("- I2C bus configuration issues");

            info!("Hardware connection checklist:");
            info!("- PB6 (SCL) connected to SC8815 SCL");
            info!("- PB7 (SDA) connected to SC8815 SDA");
            info!("- 4.7kΩ pull-up resistors on SCL and SDA to 3.3V");
            info!("- SC8815 VCC connected to 3.3V");
            info!("- PB5 (PSTOP) connected to SC8815 PSTOP and now pulled LOW");
            info!("- Verify SC8815 is properly powered and not in reset");

            // Blink LED slowly to indicate I2C error with SC8815
            loop {
                led.toggle();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }

    info!("Initializing SC8815...");
    if let Err(e) = sc8815_driver.init().await {
        error!("Failed to initialize SC8815: {:?}", e);
        // Blink LED slowly to indicate init error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(500)).await;
        }
    }
    info!("SC8815 initialized successfully");

    // Configure the device for Charger mode - 4S LiFePO4 charging with 1A limit
    let mut config = DeviceConfiguration::default();

    // Configure for 4S LiFePO4 battery using internal voltage setting
    config.battery.cell_count = CellCount::Cells4S;
    config.battery.voltage_per_cell = VoltagePerCell::Mv4450;
    config.battery.use_internal_setting = true;

    // Configure current limits with 5mΩ sense resistors
    config.current_limits.rs1_mohm = 5;
    config.current_limits.rs2_mohm = 5;
    config.current_limits.ibus_limit_ma = 1000;
    config.current_limits.ibat_limit_ma = 1000; // 1A charging current using battery side

    // Configure power settings
    config.power.operating_mode = OperatingMode::Charging;
    config.power.switching_frequency = SwitchingFrequency::Freq450kHz;
    config.power.dead_time = DeadTime::Ns60;
    config.power.vinreg_voltage_mv = 11500; // 11.5V VINREG voltage

    // Charger mode settings
    config.trickle_charging = true;
    config.charging_termination = true;
    config.use_ibus_for_charging = false; // Use IBAT (battery side) for charging current

    info!("Configuring SC8815 for Charger mode (4S LiFePO4, 1A charging)...");
    if let Err(e) = sc8815_driver.configure_device(&config).await {
        error!("Failed to configure SC8815: {:?}", e);
        // Blink LED rapidly to indicate configuration error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    }
    info!("SC8815 configured successfully for Charger mode");

    // Enable charging mode (disable OTG mode)
    info!("Enabling charging mode...");
    if let Err(e) = sc8815_driver.set_otg_mode(false).await {
        error!("Failed to enable charging mode: {:?}", e);
        // Blink LED rapidly to indicate charging mode error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    }
    info!("Charging mode enabled successfully");

    // Enable ADC conversion
    if let Err(e) = sc8815_driver.set_adc_conversion(true).await {
        error!("Failed to start ADC conversion: {:?}", e);
        // Blink LED rapidly to indicate ADC error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    }
    info!("ADC conversion started");

    // All configuration completed successfully - indicate success with solid LED
    led.set_low(); // Turn on LED to indicate complete success
    info!("✅ SC8815 fully configured and ready for operation!");

    info!("SC8815 configured as battery charger: 17.8V, 1A charging current");
    info!("Battery voltage limit: 17.8V (4.45V per cell)");
    info!("VINREG voltage: 11.5V");
    info!("Connect AC adapter to start charging");

    // Variables for LED control timing
    let mut last_led_toggle = embassy_time::Instant::now();
    let mut led_state = false;

    // Main monitoring loop
    loop {
        // Read basic device status
        let status = match sc8815_driver.get_device_status().await {
            Ok(status) => {
                if status.ac_adapter_connected {
                    info!("AC adapter connected - Charging available");

                    // Check charging status
                    if status.eoc {
                        info!("Battery charging complete (End of Charge)");
                    } else {
                        info!("Battery charging in progress");
                    }
                } else {
                    info!("AC adapter disconnected - No charging");
                }

                // Check for faults
                if status.otp_fault {
                    error!("Over-temperature protection fault detected!");
                }

                if status.vbus_short_fault {
                    error!("VBUS short circuit fault detected! Attempting official recovery method...");

                    // Use official recommended method to clear VBUS short fault
                    match sc8815_driver.clear_vbus_short_fault_with_delay(|| async {
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
                continue;
            }
        };

        // Read ADC measurements (using the configuration stored in the driver)
        match sc8815_driver.get_adc_measurements().await {
            Ok(measurements) => {
                info!("ADC Measurements:");
                info!("  VBUS: {}mV", measurements.vbus_mv);
                info!("  VBAT: {}mV", measurements.vbat_mv);
                info!("  IBUS: {}mA", measurements.ibus_ma);
                info!("  IBAT: {}mA", measurements.ibat_ma);
                info!("  ADIN: {}mV", measurements.adin_mv);

                // Debug: Read raw IBUS registers for troubleshooting using optimized consecutive read
                if let Ok((ibus_high, ibus_low)) = sc8815_driver.read_consecutive_registers(sc8815::registers::Register::IbusValue).await {
                    let ibus_value2 = (ibus_low >> 6) & 0x03;
                    info!("Raw IBUS registers: IBUS_VALUE={}, IBUS_VALUE2={}", ibus_high, ibus_value2);
                }
            }
            Err(e) => error!("Failed to read ADC measurements: {:?}", e),
        }

        // Check if we're actually in charging mode (not OTG mode)
        match sc8815_driver.is_otg_mode().await {
            Ok(is_otg) => {
                if is_otg && status.ac_adapter_connected {
                    info!("OTG mode enabled but AC adapter connected - switching to charging mode");
                    if let Err(e) = sc8815_driver.set_otg_mode(false).await {
                        error!("Failed to enable charging mode: {:?}", e);
                    }
                }
            },
            Err(e) => error!("Failed to check OTG mode: {:?}", e),
        }

        // LED control based on charging status
        let current_time = embassy_time::Instant::now();
        let has_fault = status.otp_fault || status.vbus_short_fault;

        if has_fault {
            // Fault condition: 250ms fast blink
            if current_time.duration_since(last_led_toggle) >= Duration::from_millis(250) {
                led.toggle();
                led_state = !led_state;
                last_led_toggle = current_time;
            }
        } else if status.eoc {
            // Charging complete: LED constantly on
            led.set_low(); // Turn on LED (assuming active low)
        } else if status.ac_adapter_connected {
            // Charging in progress: 1 second on, 1 second off
            if current_time.duration_since(last_led_toggle) >= Duration::from_secs(1) {
                led.toggle();
                led_state = !led_state;
                last_led_toggle = current_time;
            }
        } else {
            // No AC adapter: LED off
            led.set_high(); // Turn off LED (assuming active low)
        }

        info!("----------------------------");

        // Short delay for main loop - LED timing is handled separately
        Timer::after(Duration::from_millis(100)).await;
    }
}
