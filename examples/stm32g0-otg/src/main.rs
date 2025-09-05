#![no_std]
#![no_main]

// SC8815 STM32G0 OTG Mode Example
// Configured for configurable output voltage and current limits
// This example demonstrates using SC8815 in reverse discharge (OTG) mode
// Hardware configuration:
// - LED: PB8
// - I2C: PB6 (SCL), PB7 (SDA)
// - Battery: 4S Li-ion (4.2V per cell)
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

// === DEMO CONFIGURATION ===
// 修改这些常量来配置演示行为
#[derive(Clone, Copy, PartialEq)]
enum DemoMode {
    AlwaysOn,  // 一直输出模式 - OTG始终启用
    Toggle,    // 切换输出模式 - 定期启用/禁用OTG
}

const DEMO_MODE: DemoMode = DemoMode::Toggle;  // 修改这里来切换演示模式
const OUTPUT_VOLTAGE_MV: u16 = 19000;          // 输出电压（毫伏）- 可修改为5000, 9000, 12000, 15000等
const TOGGLE_INTERVAL_SECS: u64 = 10;         // 切换间隔（秒）- 仅在Toggle模式下有效

// === CURRENT LIMIT CONFIGURATION ===
const IBUS_LIMIT_MA: u16 = 6000;              // VBUS侧电流限制（毫安）- 输出电流限制
const IBAT_LIMIT_MA: u16 = 10000;              // VBAT侧电流限制（毫安）- 电池侧电流限制
const RS1_MOHM: u16 = 5;                      // VBUS侧电流检测电阻（毫欧）
const RS2_MOHM: u16 = 5;                      // VBAT侧电流检测电阻（毫欧）

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<I2C1>, i2c::ErrorInterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("SC8815 STM32G0 OTG Mode Example Starting");
    match DEMO_MODE {
        DemoMode::AlwaysOn => info!("Demo Mode: ALWAYS ON - OTG output continuously enabled"),
        DemoMode::Toggle => info!("Demo Mode: TOGGLE - OTG output switches every {} seconds", TOGGLE_INTERVAL_SECS),
    }
    info!("Configuration: 4S Li-ion, {}V output, {}A/{}A current limits, 450kHz switching",
          OUTPUT_VOLTAGE_MV / 1000, IBUS_LIMIT_MA as f32 / 1000.0, IBAT_LIMIT_MA as f32 / 1000.0);

    // Configure LED for status indication on PB8
    let mut led = Output::new(p.PB8, Level::High, Speed::Low);

    // Configure PSTOP pin on PB5 - CRITICAL SAFETY CONTROL
    // PSTOP = HIGH: Standby mode (power blocks disabled, I2C active) - SAFE for configuration
    // PSTOP = LOW: Active mode (power blocks enabled) - ONLY after complete configuration
    // IMPORTANT: SC8815 default factory configuration may cause hardware damage!
    let mut pstop = Output::new(p.PB5, Level::High, Speed::Low); // Start in SAFE standby mode

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
    info!("PSTOP is HIGH - SC8815 in SAFE standby mode for configuration");

    // Try to communicate with SC8815 in standby mode
    let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);
    info!("Testing I2C communication with SC8815 in standby mode...");
    match sc8815.read_register(sc8815::registers::Register::Status).await {
        Ok(status) => {
            info!("I2C communication successful in standby mode, status: 0x{:02X}", status);
        }
        Err(e) => {
            error!("I2C communication failed: {:?}", e);

            // SAFETY: PSTOP already HIGH (standby mode) - keep it that way
            error!("SAFETY: Keeping SC8815 in standby mode due to I2C failure");
            // pstop is already high, no need to change it

            // Blink LED slowly to indicate I2C error with SC8815
            loop {
                led.toggle();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }

    info!("Initializing SC8815 in standby mode...");
    match sc8815.init().await {
        Ok(()) => {
            info!("SC8815 initialized successfully in standby mode");
        }
        Err(e) => {
            error!("Failed to initialize SC8815: {:?}", e);

            // SAFETY: PSTOP already HIGH (standby mode) - keep it that way
            error!("SAFETY: Keeping SC8815 in standby mode due to initialization failure");

            // Blink LED slowly to indicate init error
            loop {
                led.toggle();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }

    // Configure the device for OTG (reverse discharge) mode with configurable limits
    let mut config = DeviceConfiguration::default();

    // Configure for 4S Li-ion battery using internal voltage setting
    config.battery.cell_count = CellCount::Cells4S;
    config.battery.voltage_per_cell = VoltagePerCell::Mv4200;
    config.battery.use_internal_setting = true;

    // Configure current limits with configurable sense resistors
    config.current_limits.rs1_mohm = RS1_MOHM;
    config.current_limits.rs2_mohm = RS2_MOHM;
    config.current_limits.ibus_limit_ma = IBUS_LIMIT_MA;
    config.current_limits.ibat_limit_ma = IBAT_LIMIT_MA;

    // Configure power settings
    config.power.operating_mode = OperatingMode::OTG;
    config.power.switching_frequency = SwitchingFrequency::Freq450kHz;
    config.power.dead_time = DeadTime::Ns80;
    config.power.vinreg_voltage_mv = OUTPUT_VOLTAGE_MV;

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

    info!("Configuring SC8815 for OTG mode ({}V output, {}A limit) in standby...",
          OUTPUT_VOLTAGE_MV / 1000, IBUS_LIMIT_MA as f32 / 1000.0);
    if let Err(e) = sc8815.configure_device(&config).await {
        error!("Failed to configure SC8815: {:?}", e);

        // SAFETY: PSTOP already HIGH (standby mode) - keep it that way
        error!("SAFETY: Keeping SC8815 in standby mode due to configuration failure");

        // Blink LED rapidly to indicate configuration error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("SC8815 configured successfully for OTG mode in standby");
    }

    // Configure OTG mode while in standby
    info!("Configuring OTG mode in standby...");
    if let Err(e) = sc8815.set_otg_mode(true).await {
        error!("Failed to configure OTG mode: {:?}", e);

        // SAFETY: PSTOP already HIGH (standby mode) - keep it that way
        error!("SAFETY: Keeping SC8815 in standby mode due to OTG mode configuration failure");

        // Blink LED rapidly to indicate OTG mode error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("OTG mode configured successfully in standby");
    }

    // Set VBUS output voltage for OTG mode
    info!("Setting VBUS output voltage to {}V...", OUTPUT_VOLTAGE_MV / 1000);
    if let Err(e) = sc8815.set_vbus_internal_voltage(OUTPUT_VOLTAGE_MV, 0).await {
        error!("Failed to set VBUS voltage: {:?}", e);

        // SAFETY: PSTOP already HIGH (standby mode) - keep it that way
        error!("SAFETY: Keeping SC8815 in standby mode due to VBUS voltage setting failure");

        // Blink LED rapidly to indicate VBUS voltage error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("VBUS voltage set to {}V successfully", OUTPUT_VOLTAGE_MV / 1000);
    }

    // Enable ADC conversion while in standby
    if let Err(e) = sc8815.set_adc_conversion(true).await {
        error!("Failed to configure ADC conversion: {:?}", e);

        // SAFETY: PSTOP already HIGH (standby mode) - keep it that way
        error!("SAFETY: Keeping SC8815 in standby mode due to ADC configuration failure");

        // Blink LED rapidly to indicate ADC error
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(250)).await;
        }
    } else {
        info!("ADC conversion configured in standby mode");
    }

    // All configuration completed successfully in standby mode
    // NOW it's safe to enable the power blocks
    info!("🔧 Configuration complete - NOW enabling power blocks (PSTOP LOW)");
    pstop.set_low(); // Enable power blocks - CRITICAL TIMING!
    Timer::after(Duration::from_millis(10)).await; // Wait for power blocks to stabilize

    info!("✅ SC8815 configured as power bank: {}V output, {}A/{}A current limits",
          OUTPUT_VOLTAGE_MV / 1000, IBUS_LIMIT_MA as f32 / 1000.0, IBAT_LIMIT_MA as f32 / 1000.0);
    info!("Connect USB load to start power delivery");
    match DEMO_MODE {
        DemoMode::AlwaysOn => info!("Demo Mode: OTG output will remain continuously enabled"),
        DemoMode::Toggle => info!("Demo Mode: OTG output will toggle every {} seconds", TOGGLE_INTERVAL_SECS),
    }

    // Variables for OTG toggle control
    let mut otg_enabled = true; // Start with OTG enabled (already configured)
    let mut last_toggle_time = embassy_time::Instant::now();
    let toggle_interval = Duration::from_secs(TOGGLE_INTERVAL_SECS);

    // Main monitoring loop
    loop {
        // Check if it's time to toggle OTG state (only in Toggle mode)
        if DEMO_MODE == DemoMode::Toggle {
            let current_time = embassy_time::Instant::now();
            if current_time.duration_since(last_toggle_time) >= toggle_interval {
                otg_enabled = !otg_enabled;
                last_toggle_time = current_time;

                if otg_enabled {
                    info!("=== ENABLING OTG OUTPUT ===");
                    if let Err(e) = sc8815.set_otg_mode(true).await {
                        error!("Failed to enable OTG mode: {:?}", e);

                        // SAFETY: If we can't communicate, disable power blocks for safety
                        error!("SAFETY: Disabling SC8815 power blocks due to communication failure");
                        pstop.set_high(); // Disable power blocks for safety

                        // Blink LED rapidly to indicate communication error
                        loop {
                            led.toggle();
                            Timer::after(Duration::from_millis(250)).await;
                        }
                    } else {
                        info!("OTG mode enabled - {}V output active", OUTPUT_VOLTAGE_MV / 1000);
                    }
                } else {
                    info!("=== DISABLING OTG OUTPUT ===");
                    if let Err(e) = sc8815.set_otg_mode(false).await {
                        error!("Failed to disable OTG mode: {:?}", e);

                        // SAFETY: If we can't communicate, disable power blocks for safety
                        error!("SAFETY: Disabling SC8815 power blocks due to communication failure");
                        pstop.set_high(); // Disable power blocks for safety
                    } else {
                        info!("OTG mode disabled - No output");
                    }
                }
            }
        }
        // Always check device status for monitoring and safety
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
                    info!("USB load detected - Providing {}V output power", OUTPUT_VOLTAGE_MV / 1000);

                    // Ensure OTG mode is enabled (but respect demo mode and toggle state)
                    if let Ok(false) = sc8815.is_otg_mode().await {
                        // Only re-enable if we're in AlwaysOn mode or (Toggle mode and otg_enabled is true)
                        let should_enable = match DEMO_MODE {
                            DemoMode::AlwaysOn => true,
                            DemoMode::Toggle => otg_enabled,
                        };

                        if should_enable {
                            info!("Re-enabling OTG mode (demo mode allows it)");
                            if let Err(e) = sc8815.set_otg_mode(true).await {
                                error!("Failed to enable OTG mode: {:?}", e);

                                // SAFETY: If we can't configure OTG mode, disable power blocks for safety
                                error!("SAFETY: Disabling SC8815 power blocks due to OTG mode re-enable failure");
                                pstop.set_high(); // Disable power blocks for safety

                                // Blink LED rapidly to indicate communication error
                                loop {
                                    led.toggle();
                                    Timer::after(Duration::from_millis(250)).await;
                                }
                            }
                        } else {
                            info!("OTG mode disabled by toggle logic - not re-enabling");
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
                    // Only re-enable if we're in AlwaysOn mode or (Toggle mode and otg_enabled is true)
                    let should_enable = match DEMO_MODE {
                        DemoMode::AlwaysOn => true,
                        DemoMode::Toggle => otg_enabled,
                    };

                    if should_enable {
                        info!("OTG mode disabled - re-enabling (demo mode allows it)");
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
                        // In Toggle mode and otg_enabled is false - this is expected
                        // LED pattern: slow blink when intentionally disabled
                        led.toggle();
                    }
                } else {
                    // OTG is enabled - LED pattern: fast blink when providing power, slow blink when idle
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

                // SAFETY: If we can't communicate with SC8815, disable power blocks for safety
                error!("SAFETY: Disabling SC8815 power blocks due to communication failure");
                pstop.set_high(); // Disable power blocks for safety

                // Blink LED rapidly to indicate communication error
                loop {
                    led.toggle();
                    Timer::after(Duration::from_millis(250)).await;
                }
            },
        };

        info!("----------------------------");

        // Wait before next measurement
        Timer::after(Duration::from_millis(800)).await;
    }
}
