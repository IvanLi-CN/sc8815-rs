#![no_std]
#![no_main]

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
    SC8815, DeviceConfiguration, BatteryConfiguration, CurrentLimitConfiguration,
    PowerConfiguration, OperatingMode, CellCount, IrCompensation, IbusRatio, IbatRatio,
    SwitchingFrequency, DeadTime, VinregRatio, VoltagePerCell, registers::constants::DEFAULT_ADDRESS,
};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("SC8815 STM32G474RE Example Starting");

    // Configure LED for status indication (PC13 - onboard LED for STM32G474RE)
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // Configure I2C pins - Use PB8/PB9 for STM32G474RE
    let scl = p.PB8;  // I2C1_SCL
    let sda = p.PB9;  // I2C1_SDA

    let mut i2c_config = Config::default();
    // Enable internal pull-ups for I2C lines
    i2c_config.scl_pullup = true;
    i2c_config.sda_pullup = true;

    let i2c = I2c::new(
        p.I2C1,
        scl,
        sda,
        Irqs,
        p.DMA1_CH5,
        p.DMA1_CH6,
        Hertz(100_000),
        i2c_config,
    );

    // Initialize SC8815 driver
    let mut sc8815 = SC8815::new(i2c, DEFAULT_ADDRESS);

    info!("Initializing SC8815...");
    match sc8815.init().await {
        Ok(()) => {
            info!("SC8815 initialized successfully");
            led.set_low(); // Turn on LED to indicate success
        }
        Err(e) => {
            error!("Failed to initialize SC8815: {:?}", e);
            // Blink LED rapidly to indicate error
            loop {
                led.toggle();
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }

    // Configure the device with example settings
    let config = DeviceConfiguration {
        battery: BatteryConfiguration {
            cell_count: CellCount::Cells4S,   // 4S battery pack
            voltage_per_cell: VoltagePerCell::Mv4200,        // 4.2V per cell
            use_internal_setting: true,       // Use internal voltage reference
            ir_compensation_mohm: IrCompensation::Mohm20, // 20mΩ IR compensation
        },
        current_limits: CurrentLimitConfiguration {
            ibus_limit_ma: 3000,              // 3A input current limit
            ibus_ratio: IbusRatio::Ratio3x,   // 3x ratio
            ibat_limit_ma: 2500,              // 2.5A battery current limit
            ibat_ratio: IbatRatio::Ratio12x,  // 12x ratio
            rs1_mohm: 10,                     // 10mΩ sense resistor
            rs2_mohm: 10,                     // 10mΩ sense resistor
        },
        power: PowerConfiguration {
            operating_mode: OperatingMode::Charging,
            switching_frequency: SwitchingFrequency::Freq300kHz, // 300kHz
            dead_time: DeadTime::Ns40,        // 40ns
            frequency_dithering: true,        // Enable dithering
            pfm_mode: false,                  // Disable PFM
            vinreg_voltage_mv: 4500,          // 4.5V VINREG
            vinreg_ratio: VinregRatio::Ratio100x, // 100x ratio
            // VBUS ratio for ADC/internal reference scaling (0: 12.5x, 1: 5x)
            vbus_ratio: sc8815::VbusRatio::Ratio12_5x,
        },
        trickle_charging: true,               // Enable trickle charging
        charging_termination: true,           // Enable auto-termination
        use_ibus_for_charging: true,          // Use IBUS for charging current
    };

    info!("Configuring SC8815...");
    if let Err(e) = sc8815.configure_device(&config).await {
        error!("Failed to configure SC8815: {:?}", e);
    } else {
        info!("SC8815 configured successfully");
    }

    // Enable ADC conversion
    if let Err(e) = sc8815.set_adc_conversion(true).await {
        error!("Failed to start ADC conversion: {:?}", e);
    } else {
        info!("ADC conversion started");
    }

    // Main monitoring loop
    loop {
        // Read device status
        match sc8815.get_device_status().await {
            Ok(status) => {
                info!("Device Status:");
                info!("  AC Adapter: {}", if status.ac_adapter_connected { "Connected" } else { "Disconnected" });
                info!("  USB Load: {}", if status.usb_load_detected { "Detected" } else { "Not detected" });
                info!("  Charging Complete: {}", if status.eoc { "Yes" } else { "No" });
                info!("  OTP Fault: {}", if status.otp_fault { "Yes" } else { "No" });
                info!("  VBUS Short: {}", if status.vbus_short_fault { "Yes" } else { "No" });
            }
            Err(e) => error!("Failed to read device status: {:?}", e),
        }

        // Read ADC measurements
        match sc8815.get_adc_measurements().await {
            Ok(measurements) => {
                info!("ADC Measurements:");
                info!("  VBUS: {}mV", measurements.vbus_mv);
                info!("  VBAT: {}mV", measurements.vbat_mv);
                info!("  IBUS: {}mA", measurements.ibus_ma);
                info!("  IBAT: {}mA", measurements.ibat_ma);
                info!("  ADIN: {}mV", measurements.adin_mv);
            }
            Err(e) => error!("Failed to read ADC measurements: {:?}", e),
        }

        // Check if charging is complete
        match sc8815.get_device_status().await {
            Ok(status) => {
                if status.eoc {
                    info!("Charging complete!");
                    // Slow blink to indicate charging complete
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_low();
                } else {
                    // Fast blink during normal operation
                    led.toggle();
                }
            }
            Err(e) => error!("Failed to check charging status: {:?}", e),
        }

        info!("----------------------------");

        // Wait before next measurement
        Timer::after(Duration::from_secs(2)).await;
    }
}
