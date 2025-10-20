#![no_std]
#![no_main]

// SC8815 + SW2303 STM32G031G8U6 FB Regulation Demo
// Demonstrates power bank configuration with SC8815 regulating VBUS through an external FB divider
// and SW2303 handling protocol negotiation / monitoring on the same bus.

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    i2c::{self, Config as I2cConfig, I2c},
    mode::Async as I2cAsyncMode,
    peripherals::I2C1,
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use sc8815::{
    CellCount, DeadTime, DeviceConfiguration, IbusRatio, OperatingMode, SC8815, SwitchingFrequency,
    VoltagePerCell, VbusRatio, registers::constants::DEFAULT_ADDRESS as SC8815_DEFAULT_ADDRESS,
};
use sw2303::{
    ProtocolType, SW2303,
    registers::constants::DEFAULT_ADDRESS as SW2303_DEFAULT_ADDRESS,
};

// External FB divider parameters: Rp = 100kΩ, Rd = 31.6kΩ
const FB_R_UP_OHM: u32 = 100_000;
const FB_R_DOWN_OHM: u32 = 31_600;
const FB_REFERENCE_MV: u16 = 1200; // Reference voltage applied to FB network
const TARGET_VBUS_MV: u16 = 5000;  // Expected output voltage after resistor scaling

const SC_IBUS_LIMIT_MA: u16 = 3_500;
const SC_IBAT_LIMIT_MA: u16 = 2_000;
const SC_RSENSE_MOHM: u16 = 5;

const SW_LOG_INTERVAL: Duration = Duration::from_millis(500);
const SC_CONFIG_TO_SW_DELAY: Duration = Duration::from_millis(500);

type SharedI2cBus = Mutex<NoopRawMutex, I2c<'static, I2cAsyncMode>>;

static I2C_BUS: StaticCell<SharedI2cBus> = StaticCell::new();

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<I2C1>, i2c::ErrorInterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("SC8815 + SW2303 FB regulation demo starting");

    // Status LED on PB8 (active low)
    let mut led = Output::new(p.PB8, Level::High, Speed::Low);

    // PSTOP on PB5 keeps SC8815 in standby until configuration is complete.
    let mut pstop = Output::new(p.PB5, Level::High, Speed::Low);
    info!("PSTOP held HIGH - SC8815 in standby for safe configuration");

    // Configure I2C1 (PB6 = SCL, PB7 = SDA) @ 100 kHz with internal pull-ups.
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;

    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Hertz(100_000),
        i2c_cfg,
    );

    // Share I2C bus between SC8815 and SW2303.
    let bus_handle: &'static SharedI2cBus = I2C_BUS.init(Mutex::new(i2c));

    let sc_i2c = I2cDevice::new(bus_handle);
    let mut sc8815 = SC8815::new(sc_i2c, SC8815_DEFAULT_ADDRESS);

    let mut sw_i2c = I2cDevice::new(bus_handle);
    let mut sw2303 = SW2303::new(&mut sw_i2c, SW2303_DEFAULT_ADDRESS);

    Timer::after(Duration::from_millis(100)).await;

    info!("Initializing SC8815 (address 0x{:02X})", SC8815_DEFAULT_ADDRESS);
    if let Err(e) = sc8815.init().await {
        error!("SC8815 init failed: {:?}", e);
        signal_error(&mut led).await;
    }

    info!("Preparing SC8815 device configuration for FB-regulated OTG output");
    let mut config = DeviceConfiguration::default();
    config.battery.cell_count = CellCount::Cells4S;
    config.battery.voltage_per_cell = VoltagePerCell::Mv4200;
    config.battery.use_internal_setting = true;
    config.trickle_charging = false;
    config.charging_termination = false;
    config.use_ibus_for_charging = false;

    config.current_limits.ibus_limit_ma = SC_IBUS_LIMIT_MA;
    config.current_limits.ibat_limit_ma = SC_IBAT_LIMIT_MA;
    config.current_limits.ibus_ratio = IbusRatio::Ratio6x;
    config.current_limits.rs1_mohm = SC_RSENSE_MOHM;
    config.current_limits.rs2_mohm = SC_RSENSE_MOHM;

    config.power.operating_mode = OperatingMode::OTG;
    config.power.switching_frequency = SwitchingFrequency::Freq450kHz;
    config.power.dead_time = DeadTime::Ns80;
    config.power.frequency_dithering = false;
    config.power.pfm_mode = false;
    config.power.vbus_ratio = VbusRatio::Ratio12_5x;

    if let Err(e) = sc8815.set_short_foldback_disable(true).await {
        warn!("Failed to disable short foldback: {:?}", e);
    }

    if let Err(e) = sc8815.configure_device(&config).await {
        error!("SC8815 configure_device failed: {:?}", e);
        signal_error(&mut led).await;
    }

    info!("Setting SC8815 external feedback reference to {}mV", FB_REFERENCE_MV);
    if let Err(e) = sc8815.set_vbus_external_reference(FB_REFERENCE_MV).await {
        error!("Failed to set external VBUS reference: {:?}", e);
        signal_error(&mut led).await;
    }

    if let Ok(vref_e_set) = sc8815
        .read_register(sc8815::registers::Register::VbusrefESet)
        .await
    {
        if let Ok(vref_e_set2) = sc8815
            .read_register(sc8815::registers::Register::VbusrefESet2)
            .await
        {
            info!(
                "VBUSREF_E registers: SET=0x{:02X}, SET2=0x{:02X}",
                vref_e_set, vref_e_set2
            );
        }
    }

    if let Err(e) = sc8815.set_vbus_slew_rate(1).await {
        warn!("Unable to configure SC8815 slew rate: {:?}", e);
    }

    if let Err(e) = sc8815.set_adc_conversion(true).await {
        error!("Failed to start SC8815 ADC conversion: {:?}", e);
        signal_error(&mut led).await;
    }

    info!("SC8815 configuration complete, enabling power blocks (PSTOP LOW)");
    pstop.set_low();
    Timer::after(Duration::from_millis(10)).await;

    info!(
        "✅ SC8815 configured as power bank: expected {}V output (external FB), {}A/{}A current limits",
        TARGET_VBUS_MV / 1000,
        SC_IBUS_LIMIT_MA as f32 / 1000.0,
        SC_IBAT_LIMIT_MA as f32 / 1000.0
    );
    info!(
        "External FB divider: Rp={}Ω, Rd={}Ω, reference {}mV",
        FB_R_UP_OHM, FB_R_DOWN_OHM, FB_REFERENCE_MV
    );

    if let Ok(ratio_reg) = sc8815
        .read_register(sc8815::registers::Register::Ratio)
        .await
    {
        info!("RATIO register: 0x{:02X}", ratio_reg);
    }

    info!("Waiting {:?} before configuring SW2303", SC_CONFIG_TO_SW_DELAY);
    Timer::after(SC_CONFIG_TO_SW_DELAY).await;

    info!("Initializing SW2303 (address 0x{:02X})", SW2303_DEFAULT_ADDRESS);
    let mut sw2303_online = false;
    let mut sw2303_error_logged = false;

    if let Err(e) = sw2303.init().await {
        error!("SW2303 init failed: {:?}", e);
        signal_error(&mut led).await;
    } else {
        match sw2303.get_system_status0().await {
            Ok(flags) => {
                info!(
                    "SW2303 initial system status0=0x{:02X}",
                    flags.bits()
                );
                sw2303_online = true;
            }
            Err(e) => {
                warn!(
                    "SW2303 not responding to initial status read (will retry): {:?}",
                    e
                );
            }
        }
    }

    info!("Configuration finished; entering monitoring loop");

    loop {
        led.toggle();

        let (sc_vbus_mv, sc_vbat_mv, sc_ibus_ma, sc_ibat_ma, _) = match sc8815.read_all_adc_values().await {
            Ok(values) => values,
            Err(e) => {
                error!("SC8815 ADC read failed: {:?}", e);
                Timer::after(SW_LOG_INTERVAL).await;
                continue;
            }
        };

        let mut sw_vbus_mv: u32 = 0;
        let mut sw_ibus_ma: u32 = 0;
        let mut negotiated: Option<ProtocolType> = None;

        if sw2303_online {
            match sw2303.read_vbus_mv_12bit().await {
                Ok(v) => sw_vbus_mv = v,
                Err(e) => {
                    warn!("SW2303 VBUS read failed: {:?} (marking device offline)", e);
                    sw2303_online = false;
                    sw2303_error_logged = false;
                }
            }

            if sw2303_online {
                match sw2303.read_ich_ma_12bit().await {
                    Ok(i) => sw_ibus_ma = i,
                    Err(e) => {
                        warn!(
                            "SW2303 IBUS read failed: {:?} (marking device offline)",
                            e
                        );
                        sw2303_online = false;
                        sw2303_error_logged = false;
                    }
                }
            }

            if sw2303_online {
                match sw2303.get_negotiated_protocol().await {
                    Ok(proto) => negotiated = proto,
                    Err(e) => {
                        warn!(
                            "SW2303 protocol query failed: {:?} (marking device offline)",
                            e
                        );
                        sw2303_online = false;
                        sw2303_error_logged = false;
                    }
                }
            }
        }

        if !sw2303_online {
            match sw2303.get_system_status0().await {
                Ok(flags) => {
                    info!(
                        "SW2303 reconnected, system status0=0x{:02X}",
                        flags.bits()
                    );
                    sw2303_online = true;
                    sw2303_error_logged = false;
                }
                Err(e) => {
                    if !sw2303_error_logged {
                        warn!("SW2303 not responding (telemetry skipped): {:?}", e);
                        sw2303_error_logged = true;
                    }
                }
            }
        }

        info!(
            "SC8815 VBAT={}mV IBAT={}mA | VBUS={}mV IBUS={}mA || SW2303 VBUS={}mV IBUS={}mA | Protocol={:?}",
            sc_vbat_mv,
            sc_ibat_ma,
            sc_vbus_mv,
            sc_ibus_ma,
            sw_vbus_mv,
            sw_ibus_ma,
            negotiated
        );

        Timer::after(SW_LOG_INTERVAL).await;
    }
}

async fn signal_error(led: &mut Output<'_>) -> ! {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(250)).await;
    }
}
