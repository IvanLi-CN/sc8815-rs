//! SC8815 data type definitions.

/// Represents the operating mode of the SC8815 device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OperatingMode {
    /// Charging mode - battery is being charged from input source.
    Charging = 0,
    /// OTG (On-The-Go) mode - battery is discharged to provide output power.
    OTG = 1,
}

// Alias for backward compatibility
pub use OperatingMode::OTG as Discharging;

/// Represents the switching frequency setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SwitchingFrequency {
    /// 150kHz switching frequency.
    Freq150kHz = 0,
    /// 300kHz switching frequency.
    Freq300kHz = 1,
    /// 450kHz switching frequency.
    Freq450kHz = 3,
}

impl Default for SwitchingFrequency {
    fn default() -> Self {
        Self::Freq300kHz
    }
}

impl From<SwitchingFrequency> for u8 {
    fn from(freq: SwitchingFrequency) -> Self {
        freq as u8
    }
}

impl From<u8> for SwitchingFrequency {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Freq150kHz,
            1 => Self::Freq300kHz,
            3 => Self::Freq450kHz,
            _ => Self::Freq300kHz, // Default to 300kHz for invalid values
        }
    }
}

/// Represents the dead time setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeadTime {
    /// 20ns dead time.
    Ns20 = 0,
    /// 40ns dead time.
    Ns40 = 1,
    /// 60ns dead time.
    Ns60 = 2,
    /// 80ns dead time.
    Ns80 = 3,
}

impl Default for DeadTime {
    fn default() -> Self {
        Self::Ns20
    }
}

impl From<DeadTime> for u8 {
    fn from(dt: DeadTime) -> Self {
        dt as u8
    }
}

impl From<u8> for DeadTime {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Ns20,
            1 => Self::Ns40,
            2 => Self::Ns60,
            3 => Self::Ns80,
            _ => Self::Ns20, // Default to 20ns for invalid values
        }
    }
}

/// Represents the power state of the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerState {
    /// Power is off.
    Off,
    /// Power is on.
    On,
}

/// Represents the charging state of the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChargingState {
    /// Not charging.
    NotCharging,
    /// Pre-charging.
    PreCharging,
    /// Constant current charging.
    ConstantCurrent,
    /// Constant voltage charging.
    ConstantVoltage,
    /// Charging complete.
    Complete,
    /// Charging fault.
    Fault,
    /// Discharging (OTG mode).
    Discharging,
    /// Charging (normal charging mode).
    Charging,
}

/// Represents the battery connection status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BatteryStatus {
    /// No battery connected.
    NotConnected,
    /// Battery connected.
    Connected,
    /// Battery fault.
    Fault,
}

/// Represents the input source status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputSourceStatus {
    /// No input source.
    NotConnected,
    /// Input source connected.
    Connected,
    /// Input source fault.
    Fault,
}

/// Represents the thermal status of the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThermalStatus {
    /// Normal temperature.
    Normal,
    /// Temperature warning.
    Warning,
    /// Thermal shutdown.
    Shutdown,
}

/// Represents the SC8815 device status based on the actual status register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SC8815Status {
    /// End of charge (EOC) conditions are satisfied.
    pub eoc: bool,
    /// Over-temperature protection (OTP) fault.
    pub otp_fault: bool,
    /// VBUS short circuit fault in discharging mode.
    pub vbus_short_fault: bool,
    /// USB-A load insert detected at INDET pin.
    pub usb_load_detected: bool,
    /// AC adapter is inserted.
    pub ac_adapter_connected: bool,
}

/// Represents ADC measurement values from the SC8815.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcMeasurements {
    /// VBUS voltage in millivolts.
    pub vbus_mv: u16,
    /// VBAT voltage in millivolts.
    pub vbat_mv: u16,
    /// IBUS current in milliamps.
    pub ibus_ma: u16,
    /// IBAT current in milliamps.
    pub ibat_ma: u16,
    /// ADIN voltage in millivolts.
    pub adin_mv: u16,
}

/// Represents the overall status of the SC8815 device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceStatus {
    /// Power state of the device.
    pub power: PowerState,
    /// Charging state.
    pub charging: ChargingState,
    /// Battery status.
    pub battery: BatteryStatus,
    /// Input source status.
    pub input_source: InputSourceStatus,
    /// Thermal status.
    pub thermal: ThermalStatus,
    /// Whether power is good.
    pub power_good: bool,
    /// Whether overcurrent protection is active.
    pub overcurrent: bool,
    /// Whether overvoltage protection is active.
    pub overvoltage: bool,
}

impl Default for DeviceStatus {
    fn default() -> Self {
        Self {
            power: PowerState::Off,
            charging: ChargingState::NotCharging,
            battery: BatteryStatus::NotConnected,
            input_source: InputSourceStatus::NotConnected,
            thermal: ThermalStatus::Normal,
            power_good: false,
            overcurrent: false,
            overvoltage: false,
        }
    }
}

/// Battery cell count setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CellCount {
    /// 1S battery pack.
    Cells1S = 1,
    /// 2S battery pack.
    Cells2S = 2,
    /// 3S battery pack.
    Cells3S = 3,
    /// 4S battery pack.
    Cells4S = 4,
}

impl Default for CellCount {
    fn default() -> Self {
        Self::Cells1S
    }
}

impl From<CellCount> for u8 {
    fn from(count: CellCount) -> Self {
        count as u8
    }
}

impl From<u8> for CellCount {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::Cells1S,
            2 => Self::Cells2S,
            3 => Self::Cells3S,
            4 => Self::Cells4S,
            _ => Self::Cells1S, // Default to 1S for invalid values
        }
    }
}

/// IR compensation setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IrCompensation {
    /// No IR compensation.
    None = 0,
    /// 20mΩ IR compensation.
    Mohm20 = 20,
    /// 40mΩ IR compensation.
    Mohm40 = 40,
    /// 80mΩ IR compensation.
    Mohm80 = 80,
}

/// Voltage per cell setting for battery configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VoltagePerCell {
    /// 4.1V per cell (4100mV).
    Mv4100 = 4100,
    /// 4.2V per cell (4200mV) - Standard Li-ion.
    Mv4200 = 4200,
    /// 4.25V per cell (4250mV).
    Mv4250 = 4250,
    /// 4.3V per cell (4300mV).
    Mv4300 = 4300,
    /// 4.35V per cell (4350mV) - High voltage Li-ion.
    Mv4350 = 4350,
    /// 4.4V per cell (4400mV).
    Mv4400 = 4400,
    /// 4.45V per cell (4450mV) - LiFePO4.
    Mv4450 = 4450,
}

impl Default for VoltagePerCell {
    fn default() -> Self {
        Self::Mv4200
    }
}

impl From<VoltagePerCell> for u16 {
    fn from(voltage: VoltagePerCell) -> Self {
        voltage as u16
    }
}

impl From<u16> for VoltagePerCell {
    fn from(value: u16) -> Self {
        match value {
            4100 => Self::Mv4100,
            4200 => Self::Mv4200,
            4250 => Self::Mv4250,
            4300 => Self::Mv4300,
            4350 => Self::Mv4350,
            4400 => Self::Mv4400,
            4450 => Self::Mv4450,
            _ => Self::Mv4200, // Default to 4.2V for invalid values
        }
    }
}

impl Default for IrCompensation {
    fn default() -> Self {
        Self::None
    }
}

impl From<IrCompensation> for u8 {
    fn from(comp: IrCompensation) -> Self {
        comp as u8
    }
}

impl From<u8> for IrCompensation {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::None,
            20 => Self::Mohm20,
            40 => Self::Mohm40,
            80 => Self::Mohm80,
            _ => Self::None, // Default to no compensation for invalid values
        }
    }
}

/// Battery configuration for the SC8815.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryConfiguration {
    /// Battery cell count.
    pub cell_count: CellCount,
    /// Voltage per cell setting.
    pub voltage_per_cell: VoltagePerCell,
    /// Whether to use internal or external VBAT voltage setting.
    pub use_internal_setting: bool,
    /// IR compensation setting.
    pub ir_compensation_mohm: IrCompensation,
}

impl Default for BatteryConfiguration {
    fn default() -> Self {
        Self {
            cell_count: CellCount::Cells1S,
            voltage_per_cell: VoltagePerCell::Mv4200,
            use_internal_setting: true,
            ir_compensation_mohm: IrCompensation::None,
        }
    }
}

/// IBUS ratio setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IbusRatio {
    /// 6x ratio.
    Ratio6x = 1,
    /// 3x ratio.
    Ratio3x = 2,
}

impl Default for IbusRatio {
    fn default() -> Self {
        Self::Ratio3x
    }
}

impl From<IbusRatio> for u8 {
    fn from(ratio: IbusRatio) -> Self {
        ratio as u8
    }
}

impl From<u8> for IbusRatio {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::Ratio6x,
            2 => Self::Ratio3x,
            _ => Self::Ratio3x, // Default to 3x for invalid values
        }
    }
}

/// IBAT ratio setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IbatRatio {
    /// 6x ratio.
    Ratio6x = 0,
    /// 12x ratio.
    Ratio12x = 1,
}

impl Default for IbatRatio {
    fn default() -> Self {
        Self::Ratio12x
    }
}

impl From<IbatRatio> for u8 {
    fn from(ratio: IbatRatio) -> Self {
        ratio as u8
    }
}

impl From<u8> for IbatRatio {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Ratio6x,
            1 => Self::Ratio12x,
            _ => Self::Ratio12x, // Default to 12x for invalid values
        }
    }
}

/// Current limit configuration for the SC8815.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentLimitConfiguration {
    /// IBUS current limit in milliamps.
    pub ibus_limit_ma: u16,
    /// IBUS ratio setting.
    pub ibus_ratio: IbusRatio,
    /// IBAT current limit in milliamps.
    pub ibat_limit_ma: u16,
    /// IBAT ratio setting.
    pub ibat_ratio: IbatRatio,
    /// VBUS side current sense resistor in milliohms.
    pub rs1_mohm: u16,
    /// VBAT side current sense resistor in milliohms.
    pub rs2_mohm: u16,
}

impl Default for CurrentLimitConfiguration {
    fn default() -> Self {
        Self {
            ibus_limit_ma: 3000,
            ibus_ratio: IbusRatio::Ratio3x,
            ibat_limit_ma: 3000,
            ibat_ratio: IbatRatio::Ratio12x,
            rs1_mohm: 5, // 5mΩ (user's configuration)
            rs2_mohm: 5, // 5mΩ (user's configuration)
        }
    }
}

/// VINREG ratio setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VinregRatio {
    /// 100x ratio.
    Ratio100x = 0,
    /// 40x ratio.
    Ratio40x = 1,
}

impl Default for VinregRatio {
    fn default() -> Self {
        Self::Ratio100x
    }
}

impl From<VinregRatio> for u8 {
    fn from(ratio: VinregRatio) -> Self {
        ratio as u8
    }
}

impl From<u8> for VinregRatio {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Ratio100x,
            1 => Self::Ratio40x,
            _ => Self::Ratio100x, // Default to 100x for invalid values
        }
    }
}

/// Power management configuration for the SC8815.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerConfiguration {
    /// Operating mode (charging or discharging).
    pub operating_mode: OperatingMode,
    /// Switching frequency setting.
    pub switching_frequency: SwitchingFrequency,
    /// Dead time setting.
    pub dead_time: DeadTime,
    /// Whether frequency dithering is enabled.
    pub frequency_dithering: bool,
    /// Whether PFM mode is enabled for light load (discharging mode only).
    pub pfm_mode: bool,
    /// VINREG voltage in millivolts.
    pub vinreg_voltage_mv: u16,
    /// VINREG ratio setting.
    pub vinreg_ratio: VinregRatio,
}

impl Default for PowerConfiguration {
    fn default() -> Self {
        Self {
            operating_mode: OperatingMode::Charging,
            switching_frequency: SwitchingFrequency::Freq300kHz,
            dead_time: DeadTime::Ns20,
            frequency_dithering: false,
            pfm_mode: false,
            vinreg_voltage_mv: 4500,
            vinreg_ratio: VinregRatio::Ratio100x,
        }
    }
}

/// Comprehensive device configuration for the SC8815.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceConfiguration {
    /// Battery configuration.
    pub battery: BatteryConfiguration,
    /// Current limit configuration.
    pub current_limits: CurrentLimitConfiguration,
    /// Power management configuration.
    pub power: PowerConfiguration,
    /// Whether trickle charging is enabled.
    pub trickle_charging: bool,
    /// Whether charging termination is enabled.
    pub charging_termination: bool,
    /// Whether to use IBUS or IBAT as charging current reference.
    pub use_ibus_for_charging: bool,
}

impl Default for DeviceConfiguration {
    fn default() -> Self {
        Self {
            battery: BatteryConfiguration::default(),
            current_limits: CurrentLimitConfiguration::default(),
            power: PowerConfiguration::default(),
            trickle_charging: true,
            charging_termination: true,
            use_ibus_for_charging: true,
        }
    }
}

/// Device identification information.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo {
    /// Device ID.
    pub device_id: u8,
    /// Vendor ID.
    pub vendor_id: u16,
    /// Product ID.
    pub product_id: u16,
    /// Revision ID.
    pub revision_id: u8,
}

/// Interrupt configuration for the SC8815.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Enable power good change interrupts.
    pub power_good_change: bool,
    /// Enable charging state change interrupts.
    pub charging_state_change: bool,
    /// Enable battery status change interrupts.
    pub battery_status_change: bool,
    /// Enable input source change interrupts.
    pub input_source_change: bool,
    /// Enable thermal status change interrupts.
    pub thermal_status_change: bool,
    /// Enable overcurrent protection interrupts.
    pub overcurrent_protection: bool,
    /// Enable overvoltage protection interrupts.
    pub overvoltage_protection: bool,
}

/// Power path management status for GPO and PGATE pins.
///
/// This structure provides the current status of the power path management
/// pins on the SC8815, which are used to control external MOSFETs and
/// isolation switches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerPathStatus {
    /// GPO (General Purpose Output) pin status.
    ///
    /// - `true`: GPO is pulling low (internal 6kΩ pull-down active)
    /// - `false`: GPO is in high-impedance state (open drain)
    pub gpo_enabled: bool,

    /// PGATE pin status for external PMOS control.
    ///
    /// - `true`: PGATE is pulling low to turn on external PMOS (6kΩ pull-down active)
    /// - `false`: PGATE is pulling high to turn off external PMOS (20kΩ pull-up active)
    pub pgate_enabled: bool,
}
