//! SC8815 register definitions and constants
//!
//! SC8815 is a power management IC for battery charging and power delivery.
//! This module defines the register addresses and bit field definitions.

use bitflags::bitflags;

/// SC8815 register addresses (based on actual datasheet)
#[allow(dead_code)] // Allow unused register definitions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Register {
    /// REG 0x00: VBAT_SET - Battery voltage setting
    VbatSet = 0x00,
    /// REG 0x01: VBUSREF_I_SET - Internal VBUS reference voltage setting
    VbusrefISet = 0x01,
    /// REG 0x02: VBUSREF_I_SET2 - Internal VBUS reference voltage setting (low bits)
    VbusrefISet2 = 0x02,
    /// REG 0x03: VBUSREF_E_SET - External VBUS reference voltage setting
    VbusrefESet = 0x03,
    /// REG 0x04: VBUSREF_E_SET2 - External VBUS reference voltage setting (low bits)
    VbusrefESet2 = 0x04,
    /// REG 0x05: IBUS_LIM_SET - IBUS current limit setting
    IbusLimSet = 0x05,
    /// REG 0x06: IBAT_LIM_SET - IBAT current limit setting
    IbatLimSet = 0x06,
    /// REG 0x07: VINREG_SET - VINREG voltage setting
    VinregSet = 0x07,
    /// REG 0x08: RATIO - Various ratio settings
    Ratio = 0x08,
    /// REG 0x09: CTRL0_SET - Control register 0
    Ctrl0Set = 0x09,
    /// REG 0x0A: CTRL1_SET - Control register 1
    Ctrl1Set = 0x0A,
    /// REG 0x0B: CTRL2_SET - Control register 2
    Ctrl2Set = 0x0B,
    /// REG 0x0C: CTRL3_SET - Control register 3
    Ctrl3Set = 0x0C,
    /// REG 0x0D: VBUS_FB_VALUE - VBUS feedback value (high 8 bits)
    VbusFbValue = 0x0D,
    /// REG 0x0E: VBUS_FB_VALUE2 - VBUS feedback value (low 2 bits)
    VbusFbValue2 = 0x0E,
    /// REG 0x0F: VBAT_FB_VALUE - VBAT feedback value (high 8 bits)
    VbatFbValue = 0x0F,
    /// REG 0x10: VBAT_FB_VALUE2 - VBAT feedback value (low 2 bits)
    VbatFbValue2 = 0x10,
    /// REG 0x11: IBUS_VALUE - IBUS current value (high 8 bits)
    IbusValue = 0x11,
    /// REG 0x12: IBUS_VALUE2 - IBUS current value (low 2 bits)
    IbusValue2 = 0x12,
    /// REG 0x13: IBAT_VALUE - IBAT current value (high 8 bits)
    IbatValue = 0x13,
    /// REG 0x14: IBAT_VALUE2 - IBAT current value (low 2 bits)
    IbatValue2 = 0x14,
    /// REG 0x15: ADIN_VALUE - ADIN voltage value (high 8 bits)
    AdinValue = 0x15,
    /// REG 0x16: ADIN_VALUE2 - ADIN voltage value (low 2 bits)
    AdinValue2 = 0x16,
    /// REG 0x17: STATUS - Status register
    Status = 0x17,
    /// REG 0x18: BATTERY_STATUS - Battery status register (hypothetical)
    BatteryStatus = 0x18,
    /// REG 0x19: MASK - Interrupt mask register
    Mask = 0x19,
    /// REG 0x1A: INPUT_SOURCE_STATUS - Input source status register (hypothetical)
    InputSourceStatus = 0x1A,
    /// REG 0x1B: THERMAL_STATUS - Thermal status register (hypothetical)
    ThermalStatus = 0x1B,
}

impl Register {
    /// Get the register address as u8.
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

bitflags! {
    /// Status register flags (REG 0x17)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct StatusFlags: u8 {
        /// Reserved bit 0
        const RESERVED_0 = 0x01;
        /// End of charge (EOC) conditions are satisfied
        const EOC = 0x02;
        /// Over-temperature protection (OTP) fault
        const OTP = 0x04;
        /// VBUS short circuit fault in discharging mode
        const VBUS_SHORT = 0x08;
        /// Reserved bit 4
        const RESERVED_4 = 0x10;
        /// USB-A load insert detected at INDET pin
        const INDET = 0x20;
        /// AC adapter is inserted
        const AC_OK = 0x40;
        /// Reserved bit 7
        const RESERVED_7 = 0x80;
    }
}

bitflags! {
    /// Interrupt mask register flags (REG 0x19)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct MaskFlags: u8 {
        /// Reserved bit 0 - Write to 1 after power up
        const RESERVED_0 = 0x01;
        /// EOC interrupt mask (1: interrupt disabled)
        const EOC_MASK = 0x02;
        /// OTP interrupt mask (1: interrupt disabled)
        const OTP_MASK = 0x04;
        /// VBUS short interrupt mask (1: interrupt disabled)
        const VBUS_SHORT_MASK = 0x08;
        /// Reserved bit 4
        const RESERVED_4 = 0x10;
        /// INDET interrupt mask (1: interrupt disabled)
        const INDET_MASK = 0x20;
        /// AC_OK interrupt mask (1: interrupt disabled)
        const AC_OK_MASK = 0x40;
        /// Reserved bit 7 - Internal use, don't overwrite
        const RESERVED_7 = 0x80;
    }
}

bitflags! {
    /// CTRL0_SET register flags (REG 0x09)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Ctrl0Flags: u8 {
        /// Dead time setting (bits 1-0)
        const DT_SET_MASK = 0x03;
        /// Switching frequency setting (bits 3-2)
        const FREQ_SET_MASK = 0x0C;
        /// VINREG setting ratio (bit 4)
        const VINREG_RATIO = 0x10;
        /// Reserved bit 5
        const RESERVED_5 = 0x20;
        /// Reserved bit 6
        const RESERVED_6 = 0x40;
        /// Enable OTG operation (bit 7)
        const EN_OTG = 0x80;
    }
}

bitflags! {
    /// CTRL1_SET register flags (REG 0x0A)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Ctrl1Flags: u8 {
        /// Reserved bit 0 - Internal use, don't overwrite
        const RESERVED_0 = 0x01;
        /// Reserved bit 1 - Internal use, don't overwrite
        const RESERVED_1 = 0x02;
        /// OVP protection setting for discharging mode
        const DIS_OVP = 0x04;
        /// Trickle charge phase threshold setting
        const TRICKLE_SET = 0x08;
        /// VBUS voltage setting control (discharging mode only)
        const FB_SEL = 0x10;
        /// Charging termination control
        const DIS_TERM = 0x20;
        /// Trickle charge control
        const DIS_TRICKLE = 0x40;
        /// Charging current selection
        const ICHAR_SEL = 0x80;
    }
}

bitflags! {
    /// CTRL2_SET register flags (REG 0x0B)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Ctrl2Flags: u8 {
        /// Slew rate setting (bits 1-0)
        const SLEW_SET_MASK = 0x03;
        /// Enable switching frequency dithering function
        const EN_DITHER = 0x04;
        /// Factory setting bit - MCU shall write this bit to 1 after power up
        const FACTORY = 0x08;
        /// Reserved bits 7-4 - Internal use, don't overwrite
        const RESERVED_4_7 = 0xF0;
    }
}

bitflags! {
    /// CTRL3_SET register flags (REG 0x0C)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Ctrl3Flags: u8 {
        /// Enable PFM mode under light load condition (discharging mode only)
        const EN_PFM = 0x01;
        /// Current threshold setting for End Of Charging (EOC) detection
        const EOC_SET = 0x02;
        /// IBUS and IBAT current foldback control for VBUS short circuit
        const DIS_SHORT_FOLDBACK = 0x04;
        /// Loop response control
        const LOOP_SET = 0x08;
        /// ILIM loop bandwidth setting
        const ILIM_BW_SEL = 0x10;
        /// ADC control
        const AD_START = 0x20;
        /// GPO output control
        const GPO_CTRL = 0x40;
        /// PGATE control
        const EN_PGATE = 0x80;
    }
}

bitflags! {
    /// RATIO register flags (REG 0x08)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct RatioFlags: u8 {
        /// VBUS ratio setting
        const VBUS_RATIO = 0x01;
        /// VBAT monitor ratio setting
        const VBAT_MON_RATIO = 0x02;
        /// IBUS ratio setting (bits 3-2)
        const IBUS_RATIO_MASK = 0x0C;
        /// IBAT ratio setting
        const IBAT_RATIO = 0x10;
        /// Reserved bit 5 - Internal use, don't overwrite
        const RESERVED_5 = 0x20;
        /// Reserved bits 7-6 - Internal use, don't overwrite
        const RESERVED_6_7 = 0xC0;
    }
}

bitflags! {
    /// VBAT_SET register flags (REG 0x00)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct VbatSetFlags: u8 {
        /// Battery voltage setting per cell (bits 2-0)
        const VCELL_SET_MASK = 0x07;
        /// Battery cell selection (bits 4-3)
        const CSEL_MASK = 0x18;
        /// VBAT voltage setting selection
        const VBAT_SEL = 0x20;
        /// Battery IR compensation setting (bits 7-6)
        const IRCOMP_MASK = 0xC0;
    }
}

bitflags! {
    /// Battery status flags (hypothetical register for battery status)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BatteryStatusFlags: u8 {
        /// Battery present
        const BATTERY_PRESENT = 0x01;
        /// Battery low voltage
        const BATTERY_LOW_VOLTAGE = 0x02;
        /// Battery overvoltage
        const BATTERY_OVERVOLTAGE = 0x04;
        /// Battery overcurrent
        const BATTERY_OVERCURRENT = 0x08;
        /// Battery temperature fault
        const BATTERY_TEMP_FAULT = 0x10;
        /// Reserved bits
        const RESERVED = 0xE0;
    }
}

bitflags! {
    /// Input source status flags (hypothetical register for input source status)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InputSourceStatusFlags: u8 {
        /// Input source present
        const INPUT_SOURCE_PRESENT = 0x01;
        /// Input undervoltage
        const INPUT_UNDERVOLTAGE = 0x02;
        /// Input overvoltage
        const INPUT_OVERVOLTAGE = 0x04;
        /// Input overcurrent
        const INPUT_OVERCURRENT = 0x08;
        /// Reserved bits
        const RESERVED = 0xF0;
    }
}

bitflags! {
    /// Thermal status flags (hypothetical register for thermal status)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ThermalStatusFlags: u8 {
        /// Temperature warning
        const TEMP_WARNING = 0x01;
        /// Thermal shutdown
        const THERMAL_SHUTDOWN = 0x02;
        /// Reserved bits
        const RESERVED = 0xFC;
    }
}

/// SC8815 device constants (based on actual datasheet)
pub mod constants {
    /// Default I2C address for SC8815 (7-bit address)
    pub const DEFAULT_ADDRESS: u8 = 0x74;
    /// 8-bit I2C address for write operations
    pub const I2C_WRITE_ADDRESS: u8 = 0xE8;
    /// 8-bit I2C address for read operations
    pub const I2C_READ_ADDRESS: u8 = 0xE9;

    /// Default timeout for operations (in milliseconds)
    pub const DEFAULT_TIMEOUT_MS: u32 = 1000;

    /// Battery voltage settings per cell (in mV)
    pub const VCELL_4_1V: u8 = 0b000;
    pub const VCELL_4_2V: u8 = 0b001; // Default
    pub const VCELL_4_25V: u8 = 0b010;
    pub const VCELL_4_3V: u8 = 0b011;
    pub const VCELL_4_35V: u8 = 0b100;
    pub const VCELL_4_4V: u8 = 0b101;
    pub const VCELL_4_45V: u8 = 0b110;

    /// Battery cell selection
    pub const CSEL_1S: u8 = 0b00; // Default
    pub const CSEL_2S: u8 = 0b01;
    pub const CSEL_3S: u8 = 0b10;
    pub const CSEL_4S: u8 = 0b11;

    /// Switching frequency settings
    pub const FREQ_150KHZ: u8 = 0b00;
    pub const FREQ_300KHZ: u8 = 0b01; // Default
    pub const FREQ_450KHZ: u8 = 0b11;

    /// Dead time settings
    pub const DT_20NS: u8 = 0b00; // Default
    pub const DT_40NS: u8 = 0b01;
    pub const DT_60NS: u8 = 0b10;
    pub const DT_80NS: u8 = 0b11;

    /// IBUS ratio settings
    pub const IBUS_RATIO_6X: u8 = 0b01;
    pub const IBUS_RATIO_3X: u8 = 0b10; // Default

    /// IBAT ratio settings
    pub const IBAT_RATIO_6X: u8 = 0;
    pub const IBAT_RATIO_12X: u8 = 1; // Default

    /// VBUS ratio settings
    pub const VBUS_RATIO_12_5X: u8 = 0; // Default
    pub const VBUS_RATIO_5X: u8 = 1;

    /// VBAT monitor ratio settings
    pub const VBAT_MON_RATIO_12_5X: u8 = 0; // Default
    pub const VBAT_MON_RATIO_5X: u8 = 1;

    /// VINREG ratio settings
    pub const VINREG_RATIO_100X: u8 = 0; // Default
    pub const VINREG_RATIO_40X: u8 = 1;

    /// Slew rate settings for VBUS dynamic change
    pub const SLEW_1MV_US: u8 = 0b00;
    pub const SLEW_2MV_US: u8 = 0b01; // Default
    pub const SLEW_4MV_US: u8 = 0b10;

    /// ADC resolution and reference
    pub const ADC_RESOLUTION_MV: u16 = 2; // 2mV per step
    pub const ADC_MAX_VOLTAGE_MV: u16 = 2048; // Maximum ADIN voltage

    /// Current sense resistor typical values (in milliohms)
    pub const TYPICAL_RS1_MOHM: u16 = 10; // VBUS side current sense resistor
    pub const TYPICAL_RS2_MOHM: u16 = 10; // VBAT side current sense resistor

    /// Voltage ranges
    pub const VBAT_RANGE_MIN_V: f32 = 2.7;
    pub const VBAT_RANGE_MAX_V: f32 = 36.0;
    pub const VBUS_RANGE_MIN_V: f32 = 2.7;
    pub const VBUS_RANGE_MAX_V: f32 = 36.0;
}
