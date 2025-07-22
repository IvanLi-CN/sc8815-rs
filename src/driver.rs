//! SC8815 driver implementation using maybe-async-cfg
//!
//! SC8815 is a power management IC for battery charging and power delivery.
//! This driver provides methods to interact with the SC8815 for power management and charging control.

use crate::data_types::{
    AdcMeasurements, BatteryStatus, ChargingState, CurrentLimitConfiguration, DeviceConfiguration,
    InputSourceStatus, OperatingMode, SC8815Status, ThermalStatus,
};
use crate::error::Error;
use crate::registers::{
    BatteryStatusFlags, Ctrl0Flags, Ctrl1Flags, Ctrl2Flags, Ctrl3Flags, InputSourceStatusFlags,
    MaskFlags, Register, StatusFlags, ThermalStatusFlags,
};

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

/// Type alias for ADC measurement results (VBUS, VBAT, IBUS, IBAT, ADIN in mV/mA)
type AdcResults<E> = Result<(u16, u16, u16, u16, u16), Error<E>>;

/// SC8815 power management IC driver.
///
/// This driver provides a high-level interface to the SC8815 power management IC,
/// supporting both synchronous and asynchronous I2C communication.
pub struct SC8815<I2C> {
    /// I2C interface for communication with the SC8815.
    i2c: I2C,
    /// I2C address of the SC8815 device.
    address: u8,
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "SC8815"),
    async(feature = "async", keep_self)
)]
impl<I2C, E> SC8815<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Create a new SC8815 driver instance.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C interface to use for communication.
    /// * `address` - The I2C address of the SC8815 device.
    ///
    /// # Returns
    ///
    /// Returns a new `SC8815` driver instance.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Release the I2C interface from the driver.
    ///
    /// # Returns
    ///
    /// Returns the I2C interface that was used by this driver.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Read a single register from the SC8815.
    ///
    /// # Arguments
    ///
    /// * `register` - The register to read from.
    ///
    /// # Returns
    ///
    /// Returns the register value on success, or an `Error` if the operation fails.
    pub async fn read_register(&mut self, register: Register) -> Result<u8, Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// Write a single register to the SC8815.
    ///
    /// # Arguments
    ///
    /// * `register` - The register to write to.
    /// * `value` - The value to write.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register.addr(), value])
            .await
            .map_err(Error::I2c)
    }

    /// Initialize the SC8815 power management IC.
    ///
    /// This method performs the initial setup of the SC8815, including chip version verification
    /// and basic configuration for power management.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if initialization fails.
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        // Read status register to verify communication
        let _status = self.read_register(Register::Status).await?;

        #[cfg(feature = "defmt")]
        defmt::info!("SC8815 status: 0x{:02X}", _status);

        // Set factory bit as required by datasheet
        let mut ctrl2 = self.read_register(Register::Ctrl2Set).await?;
        ctrl2 |= Ctrl2Flags::FACTORY.bits();
        self.write_register(Register::Ctrl2Set, ctrl2).await?;

        // Set reserved bit 0 in mask register as required by datasheet
        let mut mask = self.read_register(Register::Mask).await?;
        mask |= MaskFlags::RESERVED_0.bits();
        self.write_register(Register::Mask, mask).await?;

        #[cfg(feature = "defmt")]
        defmt::info!("SC8815 initialized successfully");

        Ok(())
    }

    /// Check if AC adapter is connected.
    ///
    /// # Returns
    ///
    /// Returns `true` if AC adapter is inserted, `false` otherwise, or an `Error` if the operation fails.
    pub async fn is_ac_adapter_connected(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::Status).await?;
        let flags = StatusFlags::from_bits_truncate(status);
        Ok(flags.contains(StatusFlags::AC_OK))
    }

    /// Check if USB-A load is detected.
    ///
    /// # Returns
    ///
    /// Returns `true` if USB-A load insert is detected, `false` otherwise, or an `Error` if the operation fails.
    pub async fn is_usb_load_detected(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::Status).await?;
        let flags = StatusFlags::from_bits_truncate(status);
        Ok(flags.contains(StatusFlags::INDET))
    }

    /// Check if end of charge (EOC) conditions are satisfied.
    ///
    /// # Returns
    ///
    /// Returns `true` if EOC conditions are satisfied, `false` otherwise, or an `Error` if the operation fails.
    pub async fn is_charging_complete(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::Status).await?;
        let flags = StatusFlags::from_bits_truncate(status);
        Ok(flags.contains(StatusFlags::EOC))
    }

    /// Check if over-temperature protection (OTP) fault occurred.
    ///
    /// # Returns
    ///
    /// Returns `true` if OTP fault occurred, `false` otherwise, or an `Error` if the operation fails.
    pub async fn is_otp_fault(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::Status).await?;
        let flags = StatusFlags::from_bits_truncate(status);
        Ok(flags.contains(StatusFlags::OTP))
    }

    /// Check if VBUS short circuit fault occurred in discharging mode.
    ///
    /// # Returns
    ///
    /// Returns `true` if VBUS short circuit fault occurred, `false` otherwise, or an `Error` if the operation fails.
    pub async fn is_vbus_short_fault(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::Status).await?;
        let flags = StatusFlags::from_bits_truncate(status);
        Ok(flags.contains(StatusFlags::VBUS_SHORT))
    }

    /// Set the device to charging mode or discharging mode.
    ///
    /// # Arguments
    ///
    /// * `enable_otg` - If `true`, enables OTG (discharging) mode. If `false`, enables charging mode.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_otg_mode(&mut self, enable_otg: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl0 = self.read_register(Register::Ctrl0Set).await?;
        let mut flags = Ctrl0Flags::from_bits_truncate(ctrl0);

        if enable_otg {
            flags.insert(Ctrl0Flags::EN_OTG);
        } else {
            flags.remove(Ctrl0Flags::EN_OTG);
        }

        self.write_register(Register::Ctrl0Set, flags.bits()).await
    }

    /// Check if the device is in OTG (discharging) mode.
    ///
    /// # Returns
    ///
    /// Returns `true` if in OTG mode, `false` if in charging mode, or an `Error` if the operation fails.
    pub async fn is_otg_mode(&mut self) -> Result<bool, Error<I2C::Error>> {
        let ctrl0 = self.read_register(Register::Ctrl0Set).await?;
        let flags = Ctrl0Flags::from_bits_truncate(ctrl0);
        Ok(flags.contains(Ctrl0Flags::EN_OTG))
    }

    /// Enable or disable trickle charging phase.
    ///
    /// # Arguments
    ///
    /// * `enable` - Whether to enable trickle charging phase.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_trickle_charging(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);

        if enable {
            flags.remove(Ctrl1Flags::DIS_TRICKLE);
        } else {
            flags.insert(Ctrl1Flags::DIS_TRICKLE);
        }

        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Enable or disable charging termination.
    ///
    /// # Arguments
    ///
    /// * `enable` - Whether to enable auto-termination.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_charging_termination(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);

        if enable {
            flags.remove(Ctrl1Flags::DIS_TERM);
        } else {
            flags.insert(Ctrl1Flags::DIS_TERM);
        }

        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Set the switching frequency.
    ///
    /// # Arguments
    ///
    /// * `freq_setting` - Frequency setting (0: 150kHz, 1: 300kHz, 3: 450kHz)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_switching_frequency(
        &mut self,
        freq_setting: u8,
    ) -> Result<(), Error<I2C::Error>> {
        if freq_setting > 3 || freq_setting == 2 {
            return Err(Error::InvalidParameter);
        }

        let mut ctrl0 = self.read_register(Register::Ctrl0Set).await?;
        // Clear frequency bits (bits 3-2) and set new value
        ctrl0 = (ctrl0 & !0x0C) | ((freq_setting & 0x03) << 2);
        self.write_register(Register::Ctrl0Set, ctrl0).await
    }

    /// Set the dead time for switching.
    ///
    /// # Arguments
    ///
    /// * `dt_setting` - Dead time setting (0: 20ns, 1: 40ns, 2: 60ns, 3: 80ns)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_dead_time(&mut self, dt_setting: u8) -> Result<(), Error<I2C::Error>> {
        if dt_setting > 3 {
            return Err(Error::InvalidParameter);
        }

        let mut ctrl0 = self.read_register(Register::Ctrl0Set).await?;
        // Clear dead time bits (bits 1-0) and set new value
        ctrl0 = (ctrl0 & !0x03) | (dt_setting & 0x03);
        self.write_register(Register::Ctrl0Set, ctrl0).await
    }

    /// Enable or disable frequency dithering.
    ///
    /// # Arguments
    ///
    /// * `enable` - Whether to enable frequency dithering.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_frequency_dithering(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl2 = self.read_register(Register::Ctrl2Set).await?;
        let mut flags = Ctrl2Flags::from_bits_truncate(ctrl2);

        if enable {
            flags.insert(Ctrl2Flags::EN_DITHER);
        } else {
            flags.remove(Ctrl2Flags::EN_DITHER);
        }

        self.write_register(Register::Ctrl2Set, flags.bits()).await
    }

    /// Enable or disable PFM mode for light load conditions (discharging mode only).
    ///
    /// # Arguments
    ///
    /// * `enable` - Whether to enable PFM mode.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_pfm_mode(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if enable {
            flags.insert(Ctrl3Flags::EN_PFM);
        } else {
            flags.remove(Ctrl3Flags::EN_PFM);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Disable short circuit current foldback for startup with load.
    ///
    /// When VBUS < 1V, SC8815 normally reduces current limits to 22% (IBUS) and 10% (IBAT).
    /// This function disables that behavior to allow startup with load connected.
    ///
    /// # Arguments
    ///
    /// * `disable` - Whether to disable short circuit foldback
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_short_foldback_disable(
        &mut self,
        disable: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if disable {
            flags.insert(Ctrl3Flags::DIS_SHORT_FOLDBACK);
        } else {
            flags.remove(Ctrl3Flags::DIS_SHORT_FOLDBACK);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Clear VBUS short circuit fault using official recommended method.
    ///
    /// According to official documentation: When VBUS_SHORT=1 is detected,
    /// clear DIS_ShortFoldBack (maintain for 10ms, then set to 1 again).
    /// This helps resolve startup issues after short circuit protection.
    ///
    /// Note: This function requires a delay implementation. For embedded systems
    /// using embassy, you can call this from an async context. For other systems,
    /// implement the delay manually between the two set_short_foldback_disable calls.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn clear_vbus_short_fault_with_delay<F, Fut>(
        &mut self,
        delay_fn: F,
    ) -> Result<(), Error<I2C::Error>>
    where
        F: FnOnce() -> Fut,
        Fut: core::future::Future<Output = ()>,
    {
        // Step 1: Clear DIS_ShortFoldBack (enable short circuit foldback temporarily)
        self.set_short_foldback_disable(false).await?;

        // Step 2: Wait 10ms as recommended by official documentation
        delay_fn().await;

        // Step 3: Set DIS_ShortFoldBack back to 1 (disable short circuit foldback)
        self.set_short_foldback_disable(true).await?;

        Ok(())
    }

    /// Clear VBUS short circuit fault using official recommended method (sync version).
    ///
    /// According to official documentation: When VBUS_SHORT=1 is detected,
    /// clear DIS_ShortFoldBack (maintain for 10ms, then set to 1 again).
    /// This helps resolve startup issues after short circuit protection.
    ///
    /// Note: This function does NOT include the 10ms delay. You must implement
    /// the delay manually between calling this function twice:
    /// 1. Call with `enable_foldback = true`
    /// 2. Wait 10ms
    /// 3. Call with `enable_foldback = false`
    ///
    /// # Arguments
    ///
    /// * `enable_foldback` - If true, enables short circuit foldback (step 1).
    ///                       If false, disables short circuit foldback (step 3).
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn clear_vbus_short_fault_step(
        &mut self,
        enable_foldback: bool,
    ) -> Result<(), Error<I2C::Error>> {
        self.set_short_foldback_disable(!enable_foldback).await
    }

    /// Control the PGATE pin for external PMOS control.
    ///
    /// # Arguments
    ///
    /// * `enable` - If `true`, PGATE outputs logic low to turn on PMOS. If `false`, outputs logic high to turn off PMOS.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_pgate_control(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if enable {
            flags.insert(Ctrl3Flags::EN_PGATE);
        } else {
            flags.remove(Ctrl3Flags::EN_PGATE);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Control the GPO (General Purpose Output) pin.
    ///
    /// # Arguments
    ///
    /// * `enable` - If `true`, GPO outputs logic low. If `false`, GPO is open drain (high impedance).
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_gpo_control(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if enable {
            flags.insert(Ctrl3Flags::GPO_CTRL);
        } else {
            flags.remove(Ctrl3Flags::GPO_CTRL);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Start or stop ADC conversion.
    ///
    /// # Arguments
    ///
    /// * `start` - If `true`, starts ADC conversion. If `false`, stops ADC conversion.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_adc_conversion(&mut self, start: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if start {
            flags.insert(Ctrl3Flags::AD_START);
        } else {
            flags.remove(Ctrl3Flags::AD_START);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Set IBUS current limit.
    ///
    /// # Arguments
    ///
    /// * `limit_ma` - Current limit in milliamps (minimum 300mA)
    /// * `ratio` - IBUS ratio setting (1: 6x, 2: 3x)
    /// * `rs1_mohm` - Current sense resistor value in milliohms (typically 10mΩ)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_ibus_limit(
        &mut self,
        limit_ma: u16,
        ratio: u8,
        rs1_mohm: u16,
    ) -> Result<(), Error<I2C::Error>> {
        if limit_ma < 300 || ratio == 0 || ratio > 2 || rs1_mohm == 0 {
            return Err(Error::InvalidParameter);
        }

        // Calculate IBUS_LIM_SET value based on formula:
        // IBUS_LIM (A) = (IBUS_LIM_SET + 1) / 256 * IBUS_RATIO * 10mΩ / RS1
        let ratio_multiplier = match ratio {
            1 => 6,
            2 => 3,
            _ => return Err(Error::InvalidParameter),
        };

        let limit_a = limit_ma as f32 / 1000.0;
        let ibus_lim_set =
            ((limit_a * 256.0 * rs1_mohm as f32) / (ratio_multiplier as f32 * 10.0)) - 1.0;

        if !(0.0..=255.0).contains(&ibus_lim_set) {
            return Err(Error::InvalidParameter);
        }

        let ibus_lim_set = ibus_lim_set as u8;

        // Set the IBUS limit register
        self.write_register(Register::IbusLimSet, ibus_lim_set)
            .await?;

        // Set the ratio in the RATIO register
        let mut ratio_reg = self.read_register(Register::Ratio).await?;
        ratio_reg = (ratio_reg & !0x0C) | ((ratio & 0x03) << 2);
        self.write_register(Register::Ratio, ratio_reg).await
    }

    /// Set IBAT current limit.
    ///
    /// # Arguments
    ///
    /// * `limit_ma` - Current limit in milliamps (minimum 300mA)
    /// * `ratio` - IBAT ratio setting (0: 6x, 1: 12x)
    /// * `rs2_mohm` - Current sense resistor value in milliohms (typically 10mΩ)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_ibat_limit(
        &mut self,
        limit_ma: u16,
        ratio: u8,
        rs2_mohm: u16,
    ) -> Result<(), Error<I2C::Error>> {
        if limit_ma < 300 || ratio > 1 || rs2_mohm == 0 {
            return Err(Error::InvalidParameter);
        }

        // Calculate IBAT_LIM_SET value based on formula:
        // IBAT_LIM (A) = (IBAT_LIM_SET + 1) / 256 * IBAT_RATIO * 10mΩ / RS2
        let ratio_multiplier = match ratio {
            0 => 6,
            1 => 12,
            _ => return Err(Error::InvalidParameter),
        };

        let limit_a = limit_ma as f32 / 1000.0;
        let ibat_lim_set =
            ((limit_a * 256.0 * rs2_mohm as f32) / (ratio_multiplier as f32 * 10.0)) - 1.0;

        if !(0.0..=255.0).contains(&ibat_lim_set) {
            return Err(Error::InvalidParameter);
        }

        let ibat_lim_set = ibat_lim_set as u8;

        // Set the IBAT limit register
        self.write_register(Register::IbatLimSet, ibat_lim_set)
            .await?;

        // Set the ratio in the RATIO register
        let mut ratio_reg = self.read_register(Register::Ratio).await?;
        if ratio == 1 {
            ratio_reg |= 0x10; // Set IBAT_RATIO bit
        } else {
            ratio_reg &= !0x10; // Clear IBAT_RATIO bit
        }
        self.write_register(Register::Ratio, ratio_reg).await
    }

    /// Set VINREG voltage threshold for charging mode.
    ///
    /// # Arguments
    ///
    /// * `voltage_mv` - VINREG voltage in millivolts
    /// * `ratio` - VINREG ratio setting (0: 100x, 1: 40x)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_vinreg_voltage(
        &mut self,
        voltage_mv: u16,
        ratio: u8,
    ) -> Result<(), Error<I2C::Error>> {
        if ratio > 1 {
            return Err(Error::InvalidParameter);
        }

        // Calculate VINREG_SET value based on formula:
        // VINREG = (VINREG_SET + 1) * VINREG_RATIO (mV)
        let ratio_multiplier = match ratio {
            0 => 100,
            1 => 40,
            _ => return Err(Error::InvalidParameter),
        };

        if voltage_mv < ratio_multiplier || voltage_mv > (255 + 1) * ratio_multiplier {
            return Err(Error::InvalidParameter);
        }

        let vinreg_set = (voltage_mv / ratio_multiplier) - 1;
        if vinreg_set > 255 {
            return Err(Error::InvalidParameter);
        }

        // Set the VINREG register
        self.write_register(Register::VinregSet, vinreg_set as u8)
            .await?;

        // Set the ratio in the CTRL0 register
        let mut ctrl0 = self.read_register(Register::Ctrl0Set).await?;
        if ratio == 1 {
            ctrl0 |= 0x10; // Set VINREG_RATIO bit
        } else {
            ctrl0 &= !0x10; // Clear VINREG_RATIO bit
        }
        self.write_register(Register::Ctrl0Set, ctrl0).await
    }

    /// Read VBUS voltage from ADC.
    ///
    /// # Arguments
    ///
    /// * `vbus_ratio` - VBUS ratio setting (0: 12.5x, 1: 5x)
    ///
    /// # Returns
    ///
    /// Returns VBUS voltage in millivolts, or an `Error` if the operation fails.
    pub async fn read_vbus_voltage(&mut self, vbus_ratio: u8) -> Result<u16, Error<I2C::Error>> {
        if vbus_ratio > 1 {
            return Err(Error::InvalidParameter);
        }

        // Read the 10-bit ADC value (high 8 bits + low 2 bits)
        let vbus_fb_high = self.read_register(Register::VbusFbValue).await?;
        let vbus_fb_low = self.read_register(Register::VbusFbValue2).await?;

        // Combine to get 10-bit value
        let vbus_fb_value = ((vbus_fb_high as u16) << 2) | (((vbus_fb_low >> 6) & 0x03) as u16);

        // Calculate voltage based on formula:
        // VBUS = (4 * VBUS_FB_VALUE + VBUS_FB_VALUE2 + 1) * VBUS_RATIO * 2 mV
        let ratio_multiplier = match vbus_ratio {
            0 => 12.5,
            1 => 5.0,
            _ => return Err(Error::InvalidParameter),
        };

        let voltage_mv = ((4 * vbus_fb_value + 1) as f32 * ratio_multiplier * 2.0) as u16;

        Ok(voltage_mv)
    }

    /// Read VBAT voltage from ADC.
    ///
    /// # Arguments
    ///
    /// * `vbat_mon_ratio` - VBAT monitor ratio setting (0: 12.5x, 1: 5x)
    ///
    /// # Returns
    ///
    /// Returns VBAT voltage in millivolts, or an `Error` if the operation fails.
    pub async fn read_vbat_voltage(
        &mut self,
        vbat_mon_ratio: u8,
    ) -> Result<u16, Error<I2C::Error>> {
        if vbat_mon_ratio > 1 {
            return Err(Error::InvalidParameter);
        }

        // Read the 10-bit ADC value (high 8 bits + low 2 bits)
        let vbat_fb_high = self.read_register(Register::VbatFbValue).await?;
        let vbat_fb_low = self.read_register(Register::VbatFbValue2).await?;

        // Calculate voltage based on formula:
        // VBAT = (4 * VBAT_FB_VALUE + VBAT_FB_VALUE2 + 1) * VBAT_MON_RATIO * 2 mV
        let ratio_multiplier = match vbat_mon_ratio {
            0 => 12.5,
            1 => 5.0,
            _ => return Err(Error::InvalidParameter),
        };

        let voltage_mv = ((4 * (vbat_fb_high as u16) + ((vbat_fb_low >> 6) & 0x03) as u16 + 1)
            as f32
            * ratio_multiplier
            * 2.0) as u16;

        Ok(voltage_mv)
    }

    /// Read IBUS current from ADC.
    ///
    /// # Arguments
    ///
    /// * `ibus_ratio` - IBUS ratio setting (1: 6x, 2: 3x)
    /// * `rs1_mohm` - Current sense resistor value in milliohms (typically 10mΩ)
    ///
    /// # Returns
    ///
    /// Returns IBUS current in milliamps, or an `Error` if the operation fails.
    pub async fn read_ibus_current(
        &mut self,
        ibus_ratio: u8,
        rs1_mohm: u16,
    ) -> Result<u16, Error<I2C::Error>> {
        if ibus_ratio == 0 || ibus_ratio > 2 || rs1_mohm == 0 {
            return Err(Error::InvalidParameter);
        }

        // Read the 10-bit ADC value (high 8 bits + low 2 bits)
        let ibus_high = self.read_register(Register::IbusValue).await?;
        let ibus_low = self.read_register(Register::IbusValue2).await?;

        // Calculate current based on SC8815 manual formula (from screenshot):
        // IBUS (A) = [(4 × IBUS_VALUE + IBUS_VALUE2 + 1) × 2] / 1200 × IBUS_RATIO × 10mΩ / RS1
        //
        // Where IBUS_RATIO is the actual multiplier value (6x or 3x)
        let ratio_multiplier = match ibus_ratio {
            1 => 6, // IbusRatio::Ratio6x = 1 -> 6x multiplier
            2 => 3, // IbusRatio::Ratio3x = 2 -> 3x multiplier
            _ => return Err(Error::InvalidParameter),
        };

        // Apply the correct formula from manual:
        // Step 1: Calculate (4 × IBUS_VALUE + IBUS_VALUE2 + 1) × 2
        let adc_value = 4 * (ibus_high as u32) + ((ibus_low >> 6) & 0x03) as u32 + 1;
        let numerator = adc_value * 2;

        // Step 2: Apply the formula: numerator / 1200 × IBUS_RATIO × 10mΩ / RS1
        let current_a =
            (numerator as f32 / 1200.0) * (ratio_multiplier as f32) * (10.0 / rs1_mohm as f32);
        let current_ma = (current_a * 1000.0) as u16;

        Ok(current_ma)
    }

    /// Read IBAT current from ADC.
    ///
    /// # Arguments
    ///
    /// * `ibat_ratio` - IBAT ratio setting (0: 6x, 1: 12x)
    /// * `rs2_mohm` - Current sense resistor value in milliohms (typically 10mΩ)
    ///
    /// # Returns
    ///
    /// Returns IBAT current in milliamps, or an `Error` if the operation fails.
    pub async fn read_ibat_current(
        &mut self,
        ibat_ratio: u8,
        rs2_mohm: u16,
    ) -> Result<u16, Error<I2C::Error>> {
        if ibat_ratio > 1 || rs2_mohm == 0 {
            return Err(Error::InvalidParameter);
        }

        // Read the 10-bit ADC value (high 8 bits + low 2 bits)
        let ibat_high = self.read_register(Register::IbatValue).await?;
        let ibat_low = self.read_register(Register::IbatValue2).await?;

        // Calculate current based on SC8815 manual formula (from screenshot):
        // IBAT (A) = [(4 × IBAT_VALUE + IBAT_VALUE2 + 1) × 2] / 1200 × IBAT_RATIO × 10mΩ / RS2
        //
        // Where IBAT_RATIO is the actual multiplier value (6x or 12x)
        let ratio_multiplier = match ibat_ratio {
            0 => 6,  // IbatRatio::Ratio6x = 0 -> 6x multiplier
            1 => 12, // IbatRatio::Ratio12x = 1 -> 12x multiplier
            _ => return Err(Error::InvalidParameter),
        };

        // Apply the correct formula from manual:
        // Step 1: Calculate (4 × IBAT_VALUE + IBAT_VALUE2 + 1) × 2
        let adc_value = 4 * (ibat_high as u32) + ((ibat_low >> 6) & 0x03) as u32 + 1;
        let numerator = adc_value * 2;

        // Step 2: Apply the formula: numerator / 1200 × IBAT_RATIO × 10mΩ / RS2
        let current_a =
            (numerator as f32 / 1200.0) * (ratio_multiplier as f32) * (10.0 / rs2_mohm as f32);
        let current_ma = (current_a * 1000.0) as u16;

        Ok(current_ma)
    }

    /// Read ADIN voltage from ADC.
    ///
    /// # Returns
    ///
    /// Returns ADIN voltage in millivolts (maximum 2048mV), or an `Error` if the operation fails.
    pub async fn read_adin_voltage(&mut self) -> Result<u16, Error<I2C::Error>> {
        // Read the 10-bit ADC value (high 8 bits + low 2 bits)
        let adin_high = self.read_register(Register::AdinValue).await?;
        let adin_low = self.read_register(Register::AdinValue2).await?;

        // Calculate voltage based on formula:
        // VADIN = (4 * ADIN_VALUE + ADIN_VALUE2 + 1) * 2 mV
        let voltage_mv = (4 * (adin_high as u16) + ((adin_low >> 6) & 0x03) as u16 + 1) * 2;

        Ok(voltage_mv)
    }

    /// Read all ADC values at once.
    ///
    /// # Arguments
    ///
    /// * `vbus_ratio` - VBUS ratio setting (0: 12.5x, 1: 5x)
    /// * `vbat_mon_ratio` - VBAT monitor ratio setting (0: 12.5x, 1: 5x)
    /// * `ibus_ratio` - IBUS ratio setting (1: 6x, 2: 3x)
    /// * `ibat_ratio` - IBAT ratio setting (0: 6x, 1: 12x)
    /// * `rs1_mohm` - VBUS side current sense resistor in milliohms
    /// * `rs2_mohm` - VBAT side current sense resistor in milliohms
    ///
    /// # Returns
    ///
    /// Returns tuple of (VBUS_mV, VBAT_mV, IBUS_mA, IBAT_mA, ADIN_mV), or an `Error` if the operation fails.
    pub async fn read_all_adc_values(
        &mut self,
        vbus_ratio: u8,
        vbat_mon_ratio: u8,
        ibus_ratio: u8,
        ibat_ratio: u8,
        rs1_mohm: u16,
        rs2_mohm: u16,
    ) -> AdcResults<I2C::Error> {
        let vbus_mv = self.read_vbus_voltage(vbus_ratio).await?;
        let vbat_mv = self.read_vbat_voltage(vbat_mon_ratio).await?;
        let ibus_ma = self.read_ibus_current(ibus_ratio, rs1_mohm).await?;
        let ibat_ma = self.read_ibat_current(ibat_ratio, rs2_mohm).await?;
        let adin_mv = self.read_adin_voltage().await?;

        Ok((vbus_mv, vbat_mv, ibus_ma, ibat_ma, adin_mv))
    }

    /// Configure interrupt mask register.
    ///
    /// # Arguments
    ///
    /// * `mask_flags` - Interrupt mask flags (1: interrupt disabled, 0: interrupt enabled)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_interrupt_mask(
        &mut self,
        mask_flags: MaskFlags,
    ) -> Result<(), Error<I2C::Error>> {
        // Always ensure reserved bit 0 is set to 1 as required by datasheet
        let mut flags = mask_flags;
        flags.insert(MaskFlags::RESERVED_0);

        self.write_register(Register::Mask, flags.bits()).await
    }

    /// Get current interrupt mask settings.
    ///
    /// # Returns
    ///
    /// Returns the current interrupt mask flags, or an `Error` if the operation fails.
    pub async fn get_interrupt_mask(&mut self) -> Result<MaskFlags, Error<I2C::Error>> {
        let mask = self.read_register(Register::Mask).await?;
        Ok(MaskFlags::from_bits_truncate(mask))
    }

    /// Enable specific interrupts.
    ///
    /// # Arguments
    ///
    /// * `interrupts` - Interrupt flags to enable
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn enable_interrupts(
        &mut self,
        interrupts: MaskFlags,
    ) -> Result<(), Error<I2C::Error>> {
        let mut current_mask = self.get_interrupt_mask().await?;
        // Remove the specified interrupt masks (0 = enabled, 1 = disabled)
        current_mask.remove(interrupts);
        self.set_interrupt_mask(current_mask).await
    }

    /// Disable specific interrupts.
    ///
    /// # Arguments
    ///
    /// * `interrupts` - Interrupt flags to disable
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn disable_interrupts(
        &mut self,
        interrupts: MaskFlags,
    ) -> Result<(), Error<I2C::Error>> {
        let mut current_mask = self.get_interrupt_mask().await?;
        // Add the specified interrupt masks (1 = disabled, 0 = enabled)
        current_mask.insert(interrupts);
        self.set_interrupt_mask(current_mask).await
    }

    /// Get current status flags from the status register.
    ///
    /// # Returns
    ///
    /// Returns the current status flags, or an `Error` if the operation fails.
    pub async fn get_status_flags(&mut self) -> Result<StatusFlags, Error<I2C::Error>> {
        let status = self.read_register(Register::Status).await?;
        Ok(StatusFlags::from_bits_truncate(status))
    }

    /// Check if any interrupts are pending by reading the status register.
    ///
    /// # Returns
    ///
    /// Returns `true` if any interrupt conditions are active, `false` otherwise, or an `Error` if the operation fails.
    pub async fn has_pending_interrupts(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.get_status_flags().await?;
        // Check if any interrupt-related status bits are set
        Ok(status.intersects(
            StatusFlags::EOC
                | StatusFlags::OTP
                | StatusFlags::VBUS_SHORT
                | StatusFlags::INDET
                | StatusFlags::AC_OK,
        ))
    }

    /// Clear interrupt conditions by reading the status register.
    /// Note: INDET bit is read-and-clear type, others represent real-time status.
    ///
    /// # Returns
    ///
    /// Returns the status flags that were active, or an `Error` if the operation fails.
    pub async fn clear_interrupts(&mut self) -> Result<StatusFlags, Error<I2C::Error>> {
        // Reading the status register clears the INDET bit
        self.get_status_flags().await
    }

    /// Enable all interrupts (unmask all).
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn enable_all_interrupts(&mut self) -> Result<(), Error<I2C::Error>> {
        // Set only reserved bits, clear all mask bits (0 = enabled)
        let mask = MaskFlags::RESERVED_0 | MaskFlags::RESERVED_7;
        self.set_interrupt_mask(mask).await
    }

    /// Disable all interrupts (mask all).
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn disable_all_interrupts(&mut self) -> Result<(), Error<I2C::Error>> {
        // Set all mask bits (1 = disabled)
        let mask = MaskFlags::all();
        self.set_interrupt_mask(mask).await
    }

    /// Check if AC adapter insertion/removal interrupt is enabled.
    ///
    /// # Returns
    ///
    /// Returns `true` if interrupt is enabled, `false` if masked, or an `Error` if the operation fails.
    pub async fn is_ac_ok_interrupt_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let mask = self.get_interrupt_mask().await?;
        Ok(!mask.contains(MaskFlags::AC_OK_MASK))
    }

    /// Check if USB load detection interrupt is enabled.
    ///
    /// # Returns
    ///
    /// Returns `true` if interrupt is enabled, `false` if masked, or an `Error` if the operation fails.
    pub async fn is_indet_interrupt_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let mask = self.get_interrupt_mask().await?;
        Ok(!mask.contains(MaskFlags::INDET_MASK))
    }

    /// Check if over-temperature protection interrupt is enabled.
    ///
    /// # Returns
    ///
    /// Returns `true` if interrupt is enabled, `false` if masked, or an `Error` if the operation fails.
    pub async fn is_otp_interrupt_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let mask = self.get_interrupt_mask().await?;
        Ok(!mask.contains(MaskFlags::OTP_MASK))
    }

    /// Set VBUS output voltage for internal reference (discharging mode).
    ///
    /// # Arguments
    ///
    /// * `voltage_mv` - Desired VBUS output voltage in millivolts
    /// * `vbus_ratio` - VBUS ratio setting (0: 12.5x for >10.24V, 1: 5x for <10.24V)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_vbus_internal_voltage(
        &mut self,
        voltage_mv: u16,
        vbus_ratio: u8,
    ) -> Result<(), Error<I2C::Error>> {
        if vbus_ratio > 1 {
            return Err(Error::InvalidParameter);
        }

        // Calculate reference voltage based on formula:
        // VBUS = VBUSREF_I * VBUS_RATIO
        // VBUSREF_I = (4 * VBUSREF_I_SET + VBUSREF_I_SET2 + 1) * 2 mV
        let ratio_multiplier = match vbus_ratio {
            0 => 12.5,
            1 => 5.0,
            _ => return Err(Error::InvalidParameter),
        };

        let vbusref_i_mv = voltage_mv as f32 / ratio_multiplier;
        if !(2.0..=2048.0).contains(&vbusref_i_mv) {
            return Err(Error::InvalidParameter);
        }

        // Calculate 10-bit reference value
        let ref_value = ((vbusref_i_mv / 2.0) - 1.0) as u16;
        if ref_value > 1023 {
            return Err(Error::InvalidParameter);
        }

        let vbusref_i_set = (ref_value >> 2) as u8;
        let vbusref_i_set2 = (ref_value & 0x03) as u8;

        // Set the reference voltage registers
        self.write_register(Register::VbusrefISet, vbusref_i_set)
            .await?;
        self.write_register(Register::VbusrefISet2, (vbusref_i_set2 << 6) | 0x3F)
            .await?;

        // Set VBUS ratio in RATIO register
        let mut ratio_reg = self.read_register(Register::Ratio).await?;
        if vbus_ratio == 1 {
            ratio_reg |= 0x01; // Set VBUS_RATIO bit
        } else {
            ratio_reg &= !0x01; // Clear VBUS_RATIO bit
        }
        self.write_register(Register::Ratio, ratio_reg).await?;

        // Ensure FB_SEL is set to 0 for internal reference
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);
        flags.remove(Ctrl1Flags::FB_SEL);
        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Set VBUS output voltage for external reference (discharging mode).
    ///
    /// # Arguments
    ///
    /// * `reference_mv` - External reference voltage in millivolts (700-2048mV)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_vbus_external_reference(
        &mut self,
        reference_mv: u16,
    ) -> Result<(), Error<I2C::Error>> {
        if !(700..=2048).contains(&reference_mv) {
            return Err(Error::InvalidParameter);
        }

        // Calculate 10-bit reference value
        // VBUSREF_E = (4 * VBUSREF_E_SET + VBUSREF_E_SET2 + 1) * 2 mV
        let ref_value = ((reference_mv as f32 / 2.0) - 1.0) as u16;
        if ref_value > 1023 {
            return Err(Error::InvalidParameter);
        }

        let vbusref_e_set = (ref_value >> 2) as u8;
        let vbusref_e_set2 = (ref_value & 0x03) as u8;

        // Set the external reference voltage registers
        self.write_register(Register::VbusrefESet, vbusref_e_set)
            .await?;
        self.write_register(Register::VbusrefESet2, (vbusref_e_set2 << 6) | 0x3F)
            .await?;

        // Set FB_SEL to 1 for external reference
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);
        flags.insert(Ctrl1Flags::FB_SEL);
        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Set slew rate for VBUS dynamic voltage changes in discharging mode.
    ///
    /// # Arguments
    ///
    /// * `slew_setting` - Slew rate setting (0: 1mV/μs, 1: 2mV/μs, 2: 4mV/μs)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_vbus_slew_rate(&mut self, slew_setting: u8) -> Result<(), Error<I2C::Error>> {
        if slew_setting > 2 {
            return Err(Error::InvalidParameter);
        }

        let mut ctrl2 = self.read_register(Register::Ctrl2Set).await?;
        // Clear slew rate bits (bits 1-0) and set new value
        ctrl2 = (ctrl2 & !0x03) | (slew_setting & 0x03);
        self.write_register(Register::Ctrl2Set, ctrl2).await
    }

    /// Configure battery voltage settings.
    ///
    /// # Arguments
    ///
    /// * `cell_count` - Number of battery cells (1-4)
    /// * `voltage_per_cell` - Voltage per cell setting
    /// * `use_internal` - Whether to use internal voltage setting
    /// * `ir_comp_mohm` - IR compensation in milliohms (0, 20, 40, or 80)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_battery_voltage(
        &mut self,
        cell_count: u8,
        voltage_per_cell: crate::VoltagePerCell,
        use_internal: bool,
        ir_comp_mohm: u8,
    ) -> Result<(), Error<I2C::Error>> {
        if cell_count == 0 || cell_count > 4 {
            return Err(Error::InvalidParameter);
        }

        let ir_comp_setting = match ir_comp_mohm {
            0 => 0b00,
            20 => 0b01,
            40 => 0b10,
            80 => 0b11,
            _ => return Err(Error::InvalidParameter),
        };

        let vcell_setting = match voltage_per_cell {
            crate::VoltagePerCell::Mv4100 => 0b000,
            crate::VoltagePerCell::Mv4200 => 0b001,
            crate::VoltagePerCell::Mv4300 => 0b011,
            crate::VoltagePerCell::Mv4350 => 0b100,
            crate::VoltagePerCell::Mv4400 => 0b101,
            crate::VoltagePerCell::Mv4450 => 0b110,
        };

        let csel_setting = match cell_count {
            1 => 0b00,
            2 => 0b01,
            3 => 0b10,
            4 => 0b11,
            _ => return Err(Error::InvalidParameter),
        };

        let mut vbat_set = 0u8;
        vbat_set |= vcell_setting; // bits 2-0
        vbat_set |= csel_setting << 3; // bits 4-3
        if !use_internal {
            vbat_set |= 0x20; // bit 5: VBAT_SEL
        }
        vbat_set |= ir_comp_setting << 6; // bits 7-6

        self.write_register(Register::VbatSet, vbat_set).await
    }

    /// Set trickle charge threshold.
    ///
    /// # Arguments
    ///
    /// * `use_60_percent` - If `true`, uses 60% of VBAT target. If `false`, uses 70%.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_trickle_threshold(
        &mut self,
        use_60_percent: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);

        if use_60_percent {
            flags.insert(Ctrl1Flags::TRICKLE_SET);
        } else {
            flags.remove(Ctrl1Flags::TRICKLE_SET);
        }

        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Set End of Charge (EOC) current threshold.
    ///
    /// # Arguments
    ///
    /// * `use_tenth` - If `true`, uses 1/10 of charging current. If `false`, uses 1/25.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_eoc_threshold(&mut self, use_tenth: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if use_tenth {
            flags.insert(Ctrl3Flags::EOC_SET);
        } else {
            flags.remove(Ctrl3Flags::EOC_SET);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Set charging current selection (IBUS vs IBAT).
    ///
    /// # Arguments
    ///
    /// * `use_ibat` - If `true`, uses IBAT as charging current reference. If `false`, uses IBUS.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_charging_current_selection(
        &mut self,
        use_ibat: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);

        if use_ibat {
            flags.insert(Ctrl1Flags::ICHAR_SEL);
        } else {
            flags.remove(Ctrl1Flags::ICHAR_SEL);
        }

        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Set loop response control for improved performance.
    ///
    /// # Arguments
    ///
    /// * `improve_response` - If `true`, improves loop response. If `false`, uses normal response.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_loop_response(
        &mut self,
        improve_response: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if improve_response {
            flags.insert(Ctrl3Flags::LOOP_SET);
        } else {
            flags.remove(Ctrl3Flags::LOOP_SET);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Set ILIM loop bandwidth.
    ///
    /// # Arguments
    ///
    /// * `use_low_bandwidth` - If `true`, uses 1.25kHz bandwidth. If `false`, uses 5kHz.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_ilim_bandwidth(
        &mut self,
        use_low_bandwidth: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if use_low_bandwidth {
            flags.insert(Ctrl3Flags::ILIM_BW_SEL);
        } else {
            flags.remove(Ctrl3Flags::ILIM_BW_SEL);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Enable or disable current foldback for VBUS short circuit protection.
    ///
    /// # Arguments
    ///
    /// * `disable_foldback` - If `true`, disables current foldback. If `false`, enables it.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_short_foldback(
        &mut self,
        disable_foldback: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let ctrl3 = self.read_register(Register::Ctrl3Set).await?;
        let mut flags = Ctrl3Flags::from_bits_truncate(ctrl3);

        if disable_foldback {
            flags.insert(Ctrl3Flags::DIS_SHORT_FOLDBACK);
        } else {
            flags.remove(Ctrl3Flags::DIS_SHORT_FOLDBACK);
        }

        self.write_register(Register::Ctrl3Set, flags.bits()).await
    }

    /// Enable or disable OVP protection for discharging mode.
    ///
    /// # Arguments
    ///
    /// * `disable_ovp` - If `true`, disables OVP protection. If `false`, enables it.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_ovp_protection(&mut self, disable_ovp: bool) -> Result<(), Error<I2C::Error>> {
        let ctrl1 = self.read_register(Register::Ctrl1Set).await?;
        let mut flags = Ctrl1Flags::from_bits_truncate(ctrl1);

        if disable_ovp {
            flags.insert(Ctrl1Flags::DIS_OVP);
        } else {
            flags.remove(Ctrl1Flags::DIS_OVP);
        }

        self.write_register(Register::Ctrl1Set, flags.bits()).await
    }

    /// Configure the device with comprehensive settings.
    ///
    /// # Arguments
    ///
    /// * `config` - Device configuration structure
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_device(
        &mut self,
        config: &DeviceConfiguration,
    ) -> Result<(), Error<I2C::Error>> {
        // First, set the FACTORY bit in CTRL2_SET register as required by the datasheet
        // This must be done at the beginning of the I2C sequence after power up
        let ctrl2_current = self.read_register(Register::Ctrl2Set).await?;
        let mut ctrl2_flags = Ctrl2Flags::from_bits_truncate(ctrl2_current);
        ctrl2_flags.insert(Ctrl2Flags::FACTORY);
        self.write_register(Register::Ctrl2Set, ctrl2_flags.bits())
            .await?;

        // Configure battery settings
        self.configure_battery_voltage(
            config.battery.cell_count.into(),
            config.battery.voltage_per_cell,
            config.battery.use_internal_setting,
            config.battery.ir_compensation_mohm.into(),
        )
        .await?;

        // Configure current limits
        self.set_ibus_limit(
            config.current_limits.ibus_limit_ma,
            config.current_limits.ibus_ratio.into(),
            config.current_limits.rs1_mohm,
        )
        .await?;

        self.set_ibat_limit(
            config.current_limits.ibat_limit_ma,
            config.current_limits.ibat_ratio.into(),
            config.current_limits.rs2_mohm,
        )
        .await?;

        // Configure power management
        self.set_otg_mode(config.power.operating_mode == OperatingMode::OTG)
            .await?;
        self.set_switching_frequency(config.power.switching_frequency.into())
            .await?;
        self.set_dead_time(config.power.dead_time.into()).await?;
        self.set_frequency_dithering(config.power.frequency_dithering)
            .await?;
        self.set_pfm_mode(config.power.pfm_mode).await?;
        self.set_vinreg_voltage(
            config.power.vinreg_voltage_mv,
            config.power.vinreg_ratio.into(),
        )
        .await?;

        // Configure charging settings
        self.set_trickle_charging(config.trickle_charging).await?;
        self.set_charging_termination(config.charging_termination)
            .await?;
        self.set_charging_current_selection(!config.use_ibus_for_charging)
            .await?;

        Ok(())
    }

    /// Get comprehensive device status.
    ///
    /// # Returns
    ///
    /// Returns the current device status, or an `Error` if the operation fails.
    pub async fn get_device_status(&mut self) -> Result<SC8815Status, Error<I2C::Error>> {
        let status_flags = self.get_status_flags().await?;

        Ok(SC8815Status {
            eoc: status_flags.contains(StatusFlags::EOC),
            otp_fault: status_flags.contains(StatusFlags::OTP),
            vbus_short_fault: status_flags.contains(StatusFlags::VBUS_SHORT),
            usb_load_detected: status_flags.contains(StatusFlags::INDET),
            ac_adapter_connected: status_flags.contains(StatusFlags::AC_OK),
        })
    }

    /// Get all ADC measurements with default ratios.
    ///
    /// # Returns
    ///
    /// Returns ADC measurements structure, or an `Error` if the operation fails.
    pub async fn get_adc_measurements(&mut self) -> Result<AdcMeasurements, Error<I2C::Error>> {
        // Use default ratios and resistor values
        let (vbus_mv, vbat_mv, ibus_ma, ibat_ma, adin_mv) = self
            .read_all_adc_values(
                0,  // VBUS ratio: 12.5x
                0,  // VBAT ratio: 12.5x
                2,  // IBUS ratio: 3x
                1,  // IBAT ratio: 12x
                10, // RS1: 10mΩ
                10, // RS2: 10mΩ
            )
            .await?;

        Ok(AdcMeasurements {
            vbus_mv,
            vbat_mv,
            ibus_ma,
            ibat_ma,
            adin_mv,
        })
    }

    /// Get all ADC measurements with custom configuration.
    ///
    /// # Arguments
    ///
    /// * `current_config` - Current limit configuration containing ratio and resistor settings
    ///
    /// # Returns
    ///
    /// Returns ADC measurements structure, or an `Error` if the operation fails.
    pub async fn get_adc_measurements_with_config(
        &mut self,
        current_config: &CurrentLimitConfiguration,
    ) -> Result<AdcMeasurements, Error<I2C::Error>> {
        let (vbus_mv, vbat_mv, ibus_ma, ibat_ma, adin_mv) = self
            .read_all_adc_values(
                0, // VBUS ratio: 12.5x (read from RATIO register if needed)
                0, // VBAT ratio: 12.5x (read from RATIO register if needed)
                current_config.ibus_ratio.into(),
                current_config.ibat_ratio.into(),
                current_config.rs1_mohm,
                current_config.rs2_mohm,
            )
            .await?;

        Ok(AdcMeasurements {
            vbus_mv,
            vbat_mv,
            ibus_ma,
            ibat_ma,
            adin_mv,
        })
    }

    /// Get the current battery status.
    ///
    /// # Returns
    ///
    /// Returns the current battery status, or an `Error` if the operation fails.
    pub async fn get_battery_status(&mut self) -> Result<BatteryStatus, Error<I2C::Error>> {
        let status = self.read_register(Register::BatteryStatus).await?;
        let flags = BatteryStatusFlags::from_bits_truncate(status);

        if flags.intersects(
            BatteryStatusFlags::BATTERY_LOW_VOLTAGE
                | BatteryStatusFlags::BATTERY_OVERVOLTAGE
                | BatteryStatusFlags::BATTERY_OVERCURRENT
                | BatteryStatusFlags::BATTERY_TEMP_FAULT,
        ) {
            Ok(BatteryStatus::Fault)
        } else if flags.contains(BatteryStatusFlags::BATTERY_PRESENT) {
            Ok(BatteryStatus::Connected)
        } else {
            Ok(BatteryStatus::NotConnected)
        }
    }

    /// Get the current input source status.
    ///
    /// # Returns
    ///
    /// Returns the current input source status, or an `Error` if the operation fails.
    pub async fn get_input_source_status(
        &mut self,
    ) -> Result<InputSourceStatus, Error<I2C::Error>> {
        let status = self.read_register(Register::InputSourceStatus).await?;
        let flags = InputSourceStatusFlags::from_bits_truncate(status);

        if flags.intersects(
            InputSourceStatusFlags::INPUT_UNDERVOLTAGE
                | InputSourceStatusFlags::INPUT_OVERVOLTAGE
                | InputSourceStatusFlags::INPUT_OVERCURRENT,
        ) {
            Ok(InputSourceStatus::Fault)
        } else if flags.contains(InputSourceStatusFlags::INPUT_SOURCE_PRESENT) {
            Ok(InputSourceStatus::Connected)
        } else {
            Ok(InputSourceStatus::NotConnected)
        }
    }

    /// Get the current thermal status.
    ///
    /// # Returns
    ///
    /// Returns the current thermal status, or an `Error` if the operation fails.
    pub async fn get_thermal_status(&mut self) -> Result<ThermalStatus, Error<I2C::Error>> {
        let status = self.read_register(Register::ThermalStatus).await?;
        let flags = ThermalStatusFlags::from_bits_truncate(status);

        if flags.contains(ThermalStatusFlags::THERMAL_SHUTDOWN) {
            Ok(ThermalStatus::Shutdown)
        } else if flags.contains(ThermalStatusFlags::TEMP_WARNING) {
            Ok(ThermalStatus::Warning)
        } else {
            Ok(ThermalStatus::Normal)
        }
    }

    /// Check if power is good (device is operating normally).
    ///
    /// # Returns
    ///
    /// Returns `true` if power is good, `false` otherwise, or an `Error` if the operation fails.
    pub async fn is_power_good(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.get_status_flags().await?;
        // Power is good if no critical faults are present
        Ok(!status.intersects(StatusFlags::OTP | StatusFlags::VBUS_SHORT))
    }

    /// Get the current charging state.
    ///
    /// # Returns
    ///
    /// Returns the current charging state, or an `Error` if the operation fails.
    pub async fn get_charging_state(&mut self) -> Result<ChargingState, Error<I2C::Error>> {
        let status = self.get_status_flags().await?;
        let is_otg = self.is_otg_mode().await?;

        if is_otg {
            Ok(ChargingState::Discharging)
        } else if status.contains(StatusFlags::EOC) {
            Ok(ChargingState::Complete)
        } else if status.contains(StatusFlags::AC_OK) {
            Ok(ChargingState::Charging)
        } else {
            Ok(ChargingState::NotCharging)
        }
    }

    /// Reset the device by clearing all control registers.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        // Clear all control registers to reset state
        self.write_register(Register::Ctrl0Set, 0x00).await?;
        self.write_register(Register::Ctrl1Set, 0x00).await?;
        self.write_register(Register::Ctrl2Set, 0x00).await?;
        self.write_register(Register::Ctrl3Set, 0x00).await?;

        // Re-initialize with default settings
        self.init().await
    }
}
