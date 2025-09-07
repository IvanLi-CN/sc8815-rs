//! Integration tests for the SC8815 driver

use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use sc8815::{SC8815, registers::constants::DEFAULT_ADDRESS};

#[cfg(not(feature = "async"))]
#[test]
fn test_device_initialization() {
    let expectations = [
        // Read status register to verify communication
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]), // AC_OK bit set
        // Read CTRL2_SET register
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        // Write CTRL2_SET with FACTORY bit set
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        // Read MASK register
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        // Write MASK register with RESERVED_0 bit set
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    let result = sc8815.init();
    assert!(result.is_ok());

    i2c.done();
}

#[cfg(not(feature = "async"))]
#[test]
fn test_ac_adapter_status_check() {
    let expectations = [
        // Initialization sequence
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        // Read status register (AC_OK bit set)
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    sc8815.init().unwrap();
    let ac_connected = sc8815.is_ac_adapter_connected().unwrap();
    assert!(ac_connected);

    i2c.done();
}

#[cfg(not(feature = "async"))]
#[test]
fn test_otg_mode_control() {
    let expectations = [
        // Initialization sequence
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        // Read CTRL0_SET register
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x09], vec![0x00]),
        // Write CTRL0_SET with EN_OTG bit set
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x09, 0x80]),
        // Read CTRL0_SET register to check OTG mode
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x09], vec![0x80]),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    sc8815.init().unwrap();
    sc8815.set_otg_mode(true).unwrap();
    let is_otg = sc8815.is_otg_mode().unwrap();
    assert!(is_otg);

    i2c.done();
}

#[cfg(not(feature = "async"))]
#[test]
fn test_device_status() {
    let expectations = [
        // Initialization sequence
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        // Read status register for device status (AC_OK and EOC bits set)
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x42]),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    sc8815.init().unwrap();
    let status = sc8815.get_device_status().unwrap();

    assert!(status.ac_adapter_connected);
    assert!(status.eoc);
    assert!(!status.otp_fault);
    assert!(!status.vbus_short_fault);
    assert!(!status.usb_load_detected);

    i2c.done();
}

#[cfg(not(feature = "async"))]
#[test]
fn test_adc_measurements() {
    let expectations = [
        // Initialization sequence
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        // Read all ADC registers at once (10 bytes starting from VBUS_FB_VALUE 0x0D)
        // This matches the read_all_adc_registers implementation
        I2cTransaction::write_read(
            DEFAULT_ADDRESS,
            vec![0x0D], // VBUS_FB_VALUE register address
            vec![0x80, 0x40, 0x60, 0x80, 0x40, 0xC0, 0x30, 0x00, 0x20, 0x40], // 10 bytes: VBUS_FB_VALUE, VBUS_FB_VALUE2, VBAT_FB_VALUE, VBAT_FB_VALUE2, IBUS_VALUE, IBUS_VALUE2, IBAT_VALUE, IBAT_VALUE2, ADIN_VALUE, ADIN_VALUE2
        ),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    sc8815.init().unwrap();
    let measurements = sc8815.get_adc_measurements().unwrap();

    // Verify that measurements are non-zero (exact values depend on calculation)
    assert!(measurements.vbus_mv > 0);
    assert!(measurements.vbat_mv > 0);
    assert!(measurements.ibus_ma > 0);
    assert!(measurements.ibat_ma > 0);
    assert!(measurements.adin_mv > 0);

    i2c.done();
}

#[cfg(not(feature = "async"))]
#[test]
fn test_interrupt_configuration() {
    let expectations = [
        // Initialization sequence
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        // Read current mask register
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x01]),
        // Write mask register to enable AC_OK interrupt (clear AC_OK_MASK bit)
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]), // Only RESERVED_0 set
        // Read mask register to verify
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x01]),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    sc8815.init().unwrap();

    // Enable AC_OK interrupt
    use sc8815::registers::MaskFlags;
    sc8815.enable_interrupts(MaskFlags::AC_OK_MASK).unwrap();

    // Check if AC_OK interrupt is enabled
    let is_enabled = sc8815.is_ac_ok_interrupt_enabled().unwrap();
    assert!(is_enabled);

    i2c.done();
}

#[cfg(feature = "async")]
mod async_tests {
    use super::*;

    #[tokio::test]
    async fn test_async_device_initialization() {
        let expectations = [
            // Read status register to verify communication
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
            // Read CTRL2_SET register
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
            // Write CTRL2_SET with FACTORY bit set
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
            // Read MASK register
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
            // Write MASK register with RESERVED_0 bit set
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

        let result = sc8815.init().await;
        assert!(result.is_ok());

        i2c.done();
    }

    #[tokio::test]
    async fn test_async_ac_adapter_status() {
        let expectations = [
            // Initialization sequence
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
            // Read status register (AC_OK bit set)
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

        sc8815.init().await.unwrap();
        let ac_connected = sc8815.is_ac_adapter_connected().await.unwrap();
        assert!(ac_connected);

        i2c.done();
    }
}

#[cfg(not(feature = "async"))]
#[test]
fn test_gpo_control_and_status() {
    let expectations = [
        // Initialization sequence
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
        // Read CTRL3 register (GPO_CTRL bit not set)
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x00]),
        // Set GPO control (enable GPO) - first read current state
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x00]),
        // Then write with GPO_CTRL bit set
        I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0C, 0x40]), // GPO_CTRL bit (bit 6)
        // Read CTRL3 register again (GPO_CTRL bit set)
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x40]),
        // Read CTRL3 for power path status (GPO_CTRL set, EN_PGATE not set)
        I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x40]),
    ];
    let mut i2c = I2cMock::new(&expectations);
    let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

    sc8815.init().unwrap();

    // Test initial GPO status (should be false/high-Z)
    let gpo_status = sc8815.is_gpo_pulled_low().unwrap();
    assert!(!gpo_status);

    // Pull GPO LOW
    sc8815.set_gpo_pull_low(true).unwrap();

    // Test GPO status after pulling low (should be true)
    let gpo_status = sc8815.is_gpo_pulled_low().unwrap();
    assert!(gpo_status);

    // Test power path status
    let power_path = sc8815.get_power_path_status().unwrap();
    assert!(power_path.gpo_pulled_low);
    assert!(!power_path.pgate_enabled); // PGATE not enabled in this test

    i2c.done();
}

#[cfg(feature = "async")]
mod async_gpo_tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[tokio::test]
    async fn test_async_gpo_control_and_status() {
        let expectations = [
            // Initialization sequence
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x17], vec![0x40]),
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0B], vec![0x00]),
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0B, 0x08]),
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x19], vec![0x00]),
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x19, 0x01]),
            // Read CTRL3 register (GPO_CTRL bit not set)
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x00]),
            // Set GPO control (enable GPO) - first read current state
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x00]),
            // Then write with GPO_CTRL bit set
            I2cTransaction::write(DEFAULT_ADDRESS, vec![0x0C, 0x40]), // GPO_CTRL bit (bit 6)
            // Read CTRL3 register again (GPO_CTRL bit set)
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x40]),
            // Read CTRL3 for power path status (GPO_CTRL set, EN_PGATE not set)
            I2cTransaction::write_read(DEFAULT_ADDRESS, vec![0x0C], vec![0x40]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sc8815 = SC8815::new(&mut i2c, DEFAULT_ADDRESS);

        sc8815.init().await.unwrap();

        // Test initial GPO status (should be false/high-Z)
        let gpo_status = sc8815.is_gpo_pulled_low().await.unwrap();
        assert!(!gpo_status);

        // Pull GPO LOW
        sc8815.set_gpo_pull_low(true).await.unwrap();

        // Test GPO status after pulling low (should be true)
        let gpo_status = sc8815.is_gpo_pulled_low().await.unwrap();
        assert!(gpo_status);

        // Test power path status
        let power_path = sc8815.get_power_path_status().await.unwrap();
        assert!(power_path.gpo_pulled_low);
        assert!(!power_path.pgate_enabled); // PGATE not enabled in this test

        i2c.done();
    }
}
