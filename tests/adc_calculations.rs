#[cfg(test)]
mod tests {
    use sc8815::driver::AdcCalculations;

    #[test]
    fn test_voltage_calculations() {
        // Test VBUS/VBAT voltage calculation with 12.5x ratio
        // Formula: Voltage = (4 * ADC_VALUE + ADC_VALUE2 + 1) * RATIO * 2 mV

        // Test case 1: ADC_VALUE = 100, ADC_VALUE2 = 2, RATIO = 12.5x (ratio = 0)
        // Expected: (4 * 100 + 2 + 1) * 12.5 * 2 = 403 * 12.5 * 2 = 10075 mV
        let high = 100;
        let low = 0b10000000; // ADC_VALUE2 = 2 in bits 7-6
        let ratio = 0; // 12.5x
        let result = AdcCalculations::calculate_voltage_mv(high, low, ratio).unwrap();
        assert_eq!(result, 10075);

        // Test case 2: ADC_VALUE = 50, ADC_VALUE2 = 1, RATIO = 5x (ratio = 1)
        // Expected: (4 * 50 + 1 + 1) * 5 * 2 = 202 * 5 * 2 = 2020 mV
        let high = 50;
        let low = 0b01000000; // ADC_VALUE2 = 1 in bits 7-6
        let ratio = 1; // 5x
        let result = AdcCalculations::calculate_voltage_mv(high, low, ratio).unwrap();
        assert_eq!(result, 2020);

        // Test case 3: Minimum values
        // ADC_VALUE = 0, ADC_VALUE2 = 0, RATIO = 12.5x
        // Expected: (4 * 0 + 0 + 1) * 12.5 * 2 = 1 * 12.5 * 2 = 25 mV
        let high = 0;
        let low = 0b00000000; // ADC_VALUE2 = 0
        let ratio = 0; // 12.5x
        let result = AdcCalculations::calculate_voltage_mv(high, low, ratio).unwrap();
        assert_eq!(result, 25);
    }

    #[test]
    fn test_current_calculations() {
        // Test IBUS/IBAT current calculation
        // Formula: Current (A) = (4 × ADC_VALUE + ADC_VALUE2 + 1) × 2 × 10mΩ × RATIO / (1200 × RS)

        // Test case 1: IBUS with 3x ratio and 5mΩ resistor
        // ADC_VALUE = 100, ADC_VALUE2 = 2, RATIO = 3x, RS = 5mΩ
        // Expected: (4 * 100 + 2 + 1) * 2 * 10 * 3 / (1200 * 5) = 403 * 60 / 6000 = 4.03 A = 4030 mA
        let high = 100;
        let low = 0b10000000; // ADC_VALUE2 = 2 in bits 7-6
        let ratio_multiplier = 3; // 3x ratio
        let rs_mohm = 5;
        let result = AdcCalculations::calculate_current_ma(high, low, ratio_multiplier, rs_mohm);
        assert_eq!(result, 4030);

        // Test case 2: IBAT with 12x ratio and 5mΩ resistor
        // ADC_VALUE = 50, ADC_VALUE2 = 1, RATIO = 12x, RS = 5mΩ
        // Expected: (4 * 50 + 1 + 1) * 2 * 10 * 12 / (1200 * 5) = 202 * 240 / 6000 = 8.08 A = 8080 mA
        let high = 50;
        let low = 0b01000000; // ADC_VALUE2 = 1 in bits 7-6
        let ratio_multiplier = 12; // 12x ratio
        let rs_mohm = 5;
        let result = AdcCalculations::calculate_current_ma(high, low, ratio_multiplier, rs_mohm);
        assert_eq!(result, 8080);

        // Test case 3: Minimum values
        // ADC_VALUE = 0, ADC_VALUE2 = 0, RATIO = 6x, RS = 10mΩ
        // Expected: (4 * 0 + 0 + 1) * 2 * 10 * 6 / (1200 * 10) = 1 * 120 / 12000 = 0.01 A = 10 mA
        let high = 0;
        let low = 0b00000000; // ADC_VALUE2 = 0
        let ratio_multiplier = 6; // 6x ratio
        let rs_mohm = 10;
        let result = AdcCalculations::calculate_current_ma(high, low, ratio_multiplier, rs_mohm);
        assert_eq!(result, 10);
    }

    #[test]
    fn test_ibus_current_calculations() {
        // Test IBUS current calculation with ratio conversion

        // Test case 1: IBUS with ratio = 2 (3x multiplier) and 5mΩ resistor
        // Expected: (4 * 100 + 2 + 1) * 2 * 10 * 3 / (1200 * 5) = 403 * 60 / 6000 = 4030 mA
        let high = 100;
        let low = 0b10000000; // ADC_VALUE2 = 2
        let ratio = 2; // Should map to 3x multiplier
        let rs1_mohm = 5;
        let result =
            AdcCalculations::calculate_ibus_current_ma(high, low, ratio, rs1_mohm).unwrap();
        assert_eq!(result, 4030);

        // Test case 2: IBUS with ratio = 1 (6x multiplier) and 10mΩ resistor
        // Expected: (4 * 50 + 1 + 1) * 2 * 10 * 6 / (1200 * 10) = 202 * 120 / 12000 = 2020 mA
        let high = 50;
        let low = 0b01000000; // ADC_VALUE2 = 1
        let ratio = 1; // Should map to 6x multiplier
        let rs1_mohm = 10;
        let result =
            AdcCalculations::calculate_ibus_current_ma(high, low, ratio, rs1_mohm).unwrap();
        assert_eq!(result, 2020);
    }

    #[test]
    fn test_ibat_current_calculations() {
        // Test IBAT current calculation with ratio conversion

        // Test case 1: IBAT with ratio = 1 (12x multiplier) and 5mΩ resistor
        // Expected: (4 * 50 + 1 + 1) * 2 * 10 * 12 / (1200 * 5) = 202 * 240 / 6000 = 8080 mA
        let high = 50;
        let low = 0b01000000; // ADC_VALUE2 = 1
        let ratio = 1; // Should map to 12x multiplier
        let rs2_mohm = 5;
        let result =
            AdcCalculations::calculate_ibat_current_ma(high, low, ratio, rs2_mohm).unwrap();
        assert_eq!(result, 8080);

        // Test case 2: IBAT with ratio = 0 (6x multiplier) and 10mΩ resistor
        // Expected: (4 * 100 + 2 + 1) * 2 * 10 * 6 / (1200 * 10) = 403 * 120 / 12000 = 4030 mA
        let high = 100;
        let low = 0b10000000; // ADC_VALUE2 = 2
        let ratio = 0; // Should map to 6x multiplier
        let rs2_mohm = 10;
        let result =
            AdcCalculations::calculate_ibat_current_ma(high, low, ratio, rs2_mohm).unwrap();
        assert_eq!(result, 4030);
    }

    #[test]
    fn test_adin_voltage_calculations() {
        // Test ADIN voltage calculation
        // Formula: VADIN = (4 * ADIN_VALUE + ADIN_VALUE2 + 1) * 2 mV

        // Test case 1: ADIN_VALUE = 100, ADIN_VALUE2 = 2
        // Expected: (4 * 100 + 2 + 1) * 2 = 403 * 2 = 806 mV
        let high = 100;
        let low = 0b10000000; // ADIN_VALUE2 = 2 in bits 7-6
        let result = AdcCalculations::calculate_adin_voltage_mv(high, low);
        assert_eq!(result, 806);

        // Test case 2: Maximum values (ADIN_VALUE = 255, ADIN_VALUE2 = 3)
        // Expected: (4 * 255 + 3 + 1) * 2 = 1024 * 2 = 2048 mV
        let high = 255;
        let low = 0b11000000; // ADIN_VALUE2 = 3 in bits 7-6
        let result = AdcCalculations::calculate_adin_voltage_mv(high, low);
        assert_eq!(result, 2048);

        // Test case 3: Minimum values
        // Expected: (4 * 0 + 0 + 1) * 2 = 1 * 2 = 2 mV
        let high = 0;
        let low = 0b00000000; // ADIN_VALUE2 = 0
        let result = AdcCalculations::calculate_adin_voltage_mv(high, low);
        assert_eq!(result, 2);
    }

    #[test]
    fn test_adc_register_combination() {
        // Test the combine_adc_registers function

        // Test case 1: high = 100, low with ADC_VALUE2 = 2
        let high = 100;
        let low = 0b10000000; // ADC_VALUE2 = 2 in bits 7-6
        let result = AdcCalculations::combine_adc_registers(high, low);
        let expected = (100 << 2) | 2; // 400 + 2 = 402
        assert_eq!(result, expected);

        // Test case 2: high = 255, low with ADC_VALUE2 = 3
        let high = 255;
        let low = 0b11000000; // ADC_VALUE2 = 3 in bits 7-6
        let result = AdcCalculations::combine_adc_registers(high, low);
        let expected = (255 << 2) | 3; // 1020 + 3 = 1023
        assert_eq!(result, expected);

        // Test case 3: Minimum values
        let high = 0;
        let low = 0b00000000; // ADC_VALUE2 = 0
        let result = AdcCalculations::combine_adc_registers(high, low);
        assert_eq!(result, 0);
    }
}
