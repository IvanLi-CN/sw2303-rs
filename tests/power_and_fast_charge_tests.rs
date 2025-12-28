//! Power (voltage/current) and Fast-Charge configuration tests for SW2303

#![cfg(all(test, not(feature = "async")))]

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use std::collections::HashMap;
use sw2303::{
    FastChargeConfiguration, PowerRequest, SW2303, error::Error,
    registers::constants::DEFAULT_ADDRESS,
};

/// Mock I2C implementation for testing
#[derive(Debug, Default)]
struct MockI2c {
    registers: HashMap<u8, u8>,
    write_enabled: bool,
}

impl MockI2c {
    fn new() -> Self {
        Self {
            registers: HashMap::new(),
            write_enabled: false,
        }
    }

    fn set_register(&mut self, reg: u8, value: u8) {
        self.registers.insert(reg, value);
    }

    fn get_register(&self, reg: u8) -> u8 {
        self.registers.get(&reg).copied().unwrap_or(0)
    }
}

impl ErrorType for MockI2c {
    type Error = ErrorKind;
}

impl I2c for MockI2c {
    fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() == 2 {
            let reg = bytes[0];
            let value = bytes[1];

            // Simulate unlock mechanism - check for writes to REG 0x12 with unlock values
            if reg == 0x12 && (value == 0x20 || value == 0x40 || value == 0x80) {
                if value == 0x80 {
                    self.write_enabled = true;
                }
                self.registers.insert(reg, value);
                return Ok(());
            }

            // Only allow writes to 0xA0-0xBF if unlocked
            if reg >= 0xA0 && reg <= 0xBF && !self.write_enabled {
                return Err(ErrorKind::Other);
            }

            self.registers.insert(reg, value);
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        _address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        if bytes.len() == 1 && buffer.len() == 1 {
            let reg = bytes[0];
            buffer[0] = self.get_register(reg);
        }
        Ok(())
    }

    fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn test_voltage_encoding_decoding_10mv_units() {
    let mut i2c = MockI2c::new();
    // REG 0x04 low nibble is reserved; keep a sentinel value to verify RMW.
    i2c.set_register(0x04, 0x0A);

    {
        let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
        sw2303.init().unwrap();

        // 5000mV -> dac_vol=500 -> 0x1F4 => REG0x03=0x1F, REG0x04[7:4]=0x4
        sw2303.set_voltage(5000).unwrap();
        let v = sw2303.get_voltage().unwrap();
        assert_eq!(v, 5000);

        let err = sw2303.set_voltage(50_000).unwrap_err();
        assert!(matches!(err, Error::InvalidParameter));
    }

    assert_eq!(i2c.get_register(0x03), 0x1F);
    assert_eq!(i2c.get_register(0x04), 0x4A);
}

#[test]
fn test_current_limit_encoding_decoding_50ma_units() {
    let mut i2c = MockI2c::new();
    // Preserve bit 7 (unknown/reserved); set it to 1 as a sentinel.
    i2c.set_register(0x05, 0x80);

    {
        let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
        sw2303.init().unwrap();

        // Manual: 1000mA + ctrl_icc[6:0] * 50mA
        // 2500mA -> ctrl_icc = (2500 - 1000) / 50 = 30
        sw2303.set_current_limit_ma(2500).unwrap();

        let i = sw2303.get_current_limit_ma().unwrap();
        assert_eq!(i, 2500);

        let err = sw2303.set_current_limit_ma(950).unwrap_err();
        assert!(matches!(err, Error::InvalidParameter));

        let err = sw2303.set_current_limit_ma(1025).unwrap_err();
        assert!(matches!(err, Error::InvalidParameter));

        let err = sw2303.set_current_limit_ma(10_000).unwrap_err();
        assert!(matches!(err, Error::InvalidParameter));
    }

    assert_eq!(i2c.get_register(0x05), 0x9E);
}

#[test]
fn test_get_power_request_decoding() {
    let mut i2c = MockI2c::new();
    // 5000mV encoded as REG0x03=0x1F, REG0x04[7:4]=0x4
    i2c.set_register(0x03, 0x1F);
    i2c.set_register(0x04, 0x4A);
    // Manual: 1000mA + ctrl_icc[6:0] * 50mA
    // 2500mA -> ctrl_icc = (2500 - 1000) / 50 = 30 -> 0x1E in bits[6:0]
    i2c.set_register(0x05, 0x80 | 0x1E);

    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
    sw2303.init().unwrap();

    let pr = sw2303.get_power_request().unwrap();
    assert_eq!(
        pr,
        PowerRequest {
            voltage_mv: 5000,
            current_limit_ma: 2500
        }
    );
}

#[test]
fn test_get_chip_version() {
    let mut i2c = MockI2c::new();
    i2c.set_register(0x01, 0b1010_0011);

    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
    sw2303.init().unwrap();

    let version = sw2303.get_chip_version().unwrap();
    assert_eq!(version, 0x03);
}

#[test]
fn test_fast_charge_configuration_roundtrip() {
    let mut i2c = MockI2c::new();
    // Seed registers with non-zero values to verify reserved bits are preserved.
    i2c.set_register(0xAD, 0xF6);
    i2c.set_register(0xAE, 0x30);
    i2c.set_register(0xB0, 0xC0);
    i2c.set_register(0xB1, 0xA0);
    i2c.set_register(0xB2, 0xC7);

    let cfg = FastChargeConfiguration {
        qc_enabled: true,
        fcp_enabled: true,
        afc_enabled: false,
        scp_enabled: true,
        pe20_enabled: false,
        sfcp_enabled: true,
        bc12_enabled: true,
        scp_current_limit: 1,
        fcp_afc_sfcp_2_25a: true,
        qc20_20v_enabled: true,
        qc30_20v_enabled: false,
        pe20_20v_enabled: true,
        pd_12v_enabled: true,
    };

    {
        let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);
        sw2303.init().unwrap();
        sw2303.unlock_write_enable_0().unwrap();

        sw2303.configure_fast_charge(cfg.clone()).unwrap();

        let got = sw2303.get_fast_charge_status().unwrap();
        assert_eq!(got, cfg);

        let err = sw2303
            .configure_fast_charge(FastChargeConfiguration {
                scp_current_limit: 3,
                ..Default::default()
            })
            .unwrap_err();
        assert!(matches!(err, Error::InvalidParameter));
    }

    assert_eq!(i2c.get_register(0xAD), 0xFE);
    assert_eq!(i2c.get_register(0xAE), 0x3E);
    assert_eq!(i2c.get_register(0xB0), 0xC0);
    assert_eq!(i2c.get_register(0xB1), 0xA6);
    assert_eq!(i2c.get_register(0xB2), 0xD7);
}
