//! Protocol Configuration Tests for SW2303
//!
//! This module contains tests for the protocol configuration functionality
//! of the SW2303 USB PD controller driver.

#![cfg(all(test, not(feature = "async")))]

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use std::collections::HashMap;
use sw2303::{
    PdConfiguration, ProtocolConfiguration, ProtocolType, SW2303, TypeCConfiguration,
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

    #[allow(dead_code)]
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
    fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        if buffer.len() == 1 {
            // Single register read - this is actually a write_read operation
            Ok(())
        } else {
            Ok(())
        }
    }

    fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() == 2 {
            let reg = bytes[0];
            let value = bytes[1];

            // Simulate unlock mechanism - check for writes to REG 0x12 with unlock values
            if reg == 0x12 && (value == 0x20 || value == 0x40 || value == 0x80) {
                if value == 0x80 {
                    // Final unlock step
                    self.write_enabled = true;
                }
                self.registers.insert(reg, value);
                return Ok(());
            }

            // Only allow writes to 0xA0-0xBF if unlocked
            if reg >= 0xA0 && reg <= 0xBF && !self.write_enabled {
                return Err(ErrorKind::Other); // Simulate write protection
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
        // Simple mock implementation
        Ok(())
    }
}

#[test]
fn test_protocol_configuration_basic() {
    let mut i2c = MockI2c::new();
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize and unlock
    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    // Test basic protocol configuration
    let config = ProtocolConfiguration {
        pd_enabled: true,
        qc20_enabled: true,
        qc30_enabled: true,
        fcp_enabled: false,
        afc_enabled: false,
        scp_enabled: false,
        pe20_enabled: false,
        bc12_enabled: false,
        sfcp_enabled: false,
    };

    // This should not panic
    let result = sw2303.configure_protocols(config);
    if let Err(e) = &result {
        println!("Error in configure_protocols: {:?}", e);
    }
    assert!(result.is_ok());
}

#[test]
fn test_pd_configuration() {
    let mut i2c = MockI2c::new();
    // REG 0xA6 has reserved bits with non‑zero defaults (bit7=1, bits5‑0=0x30).
    // Seed a realistic value to ensure configure_pd preserves reserved bits via RMW.
    i2c.set_register(0xA6, 0xB0);
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize and unlock
    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    // Test PD configuration
    let pd_config = PdConfiguration {
        enabled: true,
        vconn_swap: true,
        dr_swap: false,
        emarker_enabled: true,
        pps_enabled: true,
        fixed_voltages: [true, true, false, false], // 9V, 12V enabled
        emark_5a_bypass: false,
        emarker_60_70w: true,
    };

    let result = sw2303.configure_pd(pd_config);
    assert!(result.is_ok());

    // emark_5a_bypass=false should keep the seeded reserved bits intact.
    let pd_cfg3 = sw2303.get_pd_config_3_raw().unwrap();
    assert_eq!(pd_cfg3.bits(), 0xB0);

    // Now enable bypass and ensure only bit6 changes.
    let pd_config_bypass = PdConfiguration {
        enabled: true,
        vconn_swap: true,
        dr_swap: false,
        emarker_enabled: true,
        pps_enabled: true,
        fixed_voltages: [true, true, false, false],
        emark_5a_bypass: true,
        emarker_60_70w: true,
    };
    let result = sw2303.configure_pd(pd_config_bypass);
    assert!(result.is_ok());
    let pd_cfg3 = sw2303.get_pd_config_3_raw().unwrap();
    assert_eq!(pd_cfg3.bits(), 0xF0);
}

#[test]
fn test_type_c_configuration() {
    let mut i2c = MockI2c::new();
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize and unlock
    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    // Test Type-C configuration
    let type_c_config = TypeCConfiguration {
        current_1_5a: true,
        pd_pps_5a: true,
        cc_un_driving: false,
    };

    let result = sw2303.configure_type_c(type_c_config);
    assert!(result.is_ok());
}

#[test]
fn test_protocol_enable_disable() {
    let mut i2c = MockI2c::new();
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize and unlock
    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    // Test PD enable/disable
    assert!(sw2303.enable_pd_protocol().is_ok());
    assert!(sw2303.disable_pd_protocol().is_ok());

    // Test fast charge enable/disable
    assert!(sw2303.enable_fast_charge_protocol().is_ok());
    assert!(sw2303.disable_fast_charge_protocol().is_ok());

    // Test Type-C 1.5A enable/disable
    assert!(sw2303.enable_type_c_1_5a().is_ok());
    assert!(sw2303.disable_type_c_1_5a().is_ok());
}

#[test]
fn test_protocol_status_query() {
    let mut i2c = MockI2c::new();
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize
    sw2303.init().unwrap();

    // Test protocol status queries
    let result = sw2303.get_protocol_status();
    assert!(result.is_ok());

    let result = sw2303.is_protocol_enabled(ProtocolType::PD);
    assert!(result.is_ok());

    let result = sw2303.get_negotiated_protocol();
    assert!(result.is_ok());

    let result = sw2303.get_type_c_status();
    assert!(result.is_ok());
}

#[test]
fn test_default_configurations() {
    // Test that default configurations are sensible
    let protocol_config = ProtocolConfiguration::default();
    assert!(!protocol_config.pd_enabled); // Should be disabled by default
    assert!(!protocol_config.qc20_enabled);

    let pd_config = PdConfiguration::default();
    assert!(!pd_config.enabled);
    assert!(pd_config.vconn_swap); // Usually enabled by default
    assert!(!pd_config.dr_swap);

    let type_c_config = TypeCConfiguration::default();
    assert!(!type_c_config.current_1_5a); // Default current
    assert!(!type_c_config.cc_un_driving); // Normal operation
}

#[test]
fn test_protocol_type_enum() {
    // Test that all protocol types are covered
    let protocols = [
        ProtocolType::PD,
        ProtocolType::QC20,
        ProtocolType::QC30,
        ProtocolType::FCP,
        ProtocolType::AFC,
        ProtocolType::SCP,
        ProtocolType::PE20,
        ProtocolType::BC12,
        ProtocolType::SFCP,
    ];

    // Ensure all protocols can be used in pattern matching
    for protocol in protocols {
        match protocol {
            ProtocolType::PD => {}
            ProtocolType::QC20 => {}
            ProtocolType::QC30 => {}
            ProtocolType::FCP => {}
            ProtocolType::AFC => {}
            ProtocolType::SCP => {}
            ProtocolType::PE20 => {}
            ProtocolType::BC12 => {}
            ProtocolType::SFCP => {}
        }
    }
}
