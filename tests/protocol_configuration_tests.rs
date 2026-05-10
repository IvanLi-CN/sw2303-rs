//! Protocol Configuration Tests for SW2303
//!
//! This module contains tests for the protocol configuration functionality
//! of the SW2303 USB PD controller driver.

#![cfg(all(test, not(feature = "async")))]

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use std::collections::HashMap;
use sw2303::{
    PdConfiguration, PpsConfigMode, ProtocolConfiguration, ProtocolType, SW2303,
    TypeCConfiguration, registers::constants::DEFAULT_ADDRESS,
};

/// Mock I2C implementation for testing
#[derive(Debug, Default)]
struct MockI2c {
    registers: HashMap<u8, u8>,
    current_register: Option<u8>,
    write_enabled: bool,
}

impl MockI2c {
    fn new() -> Self {
        Self {
            registers: HashMap::new(),
            current_register: None,
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
            let reg = self.current_register.unwrap_or(0);
            buffer[0] = self.get_register(reg);
        }
        Ok(())
    }

    fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() == 1 {
            self.current_register = Some(bytes[0]);
            return Ok(());
        }

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
        pps_config_mode: PpsConfigMode::Auto,
        fixed_voltages: [true, true, false, false], // 9V, 12V enabled
        emark_5a_bypass: false,
        emarker_60_70w: true,
    };

    let result = sw2303.configure_pd(pd_config);
    assert!(result.is_ok());

    let pd_cfg1 = sw2303.get_pd_config_1_raw().unwrap();
    let pd_cfg2 = sw2303.get_pd_config_2_raw().unwrap();
    assert_eq!(pd_cfg1.bits() & 0x80, 0x00); // PPS register-config must stay in auto mode.
    assert_eq!(pd_cfg2.bits() & 0xF0, 0x00); // PPS0/1/2/3 all enabled (active-low disable bits).

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
        pps_config_mode: PpsConfigMode::Auto,
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
fn test_pd_capability_status_reports_high_voltage_pps() {
    let mut i2c = MockI2c::new();
    i2c.set_register(0xA6, 0xB0);
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    sw2303
        .configure_pd(PdConfiguration {
            enabled: true,
            vconn_swap: true,
            dr_swap: false,
            emarker_enabled: true,
            pps_enabled: true,
            pps_config_mode: PpsConfigMode::Auto,
            fixed_voltages: [true, true, true, true],
            emark_5a_bypass: false,
            emarker_60_70w: true,
        })
        .unwrap();

    let status = sw2303.get_pd_capability_status().unwrap();
    assert!(status.enabled);
    assert_eq!(status.pps_config_mode, PpsConfigMode::Auto);
    assert_eq!(status.fixed_voltages, [true, true, true, true]);
    assert_eq!(status.pps_ranges, [true, true, true, true]);
    assert_eq!(status.pps3_current_limit_ma, 5_000);
    assert!(status.supports_pps_above_11v());
    assert_eq!(status.max_pps_voltage_mv(), Some(21_000));
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
fn test_configure_protocols_clears_global_fast_charge_disable_for_non_qc_modes() {
    let mut i2c = MockI2c::new();
    i2c.set_register(0xB0, 0xFF);
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    sw2303
        .configure_protocols(ProtocolConfiguration {
            pd_enabled: false,
            qc20_enabled: false,
            qc30_enabled: false,
            fcp_enabled: true,
            afc_enabled: false,
            scp_enabled: false,
            pe20_enabled: false,
            bc12_enabled: false,
            sfcp_enabled: false,
        })
        .unwrap();

    let cfg2 = sw2303.get_fast_charge_config_2_raw().unwrap();
    assert!(!cfg2.contains(sw2303::registers::FastChargeConfig2Flags::FAST_CHARGE_DISABLE));
    assert_eq!(sw2303.get_protocol_status().unwrap().fcp_enabled, true);
}

#[test]
fn test_protocol_status_reports_qc2_and_qc3_independently() {
    let mut i2c = MockI2c::new();
    i2c.set_register(0xB0, 0x00);
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    sw2303.init().unwrap();
    sw2303.unlock_write_enable_0().unwrap();

    sw2303
        .configure_protocols(ProtocolConfiguration {
            pd_enabled: false,
            qc20_enabled: true,
            qc30_enabled: false,
            fcp_enabled: false,
            afc_enabled: false,
            scp_enabled: false,
            pe20_enabled: false,
            bc12_enabled: false,
            sfcp_enabled: false,
        })
        .unwrap();

    let status = sw2303.get_protocol_status().unwrap();
    assert!(status.qc20_enabled);
    assert!(!status.qc30_enabled);
    assert!(sw2303.is_protocol_enabled(ProtocolType::QC20).unwrap());
    assert!(!sw2303.is_protocol_enabled(ProtocolType::QC30).unwrap());
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
