//! SW2303 register definitions and constants
//!
//! SW2303 is a USB PD (Power Delivery) charging controller, not a USB hub controller.
//! This module defines the correct register addresses based on the official SW2303 register manual.

use bitflags::bitflags;

/// SW2303 register addresses based on official documentation
#[allow(dead_code)] // Allow unused register definitions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Register {
    /// REG 0x01: Chip version register
    ChipVersion = 0x01,
    /// REG 0x03: Set voltage high 8 bits
    VoltageHigh = 0x03,
    /// REG 0x04: Set voltage low 4 bits
    VoltageLow = 0x04,
    /// REG 0x05: Set current limit
    CurrentLimit = 0x05,
    /// REG 0x06: Fast charging indication
    FastChargingStatus = 0x06,
    /// REG 0x0B: System status 0
    SystemStatus0 = 0x0B,
    /// REG 0x0C: System status 1
    SystemStatus1 = 0x0C,
    /// REG 0x0D: System status 3
    SystemStatus3 = 0x0D,
    /// REG 0x12: I2C write enable control 0
    I2cWriteEnable0 = 0x12,
    /// REG 0x14: Connection control
    ConnectionControl = 0x14,
    /// REG 0x15: I2C write enable control 1
    I2cWriteEnable1 = 0x15,
    /// REG 0x16: Force control enable
    ForceControlEnable = 0x16,
    /// REG 0x30: ADC Vin data
    AdcVin = 0x30,
    /// REG 0x31: ADC Vbus data
    AdcVbus = 0x31,
    /// REG 0x33: ADC Ich data
    AdcIch = 0x33,
    /// REG 0x36: ADC Tdiet data
    AdcTdiet = 0x36,
    /// REG 0x3B: ADC configuration
    AdcConfig = 0x3B,
    /// REG 0x3C: ADC data high 8 bits
    AdcDataHigh = 0x3C,
    /// REG 0x3D: ADC data low 4 bits
    AdcDataLow = 0x3D,
}

impl Register {
    /// Get the register address as u8
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

bitflags! {
    /// System status 0 flags (REG 0x07)
    /// Note: Specific bit definitions need to be verified against official documentation
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus0Flags: u8 {
        // TODO: Define correct bits based on official SW2303 register manual
        // The previous UFP_CONNECTED definition was incorrect
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus0Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus0Flags({})", self.bits())
    }
}



bitflags! {
    /// System status 3 flags (REG 0x0D)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus3Flags: u8 {
        /// Device online status (bit 7) - indicates if sink device is connected
        /// 0: offline (no sink device)
        /// 1: online (sink device connected)
        const DEVICE_ONLINE = 0b10000000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus3Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus3Flags({})", self.bits())
    }
}

bitflags! {
    /// Fast charging status flags (REG 0x06)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FastChargingFlags: u8 {
        /// Fast charging active (bit 7)
        const FAST_CHARGING = 0b10000000;
        /// Type-C connected (bit 6)
        const TYPE_C_CONNECTED = 0b01000000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FastChargingFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FastChargingFlags({})", self.bits())
    }
}

bitflags! {
    /// Connection control flags (REG 0x14)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ConnectionControlFlags: u8 {
        /// Connection control (bit 2)
        const CONNECTION_CONTROL = 0b00000100;
        /// Type-C CC un-driving (bit 1)
        const CC_UN_DRIVING = 0b00000010;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ConnectionControlFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "ConnectionControlFlags({})", self.bits())
    }
}

/// SW2303 device constants based on official register manual
pub mod constants {
    /// Expected chip version (REG 0x01[1:0], default 0x1)
    pub const CHIP_VERSION: u8 = 0x01;
    /// Default timeout for operations (in milliseconds)
    pub const DEFAULT_TIMEOUT_MS: u32 = 1000;
    /// Default I2C address for SW2303 (verified with hardware)
    pub const DEFAULT_ADDRESS: u8 = 0x3C;

    /// ADC conversion factors
    pub mod adc {
        /// Vin/Vbus ADC conversion factor: 7.5mV per bit (12-bit mode)
        pub const VOLTAGE_FACTOR_MV: f32 = 7.5;
        /// Current ADC conversion factor: 3.125mA per bit (12-bit mode)
        pub const CURRENT_FACTOR_MA: f32 = 3.125;
        /// Temperature ADC conversion factor: 0.1488°C per bit (12-bit mode)
        pub const TEMP_FACTOR_C: f32 = 0.1488;
        /// Temperature offset: Tdiet = (adc_diet[11:0] - 1848) / 6.72°C
        pub const TEMP_OFFSET: f32 = 1848.0;
        pub const TEMP_DIVISOR: f32 = 6.72;
    }
}
