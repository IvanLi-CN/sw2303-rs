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
    /// REG 0x01: 芯片版本 (Chip version register)
    ChipVersion = 0x01,
    /// REG 0x03: 设置电压高8位 (Set voltage high 8 bits)
    VoltageHigh = 0x03,
    /// REG 0x04: 设置电压低4位 (Set voltage low 4 bits)
    VoltageLow = 0x04,
    /// REG 0x05: 设置限流 (Set current limit)
    CurrentLimit = 0x05,
    /// REG 0x06: 快充指示 (Fast charging indication)
    FastChargingStatus = 0x06,
    /// REG 0x07: 系统状态0 (System status 0)
    SystemStatus0 = 0x07,
    /// REG 0x0B: 系统状态1 (System status 1)
    SystemStatus1 = 0x0B,
    /// REG 0x0C: 系统状态2 (System status 2)
    SystemStatus2 = 0x0C,
    /// REG 0x0D: 系统状态3 (System status 3)
    SystemStatus3 = 0x0D,

    /// REG 0x12: I2C写使能控制0 (I2C write enable control 0)
    I2cWriteEnable0 = 0x12,
    /// REG 0x14: 连接控制 (Connection control)
    ConnectionControl = 0x14,
    /// REG 0x15: I2C写使能控制1 (I2C write enable control 1)
    I2cWriteEnable1 = 0x15,
    /// REG 0x16: 强制控制使能 (Force control enable)
    ForceControlEnable = 0x16,

    // ADC registers (0x30-0x3D range)
    /// REG 0x30: ADC Vin数据 (ADC Vin data)
    AdcVin = 0x30,
    /// REG 0x31: ADC Vbus数据 (ADC Vbus data)
    AdcVbus = 0x31,
    /// REG 0x33: ADC Ich数据 (ADC Ich data)
    AdcIch = 0x33,
    /// REG 0x36: ADC Tdiet数据 (ADC Tdiet data)
    AdcTdiet = 0x36,
    /// REG 0x3B: ADC配置 (ADC configuration)
    AdcConfig = 0x3B,
    /// REG 0x3C: ADC数据高8位 (ADC data high 8 bits)
    AdcDataHigh = 0x3C,
    /// REG 0x3D: ADC数据低4位 (ADC data low 4 bits)
    AdcDataLow = 0x3D,

    // Configuration registers (0xA1-0xBF range, requires unlock)
    /// REG 0xA1: 异常处理配置 (Exception handling configuration)
    ExceptionHandlingConfig = 0xA1,
    /// REG 0xA3: 输出电压偏移配置 (Output voltage offset configuration)
    OutputVoltageOffsetConfig = 0xA3,
    /// REG 0xA4: 线补阻抗配置 (Line compensation impedance configuration)
    LineCompensationConfig = 0xA4,
    /// REG 0xA5: 过压检测配置 (Overvoltage detection configuration)
    OvervoltageDetectionConfig = 0xA5,
    /// REG 0xA6: PD配置3 (PD configuration 3)
    PdConfig3 = 0xA6,
    /// REG 0xA8: 广播电流配置 (Broadcast current configuration)
    BroadcastCurrentConfig = 0xA8,
    /// REG 0xAB: 异常保护配置 (Exception protection configuration)
    ExceptionProtectionConfig = 0xAB,
    /// REG 0xAC: 在线配置 (Online configuration)
    OnlineConfig = 0xAC,
    /// REG 0xAD: 快充配置0 (Fast charging configuration 0)
    FastChargingConfig0 = 0xAD,
    /// REG 0xAE: 快充配置1 (Fast charging configuration 1)
    FastChargingConfig1 = 0xAE,
    /// REG 0xAF: 功率配置 (Power configuration)
    PowerConfig = 0xAF,
    /// REG 0xB0: 快充配置2 (Fast charging configuration 2)
    FastChargingConfig2 = 0xB0,
    /// REG 0xB1: 快充配置3 (Fast charging configuration 3)
    FastChargingConfig3 = 0xB1,
    /// REG 0xB2: 快充配置4 (Fast charging configuration 4)
    FastChargingConfig4 = 0xB2,
    /// REG 0xB3: PD配置0 (PD configuration 0)
    PdConfig0 = 0xB3,
    /// REG 0xB4: PD配置1 (PD configuration 1)
    PdConfig1 = 0xB4,
    /// REG 0xB5: PD配置2 (PD configuration 2)
    PdConfig2 = 0xB5,
    /// REG 0xB6: VID配置0 (Vendor ID Configuration 0)
    VidConfig0 = 0xB6,
    /// REG 0xB7: VID配置1 (Vendor ID Configuration 1)
    VidConfig1 = 0xB7,
    /// REG 0xB8: XID配置0 (XID Configuration 0)
    XidConfig0 = 0xB8,
    /// REG 0xB9: XID配置1 (XID Configuration 1)
    XidConfig1 = 0xB9,
    /// REG 0xBA: XID配置2 (XID Configuration 2)
    XidConfig2 = 0xBA,
    /// REG 0xBB: XID配置3 (XID Configuration 3)
    XidConfig3 = 0xBB,
    /// REG 0xBC: PID配置0 (Product ID Configuration 0)
    PidConfig0 = 0xBC,
    /// REG 0xBD: PID配置1 (Product ID Configuration 1)
    PidConfig1 = 0xBD,
    /// REG 0xBE: SVID配置0 (Standard Vendor ID Configuration 0)
    SvidConfig0 = 0xBE,
    /// REG 0xBF: SVID配置1 (Standard Vendor ID Configuration 1)
    SvidConfig1 = 0xBF,
}

impl Register {
    /// Get the register address as u8
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

bitflags! {
    /// System status 0 flags (REG 0x07)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus0Flags: u8 {
        /// PD communication status (bit 7)
        const PD_COMMUNICATION = 0b10000000;
        /// Type-C connection status (bit 6)
        const TYPE_C_CONNECTION = 0b01000000;
        /// VBUS status (bit 5)
        const VBUS_STATUS = 0b00100000;
        /// CC line status (bit 4)
        const CC_STATUS = 0b00010000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus0Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus0Flags({})", self.bits())
    }
}

bitflags! {
    /// System status 2 flags (REG 0x0C)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus2Flags: u8 {
        /// PD negotiation complete (bit 7)
        const PD_NEGOTIATION_COMPLETE = 0b10000000;
        /// Source capabilities sent (bit 6)
        const SOURCE_CAPS_SENT = 0b01000000;
        /// Sink capabilities received (bit 5)
        const SINK_CAPS_RECEIVED = 0b00100000;
        /// Contract valid (bit 4)
        const CONTRACT_VALID = 0b00010000;
        /// Hard reset received (bit 3)
        const HARD_RESET_RECEIVED = 0b00001000;
        /// Soft reset received (bit 2)
        const SOFT_RESET_RECEIVED = 0b00000100;
        /// Cable reset received (bit 1)
        const CABLE_RESET_RECEIVED = 0b00000010;
        /// Protocol error (bit 0)
        const PROTOCOL_ERROR = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus2Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus2Flags({})", self.bits())
    }
}





bitflags! {
    /// System status 1 flags (REG 0x0B)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus1Flags: u8 {
        /// Power delivery status (bit 7)
        const POWER_DELIVERY = 0b10000000;
        /// Charging status (bit 6)
        const CHARGING = 0b01000000;
        /// Overcurrent protection (bit 5)
        const OVERCURRENT = 0b00100000;
        /// Overvoltage protection (bit 4)
        const OVERVOLTAGE = 0b00010000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus1Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus1Flags({})", self.bits())
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
    /// Expected chip version (REG 0x01\[1:0\], default 0x1)
    pub const CHIP_VERSION: u8 = 0x01;
    /// Default timeout for operations (in milliseconds)
    pub const DEFAULT_TIMEOUT_MS: u32 = 1000;
    /// Default I2C address for SW2303 (verified with hardware)
    pub const DEFAULT_ADDRESS: u8 = 0x3C;

    /// Power configuration constants
    pub mod power {
        /// Use register for power configuration (bit 7 = 1)
        pub const POWER_CONFIG_REGISTER_MODE: u8 = 0x80;
        /// Use external resistor for power configuration (bit 7 = 0)
        pub const POWER_CONFIG_EXTERNAL_MODE: u8 = 0x00;
        /// Power setting mask (bits 6-0)
        pub const POWER_SETTING_MASK: u8 = 0x7F;
        /// 65W power setting (65 watts)
        pub const POWER_65W: u8 = 65;
    }

    /// ADC conversion factors
    pub mod adc {
        /// Vin/Vbus ADC conversion factor: 7.5mV per bit (12-bit mode)
        pub const VOLTAGE_FACTOR_MV: f32 = 7.5;
        /// Current ADC conversion factor: 3.125mA per bit (12-bit mode)
        pub const CURRENT_FACTOR_MA: f32 = 3.125;
        /// Temperature ADC conversion factor: 0.1488°C per bit (12-bit mode)
        pub const TEMP_FACTOR_C: f32 = 0.1488;
        /// Temperature offset: Tdiet = (adc_diet\[11:0\] - 1848) / 6.72°C
        pub const TEMP_OFFSET: f32 = 1848.0;
        pub const TEMP_DIVISOR: f32 = 6.72;

        /// ADC configuration values
        pub const ADC_12_BIT_MODE: u8 = 0x00;
        pub const ADC_10_BIT_MODE: u8 = 0x01;
        pub const ADC_8_BIT_MODE: u8 = 0x02;
    }

    /// Unlock sequence constants
    pub mod unlock {
        /// Unlock sequence for write enable 0 (unlocks reg0x14, reg0xA0-BF)
        pub const WRITE_ENABLE_0_SEQUENCE: u8 = 0x5A;
        /// Unlock sequence for write enable 1 (unlocks reg0x16)
        pub const WRITE_ENABLE_1_SEQUENCE: u8 = 0xA5;
    }

    /// PD protocol constants
    pub mod pd {
        /// PD specification revision 2.0
        pub const SPEC_REV_2_0: u8 = 0x01;
        /// PD specification revision 3.0
        pub const SPEC_REV_3_0: u8 = 0x02;
        /// PD specification revision 3.1
        pub const SPEC_REV_3_1: u8 = 0x03;

        /// Maximum voltage (20V)
        pub const MAX_VOLTAGE_20V: u16 = 20000;
        /// Maximum current (5A)
        pub const MAX_CURRENT_5A: u16 = 5000;
        /// Maximum power (100W)
        pub const MAX_POWER_100W: u16 = 100;

        /// Standard voltage levels
        pub const VOLTAGE_5V: u16 = 5000;
        pub const VOLTAGE_9V: u16 = 9000;
        pub const VOLTAGE_12V: u16 = 12000;
        pub const VOLTAGE_15V: u16 = 15000;
        pub const VOLTAGE_20V: u16 = 20000;

        /// Standard current levels
        pub const CURRENT_1_5A: u16 = 1500;
        pub const CURRENT_3A: u16 = 3000;
        pub const CURRENT_5A: u16 = 5000;
    }

    /// Type-C constants
    pub mod type_c {
        /// CC voltage thresholds (mV)
        pub const CC_THRESHOLD_200MV: u16 = 200;
        pub const CC_THRESHOLD_400MV: u16 = 400;
        pub const CC_THRESHOLD_800MV: u16 = 800;
        pub const CC_THRESHOLD_1200MV: u16 = 1200;
        pub const CC_THRESHOLD_1600MV: u16 = 1600;
        pub const CC_THRESHOLD_2000MV: u16 = 2000;

        /// Debounce times (ms)
        pub const DEBOUNCE_TIME_10MS: u16 = 10;
        pub const DEBOUNCE_TIME_100MS: u16 = 100;
        pub const DEBOUNCE_TIME_200MS: u16 = 200;

        /// VCONN current limit (mA)
        pub const VCONN_CURRENT_LIMIT: u16 = 1000;
    }

    /// Protection thresholds
    pub mod protection {
        /// Overvoltage protection thresholds
        pub const OVP_5_5V: u16 = 5500;
        pub const OVP_22V: u16 = 22000;

        /// Undervoltage protection thresholds
        pub const UVP_4_5V: u16 = 4500;
        pub const UVP_3_3V: u16 = 3300;

        /// Overcurrent protection thresholds
        pub const OCP_6A: u16 = 6000;
        pub const OCP_3_25A: u16 = 3250;

        /// Temperature protection thresholds (°C)
        pub const TEMP_WARNING_85C: i16 = 85;
        pub const TEMP_SHUTDOWN_105C: i16 = 105;
    }

    /// Timer constants
    pub mod timer {
        /// PD timers (ms)
        pub const PD_HARD_RESET_TIMER: u16 = 5000;
        pub const PD_SOURCE_CAP_TIMER: u16 = 150;
        pub const PD_SINK_REQUEST_TIMER: u16 = 100;

        /// Type-C timers (ms)
        pub const TYPE_C_CC_DEBOUNCE: u16 = 200;
        pub const TYPE_C_PD_DEBOUNCE: u16 = 20;

        /// Watchdog timer (ms)
        pub const WATCHDOG_TIMEOUT: u16 = 1000;
    }
}
