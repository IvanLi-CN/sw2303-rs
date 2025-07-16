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
    /// System status 0 flags (REG 0x07) - 根据官方文档
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus0Flags: u8 {
        /// 异常拉光耦状态 (bit 4): 0: 未出现异常拉光耦, 1: 出现异常拉光耦
        const ABNORMAL_OPTOCOUPLER = 0b00010000;
        /// CC 环路状态 (bit 2): 0: CC 环路打开, 1: CC 环路关闭
        const CC_LOOP_CLOSED = 0b00000100;
        /// 线补打开状态 (bit 1): 0: 未打开线补, 1: 线补打开
        const LINE_COMPENSATION_OPEN = 0b00000010;
        /// 通路管状态 (bit 0): 0: 通路管关闭, 1: 通路管打开
        const PASS_TRANSISTOR_OPEN = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus0Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus0Flags({})", self.bits())
    }
}

bitflags! {
    /// System status 2 flags (REG 0x0C) - 根据官方文档
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus2Flags: u8 {
        /// CC1 过压指示 (bit 7): 0: CC1 未过压, 1: CC1 过压
        const CC1_OVERVOLTAGE = 0b10000000;
        /// CC2 过压指示 (bit 6): 0: CC2 未过压, 1: CC2 过压
        const CC2_OVERVOLTAGE = 0b01000000;
        /// DP 过压指示 (bit 5): 0: DP 未过压, 1: DP 过压
        const DP_OVERVOLTAGE = 0b00100000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus2Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus2Flags({})", self.bits())
    }
}

bitflags! {
    /// System status 1 flags (REG 0x0B) - 根据官方文档
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus1Flags: u8 {
        /// Vin 超过 25V 指示 (bit 5): 0: Vin 未超过, 1: Vin 超过
        const VIN_OVER_25V = 0b00100000;
        /// 过流状态指示 (bit 4): 0: 未过流, 1: 电流超过 112.5%
        const OVERCURRENT_112_5_PERCENT = 0b00010000;
        /// Die 过温指示 (bit 3): 0: die 未过温, 1: die 过温
        const DIE_OVERTEMPERATURE = 0b00001000;
        /// CC 环路状态 (bit 2): 0: CC 环路关闭, 1: CC 环路打开
        const CC_LOOP_OPEN = 0b00000100;
        /// Vin 过压指示 (bit 1): 0: 未过压, 1: 过压
        const VIN_OVERVOLTAGE = 0b00000010;
        /// Vin 欠压指示 (bit 0): 0: 未欠压, 1: 欠压
        const VIN_UNDERVOLTAGE = 0b00000001;
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

/// SW2303 device constants
///
/// This module contains only verified constants that are either:
/// 1. Confirmed through hardware testing (e.g., I2C address)
/// 2. Basic register bit patterns that are likely correct
/// 3. Unlock sequences that are typical for this type of chip
///
/// Many constants have been removed because they were not verified against
/// the official SW2303 datasheet and appeared to be copied from other chips
/// or based on generic USB PD/Type-C standards.
///
/// **IMPORTANT**: Before using this driver in production, verify all constants
/// against the official SW2303 register manual and datasheet.
pub mod constants {

    /// Default timeout for operations (in milliseconds)
    pub const DEFAULT_TIMEOUT_MS: u32 = 1000;
    /// Default I2C address for SW2303 (verified with hardware)
    pub const DEFAULT_ADDRESS: u8 = 0x3C;

    /// Power configuration constants (verified from official datasheet)
    ///
    /// REG 0xAF: 功率配置
    /// Bit 7: 最大功率配置方式选择 (0: 外部电阻, 1: 寄存器)
    /// Bit 6-0: 最大功率设置 W/bit
    pub mod power {
        /// Use register for power configuration (bit 7 = 1)
        pub const POWER_CONFIG_REGISTER_MODE: u8 = 0x80;
        /// Use external resistor for power configuration (bit 7 = 0)
        pub const POWER_CONFIG_EXTERNAL_MODE: u8 = 0x00;
        /// Power setting mask (bits 6-0)
        pub const POWER_SETTING_MASK: u8 = 0x7F;

        /// Maximum power setting range (0-127W, as bits 6-0 = 7 bits = 0-127)
        pub const MAX_POWER_SETTING: u8 = 127;
        pub const MIN_POWER_SETTING: u8 = 0;

        /// Common power settings (W/bit according to datasheet)
        pub const POWER_65W: u8 = 65;
        pub const POWER_100W: u8 = 100;
    }

    /// ADC configuration constants
    ///
    /// These values are verified against the official SW2303 register manual.
    pub mod adc {
        /// ADC conversion factors (verified from official datasheet)

        /// REG 0x30: Vin ADC conversion factor: 7.5mV per bit (12-bit mode)
        /// Official datasheet: "7.5*16mV/bit，(存取 12bit 时分辨率为 7.5mV/bit,参见 reg0x3B)"
        pub const VIN_FACTOR_MV: f32 = 7.5;

        /// REG 0x31: Vbus ADC conversion factor: 7.5mV per bit (12-bit mode)
        /// Official datasheet: "7.5*16mV/bit，(存取 12bit 时分辨率为 7.5mV/bit,参见 reg0x3B)"
        pub const VBUS_FACTOR_MV: f32 = 7.5;

        /// REG 0x33: Ich ADC conversion factor: 50mA per bit
        /// Official datasheet: "50mA/bit，(存取 12bit 时分辨率为 3.125mA/bit,参见 reg0x3B)"
        pub const ICH_FACTOR_MA: f32 = 50.0;

        /// REG 0x36: Tdiet ADC conversion factor: 0.1488°C per bit (12-bit mode)
        /// Official datasheet: "2.38°C/bit，(存取 12bit 时分辨率为 0.1488°C/bit,参见 reg0x3B)"
        pub const TDIET_FACTOR_C: f32 = 0.1488;

        /// ADC configuration selection values (REG 0x3B bits 2-0)
        /// 1: adc_vin[11:0], 7.5mV/bit
        /// 2: adc_vbus[11:0], 7.5mV/bit
        /// 3: adc_ich[11:0], 3.125mA/bit
        /// 4: adc_diet[11:0], 0.1488°C/bit; Tdiet = (adc_diet[11:0]-1848)/6.72°C
        pub const ADC_SELECT_VIN: u8 = 1;
        pub const ADC_SELECT_VBUS: u8 = 2;
        pub const ADC_SELECT_ICH: u8 = 3;
        pub const ADC_SELECT_TDIET: u8 = 4;
    }

    /// Unlock sequence constants (verified from official datasheet)
    ///
    /// REG 0x12: I2C写使能控制0
    /// To operate registers reg0x14, reg0xA0~BF, need to execute the following operations:
    /// 1. Write reg0x12 = 0x20;
    /// 2. Write reg0x12 = 0x40;
    /// 3. Write reg0x12 = 0x80;
    pub mod unlock {
        /// First unlock sequence step for REG 0x12
        pub const WRITE_ENABLE_0_STEP1: u8 = 0x20;
        /// Second unlock sequence step for REG 0x12
        pub const WRITE_ENABLE_0_STEP2: u8 = 0x40;
        /// Third unlock sequence step for REG 0x12
        pub const WRITE_ENABLE_0_STEP3: u8 = 0x80;

        /// Legacy constants (incorrect - kept for compatibility)
        /// These were not verified against official documentation
        pub const WRITE_ENABLE_0_SEQUENCE: u8 = 0x5A; // DEPRECATED
        pub const WRITE_ENABLE_1_SEQUENCE: u8 = 0xA5; // DEPRECATED
    }

    /// PD protocol constants
    ///
    /// Note: USB PD standard constants have been removed as they are generic
    /// USB PD specification values, not SW2303-specific constants. These should
    /// be defined in a separate USB PD standard library if needed.
    pub mod pd {
        // USB PD standard constants removed - use dedicated USB PD library
    }

    /// Type-C constants
    ///
    /// Note: Type-C standard constants have been removed as they are generic
    /// Type-C specification values, not SW2303-specific constants. These should
    /// be defined in a separate Type-C standard library if needed.
    pub mod type_c {
        // Type-C standard constants removed - use dedicated Type-C library
    }

    /// Protection thresholds (verified from official datasheet)
    pub mod protection {
        /// Die temperature protection thresholds (REG 0xAB bits 3-2)
        /// Official datasheet: "Die 过温保护门限"
        /// 0: 105°C
        /// 1: 115°C
        /// 2: 125°C
        /// 3: 135°C
        pub const DIE_TEMP_PROTECTION_105C: u8 = 0;
        pub const DIE_TEMP_PROTECTION_115C: u8 = 1;
        pub const DIE_TEMP_PROTECTION_125C: u8 = 2;
        pub const DIE_TEMP_PROTECTION_135C: u8 = 3;

        /// Temperature values in Celsius
        pub const TEMP_105C: i16 = 105;
        pub const TEMP_115C: i16 = 115;
        pub const TEMP_125C: i16 = 125;
        pub const TEMP_135C: i16 = 135;
    }

    /// Timer constants
    ///
    /// Note: Timer constants have been removed as they were likely based on
    /// USB PD/Type-C standard values rather than SW2303-specific timing requirements.
    /// These values need to be determined from the official SW2303 datasheet.
    pub mod timer {
        // Timer constants removed - need verification from official datasheet
    }
}
