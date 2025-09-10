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
    /// REG 0x03: Set voltage high 8 bits
    VoltageHigh = 0x03,
    /// REG 0x04: Set voltage low 4 bits
    VoltageLow = 0x04,
    /// REG 0x05: Set current limit
    CurrentLimit = 0x05,
    /// REG 0x06: Fast charging indication
    FastChargingStatus = 0x06,
    /// REG 0x07: System status 0
    SystemStatus0 = 0x07,
    /// REG 0x09: PD status (按官方手册)
    PdStatus = 0x09,
    /// REG 0x0A: Type‑C status (按官方手册)
    TypeCStatus = 0x0A,
    /// REG 0x0B: System status 1
    SystemStatus1 = 0x0B,
    /// REG 0x0C: System status 2
    SystemStatus2 = 0x0C,
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

bitflags! {
    /// PD configuration 0 flags (REG 0xB3) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdConfig0Flags: u8 {
        /// 60~70W Emarker control (bit 7): 0: no emarker needed, 1: emarker needed
        const EMARKER_60_70W = 0b10000000;
        /// PD hard reset and PPS handling (bit 6): 0: reject hard reset, 1: enter PPS after hard reset
        const HARDRESET_PPS = 0b01000000;
        /// Reserved (bit 5)
        const RESERVED_5 = 0b00100000;
        /// Emarker enable (bit 4): 0: disable, 1: enable
        const EMARKER_ENABLE = 0b00010000;
        /// PD dr_swap function (bit 3): 0: not supported, 1: supported
        const DR_SWAP = 0b00001000;
        /// PD vconn_swap function (bit 2): 0: not supported, 1: supported
        const VCONN_SWAP = 0b00000100;
        /// Reserved (bit 1)
        const RESERVED_1 = 0b00000010;
        /// PD protocol enable (bit 0): 0: disable, 1: enable
        const PD_ENABLE = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdConfig0Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdConfig0Flags({})", self.bits())
    }
}

bitflags! {
    /// PD configuration 1 flags (REG 0xB4) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdConfig1Flags: u8 {
        /// PPS0/PPS1/PPS2/PPS3使能 (bit 7): 0: 禁用, 1: 使能
        const PPS_ENABLE = 0b10000000;
        /// PPS3电流限制 (bit 6): 0: 5A, 1: 3A
        const PPS3_3A_LIMIT = 0b01000000;
        /// Reserved (bits 5-4)
        const RESERVED_5_4 = 0b00110000;
        /// PD discovery identity (bit 3): 0: 不支持, 1: 支持
        const DISCOVERY_IDENTITY = 0b00001000;
        /// PD discovery SVID功能 (bit 2): 0: 不支持, 1: 支持
        const DISCOVERY_SVID = 0b00000100;
        /// PD peak current配置 (bits 1-0): 用于12个PDO的峰值电流设置
        const PEAK_CURRENT_MASK = 0b00000011;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdConfig1Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdConfig1Flags({})", self.bits())
    }
}

bitflags! {
    /// PD configuration 2 flags (REG 0xB5) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdConfig2Flags: u8 {
        /// 3.3~21V PPS使能(PPS3) (bit 7): 0: 禁用, 1: 使能
        const PPS3_3_3_21V = 0b10000000;
        /// 3.3~16V PPS使能(PPS2) (bit 6): 0: 禁用, 1: 使能
        const PPS2_3_3_16V = 0b01000000;
        /// 3.3~11V PPS使能(PPS1) (bit 5): 0: 禁用, 1: 使能
        const PPS1_3_3_11V = 0b00100000;
        /// 3.3~5.9V PPS使能(PPS0) (bit 4): 0: 禁用, 1: 使能
        const PPS0_3_3_5_9V = 0b00010000;
        /// PD fixed 20V使能 (bit 3): 0: 禁用, 1: 使能
        const FIXED_20V = 0b00001000;
        /// PD fixed 15V使能 (bit 2): 0: 禁用, 1: 使能
        const FIXED_15V = 0b00000100;
        /// PD fixed 12V使能 (bit 1): 0: 禁用, 1: 使能
        const FIXED_12V = 0b00000010;
        /// PD fixed 9V使能 (bit 0): 0: 禁用, 1: 使能
        const FIXED_9V = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdConfig2Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdConfig2Flags({})", self.bits())
    }
}

bitflags! {
    /// PD configuration 3 flags (REG 0xA6) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdConfig3Flags: u8 {
        /// Reserved (bit 7)
        const RESERVED_7 = 0b10000000;
        /// PD 5A emark控制 (bit 6): 0: 需要emark 5A, 1: 不需要emark，直接输出5A
        const EMARK_5A_BYPASS = 0b01000000;
        /// Reserved (bits 5-0)
        const RESERVED_5_0 = 0b00111111;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdConfig3Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdConfig3Flags({})", self.bits())
    }
}

bitflags! {
    /// Fast charging configuration 0 flags (REG 0xAD) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FastChargeConfig0Flags: u8 {
        /// QC2.0/QC3.0/PD FIX使能 (bit 7): 0: 禁用, 1: 使能
        const QC_PD_FIX_ENABLE = 0b10000000;
        /// Reserved (bits 6-4)
        const RESERVED_6_4 = 0b01110000;
        /// FCP/AFC/SFCP电流限制 (bit 3): 0: 3.25A, 1: 2.25A
        const FCP_AFC_SFCP_2_25A = 0b00001000;
        /// Reserved (bits 2-1)
        const RESERVED_2_1 = 0b00000110;
        /// PD与SCP协议优先级 (bit 0): 0: 不使能, 1: 使能
        const PD_SCP_PRIORITY = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FastChargeConfig0Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FastChargeConfig0Flags({})", self.bits())
    }
}

bitflags! {
    /// Fast charging configuration 1 flags (REG 0xAE) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FastChargeConfig1Flags: u8 {
        /// Reserved (bits 7-6)
        const RESERVED_7_6 = 0b11000000;
        /// 过压检测阈值 (bit 5): 0: 低于3.0V, 1: 低于5.0V
        const OVERVOLTAGE_5V = 0b00100000;
        /// 快充5V输出控制 (bit 4): 0: 不输出5V, 1: 输出5V
        const OUTPUT_5V = 0b00010000;
        /// QC3.0 20V使能 (bit 3): 0: 禁用, 1: 使能
        const QC30_20V = 0b00001000;
        /// QC2.0 20V使能 (bit 2): 0: 禁用, 1: 使能
        const QC20_20V = 0b00000100;
        /// PE2.0 20V使能 (bit 1): 0: 禁用, 1: 使能
        const PE20_20V = 0b00000010;
        /// PD 12V使能 (bit 0): 0: 禁用, 1: 使能
        const PD_12V = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FastChargeConfig1Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FastChargeConfig1Flags({})", self.bits())
    }
}

bitflags! {
    /// Fast charging configuration 2 flags (REG 0xB0) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FastChargeConfig2Flags: u8 {
        /// Reserved (bits 7-6)
        const RESERVED_7_6 = 0b11000000;
        /// SCP协议使能 (bit 5): 0: 禁用SCP, 1: 使能SCP
        const SCP_ENABLE = 0b00100000;
        /// SCP电流限制 (bit 4): 0: 禁用SCP, 1: 使能SCP
        const SCP_CURRENT_LIMIT = 0b00010000;
        /// QC3.0/QC2.0使能 (bit 3): 0: 禁用, 1: 使能
        const QC_ENABLE = 0b00001000;
        /// 快充协议使能 (bit 2): 0: 禁用, 1: 使能
        const FAST_CHARGE_ENABLE = 0b00000100;
        /// BC1.2使能 (bit 1): 0: 禁用, 1: 使能
        const BC12_ENABLE = 0b00000010;
        /// BC1.2 DPDM控制 (bit 0): 0: 禁用, 1: 使能
        const BC12_DPDM = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FastChargeConfig2Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FastChargeConfig2Flags({})", self.bits())
    }
}

bitflags! {
    /// Fast charging configuration 3 flags (REG 0xB1)
    /// 注意：以下若干位为“0=使能, 1=不使能”的有源低使能语义。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FastChargeConfig3Flags: u8 {
        /// Vin ADC过压检测 (bit 7): 0: 禁用, 1: 使能
        const VIN_ADC_OVERVOLTAGE = 0b10000000;
        /// Vin欠压检测 (bit 6): 0: 禁用, 1: 使能
        const VIN_UNDERVOLTAGE = 0b01000000;
        /// 1.2V检测 (bit 5): 0: 禁用, 1: 使能
        const DETECT_1_2V = 0b00100000;
        /// SFCP 协议“禁用位” (bit 4): 0: 使能, 1: 不使能
        const SFCP_ENABLE = 0b00010000;
        /// FCP 协议“禁用位” (bit 3): 0: 使能, 1: 不使能
        const FCP_ENABLE = 0b00001000;
        /// AFC 协议“禁用位” (bit 2): 0: 使能, 1: 不使能
        const AFC_ENABLE = 0b00000100;
        /// PE 协议“禁用位” (bit 1): 0: 使能, 1: 不使能
        const PE_ENABLE = 0b00000010;
        /// Reserved (bit 0)
        const RESERVED_0 = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FastChargeConfig3Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FastChargeConfig3Flags({})", self.bits())
    }
}

impl FastChargeConfig3Flags {
    /// 别名：显式表述为“禁用位”（0=使能，1=不使能）以消除歧义
    pub const SFCP_DISABLE: Self = Self::from_bits_truncate(0b0001_0000);
    pub const FCP_DISABLE: Self = Self::from_bits_truncate(0b0000_1000);
    pub const AFC_DISABLE: Self = Self::from_bits_truncate(0b0000_0100);
    pub const PE_DISABLE: Self = Self::from_bits_truncate(0b0000_0010);
}

bitflags! {
    /// Fast charging configuration 4 flags (REG 0xB2) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FastChargeConfig4Flags: u8 {
        /// Reserved (bits 7-6)
        const RESERVED_7_6 = 0b11000000;
        /// SCP协议电流限制 (bits 5-4): 0: 5.0A, 1: 4.0A, 2: 2.0A, 3: Reserved
        const SCP_CURRENT_MASK = 0b00110000;
        /// 25W 9V PDO配置 (bit 3): 0: 2.78A, 1: 2.77A
        const PDO_9V_25W_2_77A = 0b00001000;
        /// Reserved (bits 2-0)
        const RESERVED_2_0 = 0b00000111;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FastChargeConfig4Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "FastChargeConfig4Flags({})", self.bits())
    }
}

bitflags! {
    /// Broadcast current configuration flags (REG 0xA8) - Based on official register manual
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct BroadcastCurrentFlags: u8 {
        /// PD PPS 5A输出 (bit 7): 0: 不使能, 1: 使能输出5A
        const PD_PPS_5A = 0b10000000;
        /// Type-C电流广播 (bit 6): 0: 默认电流, 1: 1.5A
        const TYPE_C_1_5A = 0b01000000;
        /// Reserved (bits 5-0)
        const RESERVED_5_0 = 0b00111111;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for BroadcastCurrentFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "BroadcastCurrentFlags({})", self.bits())
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

        /// REG 0x30: Vin ADC conversion factor (8-bit direct read)
        /// Datasheet note: 8-bit path uses 7.5*16 mV/LSB; when using 12-bit via 0x3B
        /// selection, the factor is 7.5 mV/LSB (read from 0x3C/0x3D).
        pub const VIN_FACTOR_MV: f32 = 120.0; // 7.5*16 mV per LSB in 8-bit mode

        /// REG 0x31: Vbus ADC conversion factor (8-bit direct read)
        /// Datasheet note: 8-bit path uses 7.5*16 mV/LSB; 12-bit via 0x3B is 7.5 mV/LSB.
        pub const VBUS_FACTOR_MV: f32 = 120.0; // 7.5*16 mV per LSB in 8-bit mode

        /// REG 0x33: Ich ADC conversion factor: 50mA per bit
        /// Official datasheet: "50mA/bit，(存取 12bit 时分辨率为 3.125mA/bit,参见 reg0x3B)"
        pub const ICH_FACTOR_MA: f32 = 50.0;

        /// REG 0x36: Tdiet ADC conversion factor (8-bit direct read)
        /// Datasheet: 2.38°C/bit in 8-bit mode; 0.1488°C/bit in 12-bit mode via 0x3B.
        pub const TDIET_FACTOR_C: f32 = 2.38;

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

    /// PD protocol constants (verified from official SW2303 register manual)
    ///
    /// These constants are specific to SW2303 PD configuration registers
    /// and are based on the official register manual.
    pub mod pd {
        /// PD protocol IDs from REG 0x06 (快充指示) bits 5-4
        /// PD版本: 1: PD 2.0, 2: PD 3.0, other: Reserved
        pub const PD_VERSION_2_0: u8 = 1;
        pub const PD_VERSION_3_0: u8 = 2;

        /// Fast charge protocol IDs from REG 0x06 bits 3-0
        /// 1: QC2.0, 2: QC3.0, 3: FCP, 4: /, 5: SCP, 6: PD FIX, 7: PD PPS
        /// 8: PE1.1, 9: PE2.0, C: SFCP, D: AFC, other: reserved
        pub const PROTOCOL_QC20: u8 = 1;
        pub const PROTOCOL_QC30: u8 = 2;
        pub const PROTOCOL_FCP: u8 = 3;
        pub const PROTOCOL_SCP: u8 = 5;
        pub const PROTOCOL_PD_FIX: u8 = 6;
        pub const PROTOCOL_PD_PPS: u8 = 7;
        pub const PROTOCOL_PE11: u8 = 8;
        pub const PROTOCOL_PE20: u8 = 9;
        pub const PROTOCOL_SFCP: u8 = 0xC;
        pub const PROTOCOL_AFC: u8 = 0xD;

        /// PD fixed voltage levels (in volts)
        pub const VOLTAGE_9V: u16 = 9;
        pub const VOLTAGE_12V: u16 = 12;
        pub const VOLTAGE_15V: u16 = 15;
        pub const VOLTAGE_20V: u16 = 20;

        /// PPS voltage ranges (in volts)
        pub const PPS0_MIN_VOLTAGE: u16 = 3300; // 3.3V in mV
        pub const PPS0_MAX_VOLTAGE: u16 = 5900; // 5.9V in mV
        pub const PPS1_MIN_VOLTAGE: u16 = 3300; // 3.3V in mV
        pub const PPS1_MAX_VOLTAGE: u16 = 11000; // 11V in mV
        pub const PPS2_MIN_VOLTAGE: u16 = 3300; // 3.3V in mV
        pub const PPS2_MAX_VOLTAGE: u16 = 16000; // 16V in mV
        pub const PPS3_MIN_VOLTAGE: u16 = 3300; // 3.3V in mV
        pub const PPS3_MAX_VOLTAGE: u16 = 21000; // 21V in mV

        /// PD current limits (in mA)
        pub const PPS3_CURRENT_5A: u16 = 5000;
        pub const PPS3_CURRENT_3A: u16 = 3000;

        /// Peak current configuration values for REG 0xB4 bits 1-0
        pub const PEAK_CURRENT_MASK: u8 = 0x03;
    }

    /// Type-C constants (verified from official SW2303 register manual)
    ///
    /// These constants are specific to SW2303 Type-C configuration
    /// and are based on the official register manual.
    pub mod type_c {
        /// Type-C current broadcast levels from REG 0xA8
        /// bit 6: Type-C电流广播: 0: 默认电流, 1: 1.5A
        pub const DEFAULT_CURRENT: bool = false;
        pub const CURRENT_1_5A: bool = true;

        /// Type-C current values (in mA)
        pub const DEFAULT_CURRENT_MA: u16 = 500; // USB 2.0 default
        pub const CURRENT_1_5A_MA: u16 = 1500;

        /// CC line control from REG 0x14
        /// bit 1: Type-C CC un-driving: 0: 正常, 1: CC un-driving
        pub const CC_NORMAL: bool = false;
        pub const CC_UN_DRIVING: bool = true;
    }

    /// Fast charging protocol constants (verified from official SW2303 register manual)
    ///
    /// These constants are specific to SW2303 fast charging configuration
    /// and are based on the official register manual.
    pub mod fast_charge {
        /// Current limits for different protocols

        /// FCP/AFC/SFCP current limits from REG 0xAD bit 3
        /// 0: 3.25A, 1: 2.25A
        pub const FCP_AFC_SFCP_CURRENT_3_25A: u16 = 3250;
        pub const FCP_AFC_SFCP_CURRENT_2_25A: u16 = 2250;

        /// SCP current limits from REG 0xB2 bits 5-4
        /// 0: 5.0A, 1: 4.0A, 2: 2.0A, 3: Reserved
        pub const SCP_CURRENT_5_0A: u16 = 5000;
        pub const SCP_CURRENT_4_0A: u16 = 4000;
        pub const SCP_CURRENT_2_0A: u16 = 2000;

        /// SCP current configuration values
        pub const SCP_CURRENT_CONFIG_5A: u8 = 0;
        pub const SCP_CURRENT_CONFIG_4A: u8 = 1;
        pub const SCP_CURRENT_CONFIG_2A: u8 = 2;

        /// 25W 9V PDO current from REG 0xB2 bit 3
        /// 0: 2.78A, 1: 2.77A
        pub const PDO_9V_25W_CURRENT_2_78A: u16 = 2780;
        pub const PDO_9V_25W_CURRENT_2_77A: u16 = 2770;

        /// Voltage thresholds from REG 0xAE
        /// bit 5: 过压检测阈值: 0: 低于3.0V, 1: 低于5.0V
        pub const OVERVOLTAGE_THRESHOLD_3V: u16 = 3000;
        pub const OVERVOLTAGE_THRESHOLD_5V: u16 = 5000;

        /// Protocol enable bits masks for easy configuration
        pub const ALL_PROTOCOLS_DISABLED: u8 = 0x00;
        pub const QC_PROTOCOLS_MASK: u8 = 0x88; // QC2.0/3.0 enable bits
        pub const PD_PROTOCOLS_MASK: u8 = 0x81; // PD enable bits
        pub const FAST_CHARGE_PROTOCOLS_MASK: u8 = 0x1E; // FCP, AFC, SFCP, PE
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
bitflags! {
    /// PD 状态寄存器标志 (REG 0x09)
    ///
    /// 说明：部分位的具体语义因手册版本不同可能略有差异。
    /// 这里先给出位掩码，名称以位序标注，方便在上层据实解析。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdStatusFlags: u8 {
        const B0 = 0b0000_0001;
        const B1 = 0b0000_0010;
        const B2 = 0b0000_0100;
        const B3 = 0b0000_1000;
        const B4 = 0b0001_0000;
        const B5 = 0b0010_0000;
        const B6 = 0b0100_0000;
        const B7 = 0b1000_0000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdStatusFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdStatusFlags({})", self.bits())
    }
}

bitflags! {
    /// Type‑C 状态寄存器标志 (REG 0x0A)
    ///
    /// 说明：具体位语义请参照官方手册。此处提供位掩码，便于上层解析与日志输出。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct TypeCStatusFlags: u8 {
        const B0 = 0b0000_0001;
        const B1 = 0b0000_0010;
        const B2 = 0b0000_0100;
        const B3 = 0b0000_1000;
        const B4 = 0b0001_0000;
        const B5 = 0b0010_0000;
        const B6 = 0b0100_0000;
        const B7 = 0b1000_0000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TypeCStatusFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TypeCStatusFlags({})", self.bits())
    }
}
