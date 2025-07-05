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
    /// REG 0x00: Device ID
    DeviceId = 0x00,
    /// REG 0x01: Chip version register
    ChipVersion = 0x01,
    /// REG 0x02: System status 2
    SystemStatus2 = 0x02,
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
    /// REG 0x08: System status 1
    SystemStatus1 = 0x08,
    /// REG 0x09: PD status
    PdStatus = 0x09,
    /// REG 0x0A: Type-C status
    TypeCStatus = 0x0A,
    /// REG 0x0B: System status 4
    SystemStatus4 = 0x0B,
    /// REG 0x0C: System status 5
    SystemStatus5 = 0x0C,
    /// REG 0x0D: System status 3
    SystemStatus3 = 0x0D,
    /// REG 0x0E: Interrupt status 0
    InterruptStatus0 = 0x0E,
    /// REG 0x0F: Interrupt status 1
    InterruptStatus1 = 0x0F,
    /// REG 0x10: Interrupt mask 0
    InterruptMask0 = 0x10,
    /// REG 0x11: Interrupt mask 1
    InterruptMask1 = 0x11,
    /// REG 0x12: I2C write enable control 0
    I2cWriteEnable0 = 0x12,
    /// REG 0x13: Reset control
    ResetControl = 0x13,
    /// REG 0x14: Connection control
    ConnectionControl = 0x14,
    /// REG 0x15: I2C write enable control 1
    I2cWriteEnable1 = 0x15,
    /// REG 0x16: Force control enable
    ForceControlEnable = 0x16,
    /// REG 0x17: PD configuration
    PdConfig = 0x17,
    /// REG 0x18: Type-C configuration
    TypeCConfig = 0x18,
    /// REG 0x19: CC configuration
    CcConfig = 0x19,
    /// REG 0x1A: VCONN configuration
    VconnConfig = 0x1A,
    /// REG 0x1B: PD message control
    PdMessageControl = 0x1B,
    /// REG 0x1C: PD message status
    PdMessageStatus = 0x1C,
    /// REG 0x1D: Source capabilities 0
    SourceCaps0 = 0x1D,
    /// REG 0x1E: Source capabilities 1
    SourceCaps1 = 0x1E,
    /// REG 0x1F: Source capabilities 2
    SourceCaps2 = 0x1F,
    /// REG 0x20: Source capabilities 3
    SourceCaps3 = 0x20,
    /// REG 0x21: Sink capabilities 0
    SinkCaps0 = 0x21,
    /// REG 0x22: Sink capabilities 1
    SinkCaps1 = 0x22,
    /// REG 0x23: Sink capabilities 2
    SinkCaps2 = 0x23,
    /// REG 0x24: Sink capabilities 3
    SinkCaps3 = 0x24,
    /// REG 0x25: Request data object 0
    RequestDo0 = 0x25,
    /// REG 0x26: Request data object 1
    RequestDo1 = 0x26,
    /// REG 0x27: Request data object 2
    RequestDo2 = 0x27,
    /// REG 0x28: Request data object 3
    RequestDo3 = 0x28,
    /// REG 0x29: Timer configuration
    TimerConfig = 0x29,
    /// REG 0x2A: Timer status
    TimerStatus = 0x2A,
    /// REG 0x2B: Watchdog configuration
    WatchdogConfig = 0x2B,
    /// REG 0x2C: GPIO configuration
    GpioConfig = 0x2C,
    /// REG 0x2D: GPIO status
    GpioStatus = 0x2D,
    /// REG 0x2E: GPIO control
    GpioControl = 0x2E,
    /// REG 0x2F: Reserved
    Reserved2F = 0x2F,
    /// REG 0x30: ADC Vin data
    AdcVin = 0x30,
    /// REG 0x31: ADC Vbus data
    AdcVbus = 0x31,
    /// REG 0x32: ADC Vout data
    AdcVout = 0x32,
    /// REG 0x33: ADC Ich data
    AdcIch = 0x33,
    /// REG 0x34: ADC Iout data
    AdcIout = 0x34,
    /// REG 0x35: ADC Vcc data
    AdcVcc = 0x35,
    /// REG 0x36: ADC Tdiet data
    AdcTdiet = 0x36,
    /// REG 0x37: ADC Tntc data
    AdcTntc = 0x37,
    /// REG 0x38: ADC control 0
    AdcControl0 = 0x38,
    /// REG 0x39: ADC control 1
    AdcControl1 = 0x39,
    /// REG 0x3A: ADC control 2
    AdcControl2 = 0x3A,
    /// REG 0x3B: ADC configuration
    AdcConfig = 0x3B,
    /// REG 0x3C: ADC data high 8 bits
    AdcDataHigh = 0x3C,
    /// REG 0x3D: ADC data low 4 bits
    AdcDataLow = 0x3D,
    /// REG 0x3E: ADC calibration 0
    AdcCalibration0 = 0x3E,
    /// REG 0x3F: ADC calibration 1
    AdcCalibration1 = 0x3F,

    // Configuration registers (0xA0-0xBF range, requires unlock)
    /// REG 0xA0: System configuration 0
    SystemConfig0 = 0xA0,
    /// REG 0xA1: System configuration 1
    SystemConfig1 = 0xA1,
    /// REG 0xA2: System configuration 2
    SystemConfig2 = 0xA2,
    /// REG 0xA3: System configuration 3
    SystemConfig3 = 0xA3,
    /// REG 0xA4: PD protocol configuration 0
    PdProtocolConfig0 = 0xA4,
    /// REG 0xA5: PD protocol configuration 1
    PdProtocolConfig1 = 0xA5,
    /// REG 0xA6: PD protocol configuration 2
    PdProtocolConfig2 = 0xA6,
    /// REG 0xA7: Type-C protocol configuration
    TypeCProtocolConfig = 0xA7,
    /// REG 0xA8: CC detection configuration
    CcDetectionConfig = 0xA8,
    /// REG 0xA9: VCONN control configuration
    VconnControlConfig = 0xA9,
    /// REG 0xAA: Power management configuration
    PowerManagementConfig = 0xAA,
    /// REG 0xAB: Thermal management configuration
    ThermalManagementConfig = 0xAB,
    /// REG 0xAC: Safety configuration
    SafetyConfig = 0xAC,
    /// REG 0xAD: Timing configuration 0
    TimingConfig0 = 0xAD,
    /// REG 0xAE: Timing configuration 1
    TimingConfig1 = 0xAE,
    /// REG 0xAF: Power configuration
    PowerConfig = 0xAF,
    /// REG 0xB0: Voltage regulation configuration
    VoltageRegulationConfig = 0xB0,
    /// REG 0xB1: Current regulation configuration
    CurrentRegulationConfig = 0xB1,
    /// REG 0xB2: Protection configuration 0
    ProtectionConfig0 = 0xB2,
    /// REG 0xB3: Protection configuration 1
    ProtectionConfig1 = 0xB3,
    /// REG 0xB4: Calibration configuration 0
    CalibrationConfig0 = 0xB4,
    /// REG 0xB5: Calibration configuration 1
    CalibrationConfig1 = 0xB5,
    /// REG 0xB6: Test configuration 0
    TestConfig0 = 0xB6,
    /// REG 0xB7: Test configuration 1
    TestConfig1 = 0xB7,
    /// REG 0xB8: Debug configuration 0
    DebugConfig0 = 0xB8,
    /// REG 0xB9: Debug configuration 1
    DebugConfig1 = 0xB9,
    /// REG 0xBA: Reserved BA
    ReservedBA = 0xBA,
    /// REG 0xBB: Reserved BB
    ReservedBB = 0xBB,
    /// REG 0xBC: Reserved BC
    ReservedBC = 0xBC,
    /// REG 0xBD: Reserved BD
    ReservedBD = 0xBD,
    /// REG 0xBE: Reserved BE
    ReservedBE = 0xBE,
    /// REG 0xBF: Reserved BF
    ReservedBF = 0xBF,
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
    /// System status 2 flags (REG 0x02)
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
    /// System status 4 flags (REG 0x0B)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus4Flags: u8 {
        /// Temperature warning (bit 7)
        const TEMPERATURE_WARNING = 0b10000000;
        /// Temperature shutdown (bit 6)
        const TEMPERATURE_SHUTDOWN = 0b01000000;
        /// Input undervoltage (bit 5)
        const INPUT_UNDERVOLTAGE = 0b00100000;
        /// Input overvoltage (bit 4)
        const INPUT_OVERVOLTAGE = 0b00010000;
        /// Output undervoltage (bit 3)
        const OUTPUT_UNDERVOLTAGE = 0b00001000;
        /// Output overvoltage (bit 2)
        const OUTPUT_OVERVOLTAGE = 0b00000100;
        /// Reverse current protection (bit 1)
        const REVERSE_CURRENT = 0b00000010;
        /// Short circuit protection (bit 0)
        const SHORT_CIRCUIT = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus4Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus4Flags({})", self.bits())
    }
}

bitflags! {
    /// System status 5 flags (REG 0x0C)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SystemStatus5Flags: u8 {
        /// Watchdog timeout (bit 7)
        const WATCHDOG_TIMEOUT = 0b10000000;
        /// I2C communication error (bit 6)
        const I2C_ERROR = 0b01000000;
        /// ADC conversion complete (bit 5)
        const ADC_COMPLETE = 0b00100000;
        /// Timer expired (bit 4)
        const TIMER_EXPIRED = 0b00010000;
        /// GPIO interrupt (bit 3)
        const GPIO_INTERRUPT = 0b00001000;
        /// Power good (bit 2)
        const POWER_GOOD = 0b00000100;
        /// System ready (bit 1)
        const SYSTEM_READY = 0b00000010;
        /// Initialization complete (bit 0)
        const INIT_COMPLETE = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SystemStatus5Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "SystemStatus5Flags({})", self.bits())
    }
}

bitflags! {
    /// Interrupt status 0 flags (REG 0x0E)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InterruptStatus0Flags: u8 {
        /// PD interrupt (bit 7)
        const PD_INTERRUPT = 0b10000000;
        /// Type-C interrupt (bit 6)
        const TYPE_C_INTERRUPT = 0b01000000;
        /// Power interrupt (bit 5)
        const POWER_INTERRUPT = 0b00100000;
        /// Temperature interrupt (bit 4)
        const TEMPERATURE_INTERRUPT = 0b00010000;
        /// Voltage interrupt (bit 3)
        const VOLTAGE_INTERRUPT = 0b00001000;
        /// Current interrupt (bit 2)
        const CURRENT_INTERRUPT = 0b00000100;
        /// Timer interrupt (bit 1)
        const TIMER_INTERRUPT = 0b00000010;
        /// GPIO interrupt (bit 0)
        const GPIO_INTERRUPT = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for InterruptStatus0Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "InterruptStatus0Flags({})", self.bits())
    }
}

bitflags! {
    /// Interrupt status 1 flags (REG 0x0F)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InterruptStatus1Flags: u8 {
        /// System error interrupt (bit 7)
        const SYSTEM_ERROR = 0b10000000;
        /// Protection interrupt (bit 6)
        const PROTECTION = 0b01000000;
        /// Calibration interrupt (bit 5)
        const CALIBRATION = 0b00100000;
        /// Test interrupt (bit 4)
        const TEST = 0b00010000;
        /// Debug interrupt (bit 3)
        const DEBUG = 0b00001000;
        /// Reserved interrupt (bit 2)
        const RESERVED_2 = 0b00000100;
        /// Reserved interrupt (bit 1)
        const RESERVED_1 = 0b00000010;
        /// Reserved interrupt (bit 0)
        const RESERVED_0 = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for InterruptStatus1Flags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "InterruptStatus1Flags({})", self.bits())
    }
}

bitflags! {
    /// System status 1 flags (REG 0x08)
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
    /// PD status flags (REG 0x09)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdStatusFlags: u8 {
        /// PD contract established (bit 7)
        const CONTRACT_ESTABLISHED = 0b10000000;
        /// Source capabilities received (bit 6)
        const SOURCE_CAPS_RECEIVED = 0b01000000;
        /// Request message sent (bit 5)
        const REQUEST_SENT = 0b00100000;
        /// Accept message received (bit 4)
        const ACCEPT_RECEIVED = 0b00010000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdStatusFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdStatusFlags({})", self.bits())
    }
}

bitflags! {
    /// Type-C status flags (REG 0x0A)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct TypeCStatusFlags: u8 {
        /// CC1 connection detected (bit 7)
        const CC1_CONNECTED = 0b10000000;
        /// CC2 connection detected (bit 6)
        const CC2_CONNECTED = 0b01000000;
        /// VCONN enabled (bit 5)
        const VCONN_ENABLED = 0b00100000;
        /// Data role (bit 4): 0=UFP, 1=DFP
        const DATA_ROLE_DFP = 0b00010000;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TypeCStatusFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TypeCStatusFlags({})", self.bits())
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
    /// Reset control flags (REG 0x13)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ResetControlFlags: u8 {
        /// System reset (bit 7)
        const SYSTEM_RESET = 0b10000000;
        /// PD reset (bit 6)
        const PD_RESET = 0b01000000;
        /// Type-C reset (bit 5)
        const TYPE_C_RESET = 0b00100000;
        /// ADC reset (bit 4)
        const ADC_RESET = 0b00010000;
        /// Timer reset (bit 3)
        const TIMER_RESET = 0b00001000;
        /// GPIO reset (bit 2)
        const GPIO_RESET = 0b00000100;
        /// Watchdog reset (bit 1)
        const WATCHDOG_RESET = 0b00000010;
        /// Soft reset (bit 0)
        const SOFT_RESET = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ResetControlFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "ResetControlFlags({})", self.bits())
    }
}

bitflags! {
    /// PD configuration flags (REG 0x17)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PdConfigFlags: u8 {
        /// PD enable (bit 7)
        const PD_ENABLE = 0b10000000;
        /// Source mode enable (bit 6)
        const SOURCE_MODE = 0b01000000;
        /// Sink mode enable (bit 5)
        const SINK_MODE = 0b00100000;
        /// DRP mode enable (bit 4)
        const DRP_MODE = 0b00010000;
        /// VCONN enable (bit 3)
        const VCONN_ENABLE = 0b00001000;
        /// USB communication enable (bit 2)
        const USB_COMM_ENABLE = 0b00000100;
        /// PD communication enable (bit 1)
        const PD_COMM_ENABLE = 0b00000010;
        /// Auto negotiation enable (bit 0)
        const AUTO_NEGOTIATION = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PdConfigFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PdConfigFlags({})", self.bits())
    }
}

bitflags! {
    /// Type-C configuration flags (REG 0x18)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct TypeCConfigFlags: u8 {
        /// Type-C enable (bit 7)
        const TYPE_C_ENABLE = 0b10000000;
        /// CC debounce enable (bit 6)
        const CC_DEBOUNCE_ENABLE = 0b01000000;
        /// CC pull-up enable (bit 5)
        const CC_PULLUP_ENABLE = 0b00100000;
        /// CC pull-down enable (bit 4)
        const CC_PULLDOWN_ENABLE = 0b00010000;
        /// VCONN discharge enable (bit 3)
        const VCONN_DISCHARGE = 0b00001000;
        /// VBUS discharge enable (bit 2)
        const VBUS_DISCHARGE = 0b00000100;
        /// Dead battery support (bit 1)
        const DEAD_BATTERY = 0b00000010;
        /// Audio accessory support (bit 0)
        const AUDIO_ACCESSORY = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TypeCConfigFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TypeCConfigFlags({})", self.bits())
    }
}

bitflags! {
    /// GPIO configuration flags (REG 0x2C)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct GpioConfigFlags: u8 {
        /// GPIO7 enable (bit 7)
        const GPIO7_ENABLE = 0b10000000;
        /// GPIO6 enable (bit 6)
        const GPIO6_ENABLE = 0b01000000;
        /// GPIO5 enable (bit 5)
        const GPIO5_ENABLE = 0b00100000;
        /// GPIO4 enable (bit 4)
        const GPIO4_ENABLE = 0b00010000;
        /// GPIO3 enable (bit 3)
        const GPIO3_ENABLE = 0b00001000;
        /// GPIO2 enable (bit 2)
        const GPIO2_ENABLE = 0b00000100;
        /// GPIO1 enable (bit 1)
        const GPIO1_ENABLE = 0b00000010;
        /// GPIO0 enable (bit 0)
        const GPIO0_ENABLE = 0b00000001;
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for GpioConfigFlags {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "GpioConfigFlags({})", self.bits())
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
