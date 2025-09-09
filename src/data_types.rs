//! SW2303 data type definitions.

/// Represents the state of a USB port (High or Low).
/// Used for both status and control operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortState {
    /// Low state (disabled/disconnected).
    Low,
    /// High state (enabled/connected).
    High,
}

/// Represents the power state of a USB port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerState {
    /// Port power is off.
    Off,
    /// Port power is on.
    On,
}

/// Represents the connection status of a USB port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConnectionStatus {
    /// No device connected to the port.
    Disconnected,
    /// Device connected to the port.
    Connected,
}

/// Represents the speed of a connected USB device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceSpeed {
    /// Low speed device (1.5 Mbps).
    LowSpeed,
    /// Full speed device (12 Mbps).
    FullSpeed,
    /// High speed device (480 Mbps).
    HighSpeed,
}

/// Defines the individual ports of the SW2303 USB hub controller.
///
/// The SW2303 typically supports up to 4 downstream ports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Port {
    /// Port 0
    Port0 = 0,
    /// Port 1
    Port1 = 1,
    /// Port 2
    Port2 = 2,
    /// Port 3
    Port3 = 3,
}

impl Port {
    /// Get the port number as u8.
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Create a Port from a u8 value.
    /// Returns None if the value is out of range.
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Port::Port0),
            1 => Some(Port::Port1),
            2 => Some(Port::Port2),
            3 => Some(Port::Port3),
            _ => None,
        }
    }

    /// Get all available ports as an array.
    pub const fn all() -> [Port; 4] {
        [Port::Port0, Port::Port1, Port::Port2, Port::Port3]
    }
}

/// Represents the overall status of a USB port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PortStatus {
    /// Connection status of the port.
    pub connection: ConnectionStatus,
    /// Power state of the port.
    pub power: PowerState,
    /// Whether the port is enabled.
    pub enabled: bool,
    /// Whether the port is suspended.
    pub suspended: bool,
    /// Whether the port is in reset state.
    pub in_reset: bool,
    /// Whether overcurrent is detected.
    pub overcurrent: bool,
    /// Speed of the connected device (if any).
    pub device_speed: Option<DeviceSpeed>,
}

impl Default for PortStatus {
    fn default() -> Self {
        Self {
            connection: ConnectionStatus::Disconnected,
            power: PowerState::Off,
            enabled: false,
            suspended: false,
            in_reset: false,
            overcurrent: false,
            device_speed: None,
        }
    }
}

/// Configuration options for the SW2303 hub.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HubConfiguration {
    /// Whether the hub is enabled.
    pub enabled: bool,
    /// Whether overcurrent protection is enabled.
    pub overcurrent_protection: bool,
    /// Whether individual port power switching is enabled.
    pub power_switching: bool,
    /// Whether the hub operates as a compound device.
    pub compound_device: bool,
}

impl Default for HubConfiguration {
    fn default() -> Self {
        Self {
            enabled: true,
            overcurrent_protection: true,
            power_switching: false,
            compound_device: false,
        }
    }
}

/// Device identification information.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceInfo {
    /// Device ID.
    pub device_id: u8,
    /// Vendor ID.
    pub vendor_id: u16,
    /// Product ID.
    pub product_id: u16,
}

/// Interrupt configuration for the SW2303.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Enable port connection change interrupts.
    pub connection_change: bool,
    /// Enable port enable change interrupts.
    pub enable_change: bool,
    /// Enable port suspend change interrupts.
    pub suspend_change: bool,
    /// Enable overcurrent change interrupts.
    pub overcurrent_change: bool,
    /// Enable port reset change interrupts.
    pub reset_change: bool,
}

impl Default for InterruptConfig {
    fn default() -> Self {
        Self {
            connection_change: false,
            enable_change: false,
            suspend_change: false,
            overcurrent_change: false,
            reset_change: false,
        }
    }
}

/// Protocol type enumeration for SW2303 charging protocols.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtocolType {
    /// USB Power Delivery protocol
    PD,
    /// Qualcomm Quick Charge 2.0
    QC20,
    /// Qualcomm Quick Charge 3.0
    QC30,
    /// Huawei Fast Charge Protocol
    FCP,
    /// Samsung Adaptive Fast Charging
    AFC,
    /// Huawei Super Charge Protocol
    SCP,
    /// Qualcomm Power Engine 2.0
    PE20,
    /// USB Battery Charging 1.2
    BC12,
    /// Super Fast Charge Protocol
    SFCP,
}

/// Overall protocol configuration for SW2303.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProtocolConfiguration {
    /// Whether PD protocol is enabled
    pub pd_enabled: bool,
    /// Whether QC2.0 protocol is enabled
    pub qc20_enabled: bool,
    /// Whether QC3.0 protocol is enabled
    pub qc30_enabled: bool,
    /// Whether FCP protocol is enabled
    pub fcp_enabled: bool,
    /// Whether AFC protocol is enabled
    pub afc_enabled: bool,
    /// Whether SCP protocol is enabled
    pub scp_enabled: bool,
    /// Whether PE2.0 protocol is enabled
    pub pe20_enabled: bool,
    /// Whether BC1.2 protocol is enabled
    pub bc12_enabled: bool,
    /// Whether SFCP protocol is enabled
    pub sfcp_enabled: bool,
}

impl Default for ProtocolConfiguration {
    fn default() -> Self {
        Self {
            pd_enabled: false,
            qc20_enabled: false,
            qc30_enabled: false,
            fcp_enabled: false,
            afc_enabled: false,
            scp_enabled: false,
            pe20_enabled: false,
            bc12_enabled: false,
            sfcp_enabled: false,
        }
    }
}

/// PD-specific configuration for SW2303.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdConfiguration {
    /// Whether PD protocol is enabled
    pub enabled: bool,
    /// Whether VCONN swap is supported
    pub vconn_swap: bool,
    /// Whether data role swap is supported
    pub dr_swap: bool,
    /// Whether Emarker is enabled
    pub emarker_enabled: bool,
    /// Whether PPS (Programmable Power Supply) is enabled
    pub pps_enabled: bool,
    /// Fixed voltage levels to enable (9V, 12V, 15V, 20V)
    /// Each bit represents: [9V, 12V, 15V, 20V]
    pub fixed_voltages: [bool; 4],
    /// Whether to bypass 5A emark requirement
    pub emark_5a_bypass: bool,
    /// Whether to enable 60-70W emarker
    pub emarker_60_70w: bool,
}

impl Default for PdConfiguration {
    fn default() -> Self {
        Self {
            enabled: false,
            vconn_swap: true, // Usually enabled by default
            dr_swap: false,
            emarker_enabled: false,
            pps_enabled: false,
            fixed_voltages: [false; 4], // All voltages disabled by default
            emark_5a_bypass: false,
            emarker_60_70w: false,
        }
    }
}

/// Fast charging configuration for SW2303.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastChargeConfiguration {
    /// Whether QC2.0/QC3.0 protocols are enabled
    pub qc_enabled: bool,
    /// Whether FCP protocol is enabled
    pub fcp_enabled: bool,
    /// Whether AFC protocol is enabled
    pub afc_enabled: bool,
    /// Whether SCP protocol is enabled
    pub scp_enabled: bool,
    /// Whether PE2.0 protocol is enabled
    pub pe20_enabled: bool,
    /// Whether SFCP protocol is enabled
    pub sfcp_enabled: bool,
    /// Whether BC1.2 protocol is enabled
    pub bc12_enabled: bool,
    /// SCP current limit configuration (0: 5.0A, 1: 4.0A, 2: 2.0A)
    pub scp_current_limit: u8,
    /// FCP/AFC/SFCP current limit (false: 3.25A, true: 2.25A)
    pub fcp_afc_sfcp_2_25a: bool,
    /// Whether to enable 20V output for QC2.0
    pub qc20_20v_enabled: bool,
    /// Whether to enable 20V output for QC3.0
    pub qc30_20v_enabled: bool,
    /// Whether to enable 20V output for PE2.0
    pub pe20_20v_enabled: bool,
    /// Whether to enable 12V output for PD
    pub pd_12v_enabled: bool,
}

impl Default for FastChargeConfiguration {
    fn default() -> Self {
        Self {
            qc_enabled: false,
            fcp_enabled: false,
            afc_enabled: false,
            scp_enabled: false,
            pe20_enabled: false,
            sfcp_enabled: false,
            bc12_enabled: false,
            scp_current_limit: 2,     // Default to 2.0A for safety
            fcp_afc_sfcp_2_25a: true, // Default to 2.25A for safety
            qc20_20v_enabled: false,
            qc30_20v_enabled: false,
            pe20_20v_enabled: false,
            pd_12v_enabled: false,
        }
    }
}

/// Type-C configuration for SW2303.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TypeCConfiguration {
    /// Type-C current broadcast (false: default current, true: 1.5A)
    pub current_1_5a: bool,
    /// Whether PD PPS 5A output is enabled
    pub pd_pps_5a: bool,
    /// CC line control (false: normal, true: un-driving)
    pub cc_un_driving: bool,
}

impl Default for TypeCConfiguration {
    fn default() -> Self {
        Self {
            current_1_5a: false, // Default current
            pd_pps_5a: false,
            cc_un_driving: false, // Normal operation
        }
    }
}
