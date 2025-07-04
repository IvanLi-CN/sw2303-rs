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
