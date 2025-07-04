//! SW2303 driver implementation
//! 
//! SW2303 is a USB PD (Power Delivery) charging controller, not a USB hub controller.
//! This driver provides methods to interact with the SW2303 for UFP detection and charging management.

use crate::error::Error;
use crate::registers::{Register, SystemStatus0Flags, SystemStatus3Flags, FastChargingFlags, constants};

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;

/// Default I2C address for the SW2303 PD controller.
pub const DEFAULT_ADDRESS: u8 = constants::DEFAULT_ADDRESS;

/// Driver for the SW2303 USB PD (Power Delivery) controller.
///
/// This struct provides methods to interact with the SW2303 via an I2C bus,
/// allowing monitoring of UFP (USB Function Port) connection status and charging state.
///
/// It is generic over the I2C bus implementation, supporting both synchronous
/// and asynchronous `embedded-hal` traits.
pub struct SW2303<'a, I2C> {
    i2c: &'a mut I2C,
    address: u8,
}

impl<'a, I2C> SW2303<'a, I2C> {
    /// Create a new SW2303 driver instance.
    ///
    /// # Arguments
    ///
    /// * `i2c` - A mutable reference to an I2C bus implementation.
    /// * `address` - The I2C address of the SW2303 device.
    ///
    /// # Returns
    ///
    /// Returns a new `SW2303` instance.
    pub fn new(i2c: &'a mut I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Create a new SW2303 driver instance with the default I2C address.
    ///
    /// # Arguments
    ///
    /// * `i2c` - A mutable reference to an I2C bus implementation.
    ///
    /// # Returns
    ///
    /// Returns a new `SW2303` instance with the default address.
    pub fn new_with_default_address(i2c: &'a mut I2C) -> Self {
        Self::new(i2c, DEFAULT_ADDRESS)
    }

    /// Get the I2C address of this SW2303 instance.
    ///
    /// # Returns
    ///
    /// Returns the I2C address as a `u8`.
    pub fn address(&self) -> u8 {
        self.address
    }
}

// Synchronous implementation
#[cfg(not(feature = "async"))]
impl<'a, I2C> SW2303<'a, I2C>
where
    I2C: I2c,
    I2C::Error: core::fmt::Debug,
{
    /// Read a single register from the SW2303.
    ///
    /// # Arguments
    ///
    /// * `register` - The register to read from.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the register value on success, or an `Error` if the operation fails.
    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut buffer)
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// Write a single register to the SW2303.
    ///
    /// # Arguments
    ///
    /// * `register` - The register to write to.
    /// * `value` - The value to write.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register.addr(), value])
            .map_err(Error::I2c)
    }

    /// Initialize the SW2303 PD controller.
    ///
    /// This method performs the initial setup of the SW2303, including chip version verification
    /// and basic configuration for UFP detection.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if initialization fails.
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        // Verify chip version
        let chip_version = self.read_register(Register::ChipVersion)?;
        let version_bits = chip_version & 0x03; // Extract bits [1:0]
        
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 chip version: 0x{:02X} (version bits: {})", chip_version, version_bits);
        
        // Check if chip version matches expected value (default 0x1)
        if version_bits != constants::CHIP_VERSION {
            #[cfg(feature = "defmt")]
            defmt::warn!("Unexpected chip version: expected {}, got {}", constants::CHIP_VERSION, version_bits);
            // Don't fail initialization, just warn - hardware might be different variant
        }

        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 PD controller initialized successfully");

        Ok(())
    }

    /// Check if UFP (USB Function Port) is connected.
    ///
    /// DEPRECATED: This method is deprecated. Use `is_sink_device_connected()` instead,
    /// which uses the correct SystemStatus3 register for device detection.
    ///
    /// # Returns
    ///
    /// Returns `Ok(false)` always (placeholder implementation).
    #[deprecated(note = "Use is_sink_device_connected() instead")]
    pub fn is_ufp_connected(&mut self) -> Result<bool, Error<I2C::Error>> {
        // Placeholder implementation - use is_sink_device_connected() instead
        Ok(false)
    }

    /// Check if a sink device is connected (online status).
    ///
    /// This method reads the system status 3 register to determine if a sink device is connected.
    /// This is more reliable for detecting actual device connection than UFP status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if sink device is connected (online), `Ok(false)` if not connected,
    /// or an `Error` if the operation fails.
    pub fn is_sink_device_connected(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus3)?;
        let flags = SystemStatus3Flags::from_bits_truncate(status);
        Ok(flags.contains(SystemStatus3Flags::DEVICE_ONLINE))
    }

    /// Get system status 0 flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus0Flags)` on success, or an `Error` if the operation fails.
    pub fn get_system_status0(&mut self) -> Result<SystemStatus0Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus0)?;
        Ok(SystemStatus0Flags::from_bits_truncate(status))
    }



    /// Get system status 3 flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus3Flags)` on success, or an `Error` if the operation fails.
    pub fn get_system_status3(&mut self) -> Result<SystemStatus3Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus3)?;
        Ok(SystemStatus3Flags::from_bits_truncate(status))
    }

    /// Get fast charging status flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargingFlags)` on success, or an `Error` if the operation fails.
    pub fn get_fast_charging_status(&mut self) -> Result<FastChargingFlags, Error<I2C::Error>> {
        let status = self.read_register(Register::FastChargingStatus)?;
        Ok(FastChargingFlags::from_bits_truncate(status))
    }

    /// Read ADC Vin data (input voltage).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub fn read_adc_vin(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVin)
    }

    /// Read ADC Vbus data (bus voltage).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub fn read_adc_vbus(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVbus)
    }

    /// Read ADC Ich data (charging current).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub fn read_adc_ich(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcIch)
    }

    /// Read ADC Tdiet data (die temperature).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub fn read_adc_tdiet(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcTdiet)
    }
}

// Asynchronous implementation
#[cfg(feature = "async")]
impl<'a, I2C> SW2303<'a, I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
    I2C::Error: core::fmt::Debug,
{
    /// Read a single register from the SW2303 (async version).
    ///
    /// # Arguments
    ///
    /// * `register` - The register to read from.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the register value on success, or an `Error` if the operation fails.
    pub async fn read_register(&mut self, register: Register) -> Result<u8, Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// Write a single register to the SW2303 (async version).
    ///
    /// # Arguments
    ///
    /// * `register` - The register to write to.
    /// * `value` - The value to write.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register.addr(), value])
            .await
            .map_err(Error::I2c)
    }

    /// Initialize the SW2303 PD controller (async version).
    ///
    /// This method performs the initial setup of the SW2303, including chip version verification
    /// and basic configuration for UFP detection.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if initialization fails.
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        // Verify chip version
        let chip_version = self.read_register(Register::ChipVersion).await?;
        let version_bits = chip_version & 0x03; // Extract bits [1:0]

        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 chip version: 0x{:02X} (version bits: {})", chip_version, version_bits);

        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 PD controller initialized successfully");

        Ok(())
    }

    /// Check if UFP (USB Function Port) is connected (async version).
    ///
    /// DEPRECATED: This method is deprecated. Use `is_sink_device_connected()` instead,
    /// which uses the correct SystemStatus3 register for device detection.
    ///
    /// # Returns
    ///
    /// Returns `Ok(false)` always (placeholder implementation).
    #[deprecated(note = "Use is_sink_device_connected() instead")]
    pub async fn is_ufp_connected(&mut self) -> Result<bool, Error<I2C::Error>> {
        // Placeholder implementation - use is_sink_device_connected() instead
        Ok(false)
    }

    /// Check if a sink device is connected (online status) (async version).
    ///
    /// This method reads the system status 3 register to determine if a sink device is connected.
    /// This is more reliable for detecting actual device connection than UFP status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if sink device is connected (online), `Ok(false)` if not connected,
    /// or an `Error` if the operation fails.
    pub async fn is_sink_device_connected(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus3).await?;
        let flags = SystemStatus3Flags::from_bits_truncate(status);
        Ok(flags.contains(SystemStatus3Flags::DEVICE_ONLINE))
    }

    /// Get system status 0 flags (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus0Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_system_status0(&mut self) -> Result<SystemStatus0Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus0).await?;
        Ok(SystemStatus0Flags::from_bits_truncate(status))
    }



    /// Get system status 3 flags (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus3Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_system_status3(&mut self) -> Result<SystemStatus3Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus3).await?;
        Ok(SystemStatus3Flags::from_bits_truncate(status))
    }

    /// Get fast charging status flags (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargingFlags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charging_status(&mut self) -> Result<FastChargingFlags, Error<I2C::Error>> {
        let status = self.read_register(Register::FastChargingStatus).await?;
        Ok(FastChargingFlags::from_bits_truncate(status))
    }

    /// Read ADC Vin data (input voltage) (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_vin(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVin).await
    }

    /// Read ADC Vbus data (bus voltage) (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_vbus(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVbus).await
    }

    /// Read ADC Ich data (charging current) (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_ich(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcIch).await
    }

    /// Read ADC Tdiet data (die temperature) (async version).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_tdiet(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcTdiet).await
    }
}
