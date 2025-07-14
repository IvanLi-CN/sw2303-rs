//! SW2303 driver implementation using maybe-async-cfg
//!
//! SW2303 is a USB PD (Power Delivery) charging controller, not a USB hub controller.
//! This driver provides methods to interact with the SW2303 for UFP detection and charging management.

use crate::error::Error;
use crate::registers::{
    FastChargingFlags, Register, SystemStatus0Flags,
    SystemStatus1Flags, SystemStatus2Flags, SystemStatus3Flags, constants,
};

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;

#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

/// Default I2C address for the SW2303 PD controller.
pub const DEFAULT_ADDRESS: u8 = constants::DEFAULT_ADDRESS;

/// Driver for the SW2303 USB PD (Power Delivery) controller.
///
/// This struct provides methods to interact with the SW2303 via an I2C bus,
/// allowing monitoring of UFP (USB Function Port) connection status and charging state.
///
/// It is generic over the I2C bus implementation, supporting both synchronous
/// and asynchronous operations through the maybe-async-cfg pattern.
pub struct SW2303<'a, I2C> {
    /// The I2C bus instance used for communication
    i2c: &'a mut I2C,
    /// The I2C address of the SW2303 device
    address: u8,
}

impl<'a, I2C> SW2303<'a, I2C> {
    /// Create a new SW2303 driver instance.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C bus instance
    /// * `address` - The I2C address of the SW2303 device (typically 0x3C)
    ///
    /// # Returns
    ///
    /// Returns a new `SW2303` instance.
    pub fn new(i2c: &'a mut I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Get the I2C address of the device.
    ///
    /// # Returns
    ///
    /// Returns the I2C address as `u8`.
    pub fn address(&self) -> u8 {
        self.address
    }
}

// Unified implementation using maybe-async-cfg
#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "SW2303",),
    async(feature = "async", keep_self)
)]
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
    pub async fn read_register(&mut self, register: Register) -> Result<u8, Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register.addr()], &mut buffer)
            .await
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
    pub async fn write_register(
        &mut self,
        register: Register,
        value: u8,
    ) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register.addr(), value])
            .await
            .map_err(Error::I2c)
    }

    /// Initialize the SW2303 PD controller.
    ///
    /// This method performs the initial setup of the SW2303.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if initialization fails.
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 PD controller initialized successfully");

        Ok(())
    }

    /// Unlock I2C write enable control 0 (REG 0x12).
    ///
    /// This unlocks access to registers reg0x14, reg0xA0-BF.
    /// The unlock sequence requires writing 0x20, 0x40, 0x80 to REG 0x12.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the unlock sequence fails.
    pub async fn unlock_write_enable_0(&mut self) -> Result<(), Error<I2C::Error>> {
        // Unlock sequence for I2C write enable control 0 (REG 0x12)
        // This enables writing to reg0x14, reg0xA0-BF
        // Official datasheet sequence: 0x20, 0x40, 0x80
        self.write_register(Register::I2cWriteEnable0, constants::unlock::WRITE_ENABLE_0_STEP1).await?;
        self.write_register(Register::I2cWriteEnable0, constants::unlock::WRITE_ENABLE_0_STEP2).await?;
        self.write_register(Register::I2cWriteEnable0, constants::unlock::WRITE_ENABLE_0_STEP3).await?;

        Ok(())
    }

    /// Unlock I2C write enable control 1 (REG 0x15).
    ///
    /// This unlocks access to register reg0x16.
    /// The unlock sequence requires writing 0x20, 0x40, 0x80 to REG 0x15.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the unlock sequence fails.
    pub async fn unlock_write_enable_1(&mut self) -> Result<(), Error<I2C::Error>> {
        // Unlock sequence for I2C write enable control 1 (REG 0x15)
        // This enables writing to reg0x16
        self.write_register(Register::I2cWriteEnable1, 0x20).await?;
        self.write_register(Register::I2cWriteEnable1, 0x40).await?;
        self.write_register(Register::I2cWriteEnable1, 0x80).await?;

        Ok(())
    }

    /// Set the output voltage.
    ///
    /// # Arguments
    ///
    /// * `voltage_mv` - The voltage to set in millivolts
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_voltage(&mut self, voltage_mv: u16) -> Result<(), Error<I2C::Error>> {
        // Convert voltage from mV to register value (assuming 100mV per unit)
        let voltage_units = voltage_mv / 100;
        let voltage_high = (voltage_units >> 8) as u8;
        let voltage_low = (voltage_units & 0x0F) as u8;

        self.write_register(Register::VoltageHigh, voltage_high)
            .await?;
        self.write_register(Register::VoltageLow, voltage_low)
            .await?;

        Ok(())
    }



    /// Get the current voltage setting.
    ///
    /// # Returns
    ///
    /// Returns the voltage in millivolts, or an `Error` if the operation fails.
    pub async fn get_voltage(&mut self) -> Result<u16, Error<I2C::Error>> {
        let voltage_high = self.read_register(Register::VoltageHigh).await?;
        let voltage_low = self.read_register(Register::VoltageLow).await?;

        let voltage_units = ((voltage_high as u16) << 8) | (voltage_low as u16);
        // Use saturating multiplication to prevent overflow
        Ok(voltage_units.saturating_mul(100)) // Convert back to mV
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
    pub async fn is_sink_device_connected(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus3).await?;
        let flags = SystemStatus3Flags::from_bits_truncate(status);
        Ok(flags.contains(SystemStatus3Flags::DEVICE_ONLINE))
    }

    /// Get system status 0 flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus0Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_system_status0(&mut self) -> Result<SystemStatus0Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus0).await?;
        Ok(SystemStatus0Flags::from_bits_truncate(status))
    }

    /// Get system status 3 flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus3Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_system_status3(&mut self) -> Result<SystemStatus3Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus3).await?;
        Ok(SystemStatus3Flags::from_bits_truncate(status))
    }

    /// Get fast charging status flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargingFlags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charging_status(
        &mut self,
    ) -> Result<FastChargingFlags, Error<I2C::Error>> {
        let status = self.read_register(Register::FastChargingStatus).await?;
        Ok(FastChargingFlags::from_bits_truncate(status))
    }

    /// Read ADC Vin data (input voltage).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_vin(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVin).await
    }

    /// Read ADC Vbus data (bus voltage).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_vbus(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVbus).await
    }

    /// Read ADC Ich data (charging current).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_ich(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcIch).await
    }

    /// Read ADC Tdiet data (die temperature).
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the raw ADC value, or an `Error` if the operation fails.
    pub async fn read_adc_tdiet(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcTdiet).await
    }

    /// Set the power configuration using REG 0xAF.
    ///
    /// This method configures the SW2303 power setting using the power configuration register.
    /// The power is configured in register mode with the specified wattage.
    ///
    /// # Arguments
    ///
    /// * `power_watts` - The desired power in watts (e.g., 65 for 65W)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    ///
    /// # Note
    ///
    /// This method requires `unlock_write_enable_0()` to be called first,
    /// as REG 0xAF is in the locked register range (0xA0-0xBF).
    pub async fn set_power_config(&mut self, power_watts: u8) -> Result<(), Error<I2C::Error>> {
        // Ensure power setting is within valid range (0-127W)
        let power_setting = power_watts & constants::power::POWER_SETTING_MASK;

        // Configure for register mode (bit 7 = 1) with specified power
        let config_value = constants::power::POWER_CONFIG_REGISTER_MODE | power_setting;

        self.write_register(Register::PowerConfig, config_value)
            .await?;

        Ok(())
    }

    /// Get the current power configuration from REG 0xAF.
    ///
    /// This method reads the power configuration register.
    ///
    /// # Returns
    ///
    /// Returns `Ok((bool, u8))` where the first value indicates if register mode is enabled,
    /// and the second value is the power setting in watts.
    pub async fn get_power_config(&mut self) -> Result<(bool, u8), Error<I2C::Error>> {
        let config_value = self.read_register(Register::PowerConfig).await?;

        let register_mode = (config_value & constants::power::POWER_CONFIG_REGISTER_MODE) != 0;
        let power_watts = config_value & constants::power::POWER_SETTING_MASK;

        Ok((register_mode, power_watts))
    }

    // Note: PD status register (0x09) is not available in official documentation
    // pub async fn get_pd_status(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::PdStatus).await
    // }



    /// Read system status 1 flags from REG 0x08.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus1Flags)` with the current system status 1 flags.
    pub async fn get_system_status_1(&mut self) -> Result<SystemStatus1Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus1).await?;
        Ok(SystemStatus1Flags::from_bits_truncate(status))
    }

    // Note: PD status and RDO registers are not available in official documentation
    // pub async fn is_pd_contract_established(&mut self) -> Result<bool, Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(false)
    // }
    //
    // pub async fn read_request_data_object(&mut self) -> Result<[u8; 4], Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok([0, 0, 0, 0])
    // }

    // Note: Negotiated power calculation requires PD registers not available in official documentation
    // pub async fn get_negotiated_power(&mut self) -> Result<u16, Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(0)
    // }



    /// Check for overcurrent protection status (112.5% threshold).
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if overcurrent protection is triggered, `Ok(false)` otherwise.
    pub async fn is_overcurrent(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status1 = self.get_system_status_1().await?;
        Ok(status1.contains(SystemStatus1Flags::OVERCURRENT_112_5_PERCENT))
    }

    /// Read system status 2 flags from REG 0x02.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus2Flags)` with the current system status 2 flags.
    pub async fn get_system_status_2(&mut self) -> Result<SystemStatus2Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus2).await?;
        Ok(SystemStatus2Flags::from_bits_truncate(status))
    }

    // Note: SystemStatus4 and SystemStatus5 registers are not available in official documentation
    // These were likely project-specific additions
    // pub async fn get_system_status_4(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::SystemStatus4).await
    // }
    //
    // pub async fn get_system_status_5(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::SystemStatus5).await
    // }

    // Note: Interrupt status registers are not available in official documentation
    // pub async fn get_interrupt_status_0(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::InterruptStatus0).await
    // }
    //
    // pub async fn get_interrupt_status_1(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::InterruptStatus1).await
    // }



    // Note: These methods require registers not available in official documentation
    // pub async fn is_system_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(false)
    // }
    //
    // pub async fn is_temperature_warning(&mut self) -> Result<bool, Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(false)
    // }
    //
    // pub async fn is_protection_active(&mut self) -> Result<bool, Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(false)
    // }
    //
    // pub async fn system_reset(&mut self) -> Result<(), Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(())
    // }
    //
    // pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
    //     // Implementation would need to use available registers
    //     Ok(())
    // }

    // Note: PD, Type-C, and GPIO configuration registers are not available in official documentation
    // pub async fn configure_pd(&mut self, config: u8) -> Result<(), Error<I2C::Error>> {
    //     self.write_register(Register::PdConfig, config).await
    // }
    //
    // pub async fn configure_type_c(&mut self, config: u8) -> Result<(), Error<I2C::Error>> {
    //     self.write_register(Register::TypeCConfig, config).await
    // }
    //
    // pub async fn configure_gpio(&mut self, config: u8) -> Result<(), Error<I2C::Error>> {
    //     self.write_register(Register::GpioConfig, config).await
    // }

    // Note: These ADC registers are not available in official documentation
    // pub async fn read_adc_vout(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::AdcVout).await
    // }
    //
    // pub async fn read_adc_iout(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::AdcIout).await
    // }
    //
    // pub async fn read_adc_tntc(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::AdcTntc).await
    // }

    // Note: Device ID register is not available in official documentation
    // pub async fn read_device_id(&mut self) -> Result<u8, Error<I2C::Error>> {
    //     self.read_register(Register::DeviceId).await
    // }

    // Note: These registers are not available in official documentation
    // Commented out methods that use non-existent registers:
    // - set_interrupt_mask_0/1 (InterruptMask0/1 registers)
    // - read_timer_status (TimerStatus register)
    // - configure_watchdog (WatchdogConfig register)
    // - read_gpio_status (GpioStatus register)
    // - control_gpio (GpioControl register)
}
