//! SW2303 driver implementation using maybe-async-cfg
//!
//! SW2303 is a USB PD (Power Delivery) charging controller, not a USB hub controller.
//! This driver provides methods to interact with the SW2303 for UFP detection and charging management.

use crate::data_types::{
    FastChargeConfiguration, PdConfiguration, PowerRequest, ProtocolConfiguration, ProtocolType,
    TypeCConfiguration,
};
use crate::error::Error;
use crate::registers::{
    BroadcastCurrentFlags, ConnectionControlFlags, FastChargeConfig0Flags, FastChargeConfig1Flags,
    FastChargeConfig2Flags, FastChargeConfig3Flags, FastChargeConfig4Flags, FastChargingFlags,
    ForceControlFlags, PdConfig0Flags, PdConfig1Flags, PdConfig2Flags, PdConfig3Flags, Register,
    SystemStatus0Flags, SystemStatus1Flags, SystemStatus2Flags, SystemStatus3Flags, constants,
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
impl<I2C> SW2303<'_, I2C>
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
        self.write_register(
            Register::I2cWriteEnable0,
            constants::unlock::WRITE_ENABLE_0_STEP1,
        )
        .await?;
        self.write_register(
            Register::I2cWriteEnable0,
            constants::unlock::WRITE_ENABLE_0_STEP2,
        )
        .await?;
        self.write_register(
            Register::I2cWriteEnable0,
            constants::unlock::WRITE_ENABLE_0_STEP3,
        )
        .await?;

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
        // Manual: dac_vol[11:0] * 10mV
        let dac_vol = voltage_mv / 10;
        if dac_vol > 0x0FFF {
            return Err(Error::InvalidParameter);
        }

        let voltage_high = (dac_vol >> 4) as u8; // REG 0x03: [11:4]
        let voltage_low_nibble = (dac_vol & 0x0F) as u8; // REG 0x04[7:4]: [3:0]

        // Preserve REG 0x04[3:0] reserved bits
        let low_cur = self.read_register(Register::VoltageLow).await?;
        let voltage_low = (low_cur & 0x0F) | (voltage_low_nibble << 4);

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

        // Manual: REG 0x03 -> dac_vol[11:4], REG 0x04[7:4] -> dac_vol[3:0]
        let dac_vol = ((voltage_high as u16) << 4) | ((voltage_low as u16) >> 4);
        Ok(dac_vol.saturating_mul(10))
    }

    /// Set the output current limit.
    ///
    /// Manual: `ctrl_icc[6:0] * 50mA` (REG 0x05[6:0]). Bit 7 is preserved.
    pub async fn set_current_limit_ma(&mut self, current_ma: u16) -> Result<(), Error<I2C::Error>> {
        let units = current_ma / 50;
        if units > 0x7F {
            return Err(Error::InvalidParameter);
        }

        let cur = self.read_register(Register::CurrentLimit).await?;
        let value = (cur & 0x80) | (units as u8 & 0x7F);
        self.write_register(Register::CurrentLimit, value).await?;
        Ok(())
    }

    /// Get the current output current limit setpoint in milliamps.
    pub async fn get_current_limit_ma(&mut self) -> Result<u16, Error<I2C::Error>> {
        let v = self.read_register(Register::CurrentLimit).await?;
        Ok(((v & 0x7F) as u16).saturating_mul(50))
    }

    /// Get current voltage/current setpoints from registers (decoded).
    pub async fn get_power_request(&mut self) -> Result<PowerRequest, Error<I2C::Error>> {
        Ok(PowerRequest {
            voltage_mv: self.get_voltage().await?,
            current_limit_ma: self.get_current_limit_ma().await?,
        })
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

    // ========================================================================
    // ADC 12-bit convenience APIs (REG 0x3B select + 0x3C/0x3D read)
    // ========================================================================

    async fn adc_select_12bit(&mut self, sel: u8) -> Result<(), Error<I2C::Error>> {
        // Bits 2-0: 1=vin, 2=vbus, 3=ich, 4=tdiet
        self.write_register(Register::AdcConfig, sel & 0x07).await
    }

    async fn adc_read_12bit_raw(&mut self) -> Result<u16, Error<I2C::Error>> {
        let high = self.read_register(Register::AdcDataHigh).await?;
        let low = self.read_register(Register::AdcDataLow).await? & 0x0F;
        Ok(((high as u16) << 4) | (low as u16))
    }

    /// Read Vin in mV using 12-bit ADC path.
    pub async fn read_vin_mv_12bit(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.adc_select_12bit(constants::adc::ADC_SELECT_VIN)
            .await?;
        let raw = self.adc_read_12bit_raw().await? as u32;
        Ok((constants::adc::VIN_FACTOR_12BIT_MV * raw as f32) as u32)
    }

    /// Read Vbus in mV using 12-bit ADC path.
    pub async fn read_vbus_mv_12bit(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.adc_select_12bit(constants::adc::ADC_SELECT_VBUS)
            .await?;
        let raw = self.adc_read_12bit_raw().await? as u32;
        Ok((constants::adc::VBUS_FACTOR_12BIT_MV * raw as f32) as u32)
    }

    /// Read Ich in mA using 12-bit ADC path.
    pub async fn read_ich_ma_12bit(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.adc_select_12bit(constants::adc::ADC_SELECT_ICH)
            .await?;
        let raw = self.adc_read_12bit_raw().await? as u32;
        Ok((constants::adc::ICH_FACTOR_12BIT_MA * raw as f32) as u32)
    }

    /// Read die temperature in Celsius (approx) using 12-bit ADC path.
    pub async fn read_tdie_c_12bit(&mut self) -> Result<f32, Error<I2C::Error>> {
        self.adc_select_12bit(constants::adc::ADC_SELECT_TDIET)
            .await?;
        let raw = self.adc_read_12bit_raw().await? as f32;
        Ok((raw - constants::adc::TDIET_OFFSET) / constants::adc::TDIET_DIVISOR)
    }

    /// Read die temperature in Celsius using best available path.
    ///
    /// Prefer the calibrated 12-bit path; falls back to 8-bit approximation
    /// when only the raw 8-bit value is available.
    pub async fn read_tdie_c(&mut self) -> Result<f32, Error<I2C::Error>> {
        // Use 12-bit calibrated method; same I2C cost as 8-bit and more accurate
        self.read_tdie_c_12bit().await
    }

    /// Read die temperature in Celsius using 8-bit path with approximation.
    /// Formula derived from 12-bit: T = (16*t8 - 1848) / 6.72
    pub async fn read_tdie_c_8bit(&mut self) -> Result<f32, Error<I2C::Error>> {
        let t8 = self.read_adc_tdiet().await? as f32;
        let t = (16.0 * t8 - constants::adc::TDIET_OFFSET) / constants::adc::TDIET_DIVISOR;
        Ok(t)
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
    // PD 状态寄存器 (0x09) 未在已校对文档中提供稳定位义，暂不暴露。

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

    // ========================================================================
    // Protocol Configuration Methods - Low-level Register API
    // ========================================================================

    /// Set PD configuration 0 register (REG 0xB3) raw flags.
    ///
    /// This method provides direct control over PD basic configuration.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - PD configuration 0 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_pd_config_0_raw(
        &mut self,
        flags: PdConfig0Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::PdConfig0, flags.bits()).await
    }

    /// Get PD configuration 0 register (REG 0xB3) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(PdConfig0Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_pd_config_0_raw(&mut self) -> Result<PdConfig0Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::PdConfig0).await?;
        Ok(PdConfig0Flags::from_bits_truncate(value))
    }

    /// Set PD configuration 1 register (REG 0xB4) raw flags.
    ///
    /// This method provides direct control over PD advanced features.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - PD configuration 1 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_pd_config_1_raw(
        &mut self,
        flags: PdConfig1Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::PdConfig1, flags.bits()).await
    }

    /// Get PD configuration 1 register (REG 0xB4) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(PdConfig1Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_pd_config_1_raw(&mut self) -> Result<PdConfig1Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::PdConfig1).await?;
        Ok(PdConfig1Flags::from_bits_truncate(value))
    }

    /// Set PD configuration 2 register (REG 0xB5) raw flags.
    ///
    /// This method provides direct control over PD voltage levels.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - PD configuration 2 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_pd_config_2_raw(
        &mut self,
        flags: PdConfig2Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::PdConfig2, flags.bits()).await
    }

    /// Get PD configuration 2 register (REG 0xB5) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(PdConfig2Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_pd_config_2_raw(&mut self) -> Result<PdConfig2Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::PdConfig2).await?;
        Ok(PdConfig2Flags::from_bits_truncate(value))
    }

    /// Set PD configuration 3 register (REG 0xA6) raw flags.
    ///
    /// This method provides direct control over PD 5A emark configuration.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - PD configuration 3 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_pd_config_3_raw(
        &mut self,
        flags: PdConfig3Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::PdConfig3, flags.bits()).await
    }

    /// Get PD configuration 3 register (REG 0xA6) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(PdConfig3Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_pd_config_3_raw(&mut self) -> Result<PdConfig3Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::PdConfig3).await?;
        Ok(PdConfig3Flags::from_bits_truncate(value))
    }

    /// Set fast charging configuration 0 register (REG 0xAD) raw flags.
    ///
    /// This method provides direct control over fast charging basic configuration.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - Fast charging configuration 0 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_fast_charge_config_0_raw(
        &mut self,
        flags: FastChargeConfig0Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::FastChargingConfig0, flags.bits())
            .await
    }

    /// Get fast charging configuration 0 register (REG 0xAD) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargeConfig0Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charge_config_0_raw(
        &mut self,
    ) -> Result<FastChargeConfig0Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::FastChargingConfig0).await?;
        Ok(FastChargeConfig0Flags::from_bits_truncate(value))
    }

    /// Set fast charging configuration 1 register (REG 0xAE) raw flags.
    ///
    /// This method provides direct control over fast charging voltage configuration.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - Fast charging configuration 1 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_fast_charge_config_1_raw(
        &mut self,
        flags: FastChargeConfig1Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::FastChargingConfig1, flags.bits())
            .await
    }

    /// Get fast charging configuration 1 register (REG 0xAE) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargeConfig1Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charge_config_1_raw(
        &mut self,
    ) -> Result<FastChargeConfig1Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::FastChargingConfig1).await?;
        Ok(FastChargeConfig1Flags::from_bits_truncate(value))
    }

    /// Set fast charging configuration 2 register (REG 0xB0) raw flags.
    ///
    /// This method provides direct control over fast charging protocol enables.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - Fast charging configuration 2 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_fast_charge_config_2_raw(
        &mut self,
        flags: FastChargeConfig2Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::FastChargingConfig2, flags.bits())
            .await
    }

    /// Get fast charging configuration 2 register (REG 0xB0) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargeConfig2Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charge_config_2_raw(
        &mut self,
    ) -> Result<FastChargeConfig2Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::FastChargingConfig2).await?;
        Ok(FastChargeConfig2Flags::from_bits_truncate(value))
    }

    /// Set fast charging configuration 3 register (REG 0xB1) raw flags.
    ///
    /// This method provides direct control over advanced fast charging protocols.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - Fast charging configuration 3 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_fast_charge_config_3_raw(
        &mut self,
        flags: FastChargeConfig3Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::FastChargingConfig3, flags.bits())
            .await
    }

    /// Get fast charging configuration 3 register (REG 0xB1) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargeConfig3Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charge_config_3_raw(
        &mut self,
    ) -> Result<FastChargeConfig3Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::FastChargingConfig3).await?;
        Ok(FastChargeConfig3Flags::from_bits_truncate(value))
    }

    // ========================================================================
    // Identity fields: VID/PID/SVID/XID (REG 0xB6..0xBF)
    // ========================================================================

    /// Set USB-IF Vendor ID (VID). Requires unlock_write_enable_0().
    pub async fn set_vid(&mut self, vid: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::VidConfig0, (vid >> 8) as u8)
            .await?;
        self.write_register(Register::VidConfig1, (vid & 0xFF) as u8)
            .await
    }

    /// Read VID.
    pub async fn get_vid(&mut self) -> Result<u16, Error<I2C::Error>> {
        let hi = self.read_register(Register::VidConfig0).await? as u16;
        let lo = self.read_register(Register::VidConfig1).await? as u16;
        Ok((hi << 8) | lo)
    }

    /// Set Product ID (PID). Requires unlock_write_enable_0().
    pub async fn set_pid(&mut self, pid: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::PidConfig0, (pid >> 8) as u8)
            .await?;
        self.write_register(Register::PidConfig1, (pid & 0xFF) as u8)
            .await
    }

    /// Read PID.
    pub async fn get_pid(&mut self) -> Result<u16, Error<I2C::Error>> {
        let hi = self.read_register(Register::PidConfig0).await? as u16;
        let lo = self.read_register(Register::PidConfig1).await? as u16;
        Ok((hi << 8) | lo)
    }

    /// Set SVID. Requires unlock_write_enable_0().
    pub async fn set_svid(&mut self, svid: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::SvidConfig0, (svid >> 8) as u8)
            .await?;
        self.write_register(Register::SvidConfig1, (svid & 0xFF) as u8)
            .await
    }

    /// Read SVID.
    pub async fn get_svid(&mut self) -> Result<u16, Error<I2C::Error>> {
        let hi = self.read_register(Register::SvidConfig0).await? as u16;
        let lo = self.read_register(Register::SvidConfig1).await? as u16;
        Ok((hi << 8) | lo)
    }

    /// Set XID (32-bit). Requires unlock_write_enable_0().
    pub async fn set_xid(&mut self, xid: u32) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::XidConfig0, ((xid >> 24) & 0xFF) as u8)
            .await?;
        self.write_register(Register::XidConfig1, ((xid >> 16) & 0xFF) as u8)
            .await?;
        self.write_register(Register::XidConfig2, ((xid >> 8) & 0xFF) as u8)
            .await?;
        self.write_register(Register::XidConfig3, (xid & 0xFF) as u8)
            .await
    }

    /// Read XID (32-bit).
    pub async fn get_xid(&mut self) -> Result<u32, Error<I2C::Error>> {
        let b3 = self.read_register(Register::XidConfig3).await? as u32;
        let b2 = self.read_register(Register::XidConfig2).await? as u32;
        let b1 = self.read_register(Register::XidConfig1).await? as u32;
        let b0 = self.read_register(Register::XidConfig0).await? as u32;
        Ok((b0 << 24) | (b1 << 16) | (b2 << 8) | b3)
    }

    // ========================================================================
    // Force control (REG 0x16) — requires unlock_write_enable_1() before write
    // ========================================================================

    /// Get force control raw flags.
    pub async fn get_force_control_raw(&mut self) -> Result<ForceControlFlags, Error<I2C::Error>> {
        let v = self.read_register(Register::ForceControlEnable).await?;
        Ok(ForceControlFlags::from_bits_truncate(v))
    }

    /// Set force control raw flags.
    pub async fn set_force_control_raw(
        &mut self,
        flags: ForceControlFlags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::ForceControlEnable, flags.bits())
            .await
    }

    /// Force open/close path
    pub async fn force_path(&mut self, open: bool) -> Result<(), Error<I2C::Error>> {
        let mut f = self.get_force_control_raw().await?;
        // Clear both bits first
        f.remove(ForceControlFlags::FORCE_OPEN_PATH | ForceControlFlags::FORCE_CLOSE_PATH);
        if open {
            f.insert(ForceControlFlags::FORCE_OPEN_PATH);
        } else {
            f.insert(ForceControlFlags::FORCE_CLOSE_PATH);
        }
        self.set_force_control_raw(f).await
    }

    /// Force DAC control enable/disable
    pub async fn force_dac(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut f = self.get_force_control_raw().await?;
        if enable {
            f.insert(ForceControlFlags::FORCE_DAC);
        } else {
            f.remove(ForceControlFlags::FORCE_DAC);
        }
        self.set_force_control_raw(f).await
    }

    /// Force current limit control enable/disable
    pub async fn force_current_limit(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut f = self.get_force_control_raw().await?;
        if enable {
            f.insert(ForceControlFlags::FORCE_CURRENT);
        } else {
            f.remove(ForceControlFlags::FORCE_CURRENT);
        }
        self.set_force_control_raw(f).await
    }

    /// Set fast charging configuration 4 register (REG 0xB2) raw flags.
    ///
    /// This method provides direct control over SCP configuration.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - Fast charging configuration 4 flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_fast_charge_config_4_raw(
        &mut self,
        flags: FastChargeConfig4Flags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::FastChargingConfig4, flags.bits())
            .await
    }

    /// Get fast charging configuration 4 register (REG 0xB2) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(FastChargeConfig4Flags)` on success, or an `Error` if the operation fails.
    pub async fn get_fast_charge_config_4_raw(
        &mut self,
    ) -> Result<FastChargeConfig4Flags, Error<I2C::Error>> {
        let value = self.read_register(Register::FastChargingConfig4).await?;
        Ok(FastChargeConfig4Flags::from_bits_truncate(value))
    }

    /// Set broadcast current configuration register (REG 0xA8) raw flags.
    ///
    /// This method provides direct control over Type-C current broadcast.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `flags` - Broadcast current configuration flags to set
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_broadcast_current_raw(
        &mut self,
        flags: BroadcastCurrentFlags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::BroadcastCurrentConfig, flags.bits())
            .await
    }

    /// Get broadcast current configuration register (REG 0xA8) raw flags.
    ///
    /// # Returns
    ///
    /// Returns `Ok(BroadcastCurrentFlags)` on success, or an `Error` if the operation fails.
    pub async fn get_broadcast_current_raw(
        &mut self,
    ) -> Result<BroadcastCurrentFlags, Error<I2C::Error>> {
        let value = self.read_register(Register::BroadcastCurrentConfig).await?;
        Ok(BroadcastCurrentFlags::from_bits_truncate(value))
    }

    // ========================================================================
    // Protocol Configuration Methods - High-level API
    // ========================================================================

    /// Enable PD protocol.
    ///
    /// This method enables the PD protocol by setting the PD_ENABLE bit in REG 0xB3.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn enable_pd_protocol(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut flags = self.get_pd_config_0_raw().await?;
        // 有源低：清除此位即为“使能 PD”
        flags.remove(PdConfig0Flags::PD_DISABLE);
        self.set_pd_config_0_raw(flags).await
    }

    /// Disable PD protocol.
    ///
    /// This method disables the PD protocol by clearing the PD_ENABLE bit in REG 0xB3.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn disable_pd_protocol(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut flags = self.get_pd_config_0_raw().await?;
        // 有源低：置位即为“不使能 PD”
        flags.insert(PdConfig0Flags::PD_DISABLE);
        self.set_pd_config_0_raw(flags).await
    }

    /// Check if PD protocol is enabled.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if PD protocol is enabled, `Ok(false)` if disabled,
    /// or an `Error` if the operation fails.
    pub async fn is_pd_protocol_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let flags = self.get_pd_config_0_raw().await?;
        Ok(!flags.contains(PdConfig0Flags::PD_DISABLE))
    }

    /// Enable fast charging protocols.
    ///
    /// This method enables fast charging protocols by setting appropriate bits
    /// in the fast charging configuration registers.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn enable_fast_charge_protocol(&mut self) -> Result<(), Error<I2C::Error>> {
        // Enable fast charging in REG 0xB0（有源低：清除禁用位）
        let mut config2 = self.get_fast_charge_config_2_raw().await?;
        config2.remove(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        config2.remove(FastChargeConfig2Flags::QC2_DISABLE);
        config2.remove(FastChargeConfig2Flags::QC3_DISABLE);
        self.set_fast_charge_config_2_raw(config2).await
    }

    /// Disable fast charging protocols.
    ///
    /// This method disables fast charging protocols by clearing appropriate bits
    /// in the fast charging configuration registers.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn disable_fast_charge_protocol(&mut self) -> Result<(), Error<I2C::Error>> {
        // Disable fast charging in REG 0xB0（有源低：置禁用位）
        let mut config2 = self.get_fast_charge_config_2_raw().await?;
        config2.insert(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        config2.insert(FastChargeConfig2Flags::QC2_DISABLE);
        config2.insert(FastChargeConfig2Flags::QC3_DISABLE);
        self.set_fast_charge_config_2_raw(config2).await
    }

    /// Check if fast charging protocols are enabled.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if fast charging protocols are enabled, `Ok(false)` if disabled,
    /// or an `Error` if the operation fails.
    pub async fn is_fast_charge_protocol_enabled(&mut self) -> Result<bool, Error<I2C::Error>> {
        let config2 = self.get_fast_charge_config_2_raw().await?;
        Ok(!config2.contains(FastChargeConfig2Flags::FAST_CHARGE_DISABLE))
    }

    /// Configure multiple protocols at once.
    ///
    /// This method provides a convenient way to configure multiple protocols
    /// with a single call. It automatically handles the required register
    /// configurations for each protocol.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `config` - Protocol configuration specifying which protocols to enable
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_protocols(
        &mut self,
        config: ProtocolConfiguration,
    ) -> Result<(), Error<I2C::Error>> {
        // Configure PD protocol
        if config.pd_enabled {
            self.enable_pd_protocol().await?;
        } else {
            self.disable_pd_protocol().await?;
        }

        // Configure fast charging protocols
        let mut config2 = self.get_fast_charge_config_2_raw().await?;
        let mut config3 = self.get_fast_charge_config_3_raw().await?;

        // QC protocols
        // 有源低：清除禁用位=使能；置禁用位=禁用
        if config.qc20_enabled || config.qc30_enabled {
            config2.remove(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        }
        if config.qc20_enabled {
            config2.remove(FastChargeConfig2Flags::QC2_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::QC2_DISABLE);
        }
        if config.qc30_enabled {
            config2.remove(FastChargeConfig2Flags::QC3_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::QC3_DISABLE);
        }

        // SCP protocol
        if config.scp_enabled {
            config2.remove(FastChargeConfig2Flags::SCP_HV_DISABLE);
            config2.remove(FastChargeConfig2Flags::SCP_LV_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::SCP_HV_DISABLE);
            config2.insert(FastChargeConfig2Flags::SCP_LV_DISABLE);
        }

        // FCP/AFC/PE/SFCP 为“禁用位”：0=使能,1=不使能。
        // 使能 -> 清除 *_DISABLE；禁用 -> 置位 *_DISABLE。
        if config.fcp_enabled {
            config3.remove(FastChargeConfig3Flags::FCP_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::FCP_DISABLE);
        }

        if config.afc_enabled {
            config3.remove(FastChargeConfig3Flags::AFC_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::AFC_DISABLE);
        }

        if config.pe20_enabled {
            config3.remove(FastChargeConfig3Flags::PE_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::PE_DISABLE);
        }

        if config.sfcp_enabled {
            config3.remove(FastChargeConfig3Flags::SFCP_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::SFCP_DISABLE);
        }

        // BC1.2 protocol
        if config.bc12_enabled {
            config2.remove(FastChargeConfig2Flags::BC12_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::BC12_DISABLE);
        }

        // Apply all configurations
        self.set_fast_charge_config_2_raw(config2).await?;
        self.set_fast_charge_config_3_raw(config3).await?;

        Ok(())
    }

    /// Configure fast charging (QC/FCP/AFC/SCP/PE/SFCP/BC1.2) with structured settings.
    ///
    /// This method updates related fast-charge registers (0xAD/0xAE/0xB0/0xB1/0xB2) using a
    /// read‑modify‑write strategy to preserve reserved bits.
    ///
    /// Requires `unlock_write_enable_0()` to be called first.
    pub async fn configure_fast_charge(
        &mut self,
        config: FastChargeConfiguration,
    ) -> Result<(), Error<I2C::Error>> {
        if config.scp_current_limit > 2 {
            return Err(Error::InvalidParameter);
        }

        // REG 0xAD: current limit and PD/SCP interaction (other bits preserved)
        let mut config0 = self.get_fast_charge_config_0_raw().await?;
        if config.fcp_afc_sfcp_2_25a {
            config0.insert(FastChargeConfig0Flags::FCP_AFC_SFCP_2_25A);
        } else {
            config0.remove(FastChargeConfig0Flags::FCP_AFC_SFCP_2_25A);
        }
        self.set_fast_charge_config_0_raw(config0).await?;

        // REG 0xAE: voltage capability bits (other bits preserved)
        let mut config1 = self.get_fast_charge_config_1_raw().await?;
        if config.qc30_20v_enabled {
            config1.remove(FastChargeConfig1Flags::QC30_20V_DISABLE);
        } else {
            config1.insert(FastChargeConfig1Flags::QC30_20V_DISABLE);
        }
        if config.qc20_20v_enabled {
            config1.insert(FastChargeConfig1Flags::QC20_20V_ENABLE);
        } else {
            config1.remove(FastChargeConfig1Flags::QC20_20V_ENABLE);
        }
        if config.pe20_20v_enabled {
            config1.insert(FastChargeConfig1Flags::PE20_20V_ENABLE);
        } else {
            config1.remove(FastChargeConfig1Flags::PE20_20V_ENABLE);
        }
        if config.pd_12v_enabled {
            config1.remove(FastChargeConfig1Flags::NON_PD_12V_DISABLE);
        } else {
            config1.insert(FastChargeConfig1Flags::NON_PD_12V_DISABLE);
        }
        self.set_fast_charge_config_1_raw(config1).await?;

        // REG 0xB0: protocol enable/disable bits are active‑low (set = disable)
        let mut config2 = self.get_fast_charge_config_2_raw().await?;
        let any_fast = config.qc_enabled
            || config.fcp_enabled
            || config.afc_enabled
            || config.scp_enabled
            || config.pe20_enabled
            || config.sfcp_enabled;
        if any_fast {
            config2.remove(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        }

        if config.qc_enabled {
            config2.remove(FastChargeConfig2Flags::QC2_DISABLE);
            config2.remove(FastChargeConfig2Flags::QC3_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::QC2_DISABLE);
            config2.insert(FastChargeConfig2Flags::QC3_DISABLE);
        }

        if config.scp_enabled {
            config2.remove(FastChargeConfig2Flags::SCP_HV_DISABLE);
            config2.remove(FastChargeConfig2Flags::SCP_LV_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::SCP_HV_DISABLE);
            config2.insert(FastChargeConfig2Flags::SCP_LV_DISABLE);
        }

        if config.bc12_enabled {
            config2.remove(FastChargeConfig2Flags::BC12_DISABLE);
        } else {
            config2.insert(FastChargeConfig2Flags::BC12_DISABLE);
        }
        self.set_fast_charge_config_2_raw(config2).await?;

        // REG 0xB1: protocol disable bits (0=enable, 1=disable)
        let mut config3 = self.get_fast_charge_config_3_raw().await?;
        if config.fcp_enabled {
            config3.remove(FastChargeConfig3Flags::FCP_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::FCP_DISABLE);
        }
        if config.afc_enabled {
            config3.remove(FastChargeConfig3Flags::AFC_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::AFC_DISABLE);
        }
        if config.pe20_enabled {
            config3.remove(FastChargeConfig3Flags::PE_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::PE_DISABLE);
        }
        if config.sfcp_enabled {
            config3.remove(FastChargeConfig3Flags::SFCP_DISABLE);
        } else {
            config3.insert(FastChargeConfig3Flags::SFCP_DISABLE);
        }
        self.set_fast_charge_config_3_raw(config3).await?;

        // REG 0xB2: SCP broadcast current (bits 5-4)
        let mut config4 = self.get_fast_charge_config_4_raw().await?;
        let raw = (config4.bits() & !FastChargeConfig4Flags::SCP_CURRENT_MASK.bits())
            | ((config.scp_current_limit & 0x03) << 4);
        config4 = FastChargeConfig4Flags::from_bits_truncate(raw);
        self.set_fast_charge_config_4_raw(config4).await?;

        Ok(())
    }

    /// Get current fast charging configuration from registers (decoded).
    pub async fn get_fast_charge_status(
        &mut self,
    ) -> Result<FastChargeConfiguration, Error<I2C::Error>> {
        let config0 = self.get_fast_charge_config_0_raw().await?;
        let config1 = self.get_fast_charge_config_1_raw().await?;
        let config2 = self.get_fast_charge_config_2_raw().await?;
        let config3 = self.get_fast_charge_config_3_raw().await?;
        let config4 = self.get_fast_charge_config_4_raw().await?;

        let fast_ok = !config2.contains(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        let qc2_ok = fast_ok && !config2.contains(FastChargeConfig2Flags::QC2_DISABLE);
        let qc3_ok = fast_ok && !config2.contains(FastChargeConfig2Flags::QC3_DISABLE);

        Ok(FastChargeConfiguration {
            qc_enabled: qc2_ok || qc3_ok,
            fcp_enabled: !config3.contains(FastChargeConfig3Flags::FCP_DISABLE),
            afc_enabled: !config3.contains(FastChargeConfig3Flags::AFC_DISABLE),
            scp_enabled: !config2.contains(FastChargeConfig2Flags::SCP_HV_DISABLE)
                || !config2.contains(FastChargeConfig2Flags::SCP_LV_DISABLE),
            pe20_enabled: !config3.contains(FastChargeConfig3Flags::PE_DISABLE),
            sfcp_enabled: !config3.contains(FastChargeConfig3Flags::SFCP_DISABLE),
            bc12_enabled: !config2.contains(FastChargeConfig2Flags::BC12_DISABLE),
            scp_current_limit: ((config4.bits() & FastChargeConfig4Flags::SCP_CURRENT_MASK.bits())
                >> 4) as u8,
            fcp_afc_sfcp_2_25a: config0.contains(FastChargeConfig0Flags::FCP_AFC_SFCP_2_25A),
            qc20_20v_enabled: config1.contains(FastChargeConfig1Flags::QC20_20V_ENABLE),
            qc30_20v_enabled: !config1.contains(FastChargeConfig1Flags::QC30_20V_DISABLE),
            pe20_20v_enabled: config1.contains(FastChargeConfig1Flags::PE20_20V_ENABLE),
            pd_12v_enabled: !config1.contains(FastChargeConfig1Flags::NON_PD_12V_DISABLE),
        })
    }

    /// Configure PD protocol with detailed settings.
    ///
    /// This method provides fine-grained control over PD protocol configuration.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `config` - PD configuration specifying detailed PD settings
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_pd(&mut self, config: PdConfiguration) -> Result<(), Error<I2C::Error>> {
        // Configure PD Config 0 (REG 0xB3) 基于现值修改，避免破坏保留位
        let mut config0 = self.get_pd_config_0_raw().await?;
        if config.enabled {
            config0.remove(PdConfig0Flags::PD_DISABLE);
        } else {
            config0.insert(PdConfig0Flags::PD_DISABLE);
        }
        if config.vconn_swap {
            config0.insert(PdConfig0Flags::VCONN_SWAP);
        } else {
            config0.remove(PdConfig0Flags::VCONN_SWAP);
        }
        if config.dr_swap {
            config0.insert(PdConfig0Flags::DR_SWAP);
        } else {
            config0.remove(PdConfig0Flags::DR_SWAP);
        }
        if config.emarker_enabled {
            config0.remove(PdConfig0Flags::EMARKER_DETECT_DISABLE);
        } else {
            config0.insert(PdConfig0Flags::EMARKER_DETECT_DISABLE);
        }
        if config.emarker_60_70w {
            config0.insert(PdConfig0Flags::EMARKER_60_70W);
        } else {
            config0.remove(PdConfig0Flags::EMARKER_60_70W);
        }
        self.set_pd_config_0_raw(config0).await?;

        // Configure PD Config 1 (REG 0xB4) 基于现值修改
        let mut config1 = self.get_pd_config_1_raw().await?;
        if config.pps_enabled {
            config1.insert(PdConfig1Flags::PPS_REGISTER_CONFIG);
        } else {
            config1.remove(PdConfig1Flags::PPS_REGISTER_CONFIG);
        }
        if config.enabled {
            config1.insert(PdConfig1Flags::DISCOVERY_IDENTITY | PdConfig1Flags::DISCOVERY_SVID);
        }
        self.set_pd_config_1_raw(config1).await?;

        // Configure PD Config 2 (REG 0xB5) - 有源低禁用位，基于现值修改
        let mut config2 = self.get_pd_config_2_raw().await?;
        // Fixed voltages: true=使能 -> 清除禁用位；false=禁用 -> 置禁用位
        if config.fixed_voltages[0] {
            config2.remove(PdConfig2Flags::FIXED_9V_DISABLE);
        } else {
            config2.insert(PdConfig2Flags::FIXED_9V_DISABLE);
        }
        if config.fixed_voltages[1] {
            config2.remove(PdConfig2Flags::FIXED_12V_DISABLE);
        } else {
            config2.insert(PdConfig2Flags::FIXED_12V_DISABLE);
        }
        if config.fixed_voltages[2] {
            config2.remove(PdConfig2Flags::FIXED_15V_DISABLE);
        } else {
            config2.insert(PdConfig2Flags::FIXED_15V_DISABLE);
        }
        if config.fixed_voltages[3] {
            config2.remove(PdConfig2Flags::FIXED_20V_DISABLE);
        } else {
            config2.insert(PdConfig2Flags::FIXED_20V_DISABLE);
        }
        // PPS ranges
        if config.pps_enabled {
            config2.remove(
                PdConfig2Flags::PPS0_DISABLE
                    | PdConfig2Flags::PPS1_DISABLE
                    | PdConfig2Flags::PPS2_DISABLE
                    | PdConfig2Flags::PPS3_DISABLE,
            );
        } else {
            config2.insert(
                PdConfig2Flags::PPS0_DISABLE
                    | PdConfig2Flags::PPS1_DISABLE
                    | PdConfig2Flags::PPS2_DISABLE
                    | PdConfig2Flags::PPS3_DISABLE,
            );
        }
        self.set_pd_config_2_raw(config2).await?;

        // Configure PD Config 3 (REG 0xA6)
        let mut config3 = PdConfig3Flags::empty();
        if config.emark_5a_bypass {
            config3.insert(PdConfig3Flags::EMARK_5A_BYPASS);
        }
        self.set_pd_config_3_raw(config3).await?;

        Ok(())
    }

    /// Get current protocol status.
    ///
    /// This method reads the current protocol configuration from the device.
    ///
    /// # Returns
    ///
    /// Returns `Ok(ProtocolConfiguration)` with current protocol status,
    /// or an `Error` if the operation fails.
    pub async fn get_protocol_status(
        &mut self,
    ) -> Result<ProtocolConfiguration, Error<I2C::Error>> {
        let pd_config0 = self.get_pd_config_0_raw().await?;
        let fc_config2 = self.get_fast_charge_config_2_raw().await?;
        let fc_config3 = self.get_fast_charge_config_3_raw().await?;

        let fast_ok = !fc_config2.contains(FastChargeConfig2Flags::FAST_CHARGE_DISABLE);
        let qc_ok = fast_ok
            && (!fc_config2.contains(FastChargeConfig2Flags::QC2_DISABLE)
                || !fc_config2.contains(FastChargeConfig2Flags::QC3_DISABLE));

        Ok(ProtocolConfiguration {
            pd_enabled: !pd_config0.contains(PdConfig0Flags::PD_DISABLE),
            qc20_enabled: qc_ok,
            qc30_enabled: qc_ok,
            // 0=使能,1=不使能（取反）
            fcp_enabled: !fc_config3.contains(FastChargeConfig3Flags::FCP_DISABLE),
            afc_enabled: !fc_config3.contains(FastChargeConfig3Flags::AFC_DISABLE),
            scp_enabled: !fc_config2.contains(FastChargeConfig2Flags::SCP_HV_DISABLE)
                || !fc_config2.contains(FastChargeConfig2Flags::SCP_LV_DISABLE),
            pe20_enabled: !fc_config3.contains(FastChargeConfig3Flags::PE_DISABLE),
            bc12_enabled: !fc_config2.contains(FastChargeConfig2Flags::BC12_DISABLE),
            sfcp_enabled: !fc_config3.contains(FastChargeConfig3Flags::SFCP_DISABLE),
        })
    }

    /// Check if a specific protocol is enabled.
    ///
    /// # Arguments
    ///
    /// * `protocol` - The protocol type to check
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if the protocol is enabled, `Ok(false)` if disabled,
    /// or an `Error` if the operation fails.
    pub async fn is_protocol_enabled(
        &mut self,
        protocol: ProtocolType,
    ) -> Result<bool, Error<I2C::Error>> {
        match protocol {
            ProtocolType::PD => self.is_pd_protocol_enabled().await,
            ProtocolType::QC20 | ProtocolType::QC30 => {
                let config2 = self.get_fast_charge_config_2_raw().await?;
                Ok(
                    !config2.contains(FastChargeConfig2Flags::FAST_CHARGE_DISABLE)
                        && (!config2.contains(FastChargeConfig2Flags::QC2_DISABLE)
                            || !config2.contains(FastChargeConfig2Flags::QC3_DISABLE)),
                )
            }
            ProtocolType::FCP => {
                let config3 = self.get_fast_charge_config_3_raw().await?;
                Ok(!config3.contains(FastChargeConfig3Flags::FCP_DISABLE))
            }
            ProtocolType::AFC => {
                let config3 = self.get_fast_charge_config_3_raw().await?;
                Ok(!config3.contains(FastChargeConfig3Flags::AFC_DISABLE))
            }
            ProtocolType::SCP => {
                let config2 = self.get_fast_charge_config_2_raw().await?;
                Ok(!config2.contains(FastChargeConfig2Flags::SCP_HV_DISABLE)
                    || !config2.contains(FastChargeConfig2Flags::SCP_LV_DISABLE))
            }
            ProtocolType::PE20 => {
                let config3 = self.get_fast_charge_config_3_raw().await?;
                Ok(!config3.contains(FastChargeConfig3Flags::PE_DISABLE))
            }
            ProtocolType::BC12 => {
                let config2 = self.get_fast_charge_config_2_raw().await?;
                Ok(!config2.contains(FastChargeConfig2Flags::BC12_DISABLE))
            }
            ProtocolType::SFCP => {
                let config3 = self.get_fast_charge_config_3_raw().await?;
                Ok(!config3.contains(FastChargeConfig3Flags::SFCP_DISABLE))
            }
        }
    }

    /// Get the currently negotiated protocol from device status.
    ///
    /// This method reads the fast charging indicator register to determine
    /// which protocol is currently active.
    ///
    /// # Returns
    ///
    /// Returns `Ok(Some(ProtocolType))` if a protocol is active,
    /// `Ok(None)` if no protocol is active, or an `Error` if the operation fails.
    pub async fn get_negotiated_protocol(
        &mut self,
    ) -> Result<Option<ProtocolType>, Error<I2C::Error>> {
        let status = self.read_register(Register::FastChargingStatus).await?;

        // Extract protocol ID from bits 3-0
        let protocol_id = status & 0x0F;

        match protocol_id {
            constants::pd::PROTOCOL_QC20 => Ok(Some(ProtocolType::QC20)),
            constants::pd::PROTOCOL_QC30 => Ok(Some(ProtocolType::QC30)),
            constants::pd::PROTOCOL_FCP => Ok(Some(ProtocolType::FCP)),
            constants::pd::PROTOCOL_SCP => Ok(Some(ProtocolType::SCP)),
            constants::pd::PROTOCOL_PD_FIX => Ok(Some(ProtocolType::PD)),
            constants::pd::PROTOCOL_PD_PPS => Ok(Some(ProtocolType::PD)),
            constants::pd::PROTOCOL_PE20 => Ok(Some(ProtocolType::PE20)),
            constants::pd::PROTOCOL_SFCP => Ok(Some(ProtocolType::SFCP)),
            constants::pd::PROTOCOL_AFC => Ok(Some(ProtocolType::AFC)),
            _ => Ok(None), // No protocol or unknown protocol
        }
    }

    /// Configure Type-C current broadcast.
    ///
    /// This method configures the Type-C current broadcast capability.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Arguments
    ///
    /// * `config` - Type-C configuration
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_type_c(
        &mut self,
        config: TypeCConfiguration,
    ) -> Result<(), Error<I2C::Error>> {
        let mut flags = self.get_broadcast_current_raw().await?;

        if config.current_1_5a {
            flags.insert(BroadcastCurrentFlags::TYPE_C_1_5A);
        } else {
            flags.remove(BroadcastCurrentFlags::TYPE_C_1_5A);
        }

        if config.pd_pps_5a {
            flags.insert(BroadcastCurrentFlags::PD_PPS_5A);
        } else {
            flags.remove(BroadcastCurrentFlags::PD_PPS_5A);
        }

        self.set_broadcast_current_raw(flags).await?;

        // Configure CC line control if needed
        if config.cc_un_driving {
            // Trigger CC un-driving pulse
            let mut cc = self.get_connection_control_raw().await?;
            cc.insert(ConnectionControlFlags::CC_UN_DRIVING);
            self.set_connection_control_raw(cc).await?;
        }

        Ok(())
    }

    /// Get Type-C configuration status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(TypeCConfiguration)` with current Type-C settings,
    /// or an `Error` if the operation fails.
    pub async fn get_type_c_status(&mut self) -> Result<TypeCConfiguration, Error<I2C::Error>> {
        let flags = self.get_broadcast_current_raw().await?;

        Ok(TypeCConfiguration {
            current_1_5a: flags.contains(BroadcastCurrentFlags::TYPE_C_1_5A),
            pd_pps_5a: flags.contains(BroadcastCurrentFlags::PD_PPS_5A),
            cc_un_driving: false, // pulse-only
        })
    }

    // Type‑C 状态寄存器 (0x0A) 未在已校对文档中提供稳定位义，暂不暴露。

    // ========================================================================
    // Connection control (REG 0x14)
    // ========================================================================

    /// Get connection control raw flags.
    pub async fn get_connection_control_raw(
        &mut self,
    ) -> Result<ConnectionControlFlags, Error<I2C::Error>> {
        let v = self.read_register(Register::ConnectionControl).await?;
        Ok(ConnectionControlFlags::from_bits_truncate(v))
    }

    /// Set connection control raw flags (RMW recommended by caller).
    pub async fn set_connection_control_raw(
        &mut self,
        flags: ConnectionControlFlags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::ConnectionControl, flags.bits())
            .await
    }

    /// Enable or disable line compensation (0: enable/open, 1: close/disable).
    /// Requires unlock_write_enable_0() called beforehand.
    pub async fn set_line_compensation(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut cc = self.get_connection_control_raw().await?;
        if enable {
            cc.remove(ConnectionControlFlags::LINE_COMPENSATION_CLOSE);
        } else {
            cc.insert(ConnectionControlFlags::LINE_COMPENSATION_CLOSE);
        }
        self.set_connection_control_raw(cc).await
    }

    /// Trigger CC un-driving pulse; auto clears after ~1s.
    /// Requires unlock_write_enable_0() called beforehand.
    pub async fn trigger_cc_un_driving(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut cc = self.get_connection_control_raw().await?;
        cc.insert(ConnectionControlFlags::CC_UN_DRIVING);
        self.set_connection_control_raw(cc).await
    }

    /// Enable Type-C 1.5A current broadcast.
    ///
    /// This is a convenience method to quickly enable 1.5A current broadcast.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn enable_type_c_1_5a(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut flags = self.get_broadcast_current_raw().await?;
        flags.insert(BroadcastCurrentFlags::TYPE_C_1_5A);
        self.set_broadcast_current_raw(flags).await
    }

    /// Disable Type-C 1.5A current broadcast (use default current).
    ///
    /// This is a convenience method to quickly disable 1.5A current broadcast.
    /// Requires `unlock_write_enable_0()` to be called first.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn disable_type_c_1_5a(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut flags = self.get_broadcast_current_raw().await?;
        flags.remove(BroadcastCurrentFlags::TYPE_C_1_5A);
        self.set_broadcast_current_raw(flags).await
    }

    // Note: These registers are not available in official documentation
    // Commented out methods that use non-existent registers:
    // - set_interrupt_mask_0/1 (InterruptMask0/1 registers)
    // - read_timer_status (TimerStatus register)
    // - configure_watchdog (WatchdogConfig register)
    // - read_gpio_status (GpioStatus register)
    // - control_gpio (GpioControl register)
}
