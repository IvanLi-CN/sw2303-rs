//! SW2303 driver implementation using maybe-async-cfg
//!
//! SW2303 is a USB PD (Power Delivery) charging controller, not a USB hub controller.
//! This driver provides methods to interact with the SW2303 for UFP detection and charging management.

use crate::error::Error;
use crate::registers::{
    FastChargingFlags, GpioConfigFlags, InterruptStatus0Flags, InterruptStatus1Flags,
    PdConfigFlags, PdStatusFlags, Register, ResetControlFlags, SystemStatus0Flags,
    SystemStatus1Flags, SystemStatus2Flags, SystemStatus3Flags, SystemStatus4Flags,
    SystemStatus5Flags, TypeCConfigFlags, constants,
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
        defmt::info!(
            "SW2303 chip version: 0x{:02X} (version bits: {})",
            chip_version,
            version_bits
        );

        // Check if chip version matches expected value (default 0x1)
        if version_bits != constants::CHIP_VERSION {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "Unexpected chip version: expected {}, got {}",
                constants::CHIP_VERSION,
                version_bits
            );
            // Don't fail initialization, just warn - hardware might be different variant
        }

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
        self.write_register(Register::I2cWriteEnable0, 0x20).await?;
        self.write_register(Register::I2cWriteEnable0, 0x40).await?;
        self.write_register(Register::I2cWriteEnable0, 0x80).await?;

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

    /// Set the current limit.
    ///
    /// # Arguments
    ///
    /// * `current_ma` - The current limit to set in milliamps
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_current_limit(&mut self, current_ma: u16) -> Result<(), Error<I2C::Error>> {
        // Convert current from mA to register value (assuming 50mA per unit)
        let current_units = (current_ma / 50) as u8;
        self.write_register(Register::CurrentLimit, current_units)
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

    /// Get the current limit setting.
    ///
    /// # Returns
    ///
    /// Returns the current limit in milliamps, or an `Error` if the operation fails.
    pub async fn get_current_limit(&mut self) -> Result<u16, Error<I2C::Error>> {
        let current_units = self.read_register(Register::CurrentLimit).await?;
        // Use saturating multiplication to prevent overflow
        Ok((current_units as u16).saturating_mul(50)) // Convert back to mA
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

    /// Read PD status flags from REG 0x09.
    ///
    /// # Returns
    ///
    /// Returns `Ok(PdStatusFlags)` with the current PD status flags.
    pub async fn get_pd_status(&mut self) -> Result<PdStatusFlags, Error<I2C::Error>> {
        let status = self.read_register(Register::PdStatus).await?;
        Ok(PdStatusFlags::from_bits_truncate(status))
    }



    /// Read system status 1 flags from REG 0x08.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus1Flags)` with the current system status 1 flags.
    pub async fn get_system_status_1(&mut self) -> Result<SystemStatus1Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus1).await?;
        Ok(SystemStatus1Flags::from_bits_truncate(status))
    }

    /// Check if PD contract is established.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if PD contract is established, `Ok(false)` otherwise.
    pub async fn is_pd_contract_established(&mut self) -> Result<bool, Error<I2C::Error>> {
        let pd_status = self.get_pd_status().await?;
        let contract_established = pd_status.contains(PdStatusFlags::CONTRACT_ESTABLISHED);
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 PD status: 0x{:02X}, CONTRACT_ESTABLISHED: {}", pd_status.bits(), contract_established);
        Ok(contract_established)
    }

    /// Read Request Data Object (RDO) from REG 0x25-0x28.
    ///
    /// This method reads the 4-byte Request Data Object that contains the actual
    /// negotiated power parameters from the PD protocol.
    ///
    /// # Returns
    ///
    /// Returns `Ok([u8; 4])` with the 4-byte RDO data, or an `Error` if the operation fails.
    /// The bytes are returned in order: [RDO0, RDO1, RDO2, RDO3]
    pub async fn read_request_data_object(&mut self) -> Result<[u8; 4], Error<I2C::Error>> {
        let rdo0 = self.read_register(Register::RequestDo0).await?;
        let rdo1 = self.read_register(Register::RequestDo1).await?;
        let rdo2 = self.read_register(Register::RequestDo2).await?;
        let rdo3 = self.read_register(Register::RequestDo3).await?;

        Ok([rdo0, rdo1, rdo2, rdo3])
    }

    /// Get the actual negotiated power from the Request Data Object.
    ///
    /// This method reads the RDO and extracts the negotiated power value.
    /// It only returns a valid power value if a PD contract is established.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u16)` with the negotiated power in watts, or 0 if no PD contract.
    /// Returns an `Error` if the operation fails.
    pub async fn get_negotiated_power(&mut self) -> Result<u16, Error<I2C::Error>> {
        // First check if PD contract is established
        let contract_established = self.is_pd_contract_established().await?;
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 PD contract established: {}", contract_established);

        if !contract_established {
            #[cfg(feature = "defmt")]
            defmt::info!("SW2303 no PD contract, returning 0W");
            return Ok(0); // No PD contract, return 0W
        }

        // Read the Request Data Object
        let rdo = self.read_request_data_object().await?;
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 RDO data: [{:02X}, {:02X}, {:02X}, {:02X}]", rdo[0], rdo[1], rdo[2], rdo[3]);

        // Parse RDO according to USB PD specification
        // RDO format (32-bit): [RDO3][RDO2][RDO1][RDO0]
        // Operating current: bits 19-10 (10 bits) in 10mA units
        // Maximum operating current: bits 9-0 (10 bits) in 10mA units
        let rdo_u32 = ((rdo[3] as u32) << 24) | ((rdo[2] as u32) << 16) | ((rdo[1] as u32) << 8) | (rdo[0] as u32);
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 RDO as u32: 0x{:08X}", rdo_u32);

        // Extract operating current (bits 19-10) in 10mA units
        let operating_current_10ma = (rdo_u32 >> 10) & 0x3FF;
        let operating_current_ma = operating_current_10ma * 10;
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 operating current: {}mA ({}x10mA)", operating_current_ma, operating_current_10ma);

        // Get the negotiated voltage from voltage registers
        let voltage_mv = self.get_voltage().await?;
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 voltage: {}mV", voltage_mv);

        // Calculate power: P = V * I (convert to watts)
        let power_mw = (voltage_mv as u32 * operating_current_ma) / 1000;
        let power_w = (power_mw / 1000) as u16;
        #[cfg(feature = "defmt")]
        defmt::info!("SW2303 calculated power: {}mW -> {}W", power_mw, power_w);

        Ok(power_w)
    }



    /// Check if charging is active.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if charging is active, `Ok(false)` otherwise.
    pub async fn is_charging(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status1 = self.get_system_status_1().await?;
        Ok(status1.contains(SystemStatus1Flags::CHARGING))
    }

    /// Check for overcurrent protection status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if overcurrent protection is triggered, `Ok(false)` otherwise.
    pub async fn is_overcurrent(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status1 = self.get_system_status_1().await?;
        Ok(status1.contains(SystemStatus1Flags::OVERCURRENT))
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

    /// Read system status 4 flags from REG 0x0B.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus4Flags)` with the current system status 4 flags.
    pub async fn get_system_status_4(&mut self) -> Result<SystemStatus4Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus4).await?;
        Ok(SystemStatus4Flags::from_bits_truncate(status))
    }

    /// Read system status 5 flags from REG 0x0C.
    ///
    /// # Returns
    ///
    /// Returns `Ok(SystemStatus5Flags)` with the current system status 5 flags.
    pub async fn get_system_status_5(&mut self) -> Result<SystemStatus5Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::SystemStatus5).await?;
        Ok(SystemStatus5Flags::from_bits_truncate(status))
    }

    /// Read interrupt status 0 flags from REG 0x0E.
    ///
    /// # Returns
    ///
    /// Returns `Ok(InterruptStatus0Flags)` with the current interrupt status 0 flags.
    pub async fn get_interrupt_status_0(
        &mut self,
    ) -> Result<InterruptStatus0Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::InterruptStatus0).await?;
        Ok(InterruptStatus0Flags::from_bits_truncate(status))
    }

    /// Read interrupt status 1 flags from REG 0x0F.
    ///
    /// # Returns
    ///
    /// Returns `Ok(InterruptStatus1Flags)` with the current interrupt status 1 flags.
    pub async fn get_interrupt_status_1(
        &mut self,
    ) -> Result<InterruptStatus1Flags, Error<I2C::Error>> {
        let status = self.read_register(Register::InterruptStatus1).await?;
        Ok(InterruptStatus1Flags::from_bits_truncate(status))
    }

    /// Check if PD negotiation is complete.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if PD negotiation is complete, `Ok(false)` otherwise.
    pub async fn is_pd_negotiation_complete(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status2 = self.get_system_status_2().await?;
        Ok(status2.contains(SystemStatus2Flags::PD_NEGOTIATION_COMPLETE))
    }

    /// Check if system is ready.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if system is ready, `Ok(false)` otherwise.
    pub async fn is_system_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status5 = self.get_system_status_5().await?;
        Ok(status5.contains(SystemStatus5Flags::SYSTEM_READY))
    }

    /// Check for temperature warning.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if temperature warning is active, `Ok(false)` otherwise.
    pub async fn is_temperature_warning(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status4 = self.get_system_status_4().await?;
        Ok(status4.contains(SystemStatus4Flags::TEMPERATURE_WARNING))
    }

    /// Check for any protection status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(true)` if any protection is triggered, `Ok(false)` otherwise.
    pub async fn is_protection_active(&mut self) -> Result<bool, Error<I2C::Error>> {
        let status4 = self.get_system_status_4().await?;
        Ok(status4.contains(SystemStatus4Flags::INPUT_OVERVOLTAGE)
            || status4.contains(SystemStatus4Flags::INPUT_UNDERVOLTAGE)
            || status4.contains(SystemStatus4Flags::OUTPUT_OVERVOLTAGE)
            || status4.contains(SystemStatus4Flags::OUTPUT_UNDERVOLTAGE)
            || status4.contains(SystemStatus4Flags::SHORT_CIRCUIT)
            || status4.contains(SystemStatus4Flags::REVERSE_CURRENT)
            || status4.contains(SystemStatus4Flags::TEMPERATURE_SHUTDOWN))
    }

    /// Perform system reset.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn system_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(
            Register::ResetControl,
            ResetControlFlags::SYSTEM_RESET.bits(),
        )
        .await?;
        Ok(())
    }

    /// Perform soft reset.
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::ResetControl, ResetControlFlags::SOFT_RESET.bits())
            .await?;
        Ok(())
    }

    /// Configure PD protocol settings.
    ///
    /// # Arguments
    ///
    /// * `config` - PD configuration flags
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_pd(&mut self, config: PdConfigFlags) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::PdConfig, config.bits())
            .await?;
        Ok(())
    }

    /// Configure Type-C protocol settings.
    ///
    /// # Arguments
    ///
    /// * `config` - Type-C configuration flags
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_type_c(
        &mut self,
        config: TypeCConfigFlags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::TypeCConfig, config.bits())
            .await?;
        Ok(())
    }

    /// Configure GPIO settings.
    ///
    /// # Arguments
    ///
    /// * `config` - GPIO configuration flags
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_gpio(
        &mut self,
        config: GpioConfigFlags,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::GpioConfig, config.bits())
            .await?;
        Ok(())
    }

    /// Read ADC Vout (output voltage) data.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the ADC Vout reading.
    pub async fn read_adc_vout(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcVout).await
    }

    /// Read ADC Iout (output current) data.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the ADC Iout reading.
    pub async fn read_adc_iout(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcIout).await
    }

    /// Read ADC Tntc (NTC temperature) data.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the ADC Tntc reading.
    pub async fn read_adc_tntc(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::AdcTntc).await
    }

    /// Read device ID from REG 0x00.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the device ID.
    pub async fn read_device_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::DeviceId).await
    }

    /// Configure interrupt mask 0.
    ///
    /// # Arguments
    ///
    /// * `mask` - Interrupt mask flags
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_interrupt_mask_0(&mut self, mask: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::InterruptMask0, mask).await?;
        Ok(())
    }

    /// Configure interrupt mask 1.
    ///
    /// # Arguments
    ///
    /// * `mask` - Interrupt mask flags
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn set_interrupt_mask_1(&mut self, mask: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::InterruptMask1, mask).await?;
        Ok(())
    }

    /// Read timer status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the timer status.
    pub async fn read_timer_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::TimerStatus).await
    }

    /// Configure watchdog settings.
    ///
    /// # Arguments
    ///
    /// * `config` - Watchdog configuration value
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn configure_watchdog(&mut self, config: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::WatchdogConfig, config)
            .await?;
        Ok(())
    }

    /// Read GPIO status.
    ///
    /// # Returns
    ///
    /// Returns `Ok(u8)` with the GPIO status.
    pub async fn read_gpio_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Register::GpioStatus).await
    }

    /// Control GPIO outputs.
    ///
    /// # Arguments
    ///
    /// * `control` - GPIO control value
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or an `Error` if the operation fails.
    pub async fn control_gpio(&mut self, control: u8) -> Result<(), Error<I2C::Error>> {
        self.write_register(Register::GpioControl, control).await?;
        Ok(())
    }
}
