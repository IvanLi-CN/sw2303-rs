# SW2303 USB PD Controller Driver

A Rust driver for the SW2303 USB PD (Power Delivery) controller chip, designed for embedded systems.

## Features

- **No-std compatible**: Works in embedded environments without standard library
- **Async support**: Optional async/await support with `async` feature
- **Defmt logging**: Optional structured logging with `defmt` feature
- **Type-safe register access**: Strongly typed register definitions
- **Device detection**: Sink device connection monitoring
- **Error handling**: Comprehensive error types for robust applications

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
sw2303 = "0.1.0"

# For async support
sw2303 = { version = "0.1.0", features = ["async"] }

# For defmt logging support
sw2303 = { version = "0.1.0", features = ["defmt"] }

# For both async and defmt
sw2303 = { version = "0.1.0", features = ["async", "defmt"] }
```

## Usage

### Basic Usage

```rust
use sw2303::{SW2303, registers::constants::DEFAULT_ADDRESS};
use embedded_hal::i2c::I2c;

fn example<I2C: I2c>(mut i2c: I2C) -> Result<(), sw2303::error::Error<I2C::Error>>
where I2C::Error: core::fmt::Debug
{
    // Initialize the driver with an I2C interface
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize the controller
    sw2303.init()?;

    // Configure power (requires unlock for REG 0xAF)
    sw2303.unlock_write_enable_0()?;
    sw2303.set_power_config(65)?; // Set to 65W

    // Check if a sink device is connected
    if sw2303.is_sink_device_connected()? {
        // Sink device connected
    }

    // Check PD contract status
    if sw2303.is_pd_contract_established()? {
        // PD contract established
    }

    // Check Type-C connection
    if sw2303.is_type_c_connected()? {
        // Type-C device connected
    }

    // Check charging status
    if sw2303.is_charging()? {
        // Device is charging
    }

    // Check system status
    if sw2303.is_system_ready()? {
        // System is ready for operation
    }

    // Check for protection status
    if sw2303.is_protection_active()? {
        // Some protection is triggered
    }

    // Check temperature warning
    if sw2303.is_temperature_warning()? {
        // Temperature warning active
    }

    // Configure PD protocol
    use sw2303::registers::{PdConfigFlags, TypeCConfigFlags};
    let pd_config = PdConfigFlags::PD_ENABLE | PdConfigFlags::SINK_MODE;
    sw2303.configure_pd(pd_config)?;

    // Configure Type-C protocol
    let type_c_config = TypeCConfigFlags::TYPE_C_ENABLE | TypeCConfigFlags::CC_DEBOUNCE_ENABLE;
    sw2303.configure_type_c(type_c_config)?;
    Ok(())
}
```

### Async Usage

```rust
# #[cfg(feature = "async")]
# {
use sw2303::{SW2303, registers::constants::DEFAULT_ADDRESS};
use embedded_hal_async::i2c::I2c;

// With async feature enabled
async fn async_example<I2C: I2c>(mut i2c: I2C) -> Result<(), sw2303::error::Error<I2C::Error>>
where I2C::Error: core::fmt::Debug
{
    let mut sw2303 = SW2303::new(&mut i2c, DEFAULT_ADDRESS);

    // Initialize asynchronously
    sw2303.init().await?;

    // Check sink device connection
    if sw2303.is_sink_device_connected().await? {
        // Handle connected device
    }

    Ok(())
}
# }
```

## Feature Flags

- `async`: Enables async/await support for non-blocking operations
- `defmt`: Enables structured logging with defmt

## Hardware Support

This driver is designed for the SW2303 USB PD controller. The SW2303 is a USB PD controller that supports:

- USB PD charging management
- Sink device detection
- Power delivery control
- Status monitoring
- I2C configuration interface

## Register Map

The driver provides comprehensive access to all SW2303 registers:

### Status Registers

- **Device identification**: Device ID (REG 0x00), Chip version (REG 0x01)
- **System status**: Status registers 0-5 (REG 0x02, 0x07-0x0D) for comprehensive system monitoring
- **PD status**: PD protocol status (REG 0x09) including contract establishment and capabilities
- **Type-C status**: Type-C connection status (REG 0x0A) including CC line detection
- **Interrupt status**: Interrupt flags (REG 0x0E-0x0F) for event-driven operation

### Configuration Registers

- **Power configuration**: Power delivery settings (REG 0xAF) with register-based power control
- **PD configuration**: PD protocol settings (REG 0x17) for source/sink/DRP modes
- **Type-C configuration**: Type-C protocol settings (REG 0x18) for CC detection and VCONN control
- **System configuration**: Extended configuration registers (REG 0xA0-0xBF) for advanced features

### Control Registers

- **Reset control**: System and module reset (REG 0x13)
- **Connection control**: Connection management (REG 0x14)
- **Force control**: Manual control override (REG 0x16)
- **GPIO control**: GPIO configuration and control (REG 0x2C-0x2E)
- **Timer control**: Timer and watchdog configuration (REG 0x29-0x2B)

### Monitoring Registers

- **ADC readings**: Comprehensive voltage, current, and temperature monitoring (REG 0x30-0x3F)
- **Charging status**: Fast charging indication (REG 0x06)
- **Protection status**: Overvoltage, overcurrent, and thermal protection monitoring

### Security and Unlock

- **I2C write enable**: Register unlock controls (REG 0x12, 0x15) for secure configuration
- **Calibration**: ADC and system calibration registers (REG 0x3E-0x3F, 0xB4-0xB5)

## Error Handling

The driver provides comprehensive error handling through the `Error` enum:

- `Communication`: I2C communication errors
- `InvalidRegister`: Invalid register access
- `InvalidParameter`: Invalid function parameters
- `DeviceNotResponding`: Device not found or not responding
- `Timeout`: Operation timeout
- `PortConfigError`: Port configuration issues
- `InitializationFailed`: Hub initialization problems

## Development Status

This implementation provides comprehensive SW2303 register coverage:

### ✅ Completed Features

- **Complete register mapping**: All SW2303 registers (0x00-0xBF) implemented
- **Comprehensive status monitoring**: System, PD, Type-C, and interrupt status
- **Power configuration**: Register-based power control (REG 0xAF)
- **Protocol configuration**: PD and Type-C protocol settings
- **Protection monitoring**: Overvoltage, overcurrent, thermal protection
- **ADC monitoring**: Voltage, current, temperature readings
- **Control functions**: Reset, GPIO, timer, and connection control
- **Security features**: Register unlock mechanisms

### 🚧 Future Enhancements

- **Interrupt handling**: Event-driven operation with interrupt callbacks
- **Advanced PD features**: Source capabilities negotiation, PPS support
- **Calibration utilities**: ADC and system calibration helpers
- **Power optimization**: Dynamic power management algorithms
- **Comprehensive testing**: Hardware-in-the-loop testing
- **Example applications**: Real-world usage examples

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
