# SW2303 USB PD Controller Driver

A Rust driver for the SW2303 USB PD (Power Delivery) controller chip, designed for embedded systems.

## Features

- **No-std compatible**: Works in embedded environments without standard library
- **Async support**: Optional async/await support with `async` feature
- **Defmt logging**: Optional structured logging with `defmt` feature
- **Type-safe register access**: Strongly typed register definitions
- **Device detection**: Sink device connection monitoring
- **Error handling**: Comprehensive error types for robust applications

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
sw2303 = { path = "../sw2303", features = ["async", "defmt"] }
```

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

    // Check if a sink device is connected
    if sw2303.is_sink_device_connected()? {
        // Sink device connected
    }
    Ok(())
}
```

### Async Usage

```rust
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

The driver provides access to key SW2303 registers:

- Device ID and identification registers
- Charging configuration
- Device status and control
- Power management
- Current and voltage monitoring

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

This is an initial implementation. Future updates will include:

- Complete register mapping
- Advanced charging management features
- Interrupt handling
- Power delivery optimization
- Comprehensive testing
- Example applications

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
