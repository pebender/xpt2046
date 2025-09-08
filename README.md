# XPT2046 Touch Screen Controller Driver

A driver for the **XPT2046 Touch Screen Controller** connected over **SPI** that is

- written in [Rust](https://www.rust-lang.org),
- compatible with [Rust Embedded's](https://github.com/rust-embedded) [embedded-hal 1.0.0](https://docs.rs/embedded-hal/1.0.0/embedded_hal/index.html),
- no_std and
- no_alloc.

## Compatibility

While I primarily develop on the [ESP32-C6](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/product-overview.html), the driver should work on any 32-bit MCU. While I primarily develop using [esp-hal](https://docs.rs/esp-hal/latest/esp_hal/), the driver should work with any embedded-hal 1.0.0 compatible HAL. While I primarily develop using [Embassy](https://embassy.dev), the driver should work with [RTIC](https://rtic.rs).

The XPT2046 is often paired with the ILI9341 TFT LCD Single Chip Driver to create 320x240 pixel TFT LCD display touch panels that connect over SPI and come in various sizes. For example, I have been developing using the [Hosyond's 3.2" version](https://www.amazon.com/dp/B0B1M9S9V6/).

## Reasons for Forking [VersBinarii/xpt2046](https://github.com/VersBinarii/xpt2046)

I forked VersBinarii's xpt2046 driver repository because as of the time I forked the repository, the driver

- used embedded-hal 1.0.0-alpha.7,
- used the [SpiBus trait](https://docs.esp-rs.org/esp-idf-hal/embedded_hal/spi/trait.SpiBus.html) rather than the [SpiDevice trait](https://docs.esp-rs.org/esp-idf-hal/embedded_hal/spi/trait.SpiDevice.html), and
- integrated interrupt handling for the PENIRQ pin.

There are incompatibilities between embedded-hal version 1.0.0-alpha.7 and 1.0.0. As a result, the driver was not compatible with HALs that implement the final embedded-hal 1.0.0 release, including esp-hal. Therefore, I wanted an updated version of the driver that complies with embedded-hal 1.0.0.

As stated in the [embedded-hal SPI documentation](https://docs.rs/embedded-hal/latest/embedded_hal/spi/index.html), the [SpiBus trait](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiBus.html) represents exclusive ownership of the SPI bus whereas the [SpiDevice trait](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiDevice.html) represents ownership of an SPI device selected by the CS (Chip Select) pin. SPI bus sharing software provided by [embedded-hal-bus](https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/spi/index.html) and [embassy-embedded-hal](https://docs.rs/embassy-embedded-hal/latest/embassy_embedded_hal/shared_bus/index.html) follow this, and using this SPI bus sharing software simplifies async programming. Therefore, I wanted an updated version of the driver that expects an SpiDevice trait rather than an SpiBus trait.

[As stated](https://github.com/VersBinarii/xpt2046/blob/v0.3.0/src/exti_pin.rs), VersBinarii created a driver-specific trait for use in integrating PENIRQ interrupt handling into the driver because embedded-hal 1.0.0 does not expose a generic interface for working with GPIO interrupt handlers. As it turns out, this driver-specific trait was awkward to use with how the esp-hal handles GPIO interrupts. In addition, when using the driver with the esp-hal in an async framework, the only time an interrupt handler for the PENIRQ might be needed is when the PENIRQ pin is being used to wake the processor from a sleep state.

Since making the changes for which I originally forked the driver, I have made additional changes. The biggest change is the separation of the driver and the driver calibration. I did this because I felt is was cleaner and because I wanted better separation so that I could run the touch driver and the display driver in separate async tasks that communicate using channels.

One final note. I have been doing my development using [ESP32*](https://github.com/esp-rs/esp-hal) hardware and have [embassy](https://github.com/embassy-rs/embassy) experience but I have neither [STM32*](https://github.com/stm32-rs/stm32f4xx-hal) hardware nor [RTIC](https://github.com/rtic-rs/rtic) experience. Therefore, I cannot test the existing example without buying STM32F411 hardware and learning RTIC. Also, while I love playing new hardware and new software, the project in which I will be using the xpt2046 driver runs on an ESP32-C6 and uses embassy. So, examples that run on an ESP32-C6 and use embassy make the most sense right now. As a result, I will likely remove the STM32F411+RTIC example when I can no longer get it to compile.

## Documentation

There is no online documentation at this time. However, you can generate documentation by checking out the code and running the command

> cargo doc --open

## License

Licensed under either of:

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
- MIT license ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
