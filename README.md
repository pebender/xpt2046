# XPT2046 touch LCD driver

[![Crates.io](https://img.shields.io/crates/d/xpt2046.svg)](https://crates.io/crates/xpt2046)
[![Crates.io](https://img.shields.io/crates/v/xpt2046.svg)](https://crates.io/crates/xpt2046)
[![Released API docs](https://docs.rs/xpt2046/badge.svg)](https://docs.rs/xpt2046)

Rust Embedded Hal based driver for xpt2046 touch screen driver

## Reasons for Forking [VersBinarii/xpt2046](https://github.com/VersBinarii/xpt2046)

I forked VersBinarii's xpt2046 driver because I wanted to make the following changes:

- [x] Update the [embedded-hal](https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal) to 1.x,
- [x] Add an [ESP32](https://github.com/esp-rs/esp-hal)+[embassy](https://github.com/embassy-rs/embassy) example,
- [x] Refactor the PENIRQ interrupt handler code, and
- [x] Replace the driver's use of [embedded-hal's SpiBus trait](https://docs.esp-rs.org/esp-idf-hal/embedded_hal/spi/trait.SpiBus.html) with the [embedded-hal's SpiDevice trait](https://docs.esp-rs.org/esp-idf-hal/embedded_hal/spi/trait.SpiDevice.html).

There are are incompatibilities between embedded-hal version 1.0.0-alpha.7 and 1.0.0. As a result, the existing xpt2046 driver is not compatible with HALs that implement final embedded-hal 1.0.0 release.

I have [ESP32](https://github.com/esp-rs/esp-hal) hardware and [embassy](https://github.com/embassy-rs/embassy) experience but I have neither [STM32](https://github.com/stm32-rs/stm32f4xx-hal) hardware nor [RTIC](https://github.com/rtic-rs/rtic) experience. Therefore, I cannot test the existing example without buying STM32 hardware and learning RTIC. Also, while I love playing new hardware and new software, the project in which I will be using the xpt2046 driver runs on an ESP32-C6 and uses embassy. So, an example the runs on an ESP32 and uses embassy makes the most sense right now. I will remove the STM32+RTIC when I can no longer get it to compile.

[As stated](https://github.com/VersBinarii/xpt2046/blob/v0.3.0/src/exti_pin.rs), VersBinarii created the Xpt2046Exti trait because embedded-hal does not expose a generic interface for working with GPIO interrupt handlers. As it turns out, the Xpt2046Exti trait is awkward to use with how ESP32 handles GPIO interrupts. For example, the esp-hal does not expose an external interrupt peripheral that must be safely shared, so there is nothing to pass to the Xpt2046Exti trait's interrupt control functions. In addition, when using the driver with an ESP32 in an async framework, the only time that an interrupt on the PENIRQ is really needed is when the PENIRQ is being uses to wake the processor from a sleep state.

THe [embedded-hal SPI documentation](https://docs.rs/embedded-hal/latest/embedded_hal/spi/index.html) states that the [SpiBus trait](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiBus.html) represents exclusive ownership of the SPI bus whereas the [SpiDevice trait](https://docs.rs/embedded-hal/latest/embedded_hal/spi/trait.SpiDevice.html) represents ownership of an SPI device selected by the CS (Chip Select) pin. Therefore, it is more correct to pass the the xpt2046 driver something that implements the SpiDevice trait rather than something that implements the SpiBut trait and the CS pin. In addition, making this change makes it easier to use with shared SPI bus drivers such as those provided by makes it easier to use with shared SPI bus drivers such as those provided by [embedded-hal-bus](https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/spi/index.html) and [embassy](https://docs.rs/embassy-embedded-hal/latest/embassy_embedded_hal/shared_bus/index.html).

## Documentation

There is no online documentation at the moment. However, you can generate by checking out the code and running the command

> cargo doc --open --no-deps

## License

Licensed under either of:

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
- MIT license ([LICENSE-MIT](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
