# `stm32f7xx-hal`

> [HAL] for the STM32F7 family of microcontrollers

[HAL]: https://crates.io/crates/embedded-hal

[![Crates.io - stm32f7xx-hal](https://img.shields.io/crates/v/stm32f7xx-hal.svg?maxAge=2592000)](https://crates.io/crates/stm32f7xx-hal)
[![Released API docs](https://docs.rs/stm32f7xx-hal/badge.svg)](https://docs.rs/stm32f7xx-hal)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/stm32-rs/stm32f7xx-hal/workflows/Continuous%20integration/badge.svg?branch=master)](https://github.com/stm32-rs/stm32f7xx-hal/actions)

This crate is largely inspired by the awesome work done here:

- [stm32f1xx-hal](https://github.com/stm32-rs/stm32f1xx-hal)
- [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal)

## Selecting a microcontroller

This crate supports multiple microcontrollers in the
stm32f7 family. Which specific microcontroller you want to build for has to be
specified with a feature, for example `stm32f767`. 

If no microcontroller is specified, the crate will not compile.

### Supported Microcontrollers

* `stm32f722`
* `stm32f723`
* `stm32f730`
* `stm32f732`
* `stm32f733`
* `stm32f745`
* `stm32f746`
* `stm32f756`
* `stm32f765`
* `stm32f767`
* `stm32f769`
* `stm32f777`
* `stm32f778`
* `stm32f779`

## Using as a Dependency

When using this crate as a dependency in your project, the microcontroller can 
be specified as part of the `Cargo.toml` definition.

```toml
[dependencies.stm32f7xx-hal]
version = "0.7.0"
features = ["stm32f767", "rt"]
```

## Documentation

The documentation can be found at [docs.rs/stm32f7xx-hal](https://docs.rs/stm32f7xx-hal/).

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
