# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

<!-- next-header -->
## [Unreleased] - ReleaseDate

### Fixed

- Use the right addresses for `signature` module structs on f7x2, f7x3, and f730 devices

## [0.8.0] - 2024-08-02

### Changed

- Updated `stm32f7` dependency 0.14.0 -> 0.15
  - Use PascalCase for generated values of enums ([stm32-rs#727](https://github.com/stm32-rs/stm32-rs/pull/727))
- Updated `synopsys-usb-otg` dependency 0.2.3 -> 0.4.0
- Updated `stm32-fmc` dependency 0.2.0 -> 0.3
- Added Interruptable trait to Alternate mode pins
- Added a "low pin count" variant of the f730 chip to the crate features: packages <144 pins don't include a high speed USB PHY
- Added SPI2_SCK pin for stm32f769i-discovery
- Fix mass-erase triggering in `flash` on smaller chips
- Remove the need for software u64 division in the clock setup code, shrinking code (#211)
- Updated `cortex-m` dependency 0.7.4 -> 0.7.7
- Updated `nb` dependency 1.0 -> 1.1.0
- Updated `micromath` dependency 2.0 -> 2.1.0
- Updated `fugit` dependency 0.3.5 -> 0.3.7
- Updated `bitflags` dependency 1.3.2 -> 2.6.0
- Updated `embedded-hal` dependency 0.2.3 -> 0.2.7
- Updated `display-interface` dependency 0.4.1 -> 0.5.0
- Updated `cortex-m-semihosting` development dependency 0.3.3 -> 0.5.0
  - Removed unwrap from the end of hprintln / hprint for new version of semihosting
- Updated `panic-semihosting` development dependency 0.5.2 -> 0.6.0
- Updated `embedded-graphics` development dependency 0.6.1 -> 0.6.2
- Updated `usb-device` development dependency 0.2.5 -> 0.3.2
- Updated `usbd-serial` development dependency 0.1.0 -> 0.2.2
  - Updated usb serial example to use new api
- Renamed .cargo/config -> .cargo/config.toml

## [v0.7.0] - 2022-06-05

### Added

- Support for `flash` on bigger chips (#168)
- Implement `IndependentWatchdog` for the IWDG peripheral (#180)
- Implement `embedded-hal` 0.2 features for `gpio` and `timer` (#176)
- Support for different number of data and parity bits for UART (#181/#182)
- Support for PWM, counter, monotonic on timers (#179)
- Examples:
  - [Basic use of the PWM](examples/pwm.rs) (#179)
  - [Basic use of the RTC](examples/pwm-sinus.rs) (#159)
  - [Blinking a LED using a delay from a timer source](examples/delay-timer-blinky.rs) (#179)
  - [Blinking a LED from within a timer interrupt](examples/blinky-timer-irq.rs) (#179)
  - [Blinking a LED from RTIC using a timer as monotonic source](examples/rtic-tick.rs) (#179)
  - [Scanning I2C devices](examples/i2c_scanner.rs) (#155)
  - [Generating a sine wave using PWM](examples/pwm-sinus.rs) (#179)
  - [Using bit parity on UART](examples/serial_parity.rs) (#182)
  - [Using a timer using `nb::block!()`](examples/timer-periph.rs) (#179)
  - [Using the system timer using `nb::block!()`](examples/timer-syst.rs) (#179)

### Changed

- Renamed `master` branch to `main` [Updating a local clone after a branch name changes](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-branches-in-your-repository/renaming-a-branch#updating-a-local-clone-after-a-branch-name-changes)
- Split `GetBusFreq` on `BusClock` & `BusTimerClock`, use `&Clock` everywhere (#178)
- Use `fugit`-based time types instead of `embedded-time` (#177)
- Update gpios: add `DynamicPin`, add default modes, reexport pins, resort generics, etc (#176)
- Improved RCC infrastructure (#152)
- RTC support has been rewritten (#159/#160/162)
- Bump `bxcan` dependency version (#158)
- Removed `rustfmt` checks on CI (#184)

### Fixed

- Fix RAM address and add ITCM and DTCM sections (#156)
- Fix default mode for debug pins (#166)
- Use `BitsPerSeconds` instead of `BytesPerSecond` in the serial baud rate configuration (#175)

## [v0.6.0] - 2021-11-02

## [v0.5.0] - 2021-09-22

## [v0.4.0] - 2021-07-16

## [v0.3.0] - 2021-04-26

## [v0.2.0] - 2020-07-01

## [v0.1.0] - 2019-11-05


<!-- next-url -->
[Unreleased]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.8.0...HEAD
[0.8.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.7.0...v0.8.0
[v0.7.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.6.0...v0.7.0
[v0.6.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.2.0...v0.3.0
[v0.2.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.1.0...v0.2.0
