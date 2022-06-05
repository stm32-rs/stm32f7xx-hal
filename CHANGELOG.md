# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

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


[Unreleased]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.6.0...HEAD
[v0.6.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.2.0...v0.3.0
[v0.2.0]: https://github.com/stm32-rs/stm32f7xx-hal/compare/v0.1.0...v0.2.0
