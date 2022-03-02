# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- Support for flash.rs on bigger chips.

### Changed

- Update gpios: add `DynamicPin`, add default modes, reexport pins, resort generics, etc.
- Improved RCC infrastructure.
- RTC support has been rewritten.
- Bump `bxcan` dependency version.

### Fixed

- Fix RAM address and add ITCM and DTCM sections.
- Fix default mode for debug pins.
- Use `BitsPerSeconds` instead of `BytesPerSecond` in the serial baud rate configuration.

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
