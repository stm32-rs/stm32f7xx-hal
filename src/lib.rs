//! HAL for the STM32F7xx family of microcontrollers

#![no_std]
#![allow(non_camel_case_types)]

#[cfg(not(any(
    feature = "stm32f746",
    )))]
compile_error!("This crate requires one of the following device features enabled:
        stm32f746
                ");

use embedded_hal as hal;

#[cfg(feature = "stm32f746")]
pub use stm32f7::stm32f7x6 as device;

// Enable use of interrupt macro
#[cfg(feature = "rt")]
pub use crate::device::interrupt;

#[cfg(feature = "stm32f746")]
pub mod delay;

#[cfg(feature = "doc")]
pub mod examples;

#[cfg(feature = "stm32f746")]
pub mod gpio;

#[cfg(feature = "stm32f746")]
pub mod prelude;

#[cfg(feature = "stm32f746")]
pub mod rcc;

#[cfg(feature = "stm32f746")]
pub mod time;
