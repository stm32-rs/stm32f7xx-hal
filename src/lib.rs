//! HAL for the STM32F7xx family of microcontrollers

#![no_std]
#![allow(non_camel_case_types)]

#[cfg(not(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f732",
    feature = "stm32f733",
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
    )))]
compile_error!("This crate requires one of the following device features enabled:
        stm32f722
        stm32f723
        stm32f732
        stm32f733
        stm32f745
        stm32f746
        stm32f756
        stm32f765
        stm32f767
        stm32f769
        stm32f777
        stm32f778
        stm32f779
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
pub mod serial;

#[cfg(feature = "stm32f746")]
pub mod time;
