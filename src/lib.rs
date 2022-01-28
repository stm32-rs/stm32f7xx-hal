//! HAL for the STM32F7xx family of microcontrollers

#![cfg_attr(not(test), no_std)]
#![allow(non_camel_case_types)]

#[cfg(not(feature = "device-selected"))]
compile_error!(
    "This crate requires one of the following device features enabled:
        stm32f722
        stm32f723
        stm32f730
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
                "
);

pub(crate) use embedded_hal as hal;
pub use embedded_time;

#[cfg(feature = "stm32f722")]
pub use stm32f7::stm32f7x2 as pac;

#[cfg(feature = "stm32f723")]
pub use stm32f7::stm32f7x3 as pac;

#[cfg(feature = "stm32f730")]
pub use stm32f7::stm32f730 as pac;

#[cfg(feature = "stm32f732")]
pub use stm32f7::stm32f7x2 as pac;

#[cfg(feature = "stm32f733")]
pub use stm32f7::stm32f7x3 as pac;

#[cfg(feature = "stm32f745")]
pub use stm32f7::stm32f745 as pac;

#[cfg(feature = "stm32f746")]
pub use stm32f7::stm32f7x6 as pac;

#[cfg(feature = "stm32f756")]
pub use stm32f7::stm32f7x6 as pac;

#[cfg(feature = "stm32f765")]
pub use stm32f7::stm32f765 as pac;

#[cfg(feature = "stm32f767")]
pub use stm32f7::stm32f7x7 as pac;

#[cfg(feature = "stm32f769")]
pub use stm32f7::stm32f7x9 as pac;

#[cfg(feature = "stm32f777")]
pub use stm32f7::stm32f7x7 as pac;

#[cfg(feature = "stm32f778")]
pub use stm32f7::stm32f7x9 as pac;

#[cfg(feature = "stm32f779")]
pub use stm32f7::stm32f7x9 as pac;

// Enable use of interrupt macro
#[cfg(feature = "rt")]
pub use crate::pac::interrupt;

#[cfg(all(feature = "device-selected", feature = "has-can"))]
pub mod can;

#[cfg(feature = "device-selected")]
pub mod delay;

#[cfg(feature = "device-selected")]
pub mod dma;

#[cfg(all(feature = "device-selected", feature = "fmc"))]
pub mod fmc;

#[cfg(all(feature = "fmc_lcd", feature = "device-selected", feature = "fmc"))]
pub mod fmc_lcd;

#[cfg(feature = "device-selected")]
pub mod gpio;

#[cfg(feature = "device-selected")]
pub mod dac;

#[cfg(all(
    feature = "usb_fs",
    any(
        feature = "stm32f722",
        feature = "stm32f723",
        feature = "stm32f730",
        feature = "stm32f732",
        feature = "stm32f733",
        feature = "stm32f746",
        feature = "stm32f767",
    )
))]
pub mod otg_fs;

#[cfg(all(
    feature = "usb_hs",
    any(
        feature = "stm32f722",
        feature = "stm32f723",
        feature = "stm32f730",
        feature = "stm32f732",
        feature = "stm32f733",
        feature = "stm32f746",
        feature = "stm32f767",
    )
))]
pub mod otg_hs;

#[cfg(feature = "device-selected")]
pub mod prelude;

#[cfg(feature = "device-selected")]
pub mod rcc;

#[cfg(feature = "device-selected")]
pub mod rtc;

#[cfg(feature = "device-selected")]
pub mod serial;

#[cfg(feature = "device-selected")]
pub mod spi;

#[cfg(feature = "device-selected")]
pub mod timer;

#[cfg(feature = "device-selected")]
pub mod signature;

#[cfg(feature = "device-selected")]
pub mod i2c;

#[cfg(feature = "device-selected")]
pub mod rng;

#[cfg(feature = "device-selected")]
pub mod qspi;

#[cfg(any(feature = "stm32f765", feature = "stm32f767", feature = "stm32f769"))]
pub mod adc;

#[cfg(any(feature = "stm32f767", feature = "stm32f769"))]
pub mod qei;

#[cfg(feature = "ltdc")]
pub mod ltdc;

#[cfg(feature = "device-selected")]
pub mod flash;

pub mod state {
    /// Indicates that a peripheral is enabled
    pub struct Enabled;

    /// Indicates that a peripheral is disabled
    pub struct Disabled;
}

#[cfg(feature = "device-selected")]
mod sealed {
    pub trait Sealed {}
}
#[cfg(feature = "device-selected")]
pub(crate) use sealed::Sealed;
