//! HAL for the STM32F7xx family of microcontrollers

#![no_std]
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

#[cfg(feature = "device-selected")]
pub mod delay;

#[cfg(feature = "device-selected")]
pub mod dma;

#[cfg(feature = "device-selected")]
pub mod gpio;

#[cfg(feature = "device-selected")]
pub mod prelude;

#[cfg(feature = "device-selected")]
pub mod rcc;

#[cfg(feature = "device-selected")]
pub mod serial;

#[cfg(feature = "device-selected")]
pub mod spi;

#[cfg(feature = "device-selected")]
pub mod time;

#[cfg(feature = "device-selected")]
pub mod timer;

#[cfg(feature = "device-selected")]
pub mod signature;

#[cfg(feature = "device-selected")]
pub mod i2c;

#[cfg(feature = "ltdc")]
pub mod ltdc;

#[cfg(all(
    feature = "device-selected",
    not(any(
        feature = "stm32f765",
        feature = "stm32f767",
        feature = "stm32f769",
        feature = "stm32f777",
        feature = "stm32f778",
        feature = "stm32f779",
    ))
))]
pub mod flash;

pub mod state {
    /// Indicates that a peripheral is enabled
    pub struct Enabled;

    /// Indicates that a peripheral is disabled
    pub struct Disabled;
}
