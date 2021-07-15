pub use embedded_time::{duration::Extensions as _, rate::Extensions as _};

#[cfg(feature = "fmc")]
pub use crate::fmc::FmcExt as _stm327xx_hal_fmc_FmcExt;

pub use crate::gpio::GpioExt as _stm327xx_hal_gpio_GpioExt;
pub use crate::hal::digital::v2::{InputPin, OutputPin};
pub use crate::hal::prelude::*;
pub use crate::rcc::RccExt as _stm32f7xx_hal_rcc_RccExt;
pub use crate::rng::RngExt as _;
