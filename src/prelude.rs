pub use fugit::{ExtU32 as _, RateExtU32 as _};

#[cfg(feature = "fmc")]
pub use crate::fmc::FmcExt as _stm327xx_hal_fmc_FmcExt;

pub use crate::gpio::GpioExt as _stm327xx_hal_gpio_GpioExt;
pub use crate::hal::digital::v2::{InputPin, OutputPin};
pub use crate::hal::prelude::*;
pub use crate::rcc::RccExt as _stm32f7xx_hal_rcc_RccExt;
pub use crate::rng::RngExt as _;
#[cfg(feature = "rtic")]
pub use crate::timer::MonoTimerExt as _;
pub use crate::timer::PwmExt as _;
pub use crate::timer::SysTimerExt as _;
pub use crate::timer::TimerExt as _;
pub use crate::U32Ext as _;
