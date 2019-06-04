use core::fmt::{Result, Write};
use core::marker::PhantomData;
use core::ptr;

use crate::hal::prelude::*;
use crate::hal::serial;
use nb::block;

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
use crate::device::{RCC, USART1};

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
use crate::gpio::gpioa::{PA9};

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
use crate::gpio::gpiob::{PB7};

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
use crate::gpio::{Alternate, AF7};

use crate::rcc::Clocks;
use crate::time::Bps;

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

pub trait Pins<USART> {}
pub trait PinTx<USART> {}
pub trait PinRx<USART> {}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
{}

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
impl PinTx<USART1> for PA9<Alternate<AF7>> {}

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
impl PinRx<USART1> for PB7<Alternate<AF7>> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

#[macro_use]
mod macros;

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
halUsart! {
    USART1: (usart1, apb2enr, usart1en),
}

impl<USART> Write for Tx<USART>
where
    Tx<USART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> Result {
        let _ = s
            .as_bytes()
            .into_iter()
            .map(|c| block!(self.write(*c)))
            .last();
        Ok(())
    }
}
