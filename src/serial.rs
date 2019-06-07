use core::fmt::{Result, Write};
use core::marker::PhantomData;
use core::ptr;

use crate::hal::prelude::*;
use crate::hal::serial;
use crate::time::U32Ext;
use nb::block;

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
use crate::device::{RCC, USART1, USART2, USART3, USART6};

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
use crate::gpio::{
    gpioa::{PA2, PA3, PA9, PA10},
    gpiob::{PB6, PB7, PB10, PB11},
    gpioc::{PC6, PC7, PC10, PC11},
    gpiod::{PD5, PD6, PD8, PD9},
    gpiog::{PG9, PG14},
    Alternate, AF7, AF8,
};

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
impl PinTx<USART1> for PB6<Alternate<AF7>> {}
impl PinTx<USART2> for PA2<Alternate<AF7>> {}
impl PinTx<USART2> for PD5<Alternate<AF7>> {}
impl PinTx<USART3> for PB10<Alternate<AF7>> {}
impl PinTx<USART3> for PC10<Alternate<AF7>> {}
impl PinTx<USART3> for PD8<Alternate<AF7>> {}
impl PinTx<USART6> for PC6<Alternate<AF8>> {}
impl PinTx<USART6> for PG14<Alternate<AF8>> {}

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
impl PinRx<USART1> for PA10<Alternate<AF7>> {}
impl PinRx<USART1> for PB7<Alternate<AF7>> {}
impl PinRx<USART2> for PA3<Alternate<AF7>> {}
impl PinRx<USART2> for PD6<Alternate<AF7>> {}
impl PinRx<USART3> for PB11<Alternate<AF7>> {}
impl PinRx<USART3> for PC11<Alternate<AF7>> {}
impl PinRx<USART3> for PD9<Alternate<AF7>> {}
impl PinRx<USART6> for PC7<Alternate<AF8>> {}
impl PinRx<USART6> for PG9<Alternate<AF8>> {}

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


/// USART configuration
pub struct Config {
    pub baud_rate: Bps,
    pub oversampling: Oversampling,
}

pub enum Oversampling {
    By8,
    By16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baud_rate: 115_200.bps(),
            oversampling: Oversampling::By16,
        }
    }
}


macro_rules! halUsart {
    ($(
        $USARTX:ident: ($usartX:ident, $apbXenr:ident, $usartXsel:ident, $usartXen:ident),
    )+) => {
        $(
            impl<PINS> Serial<$USARTX, PINS> {
                pub fn $usartX(
                    usart: $USARTX,
                    pins: PINS,
                    clocks: Clocks,
                    config: Config) -> Self
                where
                    PINS: Pins<$USARTX>,
                {
                    // NOTE(unsafe) This executes only during initialisation
                    let rcc = unsafe { &(*RCC::ptr()) };

                    // TODO: The unsafe calls below should be replaced with accessing
                    //       the correct registers directly.

                    // Use sysclock for baudrate
                    rcc.dckcfgr2.modify(|_, w| w.$usartXsel().bits(1));

                    // Enable clock for USART
                    rcc.$apbXenr.modify(|_, w| w.$usartXen().set_bit());

                    // Calculate correct baudrate divisor on the fly
                    let brr = match config.oversampling {
                        Oversampling::By8 => {
                            usart.cr1.modify(|_, w| w.over8().set_bit());

                            let usart_div =
                                2 * clocks.sysclk().0 / config.baud_rate.0;

                            0xfff0 & usart_div
                                | 0x0008 & 0
                                | 0x0007 & ((usart_div & 0x000f) >> 1)
                        }
                        Oversampling::By16 => {
                            usart.cr1.modify(|_, w| w.over8().clear_bit());

                            clocks.sysclk().0 / config.baud_rate.0
                        }
                    };

                    usart.brr.write(|w| unsafe { w.bits(brr) });

                    // Reset other registers to disable advanced USART features
                    usart.cr2.reset();
                    usart.cr3.reset();

                    // Enable transmission and receiving
                    usart.cr1.modify(|_, w|
                        w
                            .te().enabled()
                            .re().enabled()
                            .ue().enabled()
                    );

                    Serial { usart, pins }
                }

                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }

                pub fn release(self) -> ($USARTX, PINS) {
                    (self.usart, self.pins)
                }
            }

            impl<PINS> serial::Read<u8> for Serial<$USARTX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let mut rx: Rx<$USARTX> = Rx {
                        _usart: PhantomData,
                    };
                    rx.read()
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    Err(if isr.pe().bit_is_set() {
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe {
                            ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> serial::Write<u8> for Serial<$USARTX, PINS> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.write(byte)
                }
            }

            impl serial::Write<u8> for Tx<$USARTX> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }
        )+
    }
}


#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
halUsart! {
    USART1: (usart1, apb2enr, usart1sel, usart1en),
    USART2: (usart2, apb1enr, usart2sel, usart2en),
    USART3: (usart3, apb1enr, usart3sel, usart3en),
    USART6: (usart6, apb2enr, usart6sel, usart6en),
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
