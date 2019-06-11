use core::fmt::{Result, Write};
use core::marker::PhantomData;
use core::ops::Deref;
use core::ptr;

use crate::hal::prelude::*;
use crate::hal::serial;
use crate::device;
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

impl<USART, PINS> Serial<USART, PINS> {
    pub fn new(
        usart: USART,
        pins: PINS,
        clocks: Clocks,
        config: Config) -> Self
    where
        PINS: Pins<USART>,
        USART: Instance,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // TODO: The unsafe calls below should be replaced with accessing
        //       the correct registers directly.

        USART::select_sysclock(rcc);
        USART::enable_clock(rcc);

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

    pub fn split(self) -> (Tx<USART>, Rx<USART>) {
        (
            Tx {
                _usart: PhantomData,
            },
            Rx {
                _usart: PhantomData,
            },
        )
    }

    pub fn release(self) -> (USART, PINS) {
        (self.usart, self.pins)
    }
}

impl<USART, PINS> serial::Read<u8> for Serial<USART, PINS>
    where USART: Instance
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let mut rx: Rx<USART> = Rx {
            _usart: PhantomData,
        };
        rx.read()
    }
}

impl<USART, PINS> serial::Write<u8> for Serial<USART, PINS>
    where USART: Instance
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let mut tx: Tx<USART> = Tx {
            _usart: PhantomData,
        };
        tx.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let mut tx: Tx<USART> = Tx {
            _usart: PhantomData,
        };
        tx.write(byte)
    }
}


/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

impl<USART> serial::Read<u8> for Rx<USART>
    where USART: Instance
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART::ptr()).isr.read() };

        // NOTE(unsafe): Only used for atomic writes, to clear error flags.
        let icr = unsafe { &(*USART::ptr()).icr };

        if isr.pe().bit_is_set() {
            icr.write(|w| w.pecf().clear());
            return Err(nb::Error::Other(Error::Parity));
        }
        if isr.fe().bit_is_set() {
            icr.write(|w| w.fecf().clear());
            return Err(nb::Error::Other(Error::Framing));
        }
        if isr.nf().bit_is_set() {
            icr.write(|w| w.ncf().clear());
            return Err(nb::Error::Other(Error::Noise));
        }
        if isr.ore().bit_is_set() {
            icr.write(|w| w.orecf().clear());
            return Err(nb::Error::Other(Error::Overrun));
        }

        if isr.rxne().bit_is_set() {
            // NOTE(unsafe): Atomic read with no side effects
            return Ok(unsafe {
                // Casting to `u8` should be fine, as we've configured the USART
                // to use 8 data bits.
                (*USART::ptr()).rdr.read().rdr().bits() as u8
            });
        }

        Err(nb::Error::WouldBlock)
    }
}


/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

impl<USART> serial::Write<u8> for Tx<USART>
    where USART: Instance
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART::ptr()).isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART::ptr()).isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
            unsafe {
                ptr::write_volatile(&(*USART::ptr()).tdr as *const _ as *mut _, byte)
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
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


/// Implemented by all USART instances
pub trait Instance: Deref<Target = device::usart1::RegisterBlock> {
    fn ptr() -> *const device::usart1::RegisterBlock;
    fn select_sysclock(rcc: &device::rcc::RegisterBlock);
    fn enable_clock(rcc: &device::rcc::RegisterBlock);
}

macro_rules! impl_instance {
    ($(
        $USARTX:ident: ($apbXenr:ident, $usartXsel:ident, $usartXen:ident),
    )+) => {
        $(
            impl Instance for $USARTX {
                fn ptr() -> *const device::usart1::RegisterBlock {
                    $USARTX::ptr()
                }

                fn select_sysclock(rcc: &device::rcc::RegisterBlock) {
                    rcc.dckcfgr2.modify(|_, w| w.$usartXsel().bits(1));
                }

                fn enable_clock(rcc: &device::rcc::RegisterBlock) {
                    rcc.$apbXenr.modify(|_, w| w.$usartXen().set_bit());
                }
            }
        )+
    }
}

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
))]
impl_instance! {
    USART1: (apb2enr, usart1sel, usart1en),
    USART2: (apb1enr, usart2sel, usart2en),
    USART3: (apb1enr, usart3sel, usart3en),
    USART6: (apb2enr, usart6sel, usart6en),
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
