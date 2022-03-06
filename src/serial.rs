use core::fmt;
use core::marker::PhantomData;
use core::ops::Deref;
use core::ops::DerefMut;
use core::pin::Pin;
use core::ptr;

use as_slice::{AsMutSlice, AsSlice};

use crate::dma;
use crate::hal::prelude::*;
use crate::hal::serial;
use crate::pac;
use crate::rcc::{Enable, Reset};
use crate::state;
use nb::block;

use crate::pac::{RCC, UART4, UART5, UART7, USART1, USART2, USART3, USART6};

use crate::gpio::{self, Alternate};

use crate::rcc::Clocks;
use crate::{BitsPerSecond, U32Ext};

/// Serial error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

pub trait Pins<USART> {}
pub trait PinTx<USART> {}
pub trait PinRx<USART> {}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
{
}

mod f7xx_pins {
    //table 13 in stm32f765bg.pdf
    use super::{PinRx, PinTx};
    use crate::gpio::{self, Alternate};
    use crate::pac::{UART4, UART5, UART7, USART1};
    impl PinTx<USART1> for gpio::PB14<Alternate<4>> {}
    impl PinRx<USART1> for gpio::PB15<Alternate<4>> {}

    impl PinTx<UART4> for gpio::PA11<Alternate<6>> {}
    impl PinRx<UART4> for gpio::PA12<Alternate<6>> {}

    impl PinTx<UART4> for gpio::PD1<Alternate<8>> {}
    impl PinRx<UART4> for gpio::PD0<Alternate<8>> {}

    impl PinTx<UART4> for gpio::PH13<Alternate<8>> {}
    impl PinRx<UART4> for gpio::PH14<Alternate<8>> {}

    impl PinRx<UART4> for gpio::PI9<Alternate<8>> {}

    impl PinTx<UART5> for gpio::PB6<Alternate<1>> {}
    impl PinRx<UART5> for gpio::PB5<Alternate<1>> {}

    impl PinTx<UART5> for gpio::PB9<Alternate<7>> {}
    impl PinRx<UART5> for gpio::PB8<Alternate<7>> {}

    impl PinTx<UART5> for gpio::PB13<Alternate<8>> {}
    impl PinRx<UART5> for gpio::PB12<Alternate<8>> {}

    impl PinTx<UART7> for gpio::PA15<Alternate<12>> {}
    impl PinRx<UART7> for gpio::PA8<Alternate<12>> {}

    impl PinTx<UART7> for gpio::PB4<Alternate<12>> {}
    impl PinRx<UART7> for gpio::PB3<Alternate<12>> {}
}

impl PinTx<USART1> for gpio::PA9<Alternate<7>> {}
impl PinTx<USART1> for gpio::PB6<Alternate<7>> {}
impl PinTx<USART2> for gpio::PA2<Alternate<7>> {}
impl PinTx<USART2> for gpio::PD5<Alternate<7>> {}
impl PinTx<USART3> for gpio::PB10<Alternate<7>> {}
impl PinTx<USART3> for gpio::PC10<Alternate<7>> {}
impl PinTx<USART3> for gpio::PD8<Alternate<7>> {}
impl PinTx<UART4> for gpio::PA0<Alternate<8>> {}
impl PinTx<UART4> for gpio::PC10<Alternate<8>> {}
impl PinTx<UART5> for gpio::PC12<Alternate<8>> {}
impl PinTx<USART6> for gpio::PC6<Alternate<8>> {}
impl PinTx<USART6> for gpio::PG14<Alternate<8>> {}
impl PinTx<UART7> for gpio::PE8<Alternate<8>> {}
impl PinTx<UART7> for gpio::PF7<Alternate<8>> {}

impl PinRx<USART1> for gpio::PA10<Alternate<7>> {}
impl PinRx<USART1> for gpio::PB7<Alternate<7>> {}
impl PinRx<USART2> for gpio::PA3<Alternate<7>> {}
impl PinRx<USART2> for gpio::PD6<Alternate<7>> {}
impl PinRx<USART3> for gpio::PB11<Alternate<7>> {}
impl PinRx<USART3> for gpio::PC11<Alternate<7>> {}
impl PinRx<USART3> for gpio::PD9<Alternate<7>> {}
impl PinRx<UART4> for gpio::PA1<Alternate<8>> {}
impl PinRx<UART4> for gpio::PC11<Alternate<8>> {}
impl PinRx<UART5> for gpio::PD2<Alternate<8>> {}
impl PinRx<USART6> for gpio::PC7<Alternate<8>> {}
impl PinRx<USART6> for gpio::PG9<Alternate<8>> {}
impl PinRx<UART7> for gpio::PE7<Alternate<8>> {}
impl PinRx<UART7> for gpio::PF6<Alternate<8>> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

impl<USART, PINS> Serial<USART, PINS>
where
    PINS: Pins<USART>,
    USART: Instance,
{
    pub fn new(usart: USART, pins: PINS, clocks: Clocks, config: Config) -> Self {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // TODO: The unsafe calls below should be replaced with accessing
        //       the correct registers directly.

        USART::select_sysclock(rcc);
        unsafe {
            USART::enable_unchecked();
        }

        // Calculate correct baudrate divisor on the fly
        let brr = match config.oversampling {
            Oversampling::By8 => {
                usart.cr1.modify(|_, w| w.over8().set_bit());

                let usart_div = 2 * clocks.sysclk() / config.baud_rate;

                0xfff0 & usart_div | 0x0007 & ((usart_div & 0x000f) >> 1)
            }
            Oversampling::By16 => {
                usart.cr1.modify(|_, w| w.over8().clear_bit());

                clocks.sysclk() / config.baud_rate
            }
        };

        usart.brr.write(|w| unsafe { w.bits(brr) });

        // Set character match and reset other registers to disable advanced USART features
        let ch = config.character_match.unwrap_or(0);
        usart.cr2.write(|w| w.add().bits(ch));

        // Enable transmission and receiving
        usart
            .cr1
            .modify(|_, w| w.te().enabled().re().enabled().ue().enabled());

        // Enable DMA
        usart.cr3.write(|w| w.dmat().enabled().dmar().enabled());

        Serial { usart, pins }
    }

    /// Starts listening for an interrupt event
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
            Event::CharacterMatch => self.usart.cr1.modify(|_, w| w.cmie().set_bit()),
            Event::Error => self.usart.cr3.modify(|_, w| w.eie().set_bit()),
        }
    }

    /// End listening for an interrupt event
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
            Event::CharacterMatch => self.usart.cr1.modify(|_, w| w.cmie().clear_bit()),
            Event::Error => self.usart.cr3.modify(|_, w| w.eie().clear_bit()),
        }
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
where
    USART: Instance,
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
where
    USART: Instance,
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

impl<USART> Rx<USART>
where
    USART: Instance,
    Self: dma::Target,
{
    /// Reads data using DMA until `buffer` is full
    ///
    /// DMA supports transfers up to 65535 bytes. If `buffer` is longer, this
    /// method will panic.
    pub fn read_all<B>(
        self,
        buffer: Pin<B>,
        dma: &dma::Handle<<Self as dma::Target>::Instance, state::Enabled>,
        stream: <Self as dma::Target>::Stream,
    ) -> dma::Transfer<Self, B, dma::Ready>
    where
        B: DerefMut + 'static,
        B::Target: AsMutSlice<Element = u8>,
    {
        // This is safe, as we're only using the USART instance to access the
        // address of one register.
        let address = &unsafe { &*USART::ptr() }.rdr as *const _ as _;

        // Safe, because the trait bounds on this method guarantee that `buffer`
        // can be written to safely.
        unsafe {
            dma::Transfer::new(
                dma,
                stream,
                buffer,
                self,
                address,
                dma::Direction::PeripheralToMemory,
            )
        }
    }
}

impl<USART> serial::Read<u8> for Rx<USART>
where
    USART: Instance,
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

impl<USART> Tx<USART>
where
    Self: dma::Target,
    USART: Instance,
{
    /// Writes data using DMA
    ///
    /// DMA supports transfers up to 65535 bytes. If `data` is longer, this
    /// method will panic.
    pub fn write_all<B>(
        self,
        data: Pin<B>,
        dma: &dma::Handle<<Self as dma::Target>::Instance, state::Enabled>,
        stream: <Self as dma::Target>::Stream,
    ) -> dma::Transfer<Self, B, dma::Ready>
    where
        B: Deref + 'static,
        B::Target: AsSlice<Element = u8>,
    {
        // Prepare USART for DMA. See reference manual for STM32F75xxx and
        // STM32F74xxx, section 31.5.15.
        //
        // This is safe, as we're doing just one atomic write.
        let usart = unsafe { &*USART::ptr() };
        usart.icr.write(|w| w.tccf().clear());

        // Safe, because the trait bounds on this method guarantee that `buffer`
        // can be read from safely.
        unsafe {
            dma::Transfer::new(
                dma,
                stream,
                data,
                self,
                &usart.tdr as *const _ as _,
                dma::Direction::MemoryToPeripheral,
            )
        }
    }
}

impl<USART> serial::Write<u8> for Tx<USART>
where
    USART: Instance,
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
            unsafe { ptr::write_volatile(&(*USART::ptr()).tdr as *const _ as *mut _, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// USART configuration
pub struct Config {
    pub baud_rate: BitsPerSecond,
    pub oversampling: Oversampling,
    pub character_match: Option<u8>,
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
            character_match: None,
        }
    }
}

/// Interrupt event
#[derive(Debug)]
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Character match interrupt
    CharacterMatch,
    /// Error interrupt
    Error,
}

/// Implemented by all USART instances
pub trait Instance: Deref<Target = pac::usart1::RegisterBlock> + Enable + Reset {
    fn ptr() -> *const pac::usart1::RegisterBlock;
    fn select_sysclock(rcc: &pac::rcc::RegisterBlock);
}

macro_rules! impl_instance {
    ($(
        $USARTX:ident: ($usartXsel:ident),
    )+) => {
        $(
            impl Instance for $USARTX {
                fn ptr() -> *const pac::usart1::RegisterBlock {
                    $USARTX::ptr()
                }

                fn select_sysclock(rcc: &pac::rcc::RegisterBlock) {
                    rcc.dckcfgr2.modify(|_, w| w.$usartXsel().bits(1));
                }
            }
        )+
    }
}

#[cfg(any(feature = "device-selected",))]
impl_instance! {
    USART1: (usart1sel),
    USART2: (usart2sel),
    USART3: (usart3sel),
    UART4:  (uart4sel),
    UART5:  (uart5sel),
    USART6: (usart6sel),
    UART7:  (uart7sel),
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}
