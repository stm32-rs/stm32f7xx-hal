use core::fmt;
use core::marker::PhantomData;
use core::ops::Deref;
use core::ops::DerefMut;
use core::pin::Pin;
use core::ptr;

use as_slice::{AsMutSlice, AsSlice};

use crate::dma;
use crate::embedded_time::rate::Extensions as _;
use crate::hal::prelude::*;
use crate::hal::serial;
use crate::pac;
use crate::rcc::{Enable, Reset};
use crate::state;
use nb::block;

#[cfg(any(feature = "device-selected",))]
use crate::pac::{RCC, UART4, UART5, UART7, USART1, USART2, USART3, USART6};

#[cfg(any(feature = "device-selected",))]
use crate::gpio::{
    gpioa::{PA0, PA1, PA10, PA2, PA3, PA9},
    gpiob::{PB10, PB11, PB6, PB7},
    gpioc::{PC10, PC11, PC12, PC6, PC7},
    gpiod::{PD2, PD5, PD6, PD8, PD9},
    gpioe::{PE7, PE8},
    gpiof::{PF6, PF7},
    gpiog::{PG14, PG9},
    Alternate,
};

use crate::embedded_time::rate::BitsPerSecond;
use crate::rcc::Clocks;

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
    /// UsartRx Moved
    RxMoved,
    /// UsartTx Moved
    TxMoved,
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
    use crate::gpio::{
        gpioa::{PA11, PA12, PA15, PA8},
        gpiob::{PB12, PB13, PB14, PB15, PB3, PB4, PB5, PB6, PB8, PB9},
        gpiod::{PD0, PD1},
        gpioh::{PH13, PH14},
        gpioi::PI9,
        Alternate,
    };
    use crate::pac::{UART4, UART5, UART7, USART1};
    impl PinTx<USART1> for PB14<Alternate<4>> {}
    impl PinRx<USART1> for PB15<Alternate<4>> {}

    impl PinTx<UART4> for PA11<Alternate<6>> {}
    impl PinRx<UART4> for PA12<Alternate<6>> {}

    impl PinTx<UART4> for PD1<Alternate<8>> {}
    impl PinRx<UART4> for PD0<Alternate<8>> {}

    impl PinTx<UART4> for PH13<Alternate<8>> {}
    impl PinRx<UART4> for PH14<Alternate<8>> {}

    impl PinRx<UART4> for PI9<Alternate<8>> {}

    impl PinTx<UART5> for PB6<Alternate<1>> {}
    impl PinRx<UART5> for PB5<Alternate<1>> {}

    impl PinTx<UART5> for PB9<Alternate<7>> {}
    impl PinRx<UART5> for PB8<Alternate<7>> {}

    impl PinTx<UART5> for PB13<Alternate<8>> {}
    impl PinRx<UART5> for PB12<Alternate<8>> {}

    impl PinTx<UART7> for PA15<Alternate<12>> {}
    impl PinRx<UART7> for PA8<Alternate<12>> {}

    impl PinTx<UART7> for PB4<Alternate<12>> {}
    impl PinRx<UART7> for PB3<Alternate<12>> {}
}

#[cfg(any(
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f768",
    feature = "stm32f769"
))]
pub use f7xx_pins::*;

impl PinTx<USART1> for PA9<Alternate<7>> {}
impl PinTx<USART1> for PB6<Alternate<7>> {}
impl PinTx<USART2> for PA2<Alternate<7>> {}
impl PinTx<USART2> for PD5<Alternate<7>> {}
impl PinTx<USART3> for PB10<Alternate<7>> {}
impl PinTx<USART3> for PC10<Alternate<7>> {}
impl PinTx<USART3> for PD8<Alternate<7>> {}
impl PinTx<UART4> for PA0<Alternate<8>> {}
impl PinTx<UART4> for PC10<Alternate<8>> {}
impl PinTx<UART5> for PC12<Alternate<8>> {}
impl PinTx<USART6> for PC6<Alternate<8>> {}
impl PinTx<USART6> for PG14<Alternate<8>> {}
impl PinTx<UART7> for PE8<Alternate<8>> {}
impl PinTx<UART7> for PF7<Alternate<8>> {}

impl PinRx<USART1> for PA10<Alternate<7>> {}
impl PinRx<USART1> for PB7<Alternate<7>> {}
impl PinRx<USART2> for PA3<Alternate<7>> {}
impl PinRx<USART2> for PD6<Alternate<7>> {}
impl PinRx<USART3> for PB11<Alternate<7>> {}
impl PinRx<USART3> for PC11<Alternate<7>> {}
impl PinRx<USART3> for PD9<Alternate<7>> {}
impl PinRx<UART4> for PA1<Alternate<8>> {}
impl PinRx<UART4> for PC11<Alternate<8>> {}
impl PinRx<UART5> for PD2<Alternate<8>> {}
impl PinRx<USART6> for PC7<Alternate<8>> {}
impl PinRx<USART6> for PG9<Alternate<8>> {}
impl PinRx<UART7> for PE7<Alternate<8>> {}
impl PinRx<UART7> for PF6<Alternate<8>> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
    tx: Option<Tx<USART>>,
    rx: Option<Rx<USART>>,
}

impl<USART, PINS> Serial<USART, PINS>
where
    PINS: Pins<USART>,
    USART: Instance,
    Rx<USART>: dma::Target,
    Tx<USART>: dma::Target,
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

                let usart_div = 2 * clocks.sysclk().0 / config.baud_rate.0;

                0xfff0 & usart_div | 0x0007 & ((usart_div & 0x000f) >> 1)
            }
            Oversampling::By16 => {
                usart.cr1.modify(|_, w| w.over8().clear_bit());

                clocks.sysclk().0 / config.baud_rate.0
            }
        };

        usart.brr.write(|w| unsafe { w.bits(brr) });

        // Set character match and reset other registers to disable advanced USART features
        let ch = config.character_match.unwrap_or(0);
        usart.cr2.write(|w| w.add().bits(ch));

        // TXINV | RXINV: TX/RX pin active level inversion
        match config.active_level_inversion {
            UsartInversion::Standard => {
                usart
                    .cr2
                    .modify(|_r, w| w.rxinv().clear_bit().txinv().clear_bit());
            }
            UsartInversion::Inverted => {
                usart
                    .cr2
                    .modify(|_r, w| w.rxinv().set_bit().txinv().set_bit());
            }
        }

        usart.cr1.modify(|_r, w| {
            match config.word_length {
                // M[1:0]
                WordLength::DataBits7 => {
                    // 10: 1 Start bit, 7 data bits
                    w.m0().clear_bit().m1().set_bit();
                }
                WordLength::DataBits8 => {
                    // 00: 1 Start bit, 9 data bits
                    w.m0().clear_bit().m1().clear_bit();
                }
                WordLength::DataBits9 => {
                    // 01: 1 Start bit, 9 data bits
                    w.m0().set_bit().m1().clear_bit();
                }
            }

            if let Some(parity) = config.parity_control {
                // Parity control enable
                w.pce().set_bit();
                match parity {
                    ParityControl::Even => {
                        // Parity selection: Even
                        w.ps().clear_bit();
                    }
                    ParityControl::Odd => {
                        // Parity selection: Odd
                        w.ps().set_bit();
                    }
                }
            }

            // USART enable
            w.ue()
                .set_bit()
                // Enable receiving
                .re()
                .set_bit()
                // Enable transmission
                .te()
                .set_bit()
        });

        // Enable DMA
        usart.cr3.write(|w| w.dmat().enabled().dmar().enabled());

        Serial {
            usart,
            pins,
            tx: Some(Tx {
                _usart: PhantomData,
            }),
            rx: Some(Rx {
                _usart: PhantomData,
            }),
        }
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

    pub fn read_all<B>(
        &mut self,
        buffer: Pin<B>,
        dma: &dma::Handle<<Rx<USART> as dma::Target>::Instance, state::Enabled>,
        stream: <Rx<USART> as dma::Target>::Stream,
    ) -> Result<dma::Transfer<Rx<USART>, B, dma::Ready>, Error>
    where
        B: DerefMut + 'static,
        B::Target: AsMutSlice<Element = u8>,
    {
        if let Some(usart_rx) = self.rx.take() {
            Ok(usart_rx.read_all(buffer, dma, stream))
        } else {
            Err(Error::RxMoved)
        }
    }

    pub fn return_rx(&mut self, usart_rx: Rx<USART>) {
        self.rx.replace(usart_rx);
    }

    pub fn write_all<B>(
        &mut self,
        data: Pin<B>,
        dma: &dma::Handle<<Tx<USART> as dma::Target>::Instance, state::Enabled>,
        stream: <Tx<USART> as dma::Target>::Stream,
    ) -> Result<dma::Transfer<Tx<USART>, B, dma::Ready>, Error>
    where
        B: Deref + 'static,
        B::Target: AsSlice<Element = u8>,
    {
        if let Some(usart_tx) = self.tx.take() {
            Ok(usart_tx.write_all(data, dma, stream))
        } else {
            Err(Error::RxMoved)
        }
    }

    pub fn return_tx(&mut self, usart_tx: Tx<USART>) {
        self.tx.replace(usart_tx);
    }
}

impl<USART, PINS> serial::Read<u8> for Serial<USART, PINS>
where
    USART: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        if let Some(usart_rx) = &mut self.rx {
            usart_rx.read()
        } else {
            Err(nb::Error::Other(Error::RxMoved))
        }
    }
}

impl<USART, PINS> serial::Write<u8> for Serial<USART, PINS>
where
    USART: Instance,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if let Some(usart_tx) = &mut self.tx {
            usart_tx.flush()
        } else {
            Err(nb::Error::Other(Error::TxMoved))
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        if let Some(usart_tx) = &mut self.tx {
            usart_tx.write(byte)
        } else {
            Err(nb::Error::Other(Error::TxMoved))
        }
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
    pub active_level_inversion: UsartInversion,
    pub word_length: WordLength,
    pub parity_control: Option<ParityControl>,
    pub character_match: Option<u8>,
}

pub enum UsartInversion {
    Standard,
    Inverted,
}

pub enum WordLength {
    DataBits7,
    DataBits8,
    DataBits9,
}

pub enum ParityControl {
    Even,
    Odd,
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
            active_level_inversion: UsartInversion::Standard,
            word_length: WordLength::DataBits8,
            parity_control: None,
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
