//! Serial communication using UART/USART peripherals

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
use crate::rcc::{BusClock, Enable, Reset};
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
pub trait PinCts<USART> {}
pub trait PinRts<USART> {}

impl<U, TX, RX> Pins<U> for (TX, RX)
where
    TX: PinTx<U>,
    RX: PinRx<U>,
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
// TODO where is UART8?

impl PinCts<USART1> for gpio::PA11<Alternate<7>> {}
impl PinCts<USART2> for gpio::PA0<Alternate<7>> {}
impl PinCts<USART3> for gpio::PB13<Alternate<7>> {}
impl PinCts<USART3> for gpio::PD11<Alternate<7>> {}
impl PinCts<UART4> for gpio::PA15<Alternate<8>> {}
impl PinCts<UART5> for gpio::PC9<Alternate<7>> {}
impl PinCts<USART6> for gpio::PG13<Alternate<8>> {}
impl PinCts<USART6> for gpio::PG15<Alternate<8>> {}
impl PinCts<UART7> for gpio::PE10<Alternate<8>> {}
impl PinCts<UART7> for gpio::PF9<Alternate<8>> {}

impl PinRts<USART1> for gpio::PA12<Alternate<7>> {}
impl PinRts<USART2> for gpio::PA1<Alternate<7>> {}
impl PinRts<USART3> for gpio::PB14<Alternate<7>> {}
impl PinRts<USART3> for gpio::PD12<Alternate<7>> {}
impl PinRts<UART4> for gpio::PB0<Alternate<8>> {}
impl PinRts<UART5> for gpio::PC8<Alternate<7>> {}
impl PinRts<USART6> for gpio::PG8<Alternate<8>> {}
impl PinRts<USART6> for gpio::PG12<Alternate<8>> {}
impl PinRts<UART7> for gpio::PE9<Alternate<8>> {}
impl PinRts<UART7> for gpio::PF8<Alternate<8>> {}


enum AsyncFlowControl {
    Rs232None,
    Rs232CtsRts,
    Rs232Cts,
    Rs232Rts,
    // TODO add Rs485
}

enum SerialMode {
    Async(AsyncFlowControl),
    // TODO add SingleWire, IrDA, ModbusCommunication, LIN, SnartCard, etc.
}

/// Serial abstraction
pub struct Serial<U, PINS> {
    usart: U,
    pins: PINS,
}

pub trait UART<U: Instance> {
    fn new_async_uart_no_flwctl<PINS: Pins<U>>(
        uart: U, pins: PINS, clocks: &Clocks, config: Config,
    ) -> Serial<U, PINS> {
        Serial::new(uart, pins, clocks, config, SerialMode::Async(AsyncFlowControl::Rs232None))
    }

    fn new_async_uart_rs232_cts_rts<PINS: Pins<U>, CTS: PinCts<U>, RTS: PinRts<U>>(
        uart: U, pins: PINS, clocks: &Clocks, config: Config, cts: CTS, rts: RTS,
    ) -> Serial<U, PINS> {
        // TODO Clarify if we can borrow cts and rts and keep them borrowed.
        // TODO Note that at the moment any CTS and RTS pin of this U(S)ART would be accepted
        //      This may be too flexible.
        Serial::new(uart, pins, clocks, config, SerialMode::Async(AsyncFlowControl::Rs232CtsRts))
    }

    fn new_async_uart_rs232_cts<PINS: Pins<U>, CTS: PinCts<U>, RTS: PinRts<U>>(
        uart: U, pins: PINS, clocks: &Clocks, config: Config, cts: CTS,
    ) -> Serial<U, PINS> {
        Serial::new(uart, pins, clocks, config, SerialMode::Async(AsyncFlowControl::Rs232Cts))
    }

    fn new_async_uart_rs232_rts<PINS: Pins<U>, RTS: PinRts<U>>(
        uart: U, pins: PINS, clocks: &Clocks, config: Config, rts: RTS,
    ) -> Serial<U, PINS> {
        Serial::new(uart, pins, clocks, config, SerialMode::Async(AsyncFlowControl::Rs232Rts))
    }

    // TODO Add constructors for other modes of operation
}
impl <PINS> UART<UART4> for Serial<UART4, PINS> where PINS: Pins<UART4> {}
impl <PINS> UART<UART5> for Serial<UART5, PINS> where PINS: Pins<UART5> {}
impl <PINS> UART<UART7> for Serial<UART7, PINS> where PINS: Pins<UART7> {}
// TODO where is UART8?
impl <PINS> UART<USART1> for Serial<USART1, PINS> where PINS: Pins<USART1> {}
impl <PINS> UART<USART2> for Serial<USART2, PINS> where PINS: Pins<USART2> {}
impl <PINS> UART<USART3> for Serial<USART3, PINS> where PINS: Pins<USART3> {}
impl <PINS> UART<USART6> for Serial<USART6, PINS> where PINS: Pins<USART6> {}

// TODO add USART trait which would have USART-specific modes like "Synchronous" and "SmartCard"
// TOOD add add implementation of those traits for all USART*

impl<U, PINS> Serial<U, PINS>
where
    PINS: Pins<U>,
    U: Instance,
{
    fn new(
        usart: U,
        pins: PINS,
        clocks: &Clocks,
        config: Config,
        mode: SerialMode
    ) -> Self {  // TODO remove "pub"
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // TODO: The unsafe calls below should be replaced with accessing
        //       the correct registers directly.

        U::select_sysclock(rcc, config.sysclock);
        unsafe {
            U::enable_unchecked();
        }

        let clk = if config.sysclock {
            clocks.sysclk()
        } else {
            U::clock(clocks)
        };

        // Calculate correct baudrate divisor on the fly
        let brr = match config.oversampling {
            Oversampling::By8 => {
                usart.cr1.modify(|_, w| w.over8().set_bit());

                let usart_div = 2 * clk / config.baud_rate;

                0xfff0 & usart_div | 0x0007 & ((usart_div & 0x000f) >> 1)
            }
            Oversampling::By16 => {
                usart.cr1.modify(|_, w| w.over8().clear_bit());

                clk / config.baud_rate
            }
        };

        usart.brr.write(|w| unsafe { w.bits(brr) });

        // Set character match and reset other registers to disable advanced USART features
        let ch = config.character_match.unwrap_or(0);
        usart.cr2.write(|w| w.add().bits(ch));

        // Enable tx / rx, configure data bits and parity
        usart.cr1.modify(|_, w| {
            w
                .te().enabled()
                .re().enabled()
                .ue().enabled();

            // M[1:0] are used to set data bits
            // M[1:0] = 00: 1 Start bit, 8 data bits, n stop bits
            // M[1:0] = 01: 1 Start bit, 9 data bits, n stop bits
            // M[1:0] = 10: 1 Start bit, 7 data bits, n stop bits
            match config.data_bits {
                DataBits::Bits8 => w.m1().clear_bit().m0().bit8(),
                DataBits::Bits9 => w.m1().clear_bit().m0().bit9(),
                DataBits::Bits7 => w.m0().clear_bit().m1().bit7(),
            };

            match config.parity {
                Parity::ParityEven => w.ps().even().pce().enabled(),
                Parity::ParityOdd  => w.ps().odd().pce().enabled(),
                Parity::ParityNone => w.pce().disabled(),
            }
        });

        // Enable DMA
        usart.cr3.write(|w| w.dmat().enabled().dmar().enabled());

        match mode {
            SerialMode::Async(flow_control) => match flow_control {
                AsyncFlowControl::Rs232None => {
                    usart.cr3.write(|w| w.ctse().disabled().rtse().disabled());
                },
                AsyncFlowControl::Rs232CtsRts => {
                    usart.cr3.write(|w| w.ctse().enabled().rtse().enabled());
                },
                AsyncFlowControl::Rs232Cts => {
                    usart.cr3.write(|w| w.ctse().enabled().rtse().disabled());
                },
                AsyncFlowControl::Rs232Rts => {
                    usart.cr3.write(|w| w.ctse().disabled().rtse().enabled());
                },
            }
        }

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

    pub fn split(self) -> (Tx<U>, Rx<U>) {
        (
            Tx {
                _usart: PhantomData,
            },
            Rx {
                _usart: PhantomData,
            },
        )
    }

    pub fn release(self) -> (U, PINS) {
        (self.usart, self.pins)
    }
}

impl<U, PINS> serial::Read<u8> for Serial<U, PINS>
where
    U: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let mut rx: Rx<U> = Rx {
            _usart: PhantomData,
        };
        rx.read()
    }
}

impl<U, PINS> serial::Write<u8> for Serial<U, PINS>
where
    U: Instance,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let mut tx: Tx<U> = Tx {
            _usart: PhantomData,
        };
        tx.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let mut tx: Tx<U> = Tx {
            _usart: PhantomData,
        };
        tx.write(byte)
    }
}

/// Serial receiver
pub struct Rx<U> {
    _usart: PhantomData<U>,
}

impl<U> Rx<U>
where
    U: Instance,
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
        let address = &unsafe { &*U::ptr() }.rdr as *const _ as _;

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

impl<U> serial::Read<u8> for Rx<U>
where
    U: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*U::ptr()).isr.read() };

        // NOTE(unsafe): Only used for atomic writes, to clear error flags.
        let icr = unsafe { &(*U::ptr()).icr };

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
                (*U::ptr()).rdr.read().rdr().bits() as u8
            });
        }

        Err(nb::Error::WouldBlock)
    }
}

/// Serial transmitter
pub struct Tx<U> {
    _usart: PhantomData<U>,
}

impl<U> Tx<U>
where
    Self: dma::Target,
    U: Instance,
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
        let usart = unsafe { &*U::ptr() };
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

impl<U> serial::Write<u8> for Tx<U>
where
    U: Instance,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*U::ptr()).isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*U::ptr()).isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
            unsafe { ptr::write_volatile(&(*U::ptr()).tdr as *const _ as *mut _, byte) }
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
    pub sysclock: bool,
    pub parity: Parity,
    pub data_bits: DataBits,
}

pub enum Oversampling {
    By8,
    By16,
}

/// Number of data bits
pub enum DataBits {
    /// 8 bits of data
    Bits8,
    /// 9 bits of data
    Bits9,
    /// 7 bits of data
    Bits7,
}

/// Parity generation and checking. If odd or even parity is selected, the
/// underlying USART will be configured to send/receive the parity bit in
/// addtion to the data bits.
pub enum Parity {
    /// No parity bit will be added/checked.
    ParityNone,
    /// The MSB transmitted/received will be generated/checked to have a
    /// even number of bits set.
    ParityEven,
    /// The MSB transmitted/received will be generated/checked to have a
    /// odd number of bits set.
    ParityOdd,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baud_rate: 115_200.bps(),
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
            parity: Parity::ParityNone,
            data_bits: DataBits::Bits8,
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
pub trait Instance: Deref<Target = pac::usart1::RegisterBlock> + Enable + Reset + BusClock {
    fn ptr() -> *const pac::usart1::RegisterBlock;
    fn select_sysclock(rcc: &pac::rcc::RegisterBlock, sys: bool);
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

                fn select_sysclock(rcc: &pac::rcc::RegisterBlock, sys: bool) {
                    rcc.dckcfgr2.modify(|_, w| w.$usartXsel().bits(sys as _));
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

impl<U> fmt::Write for Tx<U>
where
    Tx<U>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}
