//! Interface to the SPI peripheral
//!
//! See chapter 32 in the STM32F746 Reference Manual.

pub use embedded_hal::spi::{Mode, Phase, Polarity};

use core::{fmt, marker::PhantomData, ops::DerefMut, pin::Pin, ptr};

use as_slice::{AsMutSlice, AsSlice as _};
use embedded_hal::{
    blocking::spi::{transfer, write, write_iter},
    spi::FullDuplex,
};

use crate::{
    gpio::{self, Alternate},
    pac::{self, spi1::cr2},
    rcc::{BusClock, Clocks, Enable, RccBus},
    state,
};

use crate::dma;
use fugit::HertzU32 as Hertz;

/// Entry point to the SPI API
pub struct Spi<I, P, State> {
    spi: I,
    pins: P,
    _state: State,
}

impl<I, P> Spi<I, P, state::Disabled>
where
    I: Instance + Enable + BusClock,
    P: Pins<I>,
{
    /// Create a new instance of the SPI API
    pub fn new(instance: I, pins: P) -> Self {
        Self {
            spi: instance,
            pins,
            _state: state::Disabled,
        }
    }

    /// Initialize the SPI peripheral
    pub fn enable<Word>(
        self,
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
        apb: &mut <I as RccBus>::Bus,
    ) -> Spi<I, P, Enabled<Word>>
    where
        Word: SupportedWordSize,
    {
        I::enable(apb);
        let cpol = mode.polarity == Polarity::IdleHigh;
        let cpha = mode.phase == Phase::CaptureOnSecondTransition;

        let br = match I::clock(clocks) / freq {
            0 => unreachable!(),
            1..=2 => 0b000,
            3..=5 => 0b001,
            6..=11 => 0b010,
            12..=23 => 0b011,
            24..=47 => 0b100,
            48..=95 => 0b101,
            96..=191 => 0b110,
            _ => 0b111,
        };
        self.spi.configure::<Word>(br, cpol, cpha);

        Spi {
            spi: self.spi,
            pins: self.pins,
            _state: Enabled(PhantomData),
        }
    }
}

impl<I, P, Word> Spi<I, P, Enabled<Word>>
where
    I: Instance,
    P: Pins<I>,
    Word: SupportedWordSize,
{
    /// Start an SPI transfer using DMA
    ///
    /// Sends the data in `buffer` and writes the received data into buffer
    /// right after. Returns a [`Transfer`], to represent the ongoing SPI
    /// transfer.
    ///
    /// Please note that the word "transfer" is used with two different meanings
    /// here:
    /// - An SPI transfer, as in an SPI transaction that involves both sending
    ///   and receiving data. The method name refers to this kind of transfer.
    /// - A DMA transfer, as in an ongoing DMA operation. The name of the return
    ///   type refers to this kind of transfer.
    ///
    /// This method, as well as all other DMA-related methods in this module,
    /// requires references to two DMA handles, one each for the RX and TX
    /// streams. This will actually always be the same handle, as each SPI
    /// instance uses the same DMA instance for both sending and receiving. It
    /// would be nice to simplify that, but I believe that requires an equality
    /// constraint in the where clause, which is not supported yet by the
    /// compiler.
    pub fn transfer_all<B>(
        self,
        buffer: Pin<B>,
        dma_rx: &dma::Handle<<Rx<I> as dma::Target>::Instance, state::Enabled>,
        dma_tx: &dma::Handle<<Tx<I> as dma::Target>::Instance, state::Enabled>,
        rx: <Rx<I> as dma::Target>::Stream,
        tx: <Tx<I> as dma::Target>::Stream,
    ) -> Transfer<Word, I, P, B, Rx<I>, Tx<I>, dma::Ready>
    where
        Rx<I>: dma::Target,
        Tx<I>: dma::Target,
        B: DerefMut + 'static,
        B::Target: AsMutSlice<Element = Word>,
    {
        // Create the RX/TX tokens for the transfer. Those must only exist once,
        // otherwise it would be possible to create multiple transfers trying to
        // use the same hardware resources.
        //
        // We guarantee that they only exist once by only creating them where we
        // have access to `self`, moving `self` into the `Transfer` while they
        // are in use, and dropping them when returning `self` from the
        // transfer.
        let rx_token = Rx(PhantomData);
        let tx_token = Tx(PhantomData);

        // We need to move a buffer into each of the `dma::Transfer` instances,
        // while keeping the original buffer around to return to the caller
        // later, when the transfer is finished.
        //
        // Here we create two `Buffer` from raw pointers acquired from `buffer`.
        let rx_buffer = dma::PtrBuffer {
            ptr: buffer.as_slice().as_ptr(),
            len: buffer.as_slice().len(),
        };
        let tx_buffer = dma::PtrBuffer {
            ptr: buffer.as_slice().as_ptr(),
            len: buffer.as_slice().len(),
        };

        // Create the two DMA transfers. This is safe, for the following
        // reasons:
        // 1. The trait bounds on this method guarantee that `buffer`, which we
        //    created the two buffer instances from, can be safely read from and
        //    written to.
        // 2. The semantics of the SPI peripheral guarantee that the buffer
        //    reads/writes are synchronized, preventing race conditions.
        let rx_transfer = unsafe {
            dma::Transfer::new(
                dma_rx,
                rx,
                Pin::new(rx_buffer),
                rx_token,
                self.spi.dr_address(),
                dma::Direction::PeripheralToMemory,
            )
        };
        let tx_transfer = unsafe {
            dma::Transfer::new(
                dma_tx,
                tx,
                Pin::new(tx_buffer),
                tx_token,
                self.spi.dr_address(),
                dma::Direction::MemoryToPeripheral,
            )
        };

        Transfer {
            buffer,
            target: self,

            rx: rx_transfer,
            tx: tx_transfer,

            _state: dma::Ready,
        }
    }
}

impl<I, P, Word> FullDuplex<Word> for Spi<I, P, Enabled<Word>>
where
    I: Instance,
    P: Pins<I>,
    Word: SupportedWordSize,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<Word, Self::Error> {
        self.spi.read()
    }

    fn send(&mut self, word: Word) -> nb::Result<(), Self::Error> {
        self.spi.send(word)
    }
}

impl<I, P, Word> transfer::Default<Word> for Spi<I, P, Enabled<Word>>
where
    I: Instance,
    P: Pins<I>,
    Word: SupportedWordSize,
{
}

impl<I, P, Word> write::Default<Word> for Spi<I, P, Enabled<Word>>
where
    I: Instance,
    P: Pins<I>,
    Word: SupportedWordSize,
{
}

impl<I, P, Word> write_iter::Default<Word> for Spi<I, P, Enabled<Word>>
where
    I: Instance,
    P: Pins<I>,
    Word: SupportedWordSize,
{
}

impl<I, P, State> Spi<I, P, State>
where
    I: Instance,
    P: Pins<I>,
{
    /// Destroy the peripheral API and return a raw SPI peripheral instance
    pub fn free(self) -> (I, P) {
        (self.spi, self.pins)
    }
}

/// Implemented for all instances of the SPI peripheral
///
/// Users of this crate should not implement this trait.
pub trait Instance {
    fn configure<Word>(&self, br: u8, cpol: bool, cpha: bool)
    where
        Word: SupportedWordSize;
    fn read<Word>(&self) -> nb::Result<Word, Error>
    where
        Word: SupportedWordSize;
    fn send<Word>(&self, word: Word) -> nb::Result<(), Error>
    where
        Word: SupportedWordSize;
    fn dr_address(&self) -> u32;
}

/// Implemented for all tuples that contain a full set of valid SPI pins
pub trait Pins<I> {}

impl<I, SCK, MISO, MOSI> Pins<I> for (SCK, MISO, MOSI)
where
    SCK: Sck<I>,
    MISO: Miso<I>,
    MOSI: Mosi<I>,
{
}

/// Implemented for all pins that can function as the SCK pin
///
/// Users of this crate should not implement this trait.
pub trait Sck<I> {}

/// Implemented for all pins that can function as the MISO pin
///
/// Users of this crate should not implement this trait.
pub trait Miso<I> {}

/// Implemented for all pins that can function as the MOSI pin
///
/// Users of this crate should not implement this trait.
pub trait Mosi<I> {}

macro_rules! impl_instance {
    (
        $(
            $name:ty {
                regs: ($bus:ident, $reset:ident, $enable:ident),
                pins: {
                    SCK: [$($sck:ty,)*],
                    MISO: [$($miso:ty,)*],
                    MOSI: [$($mosi:ty,)*],
                }
            }
        )*
    ) => {
        $(
            impl Instance for $name {

                // I don't like putting this much code into the macro, but I
                // have to: There are two different SPI variants in the PAC, and
                // while I haven't found any actual differences between them,
                // they're still using different sets of types, and I need to
                // generate different methods to interface with them, even
                // though these methods end up looking identical.
                //
                // Maybe this is a problem in the SVD file that can be fixed
                // there.

                fn configure<Word>(&self, br: u8, cpol: bool, cpha: bool)
                    where Word: SupportedWordSize
                {
                    self.cr2.write(|w| {
                        // Data size
                        //
                        // This is safe, as `Word::ds` returns an enum which can
                        // only encode valid variants for this field.
                        let w = unsafe { w.ds().bits(Word::ds().into()) };

                        w
                            // FIFO reception threshold.
                            .frxth().bit(Word::frxth().into())
                            // Disable TX buffer empty interrupt
                            .txeie().masked()
                            // Disable RX buffer not empty interrupt
                            .rxneie().masked()
                            // Disable error interrupt
                            .errie().masked()
                            // Frame format
                            .frf().motorola()
                            // NSS pulse management
                            .nssp().no_pulse()
                            // SS output
                            .ssoe().disabled()
                            // Enable DMA support
                            .txdmaen().enabled()
                            .rxdmaen().enabled()
                    });

                    self.cr1.write(|w|
                        w
                            // Use two lines for MISO/MOSI
                            .bidimode().unidirectional()
                            // Disable hardware CRC calculation
                            .crcen().disabled()
                            // Enable full-duplex mode
                            .rxonly().full_duplex()
                            // Manage slave select pin manually
                            .ssm().enabled()
                            .ssi().set_bit()
                            // Transmit most significant bit first
                            .lsbfirst().msbfirst()
                            // Set baud rate value
                            .br().bits(br)
                            // Select master mode
                            .mstr().master()
                            // Select clock polarity
                            .cpol().bit(cpol)
                            // Select clock phase
                            .cpha().bit(cpha)
                            // Enable SPI
                            .spe().enabled()
                    );
                }

                fn read<Word>(&self) -> nb::Result<Word, Error> {
                    let sr = self.sr.read();

                    // Check for errors
                    //
                    // This whole code should live in a method in `Error`, but
                    // this wouldn't be straight-forward, due to the different
                    // SPI types in the PAC, explained in more detail in
                    // another comment.
                    if sr.fre().is_error() {
                        return Err(nb::Error::Other(Error::FrameFormat));
                    }
                    if sr.ovr().is_overrun() {
                        return Err(nb::Error::Other(Error::Overrun));
                    }
                    if sr.modf().is_fault() {
                        return Err(nb::Error::Other(Error::ModeFault));
                    }

                    // Did we receive something?
                    if sr.rxne().is_not_empty() {
                        // It makes a difference whether we read this register
                        // as a `u8` or `u16`, so we can't use the standard way
                        // to access it. This is safe, as `&self.dr` is a
                        // memory-mapped register.
                        let value = unsafe {
                            ptr::read_volatile(
                                &self.dr as *const _ as *const _,
                            )
                        };

                        return Ok(value);
                    }

                    Err(nb::Error::WouldBlock)
                }

                fn send<Word>(&self, word: Word) -> nb::Result<(), Error> {
                    let sr = self.sr.read();

                    // Check for errors
                    //
                    // This whole code should live in a method in `Error`, but
                    // this wouldn't be straight-forward, due to the different
                    // SPI types in the PAC, explained in more detail in
                    // another comment.
                    if sr.fre().is_error() {
                        return Err(nb::Error::Other(Error::FrameFormat));
                    }
                    if sr.ovr().is_overrun() {
                        return Err(nb::Error::Other(Error::Overrun));
                    }
                    if sr.modf().is_fault() {
                        return Err(nb::Error::Other(Error::ModeFault));
                    }

                    // Can we write to the transmit buffer?
                    if sr.txe().is_empty() {
                        // It makes a difference whether we write a `u8` or
                        // `u16` to this register, so we can't use the standard
                        // way to access it. This is safe, as `&self.dr` is a
                        // memory-mapped register.
                        unsafe {
                            ptr::write_volatile(
                                &self.dr as *const _ as *mut _,
                                word,
                            );
                        }

                        return Ok(())
                    }

                    Err(nb::Error::WouldBlock)
                }

                fn dr_address(&self) -> u32 {
                    &self.dr as *const _ as _
                }
            }

            $(
                impl Sck<$name> for $sck {}
            )*

            $(
                impl Miso<$name> for $miso {}
            )*

            $(
                impl Mosi<$name> for $mosi {}
            )*
        )*
    }
}

impl_instance!(
    pac::SPI1 {
        regs: (apb2, spi1rst, spi1en),
        pins: {
            SCK: [
                gpio::PA5<Alternate<5>>,
                gpio::PB3<Alternate<5>>,
                gpio::PG11<Alternate<5>>,
            ],
            MISO: [
                gpio::PA6<Alternate<5>>,
                gpio::PB4<Alternate<5>>,
                gpio::PG9<Alternate<5>>,
            ],
            MOSI: [
                gpio::PA7<Alternate<5>>,
                gpio::PB5<Alternate<5>>,
                gpio::PD7<Alternate<5>>,
            ],
        }
    }
    pac::SPI2 {
        regs: (apb1, spi2rst, spi2en),
        pins: {
            SCK: [
                gpio::PA9<Alternate<5>>,
                gpio::PB10<Alternate<5>>,
                gpio::PB13<Alternate<5>>,
                gpio::PD3<Alternate<5>>,
                gpio::PI1<Alternate<5>>,
            ],
            MISO: [
                gpio::PB14<Alternate<5>>,
                gpio::PC2<Alternate<5>>,
                gpio::PI2<Alternate<5>>,
            ],
            MOSI: [
                gpio::PB15<Alternate<5>>,
                gpio::PC1<Alternate<5>>,
                gpio::PC3<Alternate<5>>,
                gpio::PI3<Alternate<5>>,
            ],
        }
    }
    pac::SPI3 {
        regs: (apb1, spi3rst, spi3en),
        pins: {
            SCK: [
                gpio::PB3<Alternate<6>>,
                gpio::PC10<Alternate<6>>,
            ],
            MISO: [
                gpio::PB4<Alternate<6>>,
                gpio::PC11<Alternate<6>>,
            ],
            MOSI: [
                gpio::PB2<Alternate<7>>,
                gpio::PB5<Alternate<6>>,
                gpio::PC12<Alternate<6>>,
                gpio::PD6<Alternate<5>>,
            ],
        }
    }
    pac::SPI4 {
        regs: (apb2, spi4rst, spi4en),
        pins: {
            SCK: [
                gpio::PE2<Alternate<5>>,
                gpio::PE12<Alternate<5>>,
            ],
            MISO: [
                gpio::PE5<Alternate<5>>,
                gpio::PE13<Alternate<5>>,
            ],
            MOSI: [
                gpio::PE6<Alternate<5>>,
                gpio::PE14<Alternate<5>>,
            ],
        }
    }
    pac::SPI5 {
        regs: (apb2, spi5rst, spi5en),
        pins: {
            SCK: [
                gpio::PF7<Alternate<5>>,
                gpio::PH6<Alternate<5>>,
            ],
            MISO: [
                gpio::PF8<Alternate<5>>,
                gpio::PH7<Alternate<5>>,
            ],
            MOSI: [
                gpio::PF9<Alternate<5>>,
                gpio::PF11<Alternate<5>>,
            ],
        }
    }
);

#[cfg(any(
    feature = "stm32f745",
    feature = "stm32f746",
    feature = "stm32f756",
    feature = "stm32f765",
    feature = "stm32f767",
    feature = "stm32f769",
    feature = "stm32f777",
    feature = "stm32f778",
    feature = "stm32f779",
))]
impl_instance!(
    pac::SPI6 {
        regs: (apb2, spi6rst, spi6en),
        pins: {
            SCK: [
                gpio::PG13<Alternate<5>>,
            ],
            MISO: [
                gpio::PG12<Alternate<5>>,
            ],
            MOSI: [
                gpio::PG14<Alternate<5>>,
            ],
        }
    }
);

/// Placeholder for a pin when no SCK pin is required
pub struct NoSck;
impl<I> Sck<I> for NoSck {}

/// Placeholder for a pin when no MISO pin is required
pub struct NoMiso;
impl<I> Miso<I> for NoMiso {}

/// Placeholder for a pin when no MOSI pin is required
pub struct NoMosi;
impl<I> Mosi<I> for NoMosi {}

#[derive(Debug)]
pub enum Error {
    FrameFormat,
    Overrun,
    ModeFault,
}

/// RX token used for DMA transfers
pub struct Rx<I>(PhantomData<I>);

/// TX token used for DMA transfers
pub struct Tx<I>(PhantomData<I>);

/// A DMA transfer of the SPI peripheral
///
/// Since DMA can send and receive at the same time, using two DMA transfers and
/// two DMA streams, we need this type to represent this operation and wrap the
/// underlying [`dma::Transfer`] instances.
pub struct Transfer<Word: SupportedWordSize, I, P, Buffer, Rx: dma::Target, Tx: dma::Target, State>
{
    buffer: Pin<Buffer>,
    target: Spi<I, P, Enabled<Word>>,
    rx: dma::Transfer<Rx, dma::PtrBuffer<Word>, State>,
    tx: dma::Transfer<Tx, dma::PtrBuffer<Word>, State>,
    _state: State,
}

impl<Word, I, P, Buffer, Rx, Tx> Transfer<Word, I, P, Buffer, Rx, Tx, dma::Ready>
where
    Rx: dma::Target,
    Tx: dma::Target,
    Word: SupportedWordSize,
{
    /// Enables the given interrupts for this DMA transfer
    ///
    /// These interrupts are only enabled for this transfer. The settings
    /// doesn't affect other transfers, nor subsequent transfers using the same
    /// DMA streams.
    pub fn enable_interrupts(
        &mut self,
        rx_handle: &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle: &dma::Handle<Tx::Instance, state::Enabled>,
        interrupts: dma::Interrupts,
    ) {
        self.rx.enable_interrupts(rx_handle, interrupts);
        self.tx.enable_interrupts(tx_handle, interrupts);
    }

    /// Start the DMA transfer
    ///
    /// Consumes this instance of `Transfer` and returns another instance with
    /// its type state set to indicate the transfer has been started.
    pub fn start(
        self,
        rx_handle: &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle: &dma::Handle<Tx::Instance, state::Enabled>,
    ) -> Transfer<Word, I, P, Buffer, Rx, Tx, dma::Started> {
        Transfer {
            buffer: self.buffer,
            target: self.target,
            rx: self.rx.start(rx_handle),
            tx: self.tx.start(tx_handle),
            _state: dma::Started,
        }
    }
}

impl<Word, I, P, Buffer, Rx, Tx> Transfer<Word, I, P, Buffer, Rx, Tx, dma::Started>
where
    Rx: dma::Target,
    Tx: dma::Target,
    Word: SupportedWordSize,
{
    /// Checks whether the transfer is still ongoing
    pub fn is_active(
        &self,
        rx_handle: &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle: &dma::Handle<Tx::Instance, state::Enabled>,
    ) -> bool {
        self.rx.is_active(rx_handle) || self.tx.is_active(tx_handle)
    }

    /// Waits for the transfer to end
    ///
    /// This method will block if the transfer is still ongoing. If you want
    /// this method to return immediately, first check whether the transfer is
    /// still ongoing by calling `is_active`.
    ///
    /// An ongoing transfer needs exlusive access to some resources, namely the
    /// data buffer, the DMA stream, and the peripheral. Those have been moved
    /// into the `Transfer` instance to prevent concurrent access to them. This
    /// method returns those resources, so they can be used again.
    pub fn wait(
        self,
        rx_handle: &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle: &dma::Handle<Tx::Instance, state::Enabled>,
    ) -> WaitResult<Word, I, P, Rx, Tx, Buffer> {
        let (rx_res, rx_err) = match self.rx.wait(rx_handle) {
            Ok(res) => (res, None),
            Err((res, err)) => (res, Some(err)),
        };
        let (tx_res, tx_err) = match self.tx.wait(tx_handle) {
            Ok(res) => (res, None),
            Err((res, err)) => (res, Some(err)),
        };

        let res = TransferResources {
            rx_stream: rx_res.stream,
            tx_stream: tx_res.stream,
            target: self.target,
            buffer: self.buffer,
        };

        if let Some(err) = rx_err {
            return Err((res, err));
        }
        if let Some(err) = tx_err {
            return Err((res, err));
        }

        Ok(res)
    }
}

/// Returned by [`Transfer::wait`]
pub type WaitResult<Word, I, P, Rx, Tx, Buffer> = Result<
    TransferResources<Word, I, P, Rx, Tx, Buffer>,
    (TransferResources<Word, I, P, Rx, Tx, Buffer>, dma::Error),
>;

/// The resources that an ongoing transfer needs exclusive access to
pub struct TransferResources<Word, I, P, Rx: dma::Target, Tx: dma::Target, Buffer> {
    pub rx_stream: Rx::Stream,
    pub tx_stream: Tx::Stream,
    pub target: Spi<I, P, Enabled<Word>>,
    pub buffer: Pin<Buffer>,
}

// As `TransferResources` is used in the error variant of `Result`, it needs a
// `Debug` implementation to enable stuff like `unwrap` and `expect`. This can't
// be derived without putting requirements on the type arguments.
impl<Word, I, P, Rx, Tx, Buffer> fmt::Debug for TransferResources<Word, I, P, Rx, Tx, Buffer>
where
    Rx: dma::Target,
    Tx: dma::Target,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "TransferResources {{ .. }}")
    }
}

/// Indicates that the SPI peripheral is enabled
///
/// The `Word` type parameter indicates which word size the peripheral is
/// configured for.
pub struct Enabled<Word>(PhantomData<Word>);

pub trait SupportedWordSize: dma::SupportedWordSize + private::Sealed {
    fn frxth() -> cr2::FRXTH_A;
    fn ds() -> cr2::DS_A;
}

impl private::Sealed for u8 {}
impl SupportedWordSize for u8 {
    fn frxth() -> cr2::FRXTH_A {
        cr2::FRXTH_A::QUARTER
    }

    fn ds() -> cr2::DS_A {
        cr2::DS_A::EIGHTBIT
    }
}

impl private::Sealed for u16 {}
impl SupportedWordSize for u16 {
    fn frxth() -> cr2::FRXTH_A {
        cr2::FRXTH_A::HALF
    }

    fn ds() -> cr2::DS_A {
        cr2::DS_A::SIXTEENBIT
    }
}

mod private {
    /// Prevents code outside of the parent module from implementing traits
    ///
    /// This trait is located in a module that is not accessible outside of the
    /// parent module. This means that any trait that requires `Sealed` cannot
    /// be implemented only in the parent module.
    pub trait Sealed {}
}
