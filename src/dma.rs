//! Interface the the DMA peripheral

use core::{
    fmt,
    marker::PhantomData,
    ops::Deref,
    pin::Pin,
    sync::atomic::{self, Ordering},
};

use as_slice::AsSlice;

use crate::{
    pac::{
        self,
        dma2::{self, st::cr},
        Interrupt, DMA1, DMA2, NVIC,
    },
    rcc::Rcc,
    serial, spi, state,
};

/// Entry point to the DMA API
pub struct DMA<I> {
    /// Handle to the DMA instance
    pub handle: Handle<I, state::Disabled>,

    /// The streams associated with this DMA instance
    pub streams: Streams<I>,
}

impl<I> DMA<I>
where
    I: Instance,
{
    /// Creates a new instance of `DMA`
    ///
    /// This just wraps the PAC API, but does no initialization.
    pub fn new(instance: I) -> Self {
        DMA {
            handle: Handle::new(instance),
            streams: Streams::new(),
        }
    }
}

/// Handle to the DMA instance
///
/// Controls access to the DMA registers and makes sure that access from
/// multiple streams can not conflict.
pub struct Handle<I, State> {
    dma: I,
    _state: State,
}

impl<I> Handle<I, state::Disabled>
where
    I: Instance,
{
    fn new(instance: I) -> Self {
        Self {
            dma: instance,
            _state: state::Disabled,
        }
    }

    /// Initializes the DMA instance
    pub fn enable(self, rcc: &mut Rcc) -> Handle<I, state::Enabled> {
        I::enable(rcc);

        Handle {
            dma: self.dma,
            _state: state::Enabled,
        }
    }
}

/// Represents an ongoing DMA transfer
///
/// Peripheral APIs that support DMA have methods like `write_all` and
/// `read_all`, which return instances of this struct.
pub struct Transfer<T: Target, B, State> {
    res: TransferResources<T, B>,
    _state: State,
}

impl<T, B> Transfer<T, B, Ready>
where
    T: Target,
    B: 'static,
{
    /// Internal constructor to create a new `Transfer`
    ///
    /// # Safety
    ///
    /// [`Buffer`] can be used to acquire a raw pointer and a length. This
    /// defines a memory region, i.e. "the buffer".
    ///
    /// If this method is used to prepare a memory-to-peripheral transfer, the
    /// caller must make sure that the buffer can be read from safely.
    ///
    /// If this method is used to prepare a peripheral-to-memory transfer, the
    /// caller must make sure that the buffer can be written to safely.
    pub(crate) unsafe fn new<Word>(
        handle: &Handle<T::Instance, state::Enabled>,
        stream: T::Stream,
        buffer: Pin<B>,
        target: T,
        address: u32,
        direction: Direction,
    ) -> Self
    where
        B: Deref,
        B::Target: Buffer<Word>,
        Word: SupportedWordSize,
    {
        assert!(buffer.len() <= u16::max_value() as usize);

        // The following configuration procedure is documented in the reference
        // manual for STM32F75xxx and STM32F74xxx, section 8.3.18.

        let nr = T::Stream::number();

        // Disable stream
        handle.dma.st[nr].cr.modify(|_, w| w.en().disabled());
        while handle.dma.st[nr].cr.read().en().is_enabled() {}

        T::Stream::clear_status_flags(&handle.dma);

        // Set peripheral port register address
        handle.dma.st[nr].par.write(|w| w.pa().bits(address));

        // Set memory address
        let memory_address = buffer.as_ptr() as u32;
        handle.dma.st[nr]
            .m0ar
            .write(|w| w.m0a().bits(memory_address));

        // Write number of data items to transfer
        //
        // We've asserted that `data.len()` fits into a `u16`, so the cast
        // should be fine.
        handle.dma.st[nr]
            .ndtr
            .write(|w| w.ndt().bits(buffer.len() as u16));

        // Configure FIFO
        handle.dma.st[nr].fcr.modify(|_, w| {
            w
                // Interrupt disabled
                .feie()
                .disabled()
                // Direct mode enabled (FIFO disabled)
                .dmdis()
                .enabled()
        });

        // Select channel
        handle.dma.st[nr].cr.write(|w| {
            let w = T::Channel::select(w);

            let w = match direction {
                Direction::MemoryToPeripheral => w.dir().memory_to_peripheral(),
                Direction::PeripheralToMemory => w.dir().peripheral_to_memory(),
            };

            w
                // Single transfer
                .mburst()
                .single()
                .pburst()
                .single()
                // Double-buffer mode disabled
                .dbm()
                .disabled()
                // Very high priority
                .pl()
                .very_high()
                // Memory data size
                .msize()
                .variant(Word::msize())
                // Peripheral data size
                .psize()
                .variant(Word::psize())
                // Memory increment mode
                .minc()
                .incremented()
                // Peripheral increment mode
                .pinc()
                .fixed()
                // Circular mode disabled
                .circ()
                .disabled()
                // DMA is the flow controller
                .pfctrl()
                .dma()
                // All interrupts disabled
                .tcie()
                .disabled()
                .htie()
                .disabled()
                .teie()
                .disabled()
                .dmeie()
                .disabled()
        });

        Transfer {
            res: TransferResources {
                stream,
                buffer,
                target,
            },
            _state: Ready,
        }
    }

    /// Enables the given interrupts for this DMA transfer
    ///
    /// These interrupts are only enabled for this transfer. The settings
    /// doesn't affect other transfers, nor subsequent transfers using the same
    /// DMA stream.
    pub fn enable_interrupts(
        &mut self,
        handle: &Handle<T::Instance, state::Enabled>,
        interrupts: Interrupts,
    ) {
        handle.dma.st[T::Stream::number()].cr.modify(|_, w| {
            let w = if interrupts.transfer_complete {
                w.tcie().enabled()
            } else {
                w
            };

            let w = if interrupts.half_transfer {
                w.htie().enabled()
            } else {
                w
            };

            let w = if interrupts.transfer_error {
                w.teie().enabled()
            } else {
                w
            };

            let w = if interrupts.direct_mode_error {
                w.dmeie().enabled()
            } else {
                w
            };

            w
        });

        // Enable interrupt.
        unsafe { NVIC::unmask(T::INTERRUPT) };
    }

    /// Start the DMA transfer
    ///
    /// Consumes this instance of `Transfer` and returns another instance with
    /// its type state set to indicate the transfer has been started.
    pub fn start(self, handle: &Handle<T::Instance, state::Enabled>) -> Transfer<T, B, Started> {
        atomic::fence(Ordering::SeqCst);

        handle.dma.st[T::Stream::number()]
            .cr
            .modify(|_, w| w.en().enabled());

        Transfer {
            res: self.res,
            _state: Started,
        }
    }
}

impl<T, B> Transfer<T, B, Started>
where
    T: Target,
{
    /// Checks whether the transfer is still ongoing
    pub fn is_active(&self, handle: &Handle<T::Instance, state::Enabled>) -> bool {
        handle.dma.st[T::Stream::number()]
            .cr
            .read()
            .en()
            .is_enabled()
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
        handle: &Handle<T::Instance, state::Enabled>,
    ) -> Result<TransferResources<T, B>, (TransferResources<T, B>, Error)> {
        // Disable interrupt.
        NVIC::mask(T::INTERRUPT);

        // Wait for transfer to finish
        while self.is_active(handle) {
            if let Err(error) = Error::check::<T::Stream>(&handle.dma) {
                return Err((self.res, error));
            }
        }

        atomic::fence(Ordering::SeqCst);

        if let Err(error) = Error::check::<T::Stream>(&handle.dma) {
            return Err((self.res, error));
        }

        Ok(self.res)
    }
}

/// The resources that an ongoing transfer needs exclusive access to
pub struct TransferResources<T: Target, B> {
    pub stream: T::Stream,
    pub buffer: Pin<B>,
    pub target: T,
}

// As `TransferResources` is used in the error variant of `Result`, it needs a
// `Debug` implementation to enable stuff like `unwrap` and `expect`. This can't
// be derived without putting requirements on the type arguments.
impl<T, B> fmt::Debug for TransferResources<T, B>
where
    T: Target,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "TransferResources {{ .. }}")
    }
}

pub(crate) enum Direction {
    MemoryToPeripheral,
    PeripheralToMemory,
}

/// Implemented for all peripheral APIs that support DMA transfers
///
/// This is an internal trait. End users neither need to implement it, nor use
/// it directly.
pub trait Target {
    type Instance: Deref<Target = dma2::RegisterBlock>;
    type Stream: Stream;
    type Channel: Channel;

    const INTERRUPT: Interrupt;
}

macro_rules! impl_target {
    (
        $(
            $ty:ty,
            $instance:ty,
            $stream:ident,
            $channel:ty,
            $interrupt:ident;
        )*
    ) => {
        $(
            impl Target for $ty {
                type Instance = $instance;
                type Stream   = $stream<$instance>;
                type Channel  = $channel;

                const INTERRUPT: Interrupt = Interrupt::$interrupt;
            }
        )*
    }
}

// See section 8.3.4, tables 25 and 26
//
// Some peripherals can be used with multiple stream/channel combinations. To
// model this, we'd need to implement `Tx` multiple times, which means `Tx`
// would need a type parameter. When attempting to do this, I ran into errors
// about unconstrained type parameters when trying to use `Tx`.
//
// There's probably a smart way to achieve this, but I decided to declare
// victory and leave this problem to someone who actually needs this capability.
impl_target!(
    // SPI receive
    spi::Rx<pac::SPI1>, DMA2, Stream0, Channel3, DMA2_STREAM0;
    // SPI1 for DMA2, stream 2, channel 3 is unsupported
    spi::Rx<pac::SPI2>, DMA1, Stream3, Channel0, DMA1_STREAM3;
    spi::Rx<pac::SPI3>, DMA1, Stream0, Channel0, DMA1_STREAM0;
    // SPI3 for DMA1, stream 2, channel 0 is unsupported
    spi::Rx<pac::SPI4>, DMA2, Stream0, Channel4, DMA2_STREAM0;
    // SPI4 for DMA2, stream 3, channel 5 is unsupported
    spi::Rx<pac::SPI5>, DMA2, Stream3, Channel2, DMA2_STREAM3;
    // SPI5 for DMA2, stream 5, channel 7 is unsupported

    // SPI transmit
    spi::Tx<pac::SPI1>, DMA2, Stream3, Channel3, DMA2_STREAM3;
    // SPI1 for DMA2, stream 5, channel 3 is unsupported
    spi::Tx<pac::SPI2>, DMA1, Stream4, Channel0, DMA1_STREAM4;
    spi::Tx<pac::SPI3>, DMA1, Stream5, Channel0, DMA1_STREAM5;
    // SPI3 for DMA1, stream 7, channel 0 is unsupported
    spi::Tx<pac::SPI4>, DMA2, Stream1, Channel4, DMA2_STREAM1;
    // SPI4 for DMA2, stream 4, channel 5 is unsupported
    spi::Tx<pac::SPI5>, DMA2, Stream4, Channel2, DMA2_STREAM4;
    // SPI5 for DMA2, stream 6, channel 7 is unsupported

    // USART receive
    serial::Rx<pac::USART1>, DMA2, Stream2, Channel4, DMA2_STREAM2;
    // USART1 for DMA2, stream 5, channel 4 is unsupported
    serial::Rx<pac::USART2>, DMA1, Stream5, Channel4, DMA1_STREAM5;
    serial::Rx<pac::USART3>, DMA1, Stream1, Channel4, DMA1_STREAM1;
    serial::Rx<pac::UART4>, DMA1, Stream2, Channel4, DMA1_STREAM2;
    serial::Rx<pac::UART5>, DMA1, Stream0, Channel4, DMA1_STREAM0;
    serial::Rx<pac::USART6>, DMA2, Stream1, Channel5, DMA2_STREAM1;
    // USART6 for DMA2, stream 2, channel 5 is unsupported
    serial::Rx<pac::UART7>, DMA1, Stream3, Channel5, DMA1_STREAM3;
    serial::Rx<pac::UART8>, DMA1, Stream6, Channel5, DMA1_STREAM6;

    // USART transmit
    serial::Tx<pac::USART1>, DMA2, Stream7, Channel4, DMA2_STREAM7;
    serial::Tx<pac::USART2>, DMA1, Stream6, Channel4, DMA1_STREAM6;
    serial::Tx<pac::USART3>, DMA1, Stream3, Channel4, DMA1_STREAM3;
    // USART3 for DMA1, stream 4, channel 7 is unsupported
    serial::Tx<pac::UART4>,  DMA1, Stream4, Channel4, DMA1_STREAM4;
    serial::Tx<pac::UART5>,  DMA1, Stream7, Channel4, DMA1_STREAM7;
    serial::Tx<pac::USART6>, DMA2, Stream6, Channel5, DMA2_STREAM6;
    // USART6 for DMA2, stream 7, channel 5 is unsupported
    serial::Tx<pac::UART7>,  DMA1, Stream1, Channel5, DMA1_STREAM1;
    serial::Tx<pac::UART8>,  DMA1, Stream0, Channel5, DMA1_STREAM0;
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
impl_target!(
    spi::Rx<pac::SPI6>, DMA2, Stream6, Channel1, DMA2_STREAM6;
    spi::Tx<pac::SPI6>, DMA2, Stream5, Channel1, DMA2_STREAM5;
);

/// Implemented for all types that represent DMA streams
///
/// This is an internal trait. End users neither need to implement it, nor use
/// it directly.
pub trait Stream {
    fn number() -> usize;

    fn clear_status_flags(dma: &dma2::RegisterBlock);

    fn is_transfer_complete(dma: &dma2::RegisterBlock) -> bool;
    fn is_half_transfer(dma: &dma2::RegisterBlock) -> bool;
    fn is_transfer_error(dma: &dma2::RegisterBlock) -> bool;
    fn is_direct_mode_error(dma: &dma2::RegisterBlock) -> bool;
    fn is_fifo_error(dma: &dma2::RegisterBlock) -> bool;
}

macro_rules! impl_stream {
    (
        $(
            $name:ident,
            $name_lower:ident,
            $number:expr,
            $flag_reg:ident,
            $feif:ident,
            $dmeif:ident,
            $teif:ident,
            $htif:ident,
            $tcif:ident,
            $flag_clear_reg:ident,
            ($($flag_clear_field:ident,)*);
        )*
    ) => {
        pub struct Streams<I> {
            $(pub $name_lower: $name<I>,)*
        }

        impl<I> Streams<I> {
            fn new() -> Self {
                Self {
                    $($name_lower: $name(PhantomData),)*
                }
            }
        }


        $(
            pub struct $name<I>(PhantomData<I>);

            impl<I> Stream for $name<I> {
                fn number() -> usize { $number }

                fn clear_status_flags(dma: &dma2::RegisterBlock) {
                    dma.$flag_clear_reg.write(|w|
                        w
                            $(.$flag_clear_field().clear())*
                    );
                }

                fn is_transfer_complete(dma: &dma2::RegisterBlock) -> bool {
                    dma.$flag_reg.read().$tcif().is_complete()
                }
                fn is_half_transfer(dma: &dma2::RegisterBlock) -> bool {
                    dma.$flag_reg.read().$htif().is_half()
                }
                fn is_transfer_error(dma: &dma2::RegisterBlock) -> bool {
                    dma.$flag_reg.read().$teif().is_error()
                }
                fn is_direct_mode_error(dma: &dma2::RegisterBlock) -> bool {
                    dma.$flag_reg.read().$dmeif().is_error()
                }
                fn is_fifo_error(dma: &dma2::RegisterBlock) -> bool {
                    dma.$flag_reg.read().$feif().is_error()
                }
            }
        )*
    }
}

impl_stream!(
    Stream0, stream0, 0,
        lisr, feif0, dmeif0, teif0, htif0, tcif0,
        lifcr, (cfeif0, cdmeif0, cteif0, chtif0, ctcif0,);
    Stream1, stream1, 1,
        lisr, feif1, dmeif1, teif1, htif1, tcif1,
        lifcr, (cfeif1, cdmeif1, cteif1, chtif1, ctcif1,);
    Stream2, stream2, 2,
        lisr, feif2, dmeif2, teif2, htif2, tcif2,
        lifcr, (cfeif2, cdmeif2, cteif2, chtif2, ctcif2,);
    Stream3, stream3, 3,
        lisr, feif3, dmeif3, teif3, htif3, tcif3,
        lifcr, (cfeif3, cdmeif3, cteif3, chtif3, ctcif3,);
    Stream4, stream4, 4,
        hisr, feif4, dmeif4, teif4, htif4, tcif4,
        hifcr, (cfeif4, cdmeif4, cteif4, chtif4, ctcif4,);
    Stream5, stream5, 5,
        hisr, feif5, dmeif5, teif5, htif5, tcif5,
        hifcr, (cfeif5, cdmeif5, cteif5, chtif5, ctcif5,);
    Stream6, stream6, 6,
        hisr, feif6, dmeif6, teif6, htif6, tcif6,
        hifcr, (cfeif6, cdmeif6, cteif6, chtif6, ctcif6,);
    Stream7, stream7, 7,
        hisr, feif7, dmeif7, teif7, htif7, tcif7,
        hifcr, (cfeif7, cdmeif7, cteif7, chtif7, ctcif7,);
);

/// Implemented for all types that represent DMA channels
///
/// This is an internal trait. End users neither need to implement it, nor use
/// it directly.
pub trait Channel {
    fn select<'r>(w: &'r mut dma2::st::cr::W) -> &'r mut dma2::st::cr::W;
}

macro_rules! impl_channel {
    ($($name:ident, $number:expr;)*) => {
        $(
            pub struct $name;

            impl Channel for $name {
                fn select<'r>(w: &'r mut dma2::st::cr::W)
                    -> &'r mut dma2::st::cr::W
                {
                    // This is safe, as long as the macro caller passes in valid
                    // channel numbers.
                    w.chsel().bits($number)
                }
            }
        )*
    }
}

impl_channel!(
    Channel0, 0;
    Channel1, 1;
    Channel2, 2;
    Channel3, 3;
    Channel4, 4;
    Channel5, 5;
    Channel6, 6;
    Channel7, 7;
);

/// Implemented for all DMA instances
///
/// This is an internal trait. End users neither need to implement it, nor use
/// it directly.
pub trait Instance {
    fn enable(rcc: &mut Rcc);
}

macro_rules! impl_instance {
    ($($name:ty, $reset_reg:ident, $enable_reg:ident;)*) => {
        $(
            impl Instance for $name {
                fn enable(rcc: &mut Rcc) {
                    rcc.ahb1.rstr().modify(|_, w| w.$reset_reg().clear_bit());
                    rcc.ahb1.enr().modify(|_, w| w.$enable_reg().enabled());
                }
            }
        )*
    }
}

impl_instance!(
    DMA1, dma1rst, dma1en;
    DMA2, dma2rst, dma2en;
);

/// Used by [`Transfer::enable_interrupts`] to identify DMA interrupts
#[derive(Clone, Copy)]
pub struct Interrupts {
    pub transfer_complete: bool,
    pub half_transfer: bool,
    pub transfer_error: bool,
    pub direct_mode_error: bool,
}

impl Default for Interrupts {
    fn default() -> Self {
        Self {
            transfer_complete: false,
            half_transfer: false,
            transfer_error: false,
            direct_mode_error: false,
        }
    }
}

/// A DMA error
#[derive(Debug)]
pub enum Error {
    Transfer,
    DirectMode,
}

impl Error {
    pub(crate) fn check<S>(dma: &dma2::RegisterBlock) -> Result<(), Self>
    where
        S: Stream,
    {
        if S::is_transfer_error(dma) {
            return Err(Error::Transfer);
        }
        if S::is_direct_mode_error(dma) {
            return Err(Error::DirectMode);
        }
        // We're not checking for FIFO errors here. FIFO mode is not enabled
        // anyway, but the error flag is still set for some reason, even though
        // everything works.

        Ok(())
    }
}

/// Indicates that a DMA transfer is ready to be started
pub struct Ready;

/// Indicates that a DMA transfer has been started
pub struct Started;

/// Implemented for types that can be used as a buffer for DMA transfers
pub(crate) trait Buffer<Word> {
    fn as_ptr(&self) -> *const Word;
    fn len(&self) -> usize;
}

impl<T, Word> Buffer<Word> for T
where
    T: ?Sized + AsSlice<Element = Word>,
{
    fn as_ptr(&self) -> *const Word {
        self.as_slice().as_ptr()
    }

    fn len(&self) -> usize {
        self.as_slice().len()
    }
}

/// Can be used as a fallback [`Buffer`], if safer implementations can't be used
///
/// The `ptr` and `len` fields MUST define a valid memory region.
pub(crate) struct PtrBuffer<Word: SupportedWordSize> {
    pub ptr: *const Word,
    pub len: usize,
}

// Required to make in possible to put this in a `Pin`, in a way that satisfies
// the requirements on `Transfer::new`.
impl<Word> Deref for PtrBuffer<Word>
where
    Word: SupportedWordSize,
{
    type Target = Self;

    fn deref(&self) -> &Self::Target {
        self
    }
}

impl<Word> Buffer<Word> for PtrBuffer<Word>
where
    Word: SupportedWordSize,
{
    fn as_ptr(&self) -> *const Word {
        self.ptr
    }

    fn len(&self) -> usize {
        self.len
    }
}

pub trait SupportedWordSize: private::Sealed + Unpin + 'static {
    fn msize() -> cr::MSIZE_A;
    fn psize() -> cr::PSIZE_A;
}

impl private::Sealed for u8 {}
impl SupportedWordSize for u8 {
    fn msize() -> cr::MSIZE_A {
        cr::MSIZE_A::BITS8
    }

    fn psize() -> cr::PSIZE_A {
        cr::MSIZE_A::BITS8
    }
}

impl private::Sealed for u16 {}
impl SupportedWordSize for u16 {
    fn msize() -> cr::MSIZE_A {
        cr::MSIZE_A::BITS16
    }

    fn psize() -> cr::PSIZE_A {
        cr::MSIZE_A::BITS16
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
