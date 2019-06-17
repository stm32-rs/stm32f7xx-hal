//! Interface the the DMA peripheral


use core::{
    fmt,
    marker::PhantomData,
    ops::Deref,
    sync::atomic::{
        self,
        Ordering,
    },
};

use crate::{
    device::{
        dma2,
        DMA1,
        DMA2,
        USART1,
        USART2,
        USART3,
        UART4,
        UART5,
        USART6,
        UART7,
        UART8,
    },
    rcc::Rcc,
    serial,
};


/// Entry point to the DMA API
pub struct DMA<I> {
    /// Handle to the DMA instance
    pub handle: Handle<I, Disabled>,

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
            handle:  Handle::new(instance),
            streams: Streams::new(),
        }
    }
}


/// Handle to the DMA instance
///
/// Controls access to the DMA registers and makes sure that access from
/// multiple streams can not conflict.
pub struct Handle<I, State> {
    dma:    I,
    _state: State,
}

impl<I> Handle<I, Disabled>
    where
        I: Instance,
{
    fn new(instance: I) -> Self {
        Self {
            dma:    instance,
            _state: Disabled,
        }
    }

    /// Initializes the DMA instance
    pub fn enable(self, rcc: &mut Rcc) -> Handle<I, Enabled> {
        I::enable(rcc);

        Handle {
            dma:    self.dma,
            _state: Enabled,
        }
    }
}


/// Represents an ongoing DMA transfer
///
/// Peripheral APIs that support DMA have methods like `write_all` and
/// `read_all`, which return instances of this struct.
pub struct Transfer<Target: Tx> {
    res: TransferResources<Target>,
}

impl<Target> Transfer<Target> where Target: Tx {
    pub(crate) fn prepare(
        handle:  &Handle<Target::Instance, Enabled>,
        stream:  Target::Stream,
        source:  &'static [u8],
        target:  Target,
        address: u32,
    )
        -> Transfer<Target>
    {
        assert!(source.len() <= u16::max_value() as usize);

        // The following configuration procedure is documented in the reference
        // manual for STM32F75xxx and STM32F74xxx, section 8.3.18.

        let nr = Target::Stream::number();

        // Disable stream
        handle.dma.st[nr].cr.modify(|_, w| w.en().disabled());
        while handle.dma.st[nr].cr.read().en().is_enabled() {}

        Target::Stream::clear_status_flags(&handle.dma);

        // Set peripheral port register address
        handle.dma.st[nr].par.write(|w| w.pa().bits(address));

        // Set memory address
        let memory_address = source.as_ptr() as *const _ as u32;
        handle.dma.st[nr].m0ar.write(|w| w.m0a().bits(memory_address));

        // Write number of data items to transfer
        //
        // We've asserted that `data.len()` fits into a `u16`, so the cast
        // should be fine.
        handle.dma.st[nr].ndtr.write(|w| w.ndt().bits(source.len() as u16));

        // Configure FIFO
        handle.dma.st[nr].fcr.modify(|_, w|
            w
                // Interrupt disabled
                .feie().disabled()
                // Direct mode enabled (FIFO disabled)
                .dmdis().enabled()
        );

        // Select channel
        handle.dma.st[nr].cr.modify(|_, w| {
            let w = Target::Channel::select(w);

            w
                // Single transfer
                .mburst().single()
                .pburst().single()
                // Double-buffer mode disabled
                .dbm().disabled()
                // Very high priority
                .pl().very_high()
                // Memory data size
                .msize().byte()
                // Peripheral data size
                .psize().byte()
                // Memory increment mode
                .minc().incremented()
                // Peripheral increment mode
                .pinc().fixed()
                // Circular mode disabled
                .circ().disabled()
                // Transfer from memory to peripheral
                .dir().memory_to_peripheral()
                // DMA is the flow controller
                .pfctrl().dma()
                // All interrupts disabled
                .tcie().disabled()
                .htie().disabled()
                .teie().disabled()
                .dmeie().disabled()
        });

        Transfer {
            res: TransferResources {
                stream,
                source,
                target,
            }
        }
    }

    pub fn start(&mut self, handle: &Handle<Target::Instance, Enabled>) {
        atomic::fence(Ordering::SeqCst);

        handle.dma.st[Target::Stream::number()].cr.modify(|_, w| {
            w.en().enabled()
        });
    }

    /// Checks whether the transfer is still ongoing
    pub fn is_active(&self, handle: &Handle<Target::Instance, Enabled>)
        -> bool
    {
        handle.dma.st[Target::Stream::number()].cr.read().en().is_enabled()
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
    pub fn wait(self, handle: &Handle<Target::Instance, Enabled>)
        -> Result<TransferResources<Target>, (TransferResources<Target>, Error)>
    {
        // Wait for transfer to finish
        while self.is_active(handle) {
            if let Err(error) = Error::check::<Target::Stream>(&handle.dma) {
                return Err((self.res, error));
            }
        }

        atomic::fence(Ordering::SeqCst);

        if let Err(error) = Error::check::<Target::Stream>(&handle.dma) {
            return Err((self.res, error));
        }

        Ok(self.res)
    }
}


/// The resources that an ongoing transfer needs exclusive access to.
pub struct TransferResources<Target: Tx> {
    pub stream: Target::Stream,
    pub source: &'static [u8],
    pub target: Target,
}

// As `TransferResources` is used in the error variant of `Result`, it needs a
// `Debug` implementation to enable stuff like `unwrap` and `expect`. This can't
// be derived without putting requirements on the type arguments.
impl<Target> fmt::Debug for TransferResources<Target> where Target: Tx {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "TransferResources {{ .. }}")
    }
}


/// Implemented for all peripheral APIs that support transmitting via USART
///
/// This is an internal trait. End users neither need to implement it, nor use
/// it directly.
pub trait Tx {
    type Instance: Deref<Target = dma2::RegisterBlock>;
    type Stream: Stream;
    type Channel: Channel;
}

macro_rules! impl_tx {
    ($($ty:ty, $instance:ty, $stream:ident, $channel:ty;)*) => {
        $(
            impl Tx for $ty {
                type Instance = $instance;
                type Stream   = $stream<$instance>;
                type Channel  = $channel;
            }
        )*
    }
}

// See section 8.3.4, tables 25 and 26
//
// Some USART instances can be used with multiple stream/channel combinations.
// To model this, we'd need to implement `Tx` multiple times, which means `Tx`
// would need a type parameter. When attempting to do this, I ran into errors
// about unconstrained type parameters when trying to use `Tx`.
//
// There's probably a smart way to achieve this, but I decided to declare
// victory and leave this problem to someone who actually needs this capability.
impl_tx!(
    serial::Tx<USART1>, DMA2, Stream7, Channel4;
    serial::Tx<USART2>, DMA1, Stream6, Channel4;
    serial::Tx<USART3>, DMA1, Stream3, Channel4;
    // USART3 for DMA1, stream 4, channel 7 is unsupported
    serial::Tx<UART4>,  DMA1, Stream4, Channel4;
    serial::Tx<UART5>,  DMA1, Stream7, Channel4;
    serial::Tx<USART6>, DMA2, Stream6, Channel5;
    // USART6 for DMA2, stream 7, channel 5 is unsupported
    serial::Tx<UART7>,  DMA1, Stream1, Channel5;
    serial::Tx<UART8>,  DMA1, Stream0, Channel5;
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
    fn select<'r>(w: &'r mut dma2::st::cr::W)
        -> &'r mut dma2::st::cr::W;
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
                    unsafe { w.chsel().bits($number) }
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


/// A DMA error
#[derive(Debug)]
pub enum Error {
    Transfer,
    DirectMode,
}

impl Error {
    pub(crate) fn check<S>(dma: &dma2::RegisterBlock)
        -> Result<(), Self>
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


/// Indicates that the peripheral is enabled
pub struct Enabled;

/// Indicates that the peripheral is disabled
pub struct Disabled;
