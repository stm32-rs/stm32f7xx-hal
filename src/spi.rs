//! Interface to the SPI peripheral
//!
//! See chapter 32 in the STM32F746 Reference Manual.


pub use embedded_hal::spi::{
    Mode,
    Phase,
    Polarity,
};
pub use crate::device::spi1::cr1::BRW as ClockDivider;


use core::ptr;

use embedded_hal::{
    blocking::spi::{
        transfer,
        write,
        write_iter,
    },
    spi::FullDuplex,
};

use crate::{
    device::{
        SPI1,
        SPI2,
        SPI3,
        SPI4,
        SPI5,
        SPI6,
    },
    gpio::{
        Alternate,
        AF5,
        AF6,
        AF7,
        gpioa::{
            PA5,
            PA6,
            PA7,
            PA9,
        },
        gpiob::{
            PB2,
            PB3,
            PB4,
            PB5,
            PB10,
            PB13,
            PB14,
            PB15,
        },
        gpioc::{
            PC1,
            PC2,
            PC3,
            PC10,
            PC11,
            PC12,
        },
        gpiod::{
            PD3,
            PD6,
        },
        gpioe::{
            PE2,
            PE5,
            PE6,
            PE12,
            PE13,
            PE14,
        },
        gpiof::{
            PF7,
            PF8,
            PF9,
            PF11,
        },
        gpiog::{
            PG12,
            PG13,
            PG14,
        },
        gpioh::{
            PH6,
            PH7,
        },
        gpioi::{
            PI1,
            PI2,
            PI3,
        },
    },
    rcc::Rcc,
    state,
};


/// Entry point to the SPI API
pub struct Spi<I, P, State> {
    spi:    I,
    pins:   P,
    _state: State,
}

impl<I, P> Spi<I, P, state::Disabled>
    where
        I: Instance,
        P: Pins<I>,
{
    /// Create a new instance of the SPI API
    pub fn new(instance:  I, pins: P) -> Self {
        Self {
            spi:    instance,
            pins,
            _state: state::Disabled,
        }
    }

    /// Initialize the SPI peripheral
    pub fn enable(self, rcc: &mut Rcc, clock_divider: ClockDivider, mode: Mode)
        -> Spi<I, P, state::Enabled>
    {
        let cpol = mode.polarity == Polarity::IdleHigh;
        let cpha = mode.phase == Phase::CaptureOnSecondTransition;

        self.spi.enable_clock(rcc);
        self.spi.configure(clock_divider._bits(), cpol, cpha);

        Spi {
            spi:    self.spi,
            pins:   self.pins,
            _state: state::Enabled,
        }
    }
}

impl<I, P> FullDuplex<u8> for Spi<I, P, state::Enabled>
    where
        I: Instance,
        P: Pins<I>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.read()
    }

    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.spi.send(word)
    }
}

impl<I, P> transfer::Default<u8> for Spi<I, P, state::Enabled>
    where
        I: Instance,
        P: Pins<I>,
{}

impl<I, P> write::Default<u8> for Spi<I, P, state::Enabled>
    where
        I: Instance,
        P: Pins<I>,
{}

impl<I, P> write_iter::Default<u8> for Spi<I, P, state::Enabled>
    where
        I: Instance,
        P: Pins<I>,
{}

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
    fn enable_clock(&self, rcc: &mut Rcc);
    fn configure(&self, br: u8, cpol: bool, cpha: bool);
    fn read(&self) -> nb::Result<u8, Error>;
    fn send(&self, word: u8) -> nb::Result<(), Error>;
}

/// Implemented for all tuples that contain a full set of valid SPI pins
pub trait Pins<I> {}

impl<I, SCK, MISO, MOSI> Pins<I> for (SCK, MISO, MOSI)
    where
        SCK:  Sck<I>,
        MISO: Miso<I>,
        MOSI: Mosi<I>,
{}

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
                fn enable_clock(&self, rcc: &mut Rcc) {
                    rcc.$bus.rstr().modify(|_, w| w.$reset().clear_bit());
                    rcc.$bus.enr().modify(|_, w| w.$enable().enabled());
                }

                // I don't like putting this much code into the macro, but I
                // have to: There are two different SPI variants in the PAC, and
                // while I haven't found any actual differences between them,
                // they're still using different sets of types, and I need to
                // generate different methods to interface with them, even
                // though these methods end up looking identical.
                //
                // Maybe this is a problem in the SVD file that can be fixed
                // there.

                fn configure(&self, br: u8, cpol: bool, cpha: bool) {
                    self.cr2.write(|w|
                        w
                            // FIFO reception threshold. This is the right value
                            // for 8 bits.
                            .frxth().quarter()
                            // Data size
                            .ds().eight_bit()
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
                            // Disable DMA support
                            .txdmaen().disabled()
                            .rxdmaen().disabled()
                    );

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

                fn read(&self) -> nb::Result<u8, Error> {
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
                                &self.dr as *const _ as *const u8,
                            )
                        };

                        return Ok(value);
                    }

                    Err(nb::Error::WouldBlock)
                }

                fn send(&self, word: u8) -> nb::Result<(), Error> {
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
                                &self.dr as *const _ as *mut u8,
                                word,
                            );
                        }

                        return Ok(())
                    }

                    Err(nb::Error::WouldBlock)
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
    SPI1 {
        regs: (apb2, spi1rst, spi1en),
        pins: {
            SCK: [
                PA5<Alternate<AF5>>,
                PB3<Alternate<AF5>>,
            ],
            MISO: [
                PA6<Alternate<AF5>>,
                PB4<Alternate<AF5>>,
            ],
            MOSI: [
                PA7<Alternate<AF5>>,
                PB5<Alternate<AF5>>,
            ],
        }
    }
    SPI2 {
        regs: (apb1, spi2rst, spi2en),
        pins: {
            SCK: [
                PA9<Alternate<AF5>>,
                PB10<Alternate<AF5>>,
                PB13<Alternate<AF5>>,
                PD3<Alternate<AF5>>,
                PI1<Alternate<AF5>>,
            ],
            MISO: [
                PB14<Alternate<AF5>>,
                PC2<Alternate<AF5>>,
                PI2<Alternate<AF5>>,
            ],
            MOSI: [
                PB15<Alternate<AF5>>,
                PC1<Alternate<AF5>>,
                PC3<Alternate<AF5>>,
                PI3<Alternate<AF5>>,
            ],
        }
    }
    SPI3 {
        regs: (apb1, spi3rst, spi3en),
        pins: {
            SCK: [
                PB3<Alternate<AF6>>,
                PC10<Alternate<AF6>>,
            ],
            MISO: [
                PB4<Alternate<AF6>>,
                PC11<Alternate<AF6>>,
            ],
            MOSI: [
                PB2<Alternate<AF7>>,
                PB5<Alternate<AF6>>,
                PC12<Alternate<AF6>>,
                PD6<Alternate<AF5>>,
            ],
        }
    }
    SPI4 {
        regs: (apb2, spi4rst, spi4en),
        pins: {
            SCK: [
                PE2<Alternate<AF5>>,
                PE12<Alternate<AF5>>,
            ],
            MISO: [
                PE5<Alternate<AF5>>,
                PE13<Alternate<AF5>>,
            ],
            MOSI: [
                PE6<Alternate<AF5>>,
                PE14<Alternate<AF5>>,
            ],
        }
    }
    SPI5 {
        regs: (apb2, spi5rst, spi5en),
        pins: {
            SCK: [
                PF7<Alternate<AF5>>,
                PH6<Alternate<AF5>>,
            ],
            MISO: [
                PF8<Alternate<AF5>>,
                PH7<Alternate<AF5>>,
            ],
            MOSI: [
                PF9<Alternate<AF5>>,
                PF11<Alternate<AF5>>,
            ],
        }
    }
    SPI6 {
        regs: (apb2, spi6rst, spi6en),
        pins: {
            SCK: [
                PG13<Alternate<AF5>>,
            ],
            MISO: [
                PG12<Alternate<AF5>>,
            ],
            MOSI: [
                PG14<Alternate<AF5>>,
            ],
        }
    }
);


#[derive(Debug)]
pub enum Error {
    FrameFormat,
    Overrun,
    ModeFault,
}
