//! Inter-Integrated Circuit (I2C) bus
//! For now, only master mode is implemented

// NB : this implementation started as a modified copy of https://github.com/stm32-rs/stm32f1xx-hal/blob/master/src/i2c.rs

use micromath::F32Ext;

use crate::gpio::gpioa::PA8;
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::PC9;
use crate::gpio::gpiof::{PF0, PF1};
use crate::gpio::gpioh::{PH4, PH5, PH7, PH8};
use crate::gpio::{Alternate, AF4};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::pac::{DWT, I2C1, I2C2, I2C3};
use crate::rcc::{sealed::RccBus, Clocks, Enable, GetBusFreq, Reset};
use crate::time::{Hertz, U32Ext};
use nb::Error::{Other, WouldBlock};
use nb::{Error as NbError, Result as NbResult};

use cast::u16;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    /// Bus is busy
    Busy,
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)]
    _Extensible,
}
/// SPI mode. The user should make sure that the requested frequency can be
/// generated considering the buses clocks.
#[derive(Debug, PartialEq)]
pub enum Mode {
    Standard { frequency: Hertz },
    Fast { frequency: Hertz },
    FastPlus { frequency: Hertz },
}

impl Mode {
    pub fn standard<F: Into<Hertz>>(frequency: F) -> Self {
        Mode::Standard {
            frequency: frequency.into(),
        }
    }

    pub fn fast<F: Into<Hertz>>(frequency: F) -> Self {
        Mode::Fast {
            frequency: frequency.into(),
        }
    }

    pub fn fast_plus<F: Into<Hertz>>(frequency: F) -> Self {
        Mode::FastPlus {
            frequency: frequency.into(),
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match self {
            &Mode::Standard { frequency } => frequency,
            &Mode::Fast { frequency } => frequency,
            &Mode::FastPlus { frequency } => frequency,
        }
    }
}

struct I2cSpec {
    freq_min: u32,
    freq_max: u32,
    hddat_min: u32,
    vddat_max: u32,
    sudat_min: u32,
    lscl_min: u32,
    hscl_min: u32,
    trise: u32,
    tfall: u32,
}

const I2C_STANDARD_MODE_SPEC: I2cSpec = I2cSpec {
    freq_min: 80000,
    freq_max: 120000,
    hddat_min: 0,
    vddat_max: 3450,
    sudat_min: 250,
    lscl_min: 4700,
    hscl_min: 4000,
    trise: 640,
    tfall: 20,
};
const I2C_FAST_MODE_SPEC: I2cSpec = I2cSpec {
    freq_min: 320000,
    freq_max: 480000,
    hddat_min: 0,
    vddat_max: 900,
    sudat_min: 100,
    lscl_min: 1300,
    hscl_min: 600,
    trise: 250,
    tfall: 100,
};

const I2C_FAST_PLUS_MODE_SPEC: I2cSpec = I2cSpec {
    freq_min: 800000,
    freq_max: 1200000,
    hddat_min: 0,
    vddat_max: 450,
    sudat_min: 50,
    lscl_min: 500,
    hscl_min: 260,
    trise: 60,
    tfall: 100,
};
const I2C_VALID_TIMING_NBR: u32 = 8;
const I2C_PRESC_MAX: u32 = 16;
const I2C_SCLDEL_MAX: u32 = 16;
const I2C_SDADEL_MAX: u32 = 16;
const I2C_SCLH_MAX: u32 = 256;
const I2C_SCLL_MAX: u32 = 256;
const SEC2NSEC: u32 = 1_000_000_000;

impl I2cSpec {
    fn for_mode(m: &Mode) -> &'static I2cSpec {
        match m {
            Mode::Standard { frequency } => &I2C_STANDARD_MODE_SPEC,
            Mode::Fast { .. } => &I2C_FAST_MODE_SPEC,
            Mode::FastPlus { .. } => &I2C_FAST_PLUS_MODE_SPEC,
        }
    }

    fn calculate_timing<F, G>(&self, clkFreq: F, speed: G) -> I2cTiming
    where
        F: Into<Hertz>,
        G: Into<Hertz>,
    {
        // from Arduino code for STM32
        // https://github.com/fpistm/Arduino_Core_STM32/blob/745c200fb21ee0f800bd1bd050a650f63a5c2e07/cores/arduino/stm32/twi.c
        let clkSrcFreq = clkFreq.into().0;
        let freq = speed.into().0;
        assert!(freq >= self.freq_min);
        assert!(freq <= self.freq_max);

        let mut valid_timing_nbr: u32 = 0;
        let mut prev_presc: u32 = I2C_PRESC_MAX;

        let ti2cclk: u32 = (SEC2NSEC + (clkSrcFreq / 2)) / clkSrcFreq;
        let ti2cspeed: u32 = (SEC2NSEC + (freq / 2)) / freq;

        let tafdel_min: u32 = 0;
        let tafdel_max: u32 = 0;
        /*
         * tDNF = DNF x tI2CCLK
         * tPRESC = (PRESC+1) x tI2CCLK
         * SDADEL >= {tf +tHD;DAT(min) - tAF(min) - tDNF - [3 x tI2CCLK]} / {tPRESC}
         * SDADEL <= {tVD;DAT(max) - tr - tAF(max) - tDNF- [4 x tI2CCLK]} / {tPRESC}
         */
        let mut tsdadel_min: i32 = self.tfall as i32 + self.hddat_min as i32
            - tafdel_min as i32
            - (3 * ti2cclk as i32) as i32;
        let mut tsdadel_max: i32 =
            self.vddat_max as i32 - self.trise as i32 - tafdel_max as i32 - (4 * ti2cclk as i32);
        /* {[tr+ tSU;DAT(min)] / [tPRESC]} - 1 <= SCLDEL */
        let tscldel_min: i32 = self.trise as i32 + self.sudat_min as i32;
        if tsdadel_min <= 0 {
            tsdadel_min = 0;
        }
        if tsdadel_max <= 0 {
            tsdadel_max = 0;
        }

        let clk_max = SEC2NSEC / self.freq_min;
        let clk_min = SEC2NSEC / self.freq_max;

        let mut prev_error: u32 = ti2cspeed;
        let mut ret = I2cTiming {
            presc: 0,
            scldel: 0,
            sdadel: 0,
            sclh: 0,
            scll: 0,
        };

        for presc in 0..I2C_PRESC_MAX {
            for scldel in 0..I2C_SCLDEL_MAX {
                /* TSCLDEL = (SCLDEL+1) * (PRESC+1) * TI2CCLK */
                let tscldel: u32 = (scldel + 1) * (presc + 1) * ti2cclk;
                if tscldel >= tscldel_min as u32 {
                    for sdadel in 0..I2C_SDADEL_MAX {
                        /* TSDADEL = SDADEL * (PRESC+1) * TI2CCLK */
                        let tsdadel: u32 = (sdadel * (presc + 1)) * ti2cclk;
                        if (tsdadel >= tsdadel_min as u32) && (tsdadel <= tsdadel_max as u32) {
                            if presc != prev_presc {
                                valid_timing_nbr += 1;
                                if valid_timing_nbr >= I2C_VALID_TIMING_NBR {
                                    return ret;
                                }
                                /* tPRESC = (PRESC+1) x tI2CCLK*/
                                let tpresc: u32 = (presc + 1) * ti2cclk;
                                for scll in 0..I2C_SCLL_MAX {
                                    /* tLOW(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLL+1) x tPRESC ] */
                                    let tscl_l: u32 =
                                        tafdel_min + (2 * ti2cclk) + ((scll + 1) * tpresc);
                                    /* The I2CCLK period tI2CCLK must respect the following conditions:
                                     * tI2CCLK < (tLOW - tfilters) / 4 and tI2CCLK < tHIGH */
                                    if (tscl_l > self.lscl_min)
                                        && (ti2cclk < ((tscl_l - tafdel_min) / 4))
                                    {
                                        for sclh in 0..I2C_SCLH_MAX {
                                            /* tHIGH(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLH+1) x tPRESC] */
                                            let tscl_h: u32 =
                                                tafdel_min + (2 * ti2cclk) + ((sclh + 1) * tpresc);
                                            /* tSCL = tf + tLOW + tr + tHIGH */
                                            let tscl: u32 =
                                                tscl_l + tscl_h + self.trise + self.tfall;
                                            if (tscl >= clk_min)
                                                && (tscl <= clk_max)
                                                && (tscl_h >= self.hscl_min)
                                                && (ti2cclk < tscl_h)
                                            {
                                                let error: u32 =
                                                    (tscl as i32 - ti2cspeed as i32).abs() as u32;
                                                /* look for the timings with the lowest clock error */
                                                if error < prev_error {
                                                    prev_error = error as u32;
                                                    ret.presc = presc as u8;
                                                    ret.scldel = scldel as u8;
                                                    ret.sdadel = sdadel as u8;
                                                    ret.sclh = sclh as u8;
                                                    ret.scll = scll as u8;
                                                    prev_presc = presc;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        ret
    }
}

struct I2cTiming {
    presc: u8,
    scldel: u8,
    sdadel: u8,
    sclh: u8,
    scll: u8,
}

/// Marker trait to define SCL pins for an I2C interface.
pub trait PinScl<I2C> {}

/// Marker trait to define SDA pins for an I2C interface.
pub trait PinSda<I2C> {}

impl PinScl<I2C1> for PB6<Alternate<AF4>> {}
impl PinScl<I2C1> for PB8<Alternate<AF4>> {}
impl PinScl<I2C2> for PB10<Alternate<AF4>> {}
impl PinScl<I2C2> for PF1<Alternate<AF4>> {}
impl PinScl<I2C2> for PH4<Alternate<AF4>> {}
impl PinScl<I2C3> for PA8<Alternate<AF4>> {}
impl PinScl<I2C3> for PH7<Alternate<AF4>> {}

impl PinSda<I2C1> for PB7<Alternate<AF4>> {}
impl PinSda<I2C1> for PB9<Alternate<AF4>> {}
impl PinSda<I2C2> for PB11<Alternate<AF4>> {}
impl PinSda<I2C2> for PF0<Alternate<AF4>> {}
impl PinSda<I2C2> for PH5<Alternate<AF4>> {}
impl PinSda<I2C3> for PC9<Alternate<AF4>> {}
impl PinSda<I2C3> for PH8<Alternate<AF4>> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, SCL, SDA> {
    i2c: I2C,
    pins: (SCL, SDA),
    mode: Mode,
    pclk: u32,
}

/// embedded-hal compatible blocking I2C implementation
pub struct BlockingI2c<I2C, SCL, SDA> {
    nb: I2c<I2C, SCL, SDA>,
    data_timeout: u32,
}

impl<SCL, SDA> I2c<I2C1, SCL, SDA> {
    /// Creates a generic I2C1 object.
    pub fn i2c1(
        i2c: I2C1,
        pins: (SCL, SDA),
        mode: Mode,
        clocks: Clocks,
        apb: &mut <I2C1 as RccBus>::Bus,
    ) -> Self
    where
        SCL: PinScl<I2C1>,
        SDA: PinSda<I2C1>,
    {
        I2c::_i2c1(i2c, pins, mode, clocks, apb)
    }
}

impl<SCL, SDA> BlockingI2c<I2C1, SCL, SDA> {
    /// Creates a blocking I2C1 object using the embedded-hal `BlockingI2c` trait.
    pub fn i2c1(
        i2c: I2C1,
        pins: (SCL, SDA),
        mode: Mode,
        clocks: Clocks,
        apb: &mut <I2C1 as RccBus>::Bus,
        data_timeout_us: u32,
    ) -> Self
    where
        SCL: PinScl<I2C1>,
        SDA: PinSda<I2C1>,
    {
        BlockingI2c::_i2c1(i2c, pins, mode, clocks, apb, data_timeout_us)
    }
}

impl<SCL, SDA> I2c<I2C2, SCL, SDA> {
    /// Creates a generic I2C2 object.
    pub fn i2c2(
        i2c: I2C2,
        pins: (SCL, SDA),
        mode: Mode,
        clocks: Clocks,
        apb: &mut <I2C2 as RccBus>::Bus,
    ) -> Self
    where
        SCL: PinScl<I2C2>,
        SDA: PinSda<I2C2>,
    {
        I2c::_i2c2(i2c, pins, mode, clocks, apb)
    }
}

impl<SCL, SDA> BlockingI2c<I2C2, SCL, SDA> {
    /// Creates a blocking I2C2 object using the embedded-hal `BlockingI2c` trait.
    pub fn i2c2(
        i2c: I2C2,
        pins: (SCL, SDA),
        mode: Mode,
        clocks: Clocks,
        apb: &mut <I2C2 as RccBus>::Bus,
        data_timeout_us: u32,
    ) -> Self
    where
        SCL: PinScl<I2C2>,
        SDA: PinSda<I2C2>,
    {
        BlockingI2c::_i2c2(i2c, pins, mode, clocks, apb, data_timeout_us)
    }
}

impl<SCL, SDA> I2c<I2C3, SCL, SDA> {
    /// Creates a generic I2C3 object.
    pub fn i2c3(
        i2c: I2C3,
        pins: (SCL, SDA),
        mode: Mode,
        clocks: Clocks,
        apb: &mut <I2C3 as RccBus>::Bus,
    ) -> Self
    where
        SCL: PinScl<I2C3>,
        SDA: PinSda<I2C3>,
    {
        I2c::_i2c3(i2c, pins, mode, clocks, apb)
    }
}

impl<SCL, SDA> BlockingI2c<I2C3, SCL, SDA> {
    /// Creates a blocking I2C3 object using the embedded-hal `BlockingI2c` trait.
    pub fn i2c3(
        i2c: I2C3,
        pins: (SCL, SDA),
        mode: Mode,
        clocks: Clocks,
        apb: &mut <I2C3 as RccBus>::Bus,
        data_timeout_us: u32,
    ) -> Self
    where
        SCL: PinScl<I2C3>,
        SDA: PinSda<I2C3>,
    {
        BlockingI2c::_i2c3(i2c, pins, mode, clocks, apb, data_timeout_us)
    }
}

/// Generates a blocking I2C instance from a universal I2C object
fn blocking_i2c<I2C, SCL, SDA>(
    i2c: I2c<I2C, SCL, SDA>,
    clocks: Clocks,
    data_timeout_us: u32,
) -> BlockingI2c<I2C, SCL, SDA> {
    let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
    return BlockingI2c {
        nb: i2c,
        data_timeout: data_timeout_us * sysclk_mhz,
    };
}

macro_rules! check_status_flag {
    ($i2c:expr, $flag:ident, $status:ident) => {{
        let isr = $i2c.isr.read();

        if isr.berr().bit_is_set() {
            $i2c.icr.write(|w| w.berrcf().set_bit());
            Err(Other(Error::Bus))
        } else if isr.arlo().bit_is_set() {
            $i2c.icr.write(|w| w.arlocf().set_bit());
            Err(Other(Error::Arbitration))
        } else if isr.nackf().bit_is_set() {
            $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
            Err(Other(Error::Acknowledge))
        } else if isr.ovr().bit_is_set() {
            $i2c.icr.write(|w| w.stopcf().set_bit().ovrcf().set_bit());
            Err(Other(Error::Overrun))
        } else if isr.$flag().$status() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(WouldBlock) {
                break res;
            }
            if $exit_cond {
                break res;
            }
        }
    }};
}

macro_rules! busy_wait_cycles {
    ($nb_expr:expr, $cycles:expr) => {{
        let started = DWT::get_cycle_count();
        let cycles = $cycles;
        busy_wait!(
            $nb_expr,
            DWT::get_cycle_count().wrapping_sub(started) >= cycles
        )
    }};
}

// Generate the same code for both I2Cs
macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, SCL, SDA> {
                /// Configures the I2C peripheral to work in master mode
                fn $i2cX(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    mode: Mode,
                    clocks: Clocks,
                    apb: &mut <I2C1 as RccBus>::Bus
                ) -> Self {
                    $I2CX::enable(apb);
                    $I2CX::reset(apb);

                    let pclk = <$I2CX as RccBus>::Bus::get_frequency(&clocks).0;

                    assert!(mode.get_frequency().0 <= 400_000);

                    let mut i2c = I2c { i2c, pins, mode, pclk };
                    i2c.init();
                    i2c
                }

                /// Initializes I2C as master. Configures I2C_PRESC, I2C_SCLDEL,
                /// I2C_SDAEL, I2C_SCLH, I2C_SCLL
                ///
                /// For now, only standard mode is implemented
                fn init(&mut self) {
                    // NOTE : operations are in float for better precision,
                    // STM32F7 usually have FPU and this runs only at
                    // initialization so the footprint of such heavy calculation
                    // occurs only once

                    // Disable I2C during configuration
                    self.i2c.cr1.write(|w| w.pe().disabled());
                    let timing = I2cSpec::for_mode(&self.mode)
                        .calculate_timing(self.pclk.hz(), self.mode.get_frequency());
                    self.i2c.timingr.write(|w|
                        w.presc()
                            .bits(timing.presc)
                            .scll()
                            .bits(timing.scll)
                            .sclh()
                            .bits(timing.sclh)
                            .sdadel()
                            .bits(timing.sdadel)
                            .scldel()
                            .bits(timing.scldel)
                    );
                    self.i2c.cr1.modify(|_, w| w.pe().enabled());
                }

                /// Perform an I2C software reset
                #[allow(dead_code)]
                fn reset(&mut self) {
                    self.i2c.cr1.write(|w| w.pe().disabled());
                    // wait for disabled
                    while self.i2c.cr1.read().pe().is_enabled() {}

                    // Re-enable
                    self.i2c.cr1.write(|w| w.pe().enabled());
                }

                /// Set (7-bit) slave address, bus direction (write or read),
                /// generate START condition and set address.
                ///
                /// The user has to specify the number `n_bytes` of bytes to
                /// read. The peripheral automatically waits for the bus to be
                /// free before sending the START and address
                ///
                /// Data transfers of more than 255 bytes are not yet
                /// supported, 10-bit slave address are not yet supported
                fn start(&self, addr: u8, n_bytes: u8, read: bool, auto_stop: bool) {
                    self.i2c.cr2.write(|mut w| {
                        // Setup data
                        w = w.sadd()
                            .bits(u16(addr << 1 | 0))
                            .add10().clear_bit()
                            .nbytes()
                            .bits(n_bytes as u8)
                            .start()
                            .set_bit();

                        // Setup transfer direction
                        w = match read {
                            true => w.rd_wrn().read(),
                            false => w.rd_wrn().write()
                        };

                        // setup auto-stop
                        match auto_stop {
                            true => w.autoend().automatic(),
                            false => w.autoend().software(),
                        }
                    });
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<SCL, SDA> BlockingI2c<$I2CX, SCL, SDA> {
                fn $i2cX(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    mode: Mode,
                    clocks: Clocks,
                    apb: &mut <$I2CX as RccBus>::Bus,
                    data_timeout_us: u32
                ) -> Self {
                    blocking_i2c(I2c::$i2cX(i2c, pins, mode, clocks, apb),
                        clocks, data_timeout_us)
                }

                /// Wait for a byte to be read and return it (ie for RXNE flag
                /// to be set)
                fn wait_byte_read(&self) -> NbResult<u8, Error> {
                    // Wait until we have received something
                    busy_wait_cycles!(
                        check_status_flag!(self.nb.i2c, rxne, is_not_empty),
                        self.data_timeout
                    )?;

                    Ok(self.nb.i2c.rxdr.read().rxdata().bits())
                }

                /// Wait the write data register to be empty  (ie for TXIS flag
                /// to be set) and write the byte to it
                fn wait_byte_write(&self, byte: u8) -> NbResult<(), Error> {
                    // Wait until we are allowed to send data
                    // (START has been ACKed or last byte when through)
                    busy_wait_cycles!(
                        check_status_flag!(self.nb.i2c, txis, is_empty),
                        self.data_timeout
                    )?;

                    // Put byte on the wire
                    self.nb.i2c.txdr.write(|w| w.txdata().bits(byte));

                    Ok(())
                }

                /// Wait for any previous address sequence to end automatically.
                fn wait_start(&self) {
                    while self.nb.i2c.cr2.read().start().bit_is_set() {};
                }
            }

            impl<SCL, SDA> Write for BlockingI2c<$I2CX, SCL, SDA> {
                type Error = NbError<Error>;

                /// Write bytes to I2C. Currently, `bytes.len()` must be less or
                /// equal than 255
                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // Wait for any previous address sequence to end
                    // automatically. This could be up to 50% of a bus
                    // cycle (ie. up to 0.5/freq)
                    self.wait_start();

                    // Set START and prepare to send `bytes`. The
                    // START bit can be set even if the bus is BUSY or
                    // I2C is in slave mode.
                    self.nb.start(addr, bytes.len() as u8, false, true);

                    for byte in bytes {
                        self.wait_byte_write(*byte)?;
                    }
                    // automatic STOP

                    Ok(())
                }
            }

            impl<SCL, SDA> Read for BlockingI2c<$I2CX, SCL, SDA> {
                type Error = NbError<Error>;

                /// Reads enough bytes from slave with `address` to fill `buffer`
                fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // Wait for any previous address sequence to end
                    // automatically. This could be up to 50% of a bus
                    // cycle (ie. up to 0.5/freq)
                    self.wait_start();

                    // Set START and prepare to receive bytes into
                    // `buffer`. The START bit can be set even if the bus
                    // is BUSY or I2C is in slave mode.
                    self.nb.start(addr, buffer.len() as u8, true, true);

                    for byte in buffer {
                        *byte = self.wait_byte_read()?;
                    }

                    // automatic STOP

                    Ok(())
                }
            }

            impl<SCL, SDA> WriteRead for BlockingI2c<$I2CX, SCL, SDA> {
                type Error = NbError<Error>;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Self::Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // Start and make sure we don't send STOP after the write
                    self.wait_start();
                    self.nb.start(addr, bytes.len() as u8, false, false);

                    for byte in bytes {
                        self.wait_byte_write(*byte)?;
                    }

                    // Wait until the write finishes before beginning to read.
                    // busy_wait2!(self.nb.i2c, tc, is_complete);
                    busy_wait_cycles!(
                        check_status_flag!(self.nb.i2c, tc, is_complete),
                        self.data_timeout
                    )?;

                    // reSTART and prepare to receive bytes into `buffer`
                    self.nb.start(addr, buffer.len() as u8, true, true);

                    for byte in buffer {
                        *byte = self.wait_byte_read()?;
                    }
                    // automatic STOP

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C1: (_i2c1),
    I2C2: (_i2c2),
    I2C3: (_i2c3),
}
