//! Inter-Integrated Circuit (I2C) bus
//! For now, only master mode is implemented

// NB : this implementation started as a modified copy of https://github.com/stm32-rs/stm32f1xx-hal/blob/master/src/i2c.rs

use crate::embedded_time::rate::Hertz;
use crate::gpio::gpioa::PA8;
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::PC9;
use crate::gpio::gpiof::{PF0, PF1};
use crate::gpio::gpioh::{PH4, PH5, PH7, PH8};
use crate::gpio::AlternateOD;
use crate::hal::i2c::{
    self,
    blocking::{Read, Write, WriteRead},
};
use crate::pac::{DWT, I2C1, I2C2, I2C3};
use crate::rcc::{Clocks, Enable, GetBusFreq, RccBus, Reset};
use nb;

use cast::u16;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error occurred. e.g. A START or a STOP condition is detected and is not
    /// located after a multiple of 9 SCL clock pulses.
    Bus,
    /// The arbitration was lost, e.g. electrical problems with the clock signal
    ArbitrationLoss,
    /// A bus operation was not acknowledged, e.g. due to the addressed device not
    /// being available on the bus or the device not being ready to process requests
    /// at the moment
    NoAcknowledge(i2c::NoAcknowledgeSource),
    /// The peripheral receive buffer was overrun
    Overrun,
    /// Timeout
    Timeout,
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::Bus => i2c::ErrorKind::Bus,
            Error::ArbitrationLoss => i2c::ErrorKind::ArbitrationLoss,
            Error::NoAcknowledge(s) => i2c::ErrorKind::NoAcknowledge(s),
            Error::Overrun => i2c::ErrorKind::Overrun,
            _ => i2c::ErrorKind::Other,
        }
    }
}

/// SPI mode. The user should make sure that the requested frequency can be
/// generated considering the buses clocks.
#[derive(Debug, PartialEq)]
pub enum Mode {
    Standard { frequency: Hertz },
    Fast { frequency: Hertz },
    FastPlus { frequency: Hertz },
    Custom { timing_r: u32 },
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
}

/// Marker trait to define SCL pins for an I2C interface.
pub trait PinScl<I2C> {}

/// Marker trait to define SDA pins for an I2C interface.
pub trait PinSda<I2C> {}

impl PinScl<I2C1> for PB6<AlternateOD<4>> {}
impl PinScl<I2C1> for PB8<AlternateOD<4>> {}
impl PinScl<I2C2> for PB10<AlternateOD<4>> {}
impl PinScl<I2C2> for PF1<AlternateOD<4>> {}
impl PinScl<I2C2> for PH4<AlternateOD<4>> {}
impl PinScl<I2C3> for PA8<AlternateOD<4>> {}
impl PinScl<I2C3> for PH7<AlternateOD<4>> {}

impl PinSda<I2C1> for PB7<AlternateOD<4>> {}
impl PinSda<I2C1> for PB9<AlternateOD<4>> {}
impl PinSda<I2C2> for PB11<AlternateOD<4>> {}
impl PinSda<I2C2> for PF0<AlternateOD<4>> {}
impl PinSda<I2C2> for PH5<AlternateOD<4>> {}
impl PinSda<I2C3> for PC9<AlternateOD<4>> {}
impl PinSda<I2C3> for PH8<AlternateOD<4>> {}

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
    BlockingI2c {
        nb: i2c,
        data_timeout: data_timeout_us * sysclk_mhz,
    }
}

// hddat and vddat are removed because SDADEL is always going to be 0 in this implementation so
// condition is always met
struct I2cSpec {
    freq_max: u32,
    sudat_min: u32,
    _lscl_min: u32,
    _hscl_min: u32,
    trise_max: u32, // in ns
    _tfall_max: u32,
}

#[derive(Debug)]
struct I2cTiming {
    presc: u8,
    scldel: u8,
    sdadel: u8,
    sclh: u8,
    scll: u8,
}

// everything is in nano seconds
const I2C_STANDARD_MODE_SPEC: I2cSpec = I2cSpec {
    freq_max: 102400,
    sudat_min: 250,
    _lscl_min: 4700,
    _hscl_min: 4000,
    trise_max: 640,
    _tfall_max: 20,
};
const I2C_FAST_MODE_SPEC: I2cSpec = I2cSpec {
    freq_max: 409600,
    sudat_min: 100,
    _lscl_min: 1300,
    _hscl_min: 600,
    trise_max: 250,
    _tfall_max: 100,
};

const I2C_FAST_PLUS_MODE_SPEC: I2cSpec = I2cSpec {
    freq_max: 1024000,
    sudat_min: 50,
    _lscl_min: 500,
    _hscl_min: 260,
    trise_max: 60,
    _tfall_max: 100,
};

fn calculate_timing(
    spec: I2cSpec,
    i2c_freq: u32,
    scl_freq: u32,
    an_filter: bool,
    dnf: u8,
) -> I2cTiming {
    // This dependency is not used when `cargo test`ing. More info:
    // https://docs.rs/micromath/1.1.1/micromath/index.html#unused-import-warnings-when-linking-std
    #[cfg(not(test))]
    use micromath::F32Ext as _;

    // frequency limit check
    assert!(scl_freq <= spec.freq_max);
    // T_sync or delay introduced in SCL
    // generally it is 2-3 clock cycles
    // t_sync + dnf delay
    let t_dnf = (dnf) as f32 / i2c_freq as f32;
    // if analog filter is enabled then it offer about 50 - 70 ns delay
    let t_af: f32 = if an_filter {
        40.0 / 1_000_000_000f32
    } else {
        0.0
    };
    // t_sync = 2 to 3 * i2cclk
    let t_sync = 2.0 / (i2c_freq as f32);
    // fall or rise time
    let t_fall: f32 = 50f32 / 1_000_000_000f32;
    let t_rise: f32 = 60f32 / 1_000_000_000f32;
    let t_delay = t_fall + t_rise + 2.0 * (t_dnf + t_af + t_sync);
    // formula is like F_i2cclk/F/F_scl_clk = (scl_h+scl_l+2)*(Presc + 1)
    // consider scl_l+scl_h is 256 max. but in that case clock should always
    // be 50% duty cycle. lets consider scl_l+scl_h to be 128. so that it can
    // be changed later
    // (scl_l+scl_h+2)(presc +1 ) ==> as scl_width*presc ==F_i2cclk/F/F_scl_clk
    let product: f32 = (1.0 - t_delay * (scl_freq as f32)) * (i2c_freq / scl_freq) as f32;
    let scl_l: u8;
    let scl_h: u8;
    let mut presc: u8;
    // if ratio is > (scll+sclh)*presc. that frequancy is not possible to generate. so
    // minimum frequancy possible is generated
    if product > 8192_f32 {
        // TODO: should we panic or use minimum allowed frequancy
        scl_l = 0x7fu8;
        scl_h = 0x7fu8;
        presc = 0xfu8;
    } else {
        // smaller the minimum devition less difference between expected vs
        // actual scl clock
        let mut min_deviation = 16f32;
        // TODO: use duty cycle and based on that use precstart
        let presc_start = (product / 512.0).ceil() as u8;
        presc = presc_start;
        for tmp_presc in presc_start..17 {
            let deviation = product % tmp_presc as f32;
            if min_deviation > deviation {
                min_deviation = deviation;
                presc = tmp_presc as u8;
            }
        }
        // now that we have optimal prescalar value. optimal scl_l and scl_h
        // needs to be calculated
        let scl_width = (product / presc as f32) as u16; // it will be always less than 256
        scl_h = (scl_width / 2 - 1) as u8;
        scl_l = (scl_width - scl_h as u16 - 1) as u8; // This is to get max precision
        presc -= 1;
    }
    let scldel: u8 = (((spec.trise_max + spec.sudat_min) as f32 / 1_000_000_000.0)
        / ((presc + 1) as f32 / i2c_freq as f32)
        - 1.0)
        .ceil() as u8;
    I2cTiming {
        presc,
        scldel,
        sdadel: 0,
        sclh: scl_h,
        scll: scl_l,
    }
}

macro_rules! check_status_flag {
    ($i2c:expr, $flag:ident, $status:ident) => {{
        let isr = $i2c.isr.read();

        if isr.berr().bit_is_set() {
            $i2c.icr.write(|w| w.berrcf().set_bit());
            Err(nb::Error::Other(Error::Bus))
        } else if isr.arlo().bit_is_set() {
            $i2c.icr.write(|w| w.arlocf().set_bit());
            Err(nb::Error::Other(Error::ArbitrationLoss))
        } else if isr.nackf().bit_is_set() {
            $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
            Err(nb::Error::Other(Error::NoAcknowledge(
                i2c::NoAcknowledgeSource::Unknown,
            )))
        } else if isr.ovr().bit_is_set() {
            $i2c.icr.write(|w| w.stopcf().set_bit().ovrcf().set_bit());
            Err(nb::Error::Other(Error::Overrun))
        } else if isr.$flag().$status() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(nb::Error::WouldBlock) {
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

// Map non-blocking errors to blocking errors
macro_rules! nbError_to_Error {
    ($nb_expr:expr) => {{
        match $nb_expr {
            Ok(()) => {}
            Err(nb::Error::WouldBlock) => return Err(Error::Timeout),
            Err(nb::Error::Other(error)) => return Err(error),
        };
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

                    let an_filter:bool = self.i2c.cr1.read().anfoff().is_enabled();
                    let dnf = self.i2c.cr1.read().dnf().bits();

                    let i2c_timingr: I2cTiming =  match self.mode {
                        Mode::Standard{ frequency } => calculate_timing(I2C_STANDARD_MODE_SPEC, self.pclk, frequency.0, an_filter, dnf ),
                        Mode::Fast{ frequency } => calculate_timing(I2C_FAST_MODE_SPEC, self.pclk, frequency.0, an_filter, dnf),
                        Mode::FastPlus{ frequency } => calculate_timing(I2C_FAST_PLUS_MODE_SPEC, self.pclk, frequency.0, an_filter, dnf ),
                        Mode::Custom{ timing_r } => {
                            I2cTiming{
                                presc:  ((timing_r & 0xf000_0000) >> 28 ) as u8,
                                scldel: ((timing_r & 0x00f0_0000) >> 20 ) as u8 ,
                                sdadel: ((timing_r & 0x000f_0000) >> 16 ) as u8,
                                sclh:   ((timing_r & 0x0000_ff00) >> 8 ) as u8,
                                scll:   ((timing_r & 0x0000_00ff) >> 0 ) as u8,
                            }
                        }
                    };
                    self.i2c.timingr.write(|w|
                        w.presc()
                            .bits(i2c_timingr.presc)
                            .scll()
                            .bits(i2c_timingr.scll)
                            .sclh()
                            .bits(i2c_timingr.sclh)
                            .sdadel()
                            .bits(i2c_timingr.sdadel)
                            .scldel()
                            .bits(i2c_timingr.scldel)
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
                fn wait_byte_read(&self) -> Result<u8, Error> {
                    // Wait until we have received something
                    nbError_to_Error!(busy_wait_cycles!(check_status_flag!(self.nb.i2c, rxne, is_not_empty), self.data_timeout));

                    Ok(self.nb.i2c.rxdr.read().rxdata().bits())
                }

                /// Wait the write data register to be empty  (ie for TXIS flag
                /// to be set) and write the byte to it
                fn wait_byte_write(&self, byte: u8) -> Result<(), Error> {
                    // Wait until we are allowed to send data
                    // (START has been ACKed or last byte when through)
                    nbError_to_Error!(busy_wait_cycles!(check_status_flag!(self.nb.i2c, txis, is_empty), self.data_timeout));

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
                type Error = Error;

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
                type Error = Error;

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
                type Error = Error;

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
                    nbError_to_Error!(busy_wait_cycles!(check_status_flag!(self.nb.i2c, tc, is_complete), self.data_timeout));

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
