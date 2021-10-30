use crate::pac::DAC;
use crate::{
    gpio::{
        gpioa::{PA4, PA5},
        Analog,
    },
    rcc::{Enable, Reset},
};

/// DAC Errors
#[derive(Debug)]
pub enum Error {
    /// general
    error,
}

use core::mem;

pub struct C1;
pub struct C2;

pub trait DacOut<V> {
    fn set_value(&mut self, val: V);
    fn get_value(&mut self) -> V;
}

pub trait DacPin {
    fn enable(&mut self);
}

pub trait Pins<DAC> {
    type Output;
}

impl Pins<DAC> for PA4<Analog> {
    type Output = C1;
}

impl Pins<DAC> for PA5<Analog> {
    type Output = C2;
}

impl Pins<DAC> for (PA4<Analog>, PA5<Analog>) {
    type Output = (C1, C2);
}

pub fn dac<PINS>(_dac: DAC, _pins: PINS) -> PINS::Output
where
    PINS: Pins<DAC>,
{
    unsafe {
        DAC::enable_unchecked();
        DAC::reset_unchecked();

        // NOTE(unsafe) ZST, doesn't need initialization.
        assert!(mem::size_of::<PINS::Output>() == 0);
        #[allow(clippy::uninit_assumed_init)]
        mem::MaybeUninit::uninit().assume_init()
    }
}

macro_rules! dac {
    ($CX:ident, $en:ident, $cen:ident, $cal_flag:ident, $trim:ident, $mode:ident, $dhrx:ident, $dac_dor:ident, $daccxdhr:ident) => {
        impl DacPin for $CX {
            fn enable(&mut self) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.cr.modify(|_, w| w.$en().set_bit());
            }
        }

        impl DacOut<u16> for $CX {
            fn set_value(&mut self, val: u16) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.$dhrx.write(|w| unsafe { w.bits(val as u32) });
            }

            fn get_value(&mut self) -> u16 {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.$dac_dor.read().bits() as u16
            }
        }
    };
}

pub trait DacExt {
    fn constrain<PINS>(self, pins: PINS) -> PINS::Output
    where
        PINS: Pins<DAC>;
}

impl DacExt for DAC {
    fn constrain<PINS>(self, pins: PINS) -> PINS::Output
    where
        PINS: Pins<DAC>,
    {
        dac(self, pins)
    }
}

dac!(C1, en1, cen1, cal_flag1, otrim1, mode1, dhr12r1, dor1, dacc1dhr);
dac!(C2, en2, cen2, cal_flag2, otrim2, mode2, dhr12r2, dor2, dacc2dhr);
