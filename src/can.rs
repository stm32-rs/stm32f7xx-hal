//! # Controller Area Network (CAN) Interface
//!
//! ## Alternate function remapping
//!
//! TX: Alternate Push-Pull Output
//! RX: Input Floating Input
//!
//! ### CAN1
//!
//! | Function | NoRemap | Remap |
//! |----------|---------|-------|
//! | TX       | PA12    | PB9   |
//! | RX       | PA11    | PB8   |
//!
//! ### CAN2
//!
//! | Function | NoRemap | Remap |
//! |----------|---------|-------|
//! | TX       | PB6     | PB13  |
//! | RX       | PB5     | PB12  |

use crate::gpio::gpiob::{PB5, PB6};
use crate::gpio::{
    gpioa::{PA11, PA12},
    Alternate, Floating, Input, PushPull,
};
use crate::pac::CAN1;
use crate::pac::CAN2;
use crate::rcc::APB1;

mod sealed {
    pub trait Sealed {}
}

pub trait Pins: sealed::Sealed {
    type Instance;
}

impl sealed::Sealed for (PA12<Alternate<PushPull>>, PA11<Input<Floating>>) {}
impl Pins for (PA12<Alternate<PushPull>>, PA11<Input<Floating>>) {
    type Instance = CAN1;
}

impl sealed::Sealed for (PB6<Alternate<PushPull>>, PB5<Input<Floating>>) {}
impl Pins for (PB6<Alternate<PushPull>>, PB5<Input<Floating>>) {
    type Instance = CAN2;
}

/// Interface to the CAN peripheral.
pub struct Can<Instance> {
    _peripheral: Instance,
}

impl<Instance> Can<Instance>
where
    Instance: crate::rcc::Enable<Bus = APB1>,
{
    /// Creates a CAN interaface.
    pub fn new(can: Instance, apb: &mut APB1) -> Can<Instance> {
        Instance::enable(apb);
        Can { _peripheral: can }
    }
}

unsafe impl bxcan::Instance for Can<CAN1> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN1::ptr() as *mut _;
}

unsafe impl bxcan::Instance for Can<CAN2> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN2::ptr() as *mut _;
}

unsafe impl bxcan::FilterOwner for Can<CAN1> {
    const NUM_FILTER_BANKS: u8 = 28;
}

unsafe impl bxcan::MasterInstance for Can<CAN1> {}
