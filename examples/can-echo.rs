//! Simple CAN example.
//! Requires a transceiver connected to PA11, PA12 (CAN1) or PB5 PB6 (CAN2).

#![no_main]
#![no_std]

use panic_halt as _;

use bxcan::filter::Mask32;
use cortex_m_rt::entry;
use nb::block;
use stm32f7xx_hal::{
    can::Can,
    pac,
    prelude::*,
    rcc::{HSEClock, HSEClockMode},
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    // To meet CAN clock accuracy requirements an external crystal or ceramic
    // resonator must be used. The blue pill has a 8MHz external crystal.
    // Other boards might have a crystal with another frequency or none at all.
    let _clocks = rcc
        .cfgr
        .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Bypass))
        .sysclk(216_000_000.Hz())
        .hclk(216_000_000.Hz())
        .freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let mut can1 = {
        let rx = gpioa.pa11.into_alternate();
        let tx = gpioa.pa12.into_alternate();

        let can = Can::new(dp.CAN1, &mut rcc.apb1, (tx, rx));

        bxcan::Can::builder(can)
            // APB1 (PCLK1): 130MHz, Bit rate: 512kBit/s, Sample Point 87.5%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            .set_bit_timing(0x001e_000b)
            .enable()
    };

    // Configure filters so that can frames can be received.
    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Mask32::accept_all());

    let _can2 = {
        let rx = gpiob.pb5.into_alternate();
        let tx = gpiob.pb6.into_alternate();

        let can = Can::new(dp.CAN2, &mut rcc.apb1, (tx, rx));

        let can2 = bxcan::Can::builder(can)
            // APB1 (PCLK1): 130MHz, Bit rate: 512kBit/s, Sample Point 87.5%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            .set_bit_timing(0x001e_000b)
            .enable();

        // A total of 28 filters are shared between the two CAN instances.
        // Split them equally between CAN1 and CAN2.
        filters.set_split(14);
        let mut slave_filters = filters.slave_filters();
        slave_filters.enable_bank(14, Mask32::accept_all());
        can2
    };

    // Drop filters to leave filter configuraiton mode.
    drop(filters);

    // Select the interface.
    let mut can = can1;
    //let mut can = can2;

    // Echo back received packages in sequence.
    // See the `can-rtfm` example for an echo implementation that adheres to
    // correct frame ordering based on the transfer id.
    loop {
        if let Ok(frame) = block!(can.receive()) {
            block!(can.transmit(&frame)).unwrap();
        }
    }
}
