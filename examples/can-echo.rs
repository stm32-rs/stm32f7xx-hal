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
    rcc.cfgr
        .hse(HSEClock::new(8.mhz(), HSEClockMode::Bypass))
        .freeze();

    let mut can1 = {
        let can = Can::new(dp.CAN1, &mut rcc.apb1);
        bxcan::Can::new(can)
    };
    can1.configure(|config| {
        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        config.set_bit_timing(0x001c_0003);
    });

    // Configure filters so that can frames can be received.
    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Mask32::accept_all());

    let _can2 = {
        let can = Can::new(dp.CAN2, &mut rcc.apb1);

        let mut can2 = bxcan::Can::new(can);
        can2.configure(|config| {
            // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            config.set_bit_timing(0x001c_0003);
        });

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

    // Split the peripheral into transmitter and receiver parts.
    block!(can.enable()).unwrap();

    // Echo back received packages in sequence.
    // See the `can-rtfm` example for an echo implementation that adheres to
    // correct frame ordering based on the transfer id.
    loop {
        if let Ok(frame) = block!(can.receive()) {
            block!(can.transmit(&frame)).unwrap();
        }
    }
}
