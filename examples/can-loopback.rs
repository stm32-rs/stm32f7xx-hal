//! Showcases advanced CAN filter capabilities.
//! Does not require additional transceiver hardware.

#![no_main]
#![no_std]

use bxcan::{
    filter::{ListEntry16, ListEntry32, Mask16},
    ExtendedId, Frame, StandardId,
};
use panic_halt as _;

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

    // To meet CAN clock accuracy requirements, an external crystal or ceramic
    // resonator must be used.
    let _clocks = rcc
        .cfgr
        .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Bypass))
        .sysclk(216_000_000.Hz())
        .hclk(216_000_000.Hz())
        .freeze();

    let gpioa = dp.GPIOA.split();

    let rx = gpioa.pa11.into_alternate();
    let tx = gpioa.pa12.into_alternate();

    let can = Can::new(dp.CAN1, &mut rcc.apb1, (tx, rx));

    // Use loopback mode: No pins need to be assigned to peripheral.
    let mut can = bxcan::Can::builder(can)
        // APB1 (PCLK1): 130MHz, Bit rate: 512kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        .set_bit_timing(0x001e_000b)
        .set_loopback(true)
        .set_silent(true)
        .enable();

    let mut filters = can.modify_filters();
    assert!(filters.num_banks() > 3);

    // The order of the added filters is important: it must match configuration
    // of the `split_filters_advanced()` method.

    // 2x 11bit id + mask filter bank: Matches 0, 1, 2
    // TODO: Make this accept also ID 2
    filters.enable_bank(
        0,
        [
            // accepts 0 and 1
            Mask16::frames_with_std_id(StandardId::new(0).unwrap(), StandardId::new(1).unwrap()),
            // accepts 0 and 2
            Mask16::frames_with_std_id(StandardId::new(0).unwrap(), StandardId::new(2).unwrap()),
        ],
    );

    // 2x 29bit id filter bank: Matches 4, 5
    filters.enable_bank(
        1,
        [
            ListEntry32::data_frames_with_id(ExtendedId::new(4).unwrap()),
            ListEntry32::data_frames_with_id(ExtendedId::new(5).unwrap()),
        ],
    );

    // 4x 11bit id filter bank: Matches 8, 9, 10, 11
    filters.enable_bank(
        2,
        [
            ListEntry16::data_frames_with_id(StandardId::new(8).unwrap()),
            ListEntry16::data_frames_with_id(StandardId::new(9).unwrap()),
            ListEntry16::data_frames_with_id(StandardId::new(10).unwrap()),
            ListEntry16::data_frames_with_id(StandardId::new(11).unwrap()),
        ],
    );

    // Drop filters to leave filter configuraiton mode.
    drop(filters);

    // Some messages shall pass the filters.
    for &id in &[0, 1, 2, 8, 9, 10, 11] {
        let frame_tx = Frame::new_data(StandardId::new(id).unwrap(), [id as u8]);
        block!(can.transmit(&frame_tx)).unwrap();
        let frame_rx = block!(can.receive()).unwrap();
        assert_eq!(frame_tx, frame_rx);
    }
    for &id in &[4, 5] {
        let frame_tx = Frame::new_data(ExtendedId::new(id).unwrap(), [id as u8]);
        block!(can.transmit(&frame_tx)).unwrap();
        let frame_rx = block!(can.receive()).unwrap();
        assert_eq!(frame_tx, frame_rx);
    }

    // Some messages shall not be received.
    for &id in &[3, 6, 7, 12] {
        let frame_tx = Frame::new_data(ExtendedId::new(id).unwrap(), [id as u8]);
        block!(can.transmit(&frame_tx)).unwrap();
        while !can.is_transmitter_idle() {}

        assert!(can.receive().is_err());
    }

    loop {}
}
