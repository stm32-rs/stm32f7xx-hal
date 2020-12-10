//! Driver for the MT25QL128ABA QSPI flash device located on the STM32F746G Discovery Board.

#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use stm32f7xx_hal::{
    gpio::{GpioExt, Speed},
    pac::{self, GPIOB, GPIOD, GPIOE, QUADSPI, RCC},
    rcc::{HSEClock, HSEClockMode, RccExt},
    qspi,
    prelude::*,
};


#[entry]
fn main() -> ! {

    let pac_periph = pac::Peripherals::take().unwrap();
    let mut rcc = pac_periph.RCC;

    let mut qspi = FlashDevice::init(
        &mut rcc,
        pac_periph.GPIOB,
        pac_periph.GPIOD,
        pac_periph.GPIOE,
        pac_periph.QUADSPI,
    );

    let hse_cfg = HSEClock::new(25.mhz(), HSEClockMode::Oscillator);
    rcc.constrain().cfgr.hse(hse_cfg).sysclk(216.mhz()).freeze();
    
    loop {

    }
}
struct FlashDevice;

#[allow(dead_code)]
impl FlashDevice {
    // Device constants
    const CMD_READ_ID: u8            = 0x9F;
    const CMD_MEM_READ: u8           = 0x6B;
    const CMD_MEM_PROGRAM: u8        = 0x32;
    const CMD_BULK_ERASE: u8         = 0xC7;
    const CMD_SUBSECT_ERASE: u8      = 0x20;
    const CMD_READ_FLAG_STATUS: u8   = 0x70;
    const CMD_WRITE_ENABLE: u8       = 0x06;
    const DEVICE_ID_MANF: u8         = 0x20;
    const DEVICE_ID_MEMT: u8         = 0xBA;
    const DEVICE_ID_MEMC: u8         = 0x18;
    const DEVICE_MAX_ADDRESS: u32    = 0x00FF_FFFF;
    const DEVICE_SUBSECTOR_SIZE: u32 = 4096;
    const DEVICE_PAGE_SIZE: u32      = 256;

    pub fn init(rcc: &mut RCC, gpiob: GPIOB, gpiod: GPIOD, gpioe: GPIOE, quadspi: QUADSPI) -> qspi::Qspi {

        // Setup GPIO pins
        let gpiob = gpiob.split();
        let gpiod = gpiod.split();
        let gpioe = gpioe.split();

        let _qspi_d0 = gpiod
            .pd11
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d1 = gpiod
            .pd12
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d2 = gpioe
            .pe2
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d3 = gpiod
            .pd13
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_clk = gpiob
            .pb2
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_ncs = gpiob
            .pb6
            .into_alternate_af10()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        // Setup QSPI driver
        qspi::Qspi::new(rcc, quadspi, 24)
    }
}