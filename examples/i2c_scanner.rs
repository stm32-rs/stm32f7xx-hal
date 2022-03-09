//! Example of using I2C.
//! Scans available I2C devices on bus and print the result.

#![no_std]
#![no_main]

use core::ops::Range;

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::{hprint, hprintln};

use stm32f7xx_hal::{self as hal, gpio::GpioExt, pac, prelude::*};

const VALID_ADDR_RANGE: Range<u8> = 0x08..0x78;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpiob = dp.GPIOB.split();

    // Configure I2C1
    let scl = gpiob.pb8.into_alternate_open_drain::<4>();
    let sda = gpiob.pb7.into_alternate_open_drain::<4>();
    let mut i2c = hal::i2c::BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        hal::i2c::Mode::fast(100_000.Hz()),
        &clocks,
        &mut rcc.apb1,
        50_000,
    );

    hprintln!("Start i2c scanning...").expect("Error using hprintln.");
    hprintln!().unwrap();

    for addr in 0x00_u8..0x80 {
        // Write the empty array and check the slave response.
        let byte: [u8; 1] = [0; 1];
        if VALID_ADDR_RANGE.contains(&addr) && i2c.write(addr, &byte).is_ok() {
            hprint!("{:02x}", addr).unwrap();
        } else {
            hprint!("..").unwrap();
        }
        if addr % 0x10 == 0x0F {
            hprintln!().unwrap();
        } else {
            hprint!(" ").unwrap();
        }
    }

    hprintln!().unwrap();
    hprintln!("Done!").unwrap();

    loop {}
}
