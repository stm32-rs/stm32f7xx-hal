#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

/// generates a random number and displays it to the openocd console
use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut rng = p.RNG.init();
    let val = rng.get_rand().unwrap();
    hprintln!("random value {}", val).unwrap();
    loop {
        core::hint::spin_loop();
    }
}
