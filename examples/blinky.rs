//! Blinks an LED

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use stm32f7xx_hal as hal;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let gpioi = p.GPIOI.split();
    let mut led = gpioi.pi1.into_push_pull_output();

    loop {
        for _ in 0..10_000 {
            led.set_high().expect("GPIO can never fail");
        }
        for _ in 0..10_000 {
            led.set_low().expect("GPIO can never fail");
        }
    }
}
