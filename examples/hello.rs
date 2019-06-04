//! Prints "Hello, world" on the OpenOCD console

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_semihosting;
extern crate stm32f7xx_hal;

use cortex_m_semihosting::hprintln;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!").unwrap();
    loop {}
}
