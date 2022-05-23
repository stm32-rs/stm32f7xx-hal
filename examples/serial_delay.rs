//! Write a string to the serial port every half second.
//!
//! Note: This example is for the STM32F745/STM32F746

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use core::fmt::Write;

use cortex_m_rt::entry;
use stm32f7xx_hal::{
    pac,
    prelude::*,
    serial::{self, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216_000_000.Hz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    let tx = gpioa.pa9.into_alternate();
    let rx = gpiob.pb7.into_alternate();

    let serial = Serial::new(
        p.USART1,
        (tx, rx),
        &clocks,
        serial::Config {
            // Default to 115_200 bauds
            ..Default::default()
        },
    );
    let (mut tx, _) = serial.split();

    let hello: &str = "Hello, I'm a STM32F7xx!\r\n";
    loop {
        tx.write_str(hello).unwrap();
        delay.delay_ms(500_u16);
    }
}
