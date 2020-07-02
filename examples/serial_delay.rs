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
    delay::Delay,
    pac,
    prelude::*,
    serial::{self, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    let mut delay = Delay::new(cp.SYST, clocks);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    let tx = gpioa.pa9.into_alternate_af7();
    let rx = gpiob.pb7.into_alternate_af7();

    let serial = Serial::new(
        p.USART1,
        (tx, rx),
        clocks,
        serial::Config {
            baud_rate: 115_200.bps(),
            oversampling: serial::Oversampling::By16,
        },
    );
    let (mut tx, _) = serial.split();

    let hello: &str = "Hello, I'm a STM32F7xx!\r\n";
    loop {
        tx.write_str(hello).unwrap();
        delay.delay_ms(500_u16);
    }
}
