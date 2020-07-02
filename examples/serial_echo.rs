//! Echo characters sent back to the serial port.
//!
//! Note: This example is for the STM32F745/STM32F746

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use nb::block;

use cortex_m_rt::entry;
use stm32f7xx_hal::{
    pac,
    prelude::*,
    serial::{self, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

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

    let (mut tx, mut rx) = serial.split();

    loop {
        let received = block!(rx.read()).unwrap_or('E' as u8);
        block!(tx.write(received)).ok();
    }
}
