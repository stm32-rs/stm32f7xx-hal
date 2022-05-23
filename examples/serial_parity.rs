//! Write characters to the serial port with parity.
//!
//! Note: This example is for the STM32F767

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
    let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    let mut delay = p.TIM5.delay_ms(&clocks);

    let gpiod = p.GPIOD.split();

    let tx = gpiod.pd5.into_alternate();
    let rx = gpiod.pd6.into_alternate();

    let serial = Serial::new(
        p.USART2,
        (tx, rx),
        &clocks,
        serial::Config {
            baud_rate: 56_700.bps(),
            oversampling: serial::Oversampling::By16,
            character_match: None,
            sysclock: false,
            parity: serial::Parity::ParityEven,
        },
    );

    let (mut tx, mut _rx) = serial.split();

    let mut byte: u8 = 0;

    loop {
        block!(tx.write(byte)).ok();

        byte = byte.wrapping_add(1);

        delay.delay(10.millis());
    }
}
