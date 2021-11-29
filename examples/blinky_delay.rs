//! "Blinky" using delays instead of a timer

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use embedded_hal::delay::blocking::DelayUs;
use stm32f7xx_hal::{delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let gpioi = p.GPIOI.split();
    let mut led = gpioi.pi1.into_push_pull_output();

    // Constrain clocking registers
    let rcc = p.RCC.constrain();

    // Configure clock and freeze it
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        led.set_high();
        delay.delay_ms(500).unwrap();

        led.set_low();
        delay.delay_ms(500).unwrap();
    }
}
