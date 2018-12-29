//! "Blinky" using delays instead of a timer

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use stm32f7xx_hal::{
    prelude::*,
    device,
    delay::Delay,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = device::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let gpioi = p.GPIOI.split();
    let mut led = gpioi.pi1.into_push_pull_output();

    // Constrain clocking registers
    let rcc = p.RCC.constrain();

    // Configure clock to and freeze it
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        led.set_high();
        delay.delay_ms(500_u16);

        led.set_low();
        delay.delay_ms(500_u16);
    }
}
