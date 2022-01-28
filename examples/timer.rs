//! Test the general purpose timers

//#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::fmt::Write;

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;

use cortex_m::peripheral::NVIC;

use stm32f7xx_hal::{
    embedded_time::rate::Extensions,
    interrupt, pac,
    prelude::*,
    timer::{Event, Timer},
};

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "Starting timer...").unwrap();

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }
    let mut timer = Timer::new(dp.TIM2, &clocks).count_down();
    timer.start(1.Hz());
    timer.listen(Event::TimeOut);

    loop {}
}

#[interrupt]
fn TIM2() {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "TIM2 intr!").unwrap();
}
