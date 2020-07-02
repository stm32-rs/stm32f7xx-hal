//! Test the general purpose timers

#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::fmt::Write;

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;

use stm32f7xx_hal::{
    interrupt, pac,
    prelude::*,
    timer::{Event, Timer},
};

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "Starting timer...").unwrap();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mut nvic = cp.NVIC;
    nvic.enable(pac::Interrupt::TIM2);
    let mut timer = Timer::tim2(dp.TIM2, 1.hz(), clocks, &mut rcc.apb1);
    timer.listen(Event::TimeOut);

    loop {}
}

#[interrupt]
fn TIM2() {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "TIM2 intr!").unwrap();
}
