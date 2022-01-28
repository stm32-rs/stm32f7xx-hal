//! Test the general purpose timers

//#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use core::fmt::Write;
use core::cell::{RefCell};

use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use cortex_m_semihosting::hio;

use cortex_m::peripheral::NVIC;

use stm32f7xx_hal::{
    interrupt,
    pac::{self, TIM2},
    prelude::*,
    timer::{Event, Timer},
};

static TIMER: Mutex<RefCell<Option<Timer<TIM2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "Starting timer...").unwrap();

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mut timer = Timer::tim2(dp.TIM2, 1.Hz(), clocks, &mut rcc.apb1);
    timer.listen(Event::TimeOut);

    // Save information needed by the interrupt handler to the global variable
    free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
    });

    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }

    loop {}
}

#[interrupt]
fn TIM2() {
    free(|cs| {
        TIMER.borrow(cs).borrow_mut().as_mut().unwrap().clear_interrupt(Event::TimeOut)
    });

    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "TIM2 intr!").unwrap();
}
