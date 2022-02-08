//! Test the PWM

//#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;

use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
    let channels = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());

    let pwm = dp.TIM2.pwm(channels, 20_000u32.Hz(), &clocks);
    let (mut ch1, mut ch2) = pwm;
    let max_duty = ch1.get_max_duty();
    ch1.set_duty(max_duty / 4);
    ch1.enable();
    ch2.set_duty(max_duty / 2);
    ch2.enable();

    loop {}
}
