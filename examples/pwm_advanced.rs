//! Test the PWM

#![deny(warnings)]
#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;

use stm32f7xx_hal::pwm::{FaultMonitor, Polarity};
use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpioe = dp.GPIOE.split();

    let mut led_red = gpiob.pb14.into_push_pull_output();
    let btn = gpioc.pc13.into_pull_down_input();

    let channels = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate());
    let (mut control, pwm) = dp
        .TIM1
        .pwm_advanced(channels, &clocks)
        .prescaler(2)
        .period(250)
        .with_deadtime(200_000.nanoseconds())
        .with_break_pin(gpioe.pe15.into_alternate(), Polarity::ActiveHigh)
        .center_aligned()
        .finalize();

    let (mut ch1, mut ch2) = pwm;
    let max_duty = ch1.get_max_duty();
    ch1.set_duty(max_duty / 4);
    ch1.enable();
    ch2.set_duty(max_duty / 2);
    ch2.enable();

    loop {
        // Fault is active, turn on LED
        if control.is_fault_active() {
            led_red.set_high();
        }

        // Clear the fault when pressing the USER button
        if btn.is_high() {
            led_red.set_low();
            control.clear_fault();
        }
    }
}
