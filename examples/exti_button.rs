#![no_main]
#![no_std]

/// This example demonstrates how to use the ExtiPin trait for a GPIO input pin to capture a
/// push button press. This example was written for the Nucleo-F767ZI board from the Nucleo-144
/// family of boards, targeting the STM32F767ZI microcontroller. To port this to another board,
/// change the GPIOs used for the push button and for the debug LED. Note that the EXTI number
/// may change if using a new button, meaning that the interrupt handler will need to change also.
///
/// The intended behavior of the example is that when the user presses the button, an LED is
/// toggled.

extern crate panic_halt;

use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32f7xx_hal::gpio::{Edge, ExtiPin};
use stm32f7xx_hal::{pac, interrupt, prelude::*};

const SYSCFG_EN: u32 = 14;

#[entry]
fn main() -> ! {
    let pac_periph = pac::Peripherals::take().unwrap();

    let rcc = pac_periph.RCC.constrain();

    // TODO: This took a long time to figure out, is there a way to bake this into GPIO/EXTI?
    unsafe {
        &(*pac::RCC::ptr())
            .apb2enr
            .modify(|r, w| w.bits(r.bits() | (1 << SYSCFG_EN)));
    }

    rcc.cfgr.sysclk(216.mhz()).freeze();

    // Push button configuration
    let mut syscfg = pac_periph.SYSCFG;
    let mut exti = pac_periph.EXTI;
    let gpioc = pac_periph.GPIOC.split();
    let mut button = gpioc.pc13.into_floating_input();
    button.make_interrupt_source(&mut syscfg);
    button.trigger_on_edge(&mut exti, Edge::RISING);
    button.enable_interrupt(&mut exti);
    unsafe {
        NVIC::unmask::<interrupt>(interrupt::EXTI15_10);
    }

    loop {}
}

#[interrupt]
fn EXTI15_10() {
    static mut COUNT: u32 = 0;

    unsafe {
        // TODO: Is there a safe alternative? Using a mutable static GPIO pin is also unsafe
        let pac_periph = pac::Peripherals::steal();

        // Clear the push button interrupt
        let gpioc = pac_periph.GPIOC.split();
        let mut button = gpioc.pc13.into_floating_input();
        button.clear_interrupt_pending_bit();

        // Blink an LED for debug purposes
        let gpiob = pac_periph.GPIOB.split();
        let mut led1 = gpiob.pb0.into_push_pull_output().downgrade();
        if *COUNT & 0x1 == 0x01 {
            led1.set_high().ok();
        } else {
            led1.set_low().ok();
        }
    }

    *COUNT += 1;
}
