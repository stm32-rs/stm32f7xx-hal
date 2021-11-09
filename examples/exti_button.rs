//! This example demonstrates how to use the ExtiPin trait for a GPIO input pin to capture a
//! push button press. This example was written for the Nucleo-F767ZI board from the Nucleo-144
//! family of boards, targeting the STM32F767ZI microcontroller. To port this to another board,
//! change the GPIOs used for the push button and for the debug LED. Note that the EXTI number
//! may change if using a new button, meaning that the interrupt handler will need to change also.
//!
//! The intended behavior of the example is that when the user presses the button, an LED is
//! toggled.

#![no_main]
#![no_std]

extern crate panic_halt;

use core::cell::{Cell, RefCell};
use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32f7xx_hal::gpio::gpioc::PC13;
use stm32f7xx_hal::gpio::{Edge, ExtiPin, Floating, Input};
use stm32f7xx_hal::{interrupt, pac, prelude::*};

// Semaphore for synchronization
static SEMAPHORE: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));

// GPIO pin that main thread and interrupt handler must share
static BUTTON_PIN: Mutex<RefCell<Option<PC13<Input<Floating>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let pac_periph = pac::Peripherals::take().unwrap();

    // Debug LED configuration
    let gpiob = pac_periph.GPIOB.split();
    let mut led1 = gpiob.pb0.into_push_pull_output();

    // Freeze clocks
    let mut rcc = pac_periph.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(216_000_000.Hz()).freeze();

    // Push button configuration
    let mut syscfg = pac_periph.SYSCFG;
    let mut exti = pac_periph.EXTI;
    let gpioc = pac_periph.GPIOC.split();
    let mut button = gpioc.pc13.into_floating_input();
    button.make_interrupt_source(&mut syscfg, &mut rcc.apb2);
    button.trigger_on_edge(&mut exti, Edge::Rising);
    button.enable_interrupt(&mut exti);

    // Save information needed by the interrupt handler to the global variable
    free(|cs| {
        BUTTON_PIN.borrow(cs).replace(Some(button));
    });

    // Enable the button interrupt
    unsafe {
        NVIC::unmask::<interrupt>(interrupt::EXTI15_10);
    }

    loop {
        // Wait for the interrupt to fire
        free(|cs| {
            if SEMAPHORE.borrow(cs).get() == false {
                // Toggle debug LED
                led1.toggle();

                SEMAPHORE.borrow(cs).set(true);
            }
        });
    }
}

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        match BUTTON_PIN.borrow(cs).borrow_mut().as_mut() {
            // Clear the push button interrupt
            Some(b) => b.clear_interrupt_pending_bit(),

            // This should never happen
            None => (),
        }

        // Signal that the interrupt fired
        SEMAPHORE.borrow(cs).set(false);
    });
}
