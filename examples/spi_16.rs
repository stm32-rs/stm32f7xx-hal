#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use stm32f7xx_hal::{
    pac,
    prelude::*,
    spi::{self, Spi},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let gpioa = p.GPIOA.split();
    let gpioc = p.GPIOC.split();
    let gpiod = p.GPIOD.split();

    // Configure pin for button. This happens to be the pin for the USER button
    // on the NUCLEO-F746ZG board.
    let button = gpioc.pc13.into_floating_input();

    // Prepare pins for SPI
    let mut ncs = gpiod.pd14.into_push_pull_output();
    let sck = gpioa.pa5.into_alternate_af5();
    let mosi = gpioa.pa7.into_alternate_af5();

    // Set NCS pin to high (disabled) initially
    ncs.set_high().unwrap();

    // Initialize SPI
    let mut spi = Spi::new(p.SPI1, (sck, spi::NoMiso, mosi)).enable::<u16>(
        &mut rcc,
        spi::ClockDivider::DIV32,
        embedded_hal::spi::MODE_0,
    );

    // Use a button to control output via the Maxim Integrated MAX5214 DAC.
    loop {
        let data = if button.is_high().unwrap() {
            0xffff
        } else {
            0x0000
        };

        let word: u16 = (0b01 << 14) |   // write-through mode
            (data & 0x3fff); // data bits

        ncs.set_low().unwrap();
        spi.write(&[word]).unwrap();
        ncs.set_high().unwrap();
    }
}
