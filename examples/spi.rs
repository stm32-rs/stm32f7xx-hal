#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use embedded_hal::spi::blocking::TransferInplace;
use stm32f7xx_hal::{
    pac,
    prelude::*,
    spi::{self, Spi},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();

    // Prepare status LEDS. These happen to be the red and green ones on the
    // NUCLEO-F746ZG board.
    let mut green = gpiob.pb0.into_push_pull_output();
    let mut red = gpiob.pb14.into_push_pull_output();

    // Prepare pins for SPI
    let mut ncs = gpioc.pc9.into_push_pull_output();
    let sck = gpioc.pc10.into_alternate();
    let miso = gpioc.pc11.into_alternate();
    let mosi = gpioc.pc12.into_alternate();

    // Set NCS pin to high (disabled) initially
    ncs.set_high();

    // Initialize SPI
    let mut spi = Spi::new(p.SPI3, (sck, miso, mosi)).enable::<u8>(
        &mut rcc.apb1,
        spi::ClockDivider::DIV32,
        spi::Mode {
            polarity: spi::Polarity::IdleHigh,
            phase: spi::Phase::CaptureOnSecondTransition,
        },
    );

    loop {
        // Read WHO_AM_I register of an MPU9250 sensor.
        let mut buffer = [0; 2];
        buffer[0] = 0x75 | 0x80;
        ncs.set_low();
        spi.transfer_inplace(&mut buffer).unwrap();
        ncs.set_high();

        // The WHO_AM_I register should always return 0x71.
        if buffer[1] == 0x71 {
            green.set_high();
            red.set_low();
        } else {
            red.set_high();
            green.set_low();
        }
    }
}
