#![no_main]
#![no_std]

extern crate panic_semihosting;

use core::pin::Pin;

use cortex_m::{asm, interrupt};
use cortex_m_rt::entry;
use stm32f7xx_hal::{
    dma::{self, DMA},
    pac,
    prelude::*,
    spi::{self, Spi},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let dma = DMA::new(p.DMA1);
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();

    // Prepare status LEDS. These happen to be the red and green ones on the
    // NUCLEO-F746ZG board.
    let mut green = gpiob.pb0.into_push_pull_output();
    let mut red = gpiob.pb14.into_push_pull_output();

    // Prepare pins for SPI
    let mut ncs = gpioc.pc9.into_push_pull_output();
    let sck = gpioc.pc10.into_alternate_af6();
    let miso = gpioc.pc11.into_alternate_af6();
    let mosi = gpioc.pc12.into_alternate_af6();

    // Prepare DMA streams
    let mut rx_stream = dma.streams.stream0;
    let mut tx_stream = dma.streams.stream5;

    let dma = dma.handle.enable(&mut rcc);

    // Set NCS pin to high (disabled) initially
    ncs.set_high().unwrap();

    // Initialize SPI
    let mut spi = Spi::new(p.SPI3, (sck, miso, mosi)).enable(
        &mut rcc,
        spi::ClockDivider::DIV32,
        spi::Mode {
            polarity: spi::Polarity::IdleHigh,
            phase: spi::Phase::CaptureOnSecondTransition,
        },
    );

    // Create the buffer we're going to use for DMA. This is safe, as this
    // function won't return as long as the program runs, so there's no chance
    // of anyone else using the same static.
    static mut BUFFER: [u8; 2] = [0; 2];
    let mut buffer = unsafe { Pin::new(&mut BUFFER) };

    loop {
        // Read WHO_AM_I register of an MPU9250 sensor.
        // Write address for WHO_AM_I register an an MPU9250 sensor
        buffer[0] = 0x75 | 0x80;

        // Prepare DMA transfer
        let mut transfer = spi.transfer_all(buffer, &dma, &dma, rx_stream, tx_stream);

        // Start DMA transfer and wait for it to finish
        ncs.set_low().unwrap();
        let res = interrupt::free(|_| {
            transfer.enable_interrupts(
                &dma,
                &dma,
                dma::Interrupts {
                    transfer_complete: true,
                    transfer_error: true,
                    direct_mode_error: true,
                    ..dma::Interrupts::default()
                },
            );

            let transfer = transfer.start(&dma, &dma);

            asm::wfi();

            transfer.wait(&dma, &dma).unwrap()
        });
        ncs.set_high().unwrap();

        // Assign everything we've moved to the DMA transfer to the local
        // variables it came from, so it's available again in the next loop
        // iteration.
        buffer = res.buffer;
        spi = res.target;
        rx_stream = res.rx_stream;
        tx_stream = res.tx_stream;

        // The WHO_AM_I register should always return 0x71.
        if buffer[1] == 0x71 {
            green.set_high().unwrap();
            red.set_low().unwrap();
        } else {
            red.set_high().unwrap();
            green.set_low().unwrap();
        }
    }
}
