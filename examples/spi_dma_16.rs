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

    let dma = DMA::new(p.DMA2);
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

    // Prepare DMA streams
    let mut rx_stream = dma.streams.stream0;
    let mut tx_stream = dma.streams.stream3;

    let dma = dma.handle.enable(&mut rcc);

    // Set NCS pin to high (disabled) initially
    ncs.set_high().unwrap();

    // Initialize SPI
    let mut spi = Spi::new(p.SPI1, (sck, spi::NoMiso, mosi)).enable(
        &mut rcc,
        spi::ClockDivider::DIV32,
        embedded_hal::spi::MODE_0,
    );

    // Create the buffer we're going to use for DMA. This is safe, as this
    // function won't return as long as the program runs, so there's no chance
    // of anyone else using the same static.
    static mut BUFFER: [u16; 1] = [0; 1];
    let mut buffer = unsafe { Pin::new(&mut BUFFER) };

    // Use a button to control output via the Maxim Integrated MAX5214 DAC.
    loop {
        let data = if button.is_high().unwrap() {
            0xffff
        } else {
            0x0000
        };

        buffer[0] = (0b01 << 14) |   // write-through mode
            (data & 0x3fff); // data bits

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

        buffer = res.buffer;
        spi = res.target;
        rx_stream = res.rx_stream;
        tx_stream = res.tx_stream;
    }
}
