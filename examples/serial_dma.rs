//! Reads 4 bytes from USART then writes them back, both using DMA
//!
//! Echoing 4 bytes at a time makes for weird behavior, but is a better test for
//! DMA than doing it byte by byte would be.
//!
//! Note: This example is for the STM32F746

#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use core::pin::Pin;

use cortex_m::{asm, interrupt};
use cortex_m_rt::entry;
use stm32f7xx_hal::{
    dma::{self, DMA},
    pac,
    prelude::*,
    serial::{self, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let dma = DMA::new(p.DMA1);

    let mut rx_stream = dma.streams.stream1;
    let mut tx_stream = dma.streams.stream3;

    let dma = dma.handle.enable(&mut rcc);

    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    let gpiod = p.GPIOD.split();

    let tx = gpiod.pd8.into_alternate_af7();
    let rx = gpiod.pd9.into_alternate_af7();

    let serial = Serial::new(
        p.USART3,
        (tx, rx),
        clocks,
        serial::Config {
            baud_rate: 115_200.bps(),
            oversampling: serial::Oversampling::By16,
        },
    );
    let (mut tx, mut rx) = serial.split();

    // Create the buffer we're going to use for DMA. This is safe, as this
    // function won't return as long as the program runs, so there's no chance
    // of anyone else using the same static.
    static mut BUFFER: [u8; 4] = [0; 4];
    let mut buffer = unsafe { Pin::new(&mut BUFFER) };
    loop {
        // Read using DMA
        let mut transfer = rx.read_all(buffer, &dma, rx_stream);
        let res = interrupt::free(|_| {
            transfer.enable_interrupts(
                &dma,
                dma::Interrupts {
                    transfer_complete: true,
                    transfer_error: true,
                    direct_mode_error: true,
                    ..dma::Interrupts::default()
                },
            );

            let transfer = transfer.start(&dma);

            asm::wfi();

            transfer.wait(&dma).unwrap()
        });
        buffer = res.buffer;
        rx = res.target;
        rx_stream = res.stream;

        // Write using DMA
        let mut transfer = tx.write_all(buffer, &dma, tx_stream);
        let res = interrupt::free(|_| {
            transfer.enable_interrupts(
                &dma,
                dma::Interrupts {
                    transfer_complete: true,
                    transfer_error: true,
                    direct_mode_error: true,
                    ..dma::Interrupts::default()
                },
            );

            let transfer = transfer.start(&dma);

            asm::wfi();

            transfer.wait(&dma).unwrap()
        });
        buffer = res.buffer;
        tx = res.target;
        tx_stream = res.stream;
    }
}
