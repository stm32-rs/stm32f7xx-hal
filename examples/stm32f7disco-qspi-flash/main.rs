//! Example code showing how to use the MT25QL128ABA on the STM32F746G Discovery Board.
//! The intended behavior of the example is to write a known pattern to flash memory and
//! read it back using both polling and DMA indirect modes of the QSPI HAL driver.
//! The example will panic on failure and print messages over the debugger on success.
//! See `mt25q.rs` for more details on the QSPI HAL driver.

#![no_main]
#![no_std]

extern crate panic_semihosting;

use core::pin::Pin;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f7xx_hal::{
    dma::{Handle, Stream7, DMA},
    pac::{self, DMA2},
    prelude::*,
    rcc::{HSEClock, HSEClockMode, RccExt},
    state,
};

mod mt25q;

#[entry]
fn main() -> ! {
    let pac_periph = pac::Peripherals::take().unwrap();
    let mut rcc = pac_periph.RCC;

    // Initialize flash driver, which will initialize QSPI driver
    let mut mt25q = mt25q::Mt25q::new(
        &mut rcc,
        pac_periph.GPIOB,
        pac_periph.GPIOD,
        pac_periph.GPIOE,
        pac_periph.QUADSPI,
    );

    // Init clocks
    let hse_cfg = HSEClock::new(25_000_000.Hz(), HSEClockMode::Oscillator);
    let mut rcc = rcc.constrain();

    // Setup DMA
    let dma = DMA::new(pac_periph.DMA2);
    let stream = dma.streams.stream7;
    let dma = dma.handle.enable(&mut rcc.ahb1);

    // Ramp up clocks to 216 MHz
    rcc.cfgr.hse(hse_cfg).sysclk(216_000_000.Hz()).freeze();

    // Check that we can communicate with the flash device
    mt25q.check_id();

    memory_example_polling(&mut mt25q);
    memory_example_dma(&mut mt25q, &dma, stream);

    loop {}
}

fn memory_example_polling(mt25q: &mut mt25q::Mt25q) {
    // Create a set of buffers in RAM that will mirror flash memory at address `ADDR` of size `LEN`
    const ADDR: u32 = 0x7003;
    const LEN: usize = 1035;
    let mut read_buffer: [u8; LEN] = [0; LEN];
    let mut write_buffer: [u8; LEN] = [0; LEN];
    for i in 0..LEN {
        write_buffer[i] = i as u8;
    }

    ///////////////////////
    // Test erase + read //
    ///////////////////////

    let (num_erase, addr_erase) = mt25q.erase(ADDR, LEN);
    assert!(LEN <= num_erase as usize);
    assert!(addr_erase <= ADDR);

    mt25q.read(&mut read_buffer, ADDR, LEN);
    for i in 0..LEN {
        assert!(read_buffer[i] == 0xFF);
    }

    ///////////////////////
    // Test write + read //
    ///////////////////////

    mt25q.write(ADDR, &mut write_buffer, LEN);
    mt25q.read(&mut read_buffer, ADDR, LEN);
    for i in 0..LEN {
        if write_buffer[i] != read_buffer[i] {
            panic!(
                "Error: Mismatch at address {:X}. Expected {:X} but read {:X}",
                ADDR + i as u32,
                write_buffer[i],
                read_buffer[i]
            );
        }
    }

    hprintln!("Flash device memory test successful!").unwrap();
}

fn memory_example_dma(
    mt25q: &mut mt25q::Mt25q,
    dma: &Handle<DMA2, state::Enabled>,
    stream: Stream7<DMA2>,
) {
    // Create a buffer in RAM that will mirror flash memory at address `ADDR` of size `LEN`
    const ADDR: u32 = 0x7000;
    const LEN: usize = mt25q::SUBSECTOR_SIZE as usize;
    static mut READ_BUFFER: [u8; LEN] = [0; LEN];

    // Temporary working memory
    const PAGE_SIZE: usize = mt25q::PAGE_SIZE as usize;
    static mut PAGE_BUFFER: [u8; PAGE_SIZE] = [0; PAGE_SIZE];

    // Set the page buffer to some random data
    unsafe {
        for i in 0..PAGE_SIZE {
            PAGE_BUFFER[i] = i as u8;
        }
    }

    // Create pinned versions for DMA transfers
    let mut stream = stream;
    let mut read_buffer = unsafe { Pin::new(&mut READ_BUFFER) };
    let mut page_buffer = unsafe { Pin::new(&mut PAGE_BUFFER) };

    ///////////////////////
    // Test erase + read //
    ///////////////////////

    let (num_erase, addr_erase) = mt25q.erase(ADDR, LEN);
    assert!(LEN <= num_erase as usize);
    assert!(addr_erase <= ADDR);

    let read_resources = mt25q.read_dma(read_buffer, ADDR, LEN, dma, stream);
    for i in 0..LEN {
        assert!(read_resources.buffer[i] == 0xFF);
    }

    stream = read_resources.stream;
    read_buffer = read_resources.buffer;

    ///////////////////////
    // Test write + read //
    ///////////////////////

    // For writing with DMA, caller must break the writes down into flash memory pages.
    // Note that since ADDR is page aligned and LEN is a multiple of the page size some
    // page boundry math is ignored here. See the polling write function in the `mt25q`
    // driver for a more advanced example dealing with unaligned page boundries.
    let mut curr_addr: u32 = ADDR;
    let mut bytes_written: usize = 0;
    while bytes_written < LEN {
        let write_resources = mt25q.write_page_dma(curr_addr, page_buffer, PAGE_SIZE, dma, stream);
        stream = write_resources.stream;
        page_buffer = write_resources.buffer;
        bytes_written += PAGE_SIZE;
        curr_addr += PAGE_SIZE as u32;
    }

    mt25q.read_dma(read_buffer, ADDR, LEN, dma, stream);
    for i in 0..LEN {
        unsafe {
            if PAGE_BUFFER[i % PAGE_SIZE] != READ_BUFFER[i] {
                panic!(
                    "Error: Mismatch at address {:X}. Expected {:X} but read {:X}",
                    ADDR + i as u32,
                    PAGE_BUFFER[i % PAGE_SIZE],
                    READ_BUFFER[i]
                );
            }
        }
    }

    hprintln!("Flash device memory DMA test successful!").unwrap();
}
