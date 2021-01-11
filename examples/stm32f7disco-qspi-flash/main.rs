//! Example code showing how to use the MT25QL128ABA on the STM32F746G Discovery Board.
//! The intended behavior of the example is to write a known pattern to flash memory and
//! read it back using both polling and DMA indirect modes of the QSPI HAL driver.
//! The example will panic on failure and print messages over the debugger on success.
//! See `mt25q.rs` for more details on the QSPI HAL driver.

#![no_main]
#![no_std]

extern crate panic_semihosting;

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
    let hse_cfg = HSEClock::new(25.mhz(), HSEClockMode::Oscillator);
    let mut rcc = rcc.constrain();

    // Setup DMA
    let dma = DMA::new(pac_periph.DMA2);
    let stream = dma.streams.stream7;
    let dma = dma.handle.enable(&mut rcc.ahb1);

    // Ramp up clocks to 216 MHz
    rcc.cfgr.hse(hse_cfg).sysclk(216.mhz()).freeze();

    // Check that we can communicate with the flash device
    mt25q.check_id();

    memory_example_polling(&mut mt25q);
    memory_example_dma(&mut mt25q, &dma, stream);

    loop {}
}

fn memory_example_polling(mt25q: &mut mt25q::Mt25q) {
    // Create a set of buffers for a memory at address `ADDR` of size `LEN` bytes
    const ADDR: u32 = 0x7003;
    const LEN: usize = 1035;
    let mut read_buffer: [u8; LEN] = [0; LEN];
    let mut write_buffer: [u8; LEN] = [0; LEN];
    for i in 0..LEN {
        write_buffer[i] = i as u8;
    }

    // Test erase + read
    let (num_erase, addr_erase) = mt25q.erase(ADDR, LEN);
    assert!(LEN <= num_erase as usize);
    assert!(addr_erase <= ADDR);

    mt25q.read(&mut read_buffer, ADDR, LEN);
    for i in 0..LEN {
        assert!(read_buffer[i] == 0xFF);
    }

    // Test write + read
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
    // Create a set of buffers for a memory at address `ADDR` of size `LEN` bytes
    const ADDR: u32 = 0x7000;
    const LEN: usize = 4096;
    let mut read_buffer: [u8; LEN] = [0; LEN];
    let mut write_buffer: [u8; LEN] = [0; LEN];
    for i in 0..LEN {
        write_buffer[i] = i as u8;
    }

    // Test erase + read
    let (num_erase, addr_erase) = mt25q.erase(ADDR, LEN);
    assert!(LEN <= num_erase as usize);
    assert!(addr_erase <= ADDR);

    let stream = mt25q.read_dma(&mut read_buffer, ADDR, LEN, dma, stream);
    for i in 0..LEN {
        assert!(read_buffer[i] == 0xFF);
    }

    // Test write + read
    let stream = mt25q.write_dma(ADDR, &mut write_buffer, LEN, dma, stream);
    mt25q.read_dma(&mut read_buffer, ADDR, LEN, dma, stream);
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

    hprintln!("Flash device memory DMA test successful!").unwrap();
}
