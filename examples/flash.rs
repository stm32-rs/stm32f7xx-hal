//! Erases a flash sector and programs data.

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use stm32f7xx_hal::{flash::Flash, pac};

const DATA: &[u8] = &[0, 1, 2, 3, 4];

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = Flash::new(p.FLASH);

    // The flash needs to be unlocked before any erase or program operations.
    flash.unlock();

    // Erase flash sector 3, which is located at address 0x0800C000
    flash.blocking_erase_sector(3).unwrap();

    // Program the DATA slice into the flash memory starting at offset 0xC00 from the
    // beginning of the flash memory.
    flash.blocking_program(0xC000, &DATA).unwrap();

    // Lock the flash memory to prevent any accidental modification of the flash content.
    flash.lock();

    // Create a slice that can be used to read the written data.
    #[allow(unsafe_code)]
    let flash_data = unsafe { core::slice::from_raw_parts(0x0800C000 as *const u8, DATA.len()) };

    // Compare the written data with the expected value.
    if flash_data == DATA {
        hprintln!("Flash programming successful").unwrap();
    }

    loop {}
}
