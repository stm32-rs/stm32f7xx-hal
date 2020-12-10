//! Example program showing how to use the QSPI driver in the HAL to interface
//! with a QSPI flash memory device. The MT25QL128ABA QSPI flash device (16 MB)
//! located on the STM32F746G Discovery Board is used.

#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f7xx_hal::{
    gpio::{GpioExt, Speed},
    pac::{self, GPIOB, GPIOD, GPIOE, QUADSPI, RCC},
    prelude::*,
    qspi::{Qspi, QspiTransaction, QspiWidth},
    rcc::{HSEClock, HSEClockMode, RccExt},
};

#[entry]
fn main() -> ! {
    let pac_periph = pac::Peripherals::take().unwrap();
    let mut rcc = pac_periph.RCC;

    // Initialize flash driver, which will initialize QSPI driver
    let mut qspi_driver: Qspi = flash_driver::init(
        &mut rcc,
        pac_periph.GPIOB,
        pac_periph.GPIOD,
        pac_periph.GPIOE,
        pac_periph.QUADSPI,
    );

    // Ramp up clocks to 216 MHz
    let hse_cfg = HSEClock::new(25.mhz(), HSEClockMode::Oscillator);
    rcc.constrain().cfgr.hse(hse_cfg).sysclk(216.mhz()).freeze();

    // Check that we can communicate with the flash device
    flash_driver::check_id(&mut qspi_driver);

    // Create a set of buffers for a memory at address `ADDR` of size `LEN` bytes
    const ADDR: u32 = 0x7003;
    const LEN: usize = 4121;
    let mut read_buffer: [u8; LEN] = [0; LEN];
    let mut write_buffer: [u8; LEN] = [0; LEN];
    for i in 0..LEN {
        write_buffer[i] = i as u8;
    }

    // Test erase + read
    let (num_erase, addr_erase) = flash_driver::erase(&mut qspi_driver, ADDR, LEN);
    assert!(LEN <= num_erase as usize);
    assert!(addr_erase <= ADDR);

    flash_driver::read(&mut qspi_driver, &mut read_buffer, ADDR, LEN);
    for i in 0..LEN {
        assert!(read_buffer[i] == 0xFF);
    }

    // Test write + read
    flash_driver::write(&mut qspi_driver, ADDR, &mut write_buffer, LEN);
    flash_driver::read(&mut qspi_driver, &mut read_buffer, ADDR, LEN);
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

    loop {}
}

/// Basic driver for the MT25QL128ABA using the QSPI driver in the HAL.
#[allow(dead_code)]
mod flash_driver {
    use super::*;

    // Device constants
    const CMD_READ_ID: u8 = 0x9F;
    const CMD_MEM_READ: u8 = 0x6B;
    const CMD_MEM_PROGRAM: u8 = 0x32;
    const CMD_BULK_ERASE: u8 = 0xC7;
    const CMD_SUBSECT_ERASE: u8 = 0x20;
    const CMD_READ_FLAG_STATUS: u8 = 0x70;
    const CMD_WRITE_ENABLE: u8 = 0x06;
    const DEVICE_ID_MANF: u8 = 0x20;
    const DEVICE_ID_MEMT: u8 = 0xBA;
    const DEVICE_ID_MEMC: u8 = 0x18;
    const DEVICE_MAX_ADDRESS: u32 = 0x00FF_FFFF;
    const DEVICE_SUBSECTOR_SIZE: u32 = 4096;
    const DEVICE_PAGE_SIZE: u32 = 256;

    pub fn init(rcc: &mut RCC, gpiob: GPIOB, gpiod: GPIOD, gpioe: GPIOE, quadspi: QUADSPI) -> Qspi {
        // Setup GPIO pins
        let gpiob = gpiob.split();
        let gpiod = gpiod.split();
        let gpioe = gpioe.split();

        let _qspi_d0 = gpiod
            .pd11
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d1 = gpiod
            .pd12
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d2 = gpioe
            .pe2
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d3 = gpiod
            .pd13
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_clk = gpiob
            .pb2
            .into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_ncs = gpiob
            .pb6
            .into_alternate_af10()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        // Setup QSPI driver
        Qspi::new(rcc, quadspi, 24)
    }

    /// Check the identification bytes of the flash device to validate communication.
    pub fn check_id(qspi_driver: &mut Qspi) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: CMD_READ_ID,
            address: None,
            dummy: 0,
            data_len: Some(3),
        };

        let mut device_id = [0, 0, 0];
        qspi_driver.polling_read(&mut device_id, transaction);

        hprintln!("Device ID:").unwrap();
        hprintln!("\t{:X}", device_id[0]).unwrap();
        hprintln!("\t{:X}", device_id[1]).unwrap();
        hprintln!("\t{:X}", device_id[2]).unwrap();

        if device_id[0] != DEVICE_ID_MANF
            || device_id[1] != DEVICE_ID_MEMT
            || device_id[2] != DEVICE_ID_MEMC
        {
            panic!("Error: Device ID mismatch!");
        }
    }

    /// Blocking read.
    pub fn read(qspi_driver: &mut Qspi, dst: &mut [u8], src: u32, len: usize) {
        assert!(len > 0);
        assert!(src + (len as u32) <= DEVICE_MAX_ADDRESS);

        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_MEM_READ,
            address: Some(src & DEVICE_MAX_ADDRESS),
            dummy: 8,
            data_len: Some(len),
        };

        qspi_driver.polling_read(dst, transaction);
    }

    /// Blocking write.
    pub fn write(qspi_driver: &mut Qspi, dst: u32, src: &mut [u8], len: usize) {
        assert!(len > 0);
        assert!(dst + (len as u32) <= DEVICE_MAX_ADDRESS);

        let mut outer_idx: usize = 0;
        let mut curr_addr: u32 = dst;
        let mut curr_len: usize = len;

        // Constraints for writes: (1) Must be <= 256 bytes, (2) must not cross a page boundry
        while curr_len > 0 {
            write_enable(qspi_driver);

            let start_page = curr_addr - (curr_addr % DEVICE_PAGE_SIZE);
            let end_page = start_page + DEVICE_PAGE_SIZE;
            let size: usize = if curr_addr + (curr_len as u32) > end_page {
                (end_page - curr_addr) as usize
            } else {
                curr_len
            };

            let transaction = QspiTransaction {
                iwidth: QspiWidth::SING,
                awidth: QspiWidth::SING,
                dwidth: QspiWidth::QUAD,
                instruction: CMD_MEM_PROGRAM,
                address: Some(curr_addr & DEVICE_MAX_ADDRESS),
                dummy: 0,
                data_len: Some(size),
            };

            qspi_driver.polling_write(src, transaction, outer_idx);
            poll_status(qspi_driver);

            curr_addr += size as u32;
            curr_len -= size;
            outer_idx += size;
        }
    }

    /// Erase `len` bytes at address `src` sector-by-sector. If `src` is not sector aligned, the
    /// start of sector it resides in will be the starting address for the erase. A pair is
    /// returned containing the total number of bytes erased and the erase starting address.
    pub fn erase(qspi_driver: &mut Qspi, src: u32, len: usize) -> (u32, u32) {
        assert!(len > 0);
        assert!(src + (len as u32) <= DEVICE_MAX_ADDRESS);

        let mut num_erased_bytes: u32 = 0;
        let mut addr: u32 = src - (src % DEVICE_SUBSECTOR_SIZE);
        let start_addr = addr;

        // The smallest possible erase is a subsector (4KB)
        while num_erased_bytes < (len as u32) {
            write_enable(qspi_driver);

            let transaction = QspiTransaction {
                iwidth: QspiWidth::SING,
                awidth: QspiWidth::SING,
                dwidth: QspiWidth::NONE,
                instruction: CMD_SUBSECT_ERASE,
                address: Some(addr & DEVICE_MAX_ADDRESS),
                dummy: 0,
                data_len: None,
            };

            let mut dummy = [0];
            qspi_driver.polling_read(&mut dummy, transaction);

            num_erased_bytes += DEVICE_SUBSECTOR_SIZE;
            addr += DEVICE_SUBSECTOR_SIZE;

            poll_status(qspi_driver);
        }

        (num_erased_bytes, start_addr)
    }

    /// Poll the flag status register on the flash device until the the operation is complete.
    pub fn poll_status(qspi_driver: &mut Qspi) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: CMD_READ_FLAG_STATUS,
            address: None,
            dummy: 0,
            data_len: Some(1),
        };

        let mut status = [0];
        while status[0] & 0x80 == 0 {
            qspi_driver.polling_read(&mut status, transaction.clone());
        }
    }

    /// Send the write enable command to the flash device.
    pub fn write_enable(qspi_driver: &mut Qspi) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::NONE,
            instruction: CMD_WRITE_ENABLE,
            address: None,
            dummy: 0,
            data_len: None,
        };

        let mut dummy = [0];
        qspi_driver.polling_read(&mut dummy, transaction)
    }
}
