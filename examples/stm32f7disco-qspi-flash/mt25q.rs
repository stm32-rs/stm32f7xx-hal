//! Basic driver for the MT25QL128ABA. Demonstrates how the QSPI HAL
//! driver can be used to create a driver for a QSPI flash memory chip.

use as_slice::AsSlice;
use core::ops::Deref;
use core::pin::Pin;
use stm32f7xx_hal::{
    dma::{Handle, Stream7, TransferResources},
    gpio::{GpioExt, Speed},
    pac::{DMA2, GPIOB, GPIOD, GPIOE, QUADSPI, RCC},
    qspi::{Qspi, QspiTransaction, QspiWidth, RxTx},
    state,
};

const CMD_READ_ID: u8 = 0x9F;
const CMD_MEM_READ: u8 = 0x6B;
const CMD_MEM_PROGRAM: u8 = 0x32;
const CMD_SUBSECT_ERASE: u8 = 0x20;
const CMD_READ_STATUS: u8 = 0x70;
const CMD_WRITE_ENABLE: u8 = 0x06;
const ID_MANF: u8 = 0x20;
const ID_MEMT: u8 = 0xBA;
const ID_MEMC: u8 = 0x18;
const MAX_ADDR: u32 = 0x00FF_FFFF;
pub const SUBSECTOR_SIZE: u32 = 4096;
pub const PAGE_SIZE: u32 = 256;

pub struct Mt25q {
    driver: Qspi,
}

impl Mt25q {
    /// Initialize driver.
    pub fn new(rcc: &mut RCC, gpiob: GPIOB, gpiod: GPIOD, gpioe: GPIOE, quadspi: QUADSPI) -> Self {
        let gpiob = gpiob.split();
        let gpiod = gpiod.split();
        let gpioe = gpioe.split();

        let _qspi_d0 = gpiod
            .pd11
            .into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d1 = gpiod
            .pd12
            .into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d2 = gpioe
            .pe2
            .into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_d3 = gpiod
            .pd13
            .into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_clk = gpiob
            .pb2
            .into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _qspi_ncs = gpiob
            .pb6
            .into_alternate::<10>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let driver = Qspi::new(rcc, quadspi, 24, 3);
        Mt25q { driver }
    }

    /// Check the identification bytes of the flash device to validate communication.
    pub fn check_id(&mut self) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: CMD_READ_ID,
            address: None,
            dummy: 0,
            data_len: Some(3),
        };

        let mut id = [0, 0, 0];
        self.driver.read(&mut id, transaction).unwrap();

        if id[0] != ID_MANF || id[1] != ID_MEMT || id[2] != ID_MEMC {
            panic!("Error: Device ID mismatch!");
        }
    }

    /// Blocking DMA read.
    ///
    /// NOTE: This function could be easily modified for non-blocking by returning
    /// the handle to the ongoing DMA `Transfer`.
    pub fn read_dma<B>(
        &mut self,
        dst: Pin<B>,
        src: u32,
        len: usize,
        dma: &Handle<DMA2, state::Enabled>,
        stream: Stream7<DMA2>,
    ) -> TransferResources<RxTx<QUADSPI>, B>
    where
        B: Deref + 'static,
        B::Target: AsSlice<Element = u8>,
    {
        assert!(len > 0);
        assert!(src + (len as u32) <= MAX_ADDR);

        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_MEM_READ,
            address: Some(src & MAX_ADDR),
            dummy: 8,
            data_len: Some(len),
        };

        // Start the DMA read
        let rx_transfer = self.driver.read_all(dst, transaction, dma, stream).unwrap();

        // Wait for DMA read to finish
        rx_transfer.wait(&dma).unwrap()
    }

    /// Blocking DMA page write.
    ///
    /// NOTE: This function could be easily modified for non-blocking by returning
    /// the handle to the ongoing DMA `Transfer`. However for this flash chip it
    /// would not be very useful, since writes are limited by page size. The only
    /// way to acheive non-blocking writes would be to use interrupts to reload DMA
    /// for each page.
    pub fn write_page_dma<B>(
        &mut self,
        dst: u32,
        src: Pin<B>,
        len: usize,
        dma: &Handle<DMA2, state::Enabled>,
        stream: Stream7<DMA2>,
    ) -> TransferResources<RxTx<QUADSPI>, B>
    where
        B: Deref + 'static,
        B::Target: AsSlice<Element = u8>,
    {
        assert!(len > 0);
        assert!(dst + (len as u32) <= MAX_ADDR);

        // Constraints for writes: (1) Must be <= 256 bytes, (2) must not cross a page boundry
        assert!(len as u32 <= PAGE_SIZE);
        assert!(dst % PAGE_SIZE == 0);

        self.write_enable();

        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_MEM_PROGRAM,
            address: Some(dst & MAX_ADDR),
            dummy: 0,
            data_len: Some(len),
        };

        // Start the DMA write
        let tx_transfer = self
            .driver
            .write_all(src, transaction, dma, stream)
            .unwrap();

        // Wait for DMA write to finish
        let resources = tx_transfer.wait(&dma).unwrap();

        self.poll_status();

        resources
    }

    /// Blocking polling read.
    pub fn read(&mut self, dst: &mut [u8], src: u32, len: usize) {
        assert!(len > 0);
        assert!(src + (len as u32) <= MAX_ADDR);

        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_MEM_READ,
            address: Some(src & MAX_ADDR),
            dummy: 8,
            data_len: Some(len),
        };

        self.driver.read(dst, transaction).unwrap();
    }

    /// Blocking polling write.
    pub fn write(&mut self, dst: u32, src: &mut [u8], len: usize) {
        assert!(len > 0);
        assert!(dst + (len as u32) <= MAX_ADDR);

        let mut outer_idx: usize = 0;
        let mut curr_addr: u32 = dst;
        let mut curr_len: usize = len;

        // Constraints for writes: (1) Must be <= 256 bytes, (2) must not cross a page boundry
        while curr_len > 0 {
            self.write_enable();

            let start_page = curr_addr - (curr_addr % PAGE_SIZE);
            let end_page = start_page + PAGE_SIZE;
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
                address: Some(curr_addr & MAX_ADDR),
                dummy: 0,
                data_len: Some(size),
            };

            let buf = unsafe { core::slice::from_raw_parts(&src[outer_idx] as *const u8, size) };
            self.driver.write(buf, transaction).unwrap();
            self.poll_status();

            outer_idx += size;
            curr_addr += size as u32;
            curr_len -= size;
        }
    }

    /// Erase `len` bytes at address `src` sector-by-sector. If `src` is not sector aligned, the
    /// start of sector it resides in will be the starting address for the erase. A pair is
    /// returned containing the total number of bytes erased and the erase starting address.
    pub fn erase(&mut self, src: u32, len: usize) -> (u32, u32) {
        assert!(len > 0);
        assert!(src + (len as u32) <= MAX_ADDR);

        let mut num_erased_bytes: u32 = 0;
        let mut addr: u32 = src - (src % SUBSECTOR_SIZE);
        let start_addr = addr;

        // The smallest possible erase is a subsector (4KB)
        while num_erased_bytes < (len as u32) {
            self.write_enable();

            let transaction = QspiTransaction {
                iwidth: QspiWidth::SING,
                awidth: QspiWidth::SING,
                dwidth: QspiWidth::NONE,
                instruction: CMD_SUBSECT_ERASE,
                address: Some(addr & MAX_ADDR),
                dummy: 0,
                data_len: None,
            };

            let mut dummy = [0];
            self.driver.read(&mut dummy, transaction).unwrap();

            num_erased_bytes += SUBSECTOR_SIZE;
            addr += SUBSECTOR_SIZE;

            self.poll_status();
        }

        (num_erased_bytes, start_addr)
    }

    /// Poll the flag status register on the flash device until the the operation is complete.
    pub fn poll_status(&mut self) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: CMD_READ_STATUS,
            address: None,
            dummy: 0,
            data_len: Some(1),
        };

        let mut status = [0];
        while status[0] & 0x80 == 0 {
            self.driver.read(&mut status, transaction.clone()).unwrap();
        }
    }

    /// Send the write enable command to the flash device.
    pub fn write_enable(&mut self) {
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
        self.driver.read(&mut dummy, transaction).unwrap()
    }
}
