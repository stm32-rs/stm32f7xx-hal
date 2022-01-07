//! CDC-ACM serial port example using polling in a busy loop.
//! Target board: any STM32F7 with an OTG FS/HS peripheral and a 25MHz HSE generator
//! This example works on the 32F723EDISCOVERY board.
//!
//! For FS operation:
//! > cargo run --example usb_serial --features  "stm32f723, rt, usb_fs" --release
//!
//! For HS operation:
//! > cargo run --example usb_serial --features  "stm32f723, rt, usb_hs" --release
//!
//! Note that `usbd-serial` library used in this example doesn't support
//! HighSpeed mode properly at the moment. See
//! https://github.com/mvirkkunen/usbd-serial/pull/14 for a potential workaround.
#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_rt::entry;
#[cfg(feature = "usb_fs")]
use stm32f7xx_hal::otg_fs::{UsbBus, USB};
#[cfg(feature = "usb_hs")]
use stm32f7xx_hal::otg_hs::{UsbBus, USB};
use stm32f7xx_hal::pac;
use stm32f7xx_hal::prelude::*;
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode, PLL48CLK};
use usb_device::prelude::*;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Bypass))
        .use_pll()
        .use_pll48clk(PLL48CLK::Pllq)
        .sysclk(216_000_000.Hz())
        .freeze();

    #[cfg(feature = "usb_fs")]
    let gpioa = dp.GPIOA.split();
    #[cfg(feature = "usb_hs")]
    let gpiob = dp.GPIOB.split();

    #[cfg(feature = "usb_fs")]
    let usb = USB::new(
        dp.OTG_FS_GLOBAL,
        dp.OTG_FS_DEVICE,
        dp.OTG_FS_PWRCLK,
        (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate()),
        clocks,
    );
    #[cfg(all(feature = "usb_hs", not(feature = "usb_hs_phy")))]
    let usb = USB::new(
        dp.OTG_HS_GLOBAL,
        dp.OTG_HS_DEVICE,
        dp.OTG_HS_PWRCLK,
        (gpiob.pb14.into_alternate(), gpiob.pb15.into_alternate()),
        clocks,
    );
    #[cfg(all(feature = "usb_hs", feature = "usb_hs_phy"))]
    let usb = USB::new_with_internal_hs_phy(
        dp.OTG_HS_GLOBAL,
        dp.OTG_HS_DEVICE,
        dp.OTG_HS_PWRCLK,
        dp.USBPHYC,
        (gpiob.pb14.into_alternate(), gpiob.pb15.into_alternate()),
        clocks,
    );

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .max_packet_size_0(64) // Size required for HS, and ok for FS
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 512];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}
