//! USB OTG full-speed peripheral
//!
//! Requires the `usb_fs` feature.
//! Only one of the `usb_fs`/`usb_hs` features can be selected at the same time.

use crate::pac;

use crate::gpio::{
    gpioa::{PA11, PA12},
    Alternate,
};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use fugit::HertzU32 as Hertz;

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

pub struct USB {
    pub usb_global: pac::OTG_FS_GLOBAL,
    pub usb_device: pac::OTG_FS_DEVICE,
    pub usb_pwrclk: pac::OTG_FS_PWRCLK,
    pub pin_dm: PA11<Alternate<10>>,
    pub pin_dp: PA12<Alternate<10>>,
    pub hclk: Hertz,
}

impl USB {
    /// Construct a USB peripheral wrapper.
    ///
    /// Call `UsbBus::new` to construct and initialize the USB peripheral driver.
    pub fn new(
        usb_global: pac::OTG_FS_GLOBAL,
        usb_device: pac::OTG_FS_DEVICE,
        usb_pwrclk: pac::OTG_FS_PWRCLK,
        pins: (PA11<Alternate<10>>, PA12<Alternate<10>>),
        clocks: &Clocks,
    ) -> Self {
        Self {
            usb_global,
            usb_device,
            usb_pwrclk,
            pin_dm: pins.0,
            pin_dp: pins.1,
            hclk: pac::OTG_FS_GLOBAL::clock(clocks),
        }
    }
}

unsafe impl Sync for USB {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = pac::OTG_FS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320;
    const ENDPOINT_COUNT: usize = 6;

    fn enable() {
        cortex_m::interrupt::free(|_| unsafe {
            // Enable USB peripheral
            pac::OTG_FS_GLOBAL::enable_unchecked();

            // Reset USB peripheral
            pac::OTG_FS_GLOBAL::reset_unchecked();
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.raw()
    }
}

pub type UsbBusType = UsbBus<USB>;
