//! USB OTG full-speed peripheral
//!
//! Requires the `usb_fs` feature.
//! Only one of the `usb_fs`/`usb_hs` features can be selected at the same time.

use crate::pac;

use crate::gpio::{Alternate, AF10, gpioa::{PA11, PA12}};
use crate::time::Hertz;

use synopsys_usb_otg::UsbPeripheral;
pub use synopsys_usb_otg::UsbBus;

pub struct USB {
    pub usb_global: pac::OTG_FS_GLOBAL,
    pub usb_device: pac::OTG_FS_DEVICE,
    pub usb_pwrclk: pac::OTG_FS_PWRCLK,
    pub pin_dm: PA11<Alternate<AF10>>,
    pub pin_dp: PA12<Alternate<AF10>>,
    pub hclk: Hertz,
}

unsafe impl Sync for USB {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = pac::OTG_FS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320;
    const ENDPOINT_COUNT: usize = 6;

    fn enable() {
        let rcc = unsafe { &*pac::RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.ahb2enr.modify(|_, w| w.otgfsen().set_bit());

            // Reset USB peripheral
            rcc.ahb2rstr.modify(|_, w| w.otgfsrst().set_bit());
            rcc.ahb2rstr.modify(|_, w| w.otgfsrst().clear_bit());
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.0
    }
}

pub type UsbBusType = UsbBus<USB>;
