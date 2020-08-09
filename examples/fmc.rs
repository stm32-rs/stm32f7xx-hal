//! Initialises FMC controller

#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use core::{mem, slice};
use stm32f7xx_hal as hal;

use crate::hal::gpio::Speed;
use crate::hal::{delay::Delay, pac, prelude::*};
use cortex_m_rt::entry;

use stm32_fmc::devices::is42s32800g_6;

/// Configure pins for the FMC controller
macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .set_speed(Speed::VeryHigh)
                    .into_alternate_af12()
                    .internal_pull_up(true)
            ),*
        )
    };
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // Get the delay provider.
    let mut delay = Delay::new(cp.SYST, clocks);

    // IO
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();
    let gpiof = dp.GPIOF.split();
    let gpiog = dp.GPIOG.split();
    let gpioh = dp.GPIOH.split();
    let gpioi = dp.GPIOI.split();

    // Initialise SDRAM...
    let fmc_io = fmc_pins! {
        // A0-A11
        gpiof.pf0, gpiof.pf1, gpiof.pf2, gpiof.pf3,
        gpiof.pf4, gpiof.pf5, gpiof.pf12, gpiof.pf13,
        gpiof.pf14, gpiof.pf15, gpiog.pg0, gpiog.pg1,
        // BA0-BA1
        gpiog.pg4, gpiog.pg5,
        // D0-D31
        gpiod.pd14, gpiod.pd15, gpiod.pd0, gpiod.pd1,
        gpioe.pe7, gpioe.pe8, gpioe.pe9, gpioe.pe10,
        gpioe.pe11, gpioe.pe12, gpioe.pe13, gpioe.pe14,
        gpioe.pe15, gpiod.pd8, gpiod.pd9, gpiod.pd10,
        gpioh.ph8, gpioh.ph9, gpioh.ph10, gpioh.ph11,
        gpioh.ph12, gpioh.ph13, gpioh.ph14, gpioh.ph15,
        gpioi.pi0, gpioi.pi1, gpioi.pi2, gpioi.pi3,
        gpioi.pi6, gpioi.pi7, gpioi.pi9, gpioi.pi10,
        // NBL0 - NBL3
        gpioe.pe0, gpioe.pe1, gpioi.pi4, gpioi.pi5,
        gpioh.ph7,              // SDCKE1
        gpiog.pg8,              // SDCLK
        gpiog.pg15,             // SDNCAS
        gpioh.ph6,              // SDNE1 (!CS)
        gpiof.pf11,             // SDRAS
        gpioh.ph5               // SDNWE
    };

    // New SDRAM
    let mut sdram = dp.FMC.sdram(fmc_io, is42s32800g_6::Is42s32800g {}, &clocks);

    // Initialise controller and SDRAM
    let ram = unsafe {
        let ram_ptr: *mut u32 = sdram.init(&mut delay);
        let ram_size_bytes = 32 * 1024 * 1024;

        // Configure MPU if required

        slice::from_raw_parts_mut(ram_ptr, ram_size_bytes / mem::size_of::<u32>())
    };

    // ----------------------------------------------------------
    // Main application loop
    let len = 8 * 1024 * 1024;

    for a in 0..len {
        ram[a] = a as u32;
    }

    loop {}
}
