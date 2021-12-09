//! This example demonstrates how to use the RTC.
//! Note that the LSI can be quite inaccurate.
//! The tolerance is up to ±47% (Min 17 kHz, Typ 32 kHz, Max 47 kHz).

#![no_main]
#![no_std]

extern crate panic_halt as _;
use cortex_m_rt::entry;

use cortex_m_semihosting::hprintln;
use stm32f7xx_hal::{
    pac,
    prelude::*,
    rtc::{Rtc, RtcClock},
};
use time::{
    macros::{date, time},
    PrimitiveDateTime,
};

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.lsi().freeze();

    let mut rtc = Rtc::new(
        p.RTC,
        249,
        127,
        RtcClock::Lsi,
        clocks,
        &mut rcc.apb1,
        &mut p.PWR,
    )
    .unwrap();

    rtc.set_datetime(&PrimitiveDateTime::new(date!(2019 - 01 - 01), time!(23:59)))
        .unwrap();
    // Alternatively:
    // rtc.set_date(&date!(2019 - 01 - 01)).unwrap();
    // rtc.set_time(&time!(23:59)).unwrap();
    // Or:
    // rtc.set_year(2019).unwrap();
    // rtc.set_month(12).unwrap();
    // rtc.set_day(31).unwrap();
    // rtc.set_hours(23).unwrap();
    // rtc.set_minutes(59).unwrap();
    // rtc.set_seconds(59).unwrap();
    loop {
        hprintln!("{}", rtc.get_datetime()).unwrap();
    }
}
