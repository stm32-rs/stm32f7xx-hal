//! This example demonstrates how to use the RTC.
//! Note that the LSI can be quite inaccurate.
//! The tolerance is up to ±47% (Min 17 kHz, Typ 32 kHz, Max 47 kHz).

#![no_main]
#![no_std]

extern crate panic_halt as _;
use cortex_m_rt::entry;

use cortex_m_semihosting::hprintln;
use rtcc::{Hours, NaiveDate, NaiveTime, Rtcc};
use stm32f7xx_hal::{
    pac,
    prelude::*,
    rtc::{Rtc, RtcClock},
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

    rtc.set_24h_fmt();
    rtc.set_time(&NaiveTime::from_hms(12, 30, 00)).unwrap();
    rtc.set_date(&NaiveDate::from_ymd(2021, 11, 25)).unwrap();
    loop {
        if let Hours::H24(h) = rtc.get_hours().unwrap() {
            hprintln!(
                "{}-{}-{} {}:{}:{}",
                rtc.get_year().unwrap(),
                rtc.get_month().unwrap(),
                rtc.get_day().unwrap(),
                h,
                rtc.get_minutes().unwrap(),
                rtc.get_seconds().unwrap()
            )
            .unwrap();
        }
    }
}
