#![deny(warnings)]
#![no_main]
#![no_std]

// Required
extern crate panic_semihosting;

use cortex_m_rt::entry;
use embedded_graphics::{
    egcircle, egrectangle, egtext,
    fonts::Font6x8,
    pixelcolor::{Rgb565, RgbColor},
    prelude::*,
    primitive_style, text_style,
};

use stm32f7xx_hal::{
    gpio::Speed,
    ltdc::{Layer, PixelFormat},
    pac,
    prelude::*,
    rcc::{HSEClock, HSEClockMode, Rcc},
};

mod screen;

// DIMENSIONS
const WIDTH: u16 = 480;
const HEIGHT: u16 = 272;

// Graphics framebuffer
const FB_GRAPHICS_SIZE: usize = (WIDTH as usize) * (HEIGHT as usize);
static mut FB_LAYER1: [u16; FB_GRAPHICS_SIZE] = [0; FB_GRAPHICS_SIZE];

#[entry]
fn main() -> ! {
    let perif = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let rcc_hal: Rcc = perif.RCC.constrain();

    // Set up pins
    let _gpioa = perif.GPIOA.split();
    let _gpiob = perif.GPIOB.split();
    let gpioe = perif.GPIOE.split();
    let gpiog = perif.GPIOG.split();
    let gpioh = perif.GPIOH.split();
    let gpioi = perif.GPIOI.split();
    let gpioj = perif.GPIOJ.split();
    let gpiok = perif.GPIOK.split();

    gpioe.pe4.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_B0

    gpiog.pg12.into_alternate_af9().set_speed(Speed::VeryHigh); // LTCD_B4

    gpioi.pi9.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_VSYNC
    gpioi.pi10.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_HSYNC
    gpioi.pi13.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioi.pi14.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_CLK
    gpioi.pi15.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R0

    gpioj.pj0.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R1
    gpioj.pj1.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R2
    gpioj.pj2.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R3
    gpioj.pj3.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R4
    gpioj.pj4.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R5
    gpioj.pj5.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R6
    gpioj.pj6.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_R7
    gpioj.pj7.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G0
    gpioj.pj8.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G1
    gpioj.pj9.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G2
    gpioj.pj10.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G3
    gpioj.pj11.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G4
    gpioj.pj13.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_B1
    gpioj.pj14.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_B2
    gpioj.pj15.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_B3

    gpiok.pk0.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G5
    gpiok.pk1.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G6
    gpiok.pk2.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_G7
    gpiok.pk4.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_B5
    gpiok.pk5.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_B6
    gpiok.pk6.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_D7
    gpiok.pk7.into_alternate_af14().set_speed(Speed::VeryHigh); // LTCD_E

    // HSE osc out in High Z
    gpioh.ph1.into_floating_input();
    let _clocks = rcc_hal
        .cfgr
        .hse(HSEClock::new(25.mhz(), HSEClockMode::Bypass))
        .sysclk(216.mhz())
        .hclk(216.mhz())
        .freeze();

    // LCD enable: set it low first to avoid LCD bleed while setting up timings
    let mut disp_on = gpioi.pi12.into_push_pull_output();
    disp_on.set_low().ok();

    // LCD backlight enable
    let mut backlight = gpiok.pk3.into_push_pull_output();
    backlight.set_high().ok();

    let mut display = screen::Stm32F7DiscoDisplay::new(perif.LTDC, perif.DMA2D);
    display
        .controller
        .config_layer(Layer::L1, unsafe { &mut FB_LAYER1 }, PixelFormat::RGB565);

    display.controller.enable_layer(Layer::L1);
    display.controller.reload();

    let display = &mut display;

    // LCD enable: activate LCD !
    disp_on.set_high().ok();

    let r = egrectangle!(
        top_left = (0, 0),
        bottom_right = (479, 271),
        style = primitive_style!(fill_color = Rgb565::new(0, 0b11110, 0b11011))
    );
    r.draw(display).ok();

    let c1 = egcircle!(
        center = (20, 20),
        radius = 8,
        style = primitive_style!(fill_color = Rgb565::new(0, 63, 0))
    );

    let c2 = egcircle!(
        center = (25, 20),
        radius = 8,
        style = primitive_style!(fill_color = Rgb565::new(31, 0, 0))
    );

    let t = egtext!(
        text = "Hello Rust!",
        top_left = (100, 100),
        style = text_style!(font = Font6x8, text_color = RgbColor::WHITE)
    );

    c1.draw(display).ok();
    c2.draw(display).ok();
    t.draw(display).ok();

    for i in 0..300 {
        let c1 = egcircle!(
            center = (20 + i, 20),
            radius = 8,
            style = primitive_style!(fill_color = RgbColor::GREEN)
        );
        c1.draw(display).ok();
    }

    loop {}
}
