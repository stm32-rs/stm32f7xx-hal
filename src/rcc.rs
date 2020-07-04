use core::cmp::min;

use micromath::F32Ext;

use crate::pac::{rcc, FLASH, RCC};
use crate::time::Hertz;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1 { _0: () },
            ahb2: AHB2 { _0: () },
            ahb3: AHB3 { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                timclk1: None,
                timclk2: None,
                pll48clk: false,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// Advanced High-Performance Bus 1 (AHB1) registers
    pub ahb1: AHB1,
    /// Advanced High-Performance Bus 2 (AHB2) registers
    pub ahb2: AHB2,
    /// Advanced High-Performance Bus 3 (AHB3) registers
    pub ahb3: AHB3,

    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    pub cfgr: CFGR,
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

/// Advanced High-performance Bus 1 (AHB1) registers
pub struct AHB1 {
    _0: (),
}

impl AHB1 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1rstr }
    }
}

/// Advanced High-performance Bus 2 (AHB2) registers
pub struct AHB2 {
    _0: (),
}

#[allow(dead_code)]
impl AHB2 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2rstr }
    }
}

/// Advanced High-performance Bus 3 (AHB3) registers
pub struct AHB3 {
    _0: (),
}

#[allow(dead_code)]
impl AHB3 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB3ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB3RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3rstr }
    }
}

/// HSE Clock modes
///     * `Oscillator`: Use of an external crystal/ceramic resonator
///     * `Bypass`: Use of an external user clock
pub enum HSEClockMode {
    Oscillator,
    Bypass,
}

/// HSE Clock
pub struct HSEClock {
    pub freq: u32,
    pub mode: HSEClockMode,
}

impl HSEClock {
    /// Provide HSE frequency. Must be between 4 and 26 MHz
    pub fn new<F>(freq: F, mode: HSEClockMode) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;

        assert!(4_000_000 <= f && f <= 26_000_000);
        HSEClock { freq: f, mode }
    }
}

const HSI: u32 = 16_000_000; // Hz

pub struct CFGR {
    hse: Option<HSEClock>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    timclk1: Option<u32>,
    timclk2: Option<u32>,
    pll48clk: bool,
}

impl CFGR {
    /// Declare an HSE clock if available.
    pub fn hse(mut self, hse: HSEClock) -> Self {
        self.hse = Some(hse);
        self
    }

    /// Set HCLK Clock (AHB bus, core, memory and DMA.
    /// Specified frequency must be <= 216 MHz
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;
        assert!(f <= 216_000_000);

        self.hclk = Some(f);
        self
    }

    /// Set PCLK1 Clock (APB1 clock). Must be <= 54 Mhz. By default, max
    /// frequency is chosen
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;
        assert!(12_500_000 <= f && f <= 54_000_000);

        self.pclk1 = Some(f);
        self
    }

    /// Set PCLK2 Clock (APB2 clock). Must be <= 108 Mhz. By default, max
    /// frequency is chosen
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;
        assert!(12_500_000 <= f && f <= 108_000_000);

        self.pclk2 = Some(f);
        self
    }

    /// Set SYSCLK Clock. It must be between 12.5 Mhz and 216 Mhz.
    /// If the ethernet peripheral is on, the user should set a
    /// frequency higher than 25 Mhz
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;
        assert!(12_500_000 <= f && f <= 216_000_000);

        self.sysclk = Some(f);
        self
    }

    pub fn timclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.timclk1 = Some(freq.into().0);
        self
    }

    pub fn timclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.timclk2 = Some(freq.into().0);
        self
    }

    pub fn require_pll48clk(mut self) -> Self {
        self.pll48clk = true;
        self
    }

    /// Configure the "mandatory" clocks (`sysclk`, `hclk`, `pclk1` and `pclk2')
    /// and return them via the `Clocks` struct.
    ///
    /// The user shouldn't call freeze more than once as the clocks parameters
    /// cannot be changed after the clocks have started.
    ///
    /// The implementation makes the following choice: HSI is always chosen over
    /// HSE except when HSE is provided. When HSE is provided, HSE is used
    /// wherever it is possible.
    pub fn freeze(self) -> Clocks {
        let flash = unsafe { &(*FLASH::ptr()) };
        let rcc = unsafe { &(*RCC::ptr()) };

        // Switch to fail-safe clock settings.
        // This is useful when booting from a bootloader that alters clock tree configuration.
        // Turn on HSI
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsirdy().bit_is_clear() {}
        // Switch to HSI
        rcc.cfgr.modify(|_, w| w.sw().hsi());

        let base_clk = match self.hse.as_ref() {
            Some(hse) => hse.freq,
            None => HSI,
        };
        // SYSCLK, must be <= 216 Mhz. By default, HSI/HSE frequency is chosen
        let mut sysclk = self.sysclk.unwrap_or(base_clk);

        // Configure HSE if provided
        if self.hse.is_some() {
            // Configure the HSE mode
            match self.hse.as_ref().unwrap().mode {
                HSEClockMode::Bypass => rcc.cr.modify(|_, w| w.hsebyp().bypassed()),
                HSEClockMode::Oscillator => rcc.cr.modify(|_, w| w.hsebyp().not_bypassed()),
            }
            // Start HSE
            rcc.cr.modify(|_, w| w.hseon().on());
            while rcc.cr.read().hserdy().is_not_ready() {}
        }

        let mut use_pll = false;

        let mut pll48clk: Option<Hertz> = None;

        if sysclk != base_clk {
            // Sysclk output divisor must be one of 2, 4, 6 or 8
            let sysclk_div = core::cmp::min(8, (432_000_000 / sysclk) & !1);

            // Input divisor from PLL source clock, must result to frequency in
            // the range from 1 to 2 MHz
            let pllm_min = (sysclk + 1_999_999) / 2_000_000;
            let pllm_max = sysclk / 1_000_000;

            let target_freq = if self.pll48clk {
                // set source clock for 48 MHz to main PLL
                rcc.dckcfgr2.modify(|_, w| w.ck48msel().bit(false));

                48_000_000
            } else {
                sysclk * sysclk_div
            };

            // Find the lowest pllm value that minimize the difference between
            // target frequency and the real vco_out frequency.
            let pllm = (pllm_min..=pllm_max)
                .min_by_key(|pllm| {
                    let vco_in = sysclk / pllm;
                    let plln = target_freq / vco_in;
                    target_freq - vco_in * plln
                })
                .unwrap();

            let vco_in = sysclk / pllm;
            assert!(vco_in >= 1_000_000 && vco_in <= 2_000_000);

            // PLLN, main scaler, must result in >= 100MHz and <= 432MHz, min
            // 50, max 432, this constraint is allways respected when vco_clkin
            // <= 2 MHz
            let plln = if self.pll48clk {
                // try the different valid pllq according to the valid
                // main scaller values, and take the best
                let pllq = (4..=9)
                    .min_by_key(|pllq| {
                        let plln = 48_000_000 * pllq / vco_in;
                        let pll48_diff = 48_000_000 - vco_in * plln / pllq;
                        let sysclk_diff =
                            (sysclk as i32 - (vco_in * plln / sysclk_div) as i32).abs();
                        (pll48_diff, sysclk_diff)
                    })
                    .unwrap();
                48_000_000 * pllq / vco_in
            } else {
                sysclk * sysclk_div / vco_in
            };
            let pllp = (sysclk_div / 2) - 1;

            let pllq = (vco_in * plln + 47_999_999) / 48_000_000;
            pll48clk = Some(Hertz(vco_in * plln / pllq));

            rcc.pllcfgr.write(|w| unsafe {
                w.pllm().bits(pllm as u8);
                w.plln().bits(plln as u16);
                w.pllp().bits(pllp as u8);
                w.pllq().bits(pllq as u8);
                w.pllsrc().bit(self.hse.is_some())
            });

            // Enable PLL
            rcc.cr.modify(|_, w| w.pllon().on());
            // // Wait for PLL to stabilise
            while rcc.cr.read().pllrdy().is_not_ready() {}

            use_pll = true;

            sysclk = vco_in * plln / sysclk_div;
        }

        // HCLK. By default, SYSCLK frequency is chosen. Because of the method
        // of clock multiplication and division, even if `sysclk` is set to be
        // the same as `hclk`, it can be slightly inferior to `sysclk` after
        // pllm, pllp... calculations
        let mut hclk: u32 = sysclk; // min(sysclk, self.hclk.unwrap_or(sysclk));

        // Configure HPRE.
        let hpre_val: f32 = (sysclk as f32 / hclk as f32).ceil();

        // The real value of hpre is computed to be as near as possible to the
        // desired value, this leads to a quantization error
        let (hpre_val, hpre): (f32, u8) = match hpre_val as u32 {
            0 => unreachable!(),
            1 => (1.0, 0b000),
            2 => (2.0, 0b1000),
            3..=5 => (4.0, 0b1001),
            6..=11 => (8.0, 0b1010),
            12..=39 => (16.0, 0b1011),
            40..=95 => (64.0, 0b1100),
            96..=191 => (128.0, 0b1101),
            192..=383 => (256.0, 0b1110),
            _ => (512.0, 0b1111),
        };
        // update hclk with the real value
        hclk = (sysclk as f32 / hpre_val).floor() as u32;

        // PCLK1 (APB1). Must be <= 54 Mhz. By default, min(hclk, 54Mhz) is
        // chosen
        let mut pclk1: u32 = min(54_000_000, self.pclk1.unwrap_or(hclk));
        // PCLK2 (APB2). Must be <= 108 Mhz. By default, min(hclk, 108Mhz) is
        // chosen
        let mut pclk2: u32 = min(108_000_000, self.pclk2.unwrap_or(hclk));

        // Configure PPRE1
        let mut ppre1_val: u32 = (hclk as f32 / pclk1 as f32).ceil() as u32;
        let ppre1: u32 = match ppre1_val {
            0 => unreachable!(),
            1 => {
                ppre1_val = 1;
                0b000
            }
            2 => {
                ppre1_val = 2;
                0b100
            }
            3..=6 => {
                ppre1_val = 4;
                0b101
            }
            7..=12 => {
                ppre1_val = 8;
                0b110
            }
            _ => {
                ppre1_val = 16;
                0b111
            }
        };
        // update pclk1 with the real value
        pclk1 = hclk / ppre1_val;

        // Configure PPRE2
        let mut ppre2_val: u32 = (hclk as f32 / pclk2 as f32).ceil() as u32;
        let ppre2: u32 = match ppre2_val {
            0 => unreachable!(),
            1 => {
                ppre2_val = 1;
                0b000
            }
            2 => {
                ppre2_val = 2;
                0b100
            }
            3..=6 => {
                ppre2_val = 4;
                0b101
            }
            7..=12 => {
                ppre2_val = 8;
                0b110
            }
            _ => {
                ppre2_val = 16;
                0b111
            }
        };
        // update pclk2 with the real value
        pclk2 = hclk / ppre2_val;

        // Assumes TIMPRE bit of RCC_DCKCFGR1 is reset (0)
        let timclk1 = if ppre1_val == 1 { pclk1 } else { 2 * pclk1 };
        let timclk2 = if ppre2_val == 1 { pclk2 } else { 2 * pclk2 };

        // Adjust flash wait states
        flash.acr.write(|w| {
            w.latency().bits(if sysclk <= 30_000_000 {
                0b0000
            } else if sysclk <= 60_000_000 {
                0b0001
            } else if sysclk <= 90_000_000 {
                0b0010
            } else if sysclk <= 120_000_000 {
                0b0011
            } else if sysclk <= 150_000_000 {
                0b0100
            } else if sysclk <= 180_000_000 {
                0b0101
            } else if sysclk <= 210_000_000 {
                0b0110
            } else {
                0b0111
            })
        });

        // Select SYSCLK source
        if use_pll {
            rcc.cfgr.modify(|_, w| w.sw().pll());
            while !rcc.cfgr.read().sws().is_pll() {}
        } else if self.hse.is_some() {
            rcc.cfgr.modify(|_, w| w.sw().hse());
            while !rcc.cfgr.read().sws().is_hse() {}
        } else {
            rcc.cfgr.modify(|_, w| w.sw().hsi());
            while !rcc.cfgr.read().sws().is_hsi() {}
        }

        // Configure HCLK, PCLK1, PCLK2
        rcc.cfgr.modify(|_, w| unsafe {
            w.ppre1()
                .bits(ppre1 as u8)
                .ppre2()
                .bits(ppre2 as u8)
                .hpre()
                .bits(hpre as u8)
        });

        // As requested by user manual we need to wait 16 ticks before the right
        // predivision is applied
        cortex_m::asm::delay(16);

        let clocks = Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            sysclk: Hertz(sysclk),
            timclk1: Hertz(timclk1),
            timclk2: Hertz(timclk2),
            pll48clk,
        };

        if self.pll48clk {
            assert!(clocks.is_pll48clk_valid());
        }

        clocks
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    sysclk: Hertz,
    timclk1: Hertz,
    timclk2: Hertz,
    pll48clk: Option<Hertz>,
}

impl Clocks {
    /// Returns the frequency of the AHB1
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the frequency for timers on APB1
    pub fn timclk1(&self) -> Hertz {
        self.timclk1
    }

    /// Returns the frequency for timers on APB1
    pub fn timclk2(&self) -> Hertz {
        self.timclk2
    }

    /// Returns the frequency of the PLL48 clock line
    pub fn pll48clk(&self) -> Option<Hertz> {
        self.pll48clk
    }

    /// Returns true if the PLL48 clock is within USB
    /// specifications. It is required to use the USB functionality.
    pub fn is_pll48clk_valid(&self) -> bool {
        // USB specification allow +-0.25%
        self.pll48clk
            .map(|freq| (48_000_000 - freq.0 as i32).abs() <= 48_000_000 * 25 / 10000)
            .unwrap_or(false)
    }
}

pub trait GetBusFreq {
    fn get_frequency(clocks: &Clocks) -> Hertz;
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        Self::get_frequency(clocks)
    }
}

impl GetBusFreq for AHB1 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl GetBusFreq for AHB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl GetBusFreq for AHB3 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl GetBusFreq for APB1 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk1
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.timclk1()
    }
}

impl GetBusFreq for APB2 {
    fn get_frequency(clocks: &Clocks) -> Hertz {
        clocks.pclk2
    }
    fn get_timer_frequency(clocks: &Clocks) -> Hertz {
        clocks.timclk2()
    }
}

pub(crate) mod sealed {
    /// Bus associated to peripheral
    pub trait RccBus {
        /// Bus type;
        type Bus;
    }
}
use sealed::RccBus;

/// Enable/disable peripheral
pub trait Enable: RccBus {
    fn enable(apb: &mut Self::Bus);
    fn disable(apb: &mut Self::Bus);
}

/// Reset peripheral
pub trait Reset: RccBus {
    fn reset(apb: &mut Self::Bus);
}

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $peren:ident, $perrst:ident),)+) => {
        $(
            impl RccBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().set_bit());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.enr().modify(|_, w| w.$peren().clear_bit());
                }
            }
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset(apb: &mut Self::Bus) {
                    apb.rstr().modify(|_, w| w.$perrst().set_bit());
                    apb.rstr().modify(|_, w| w.$perrst().clear_bit());
                }
            }
        )+
    }
}

// Peripherals respective buses
// TODO: check which processor has which peripheral and add them
bus! {
    I2C1 => (APB1, i2c1en, i2c1rst),
    I2C2 => (APB1, i2c2en, i2c2rst),
    I2C3 => (APB1, i2c3en, i2c3rst),

    SPI1 => (APB2, spi1en, spi1rst),
    SPI2 => (APB1, spi2en, spi2rst),
    SPI3 => (APB1, spi3en, spi3rst),

    USART1 => (APB2, usart1en, usart1rst),
    USART2 => (APB1, usart2en, uart2rst),
    USART3 => (APB1, usart3en, uart3rst),
    UART4 => (APB1, uart4en, uart4rst),
    UART5 => (APB1, uart5en, uart5rst),
    USART6 => (APB2, usart6en, usart6rst),
    UART7 => (APB1, uart7en, uart7rst),
    UART8 => (APB1, uart8en, uart8rst),

    WWDG => (APB1, wwdgen, wwdgrst),

    DMA1 => (AHB1, dma1en, dma1rst),
    DMA2 => (AHB1, dma2en, dma2rst),

    GPIOA => (AHB1, gpioaen, gpioarst),
    GPIOB => (AHB1, gpioben, gpiobrst),
    GPIOC => (AHB1, gpiocen, gpiocrst),
    GPIOD => (AHB1, gpioden, gpiodrst),
    GPIOE => (AHB1, gpioeen, gpioerst),
    GPIOF => (AHB1, gpiofen, gpiofrst),
    GPIOG => (AHB1, gpiogen, gpiogrst),
    GPIOH => (AHB1, gpiohen, gpiohrst),
    GPIOI => (AHB1, gpioien, gpioirst),

    TIM1 => (APB2, tim1en, tim1rst),
    TIM2 => (APB1, tim2en, tim2rst),
    TIM3 => (APB1, tim3en, tim3rst),
    TIM4 => (APB1, tim4en, tim4rst),
    TIM5 => (APB1, tim5en, tim5rst),
    TIM6 => (APB1, tim6en, tim6rst),
    TIM7 => (APB1, tim7en, tim7rst),
    TIM8 => (APB2, tim8en, tim8rst),
    TIM9 => (APB2, tim9en, tim9rst),
    TIM10 => (APB2, tim10en, tim10rst),
    TIM11 => (APB2, tim11en, tim11rst),
    TIM12 => (APB1, tim12en, tim12rst),
    TIM13 => (APB1, tim13en, tim13rst),
    TIM14 => (APB1, tim14en, tim14rst),
}

#[cfg(not(any(
    feature = "stm32f722",
    feature = "stm32f723",
    feature = "stm32f730",
    feature = "stm32f732",
    feature = "stm32f733"
)))]
bus! {
    I2C4 => (APB1, i2c4en, i2c4rst),

    GPIOJ => (AHB1, gpiojen, gpiojrst),
    GPIOK => (AHB1, gpioken, gpiokrst),

    DMA2D => (AHB1, dma2den, dma2drst),
}
