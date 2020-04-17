use core::cmp::min;

use micromath::F32Ext;

use crate::device::{rcc, FLASH, RCC};
use crate::time::Hertz;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1(()),
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
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// Advanced High-Performance Bus 1 (AHB1) registers
    pub ahb1: AHB1,

    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    pub cfgr: CFGR,
}

/// Advanced High-Performance Bus 1 (AHB1) registers
pub struct AHB1(());

impl AHB1 {
    pub fn enr(&mut self) -> &rcc::AHB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1enr }
    }

    pub fn rstr(&mut self) -> &rcc::AHB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1rstr }
    }
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


/// HSE Clock modes
///     * `Oscillator`: Use of an external crystal/ceramic resonator
///     * `Bypass`: Use of an external user clock
pub enum HSEClockMode {
    Oscillator,
    Bypass
}

/// HSE Clock
pub struct HSEClock {
    pub freq: u32,
    pub mode: HSEClockMode
}

impl HSEClock {
    /// Provide HSE frequence. Must be between 4 and 26 MHz
    pub fn new<F>(freq: F, mode: HSEClockMode) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;

        assert!(4_000_000 <= f && f <= 26_000_000);
        HSEClock{
            freq: f,
            mode: mode
        }
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
}

impl CFGR {
    /// Declare an HSE clock if available.
    pub fn hse(mut self, hse: HSEClock) -> Self
    {
        self.hse = Some(hse);
        self
    }

    /// Set HCLK Clock (AHB bus, core, memory and DMA.
    /// Specified frequence must be <= 216 MHz
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

    /// Set SYSCLK Clock. It must be between 12,5 Mhz and 216 Mhz.
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

    /// Configure the "mandatory" clocks (`sysclk`, `hclk`, `pclk1` and `pclk2)
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

        // If HSE is provided by the user
        let hse_freq: u32 = self.hse.as_ref().map_or(0, |c| c.freq);
        // SYSCLOCK, must be <= 216 Mhz. By default, HSI frequence is chosen
        let mut sysclk = self.sysclk.unwrap_or(HSI);
        let base_clk = match hse_freq {
            0 => HSI,
            _ => hse_freq
        };

        // Configure HSE if provided
        if self.hse.is_some() {
            // Configure the HSE mode
            match self.hse.as_ref().unwrap().mode {
                HSEClockMode::Bypass => { rcc.cr.modify(
                    |_, w| w.hsebyp().bypassed() 
                )},
                HSEClockMode::Oscillator => { rcc.cr.modify(
                    |_, w| w.hsebyp().not_bypassed()
                )}
            }
            // Start HSE
            rcc.cr.modify(|_, w| w.hseon().on());
            while rcc.cr.read().hserdy().is_not_ready() {}
        }

        let mut use_pll = false;

        if sysclk != base_clk { // A PLL is needed to multiply / divide the frequence
            // Input divisor from HSI/HSE clock, must result in less than 2MHz, and
            // must be between 2 and 63. In this case, the condition is always
            // respected. We set it at 2MHz as recommanded by the user manual. 
            let pllm = ((base_clk as f32) / (2_000_000 as f32)).ceil() as u8;
            let vco_clkin_mhz = (base_clk as f32 / pllm as f32) / 1_000_000.0;
            let mut sysclk_mhz: f32 = sysclk as f32 / 1_000_000.0;

            // PLLN, main scaler, must result in >= 192MHz and <= 432MHz, min
            // 50, max 432, this constraint is allways respected when vco_clkin
            // <= 2 MHz
            let mut plln: f32 = 100.0;
            let allowed_pllp: [u8; 4] = [2, 4, 6, 8];
            let pllp_val = *allowed_pllp.iter().min_by_key(|&pllp| {
                plln = ((sysclk_mhz * (*pllp as f32)) / vco_clkin_mhz).floor();
                let error = sysclk_mhz - ((plln/(*pllp as f32)) * vco_clkin_mhz);

                if error < 0.0
                    || plln * vco_clkin_mhz > 432.0 
                    || plln > 432.0 || plln < 100.0 { core::u32::MAX }
                else { (error*1_000.0) as u32 }
            }).unwrap();

            // PLLN coresponding to the best pllp_val
            plln = ((sysclk_mhz * (pllp_val as f32)) / vco_clkin_mhz).floor();

            // Pllp bits to be written in the register
            let pllp = match pllp_val {
                2 => 0b00,
                4 => 0b01,
                6 => 0b10,
                8 => 0b11,
                _ => unreachable!()
            };

            // Update the real sysclk value
            sysclk_mhz = (vco_clkin_mhz * plln) / (pllp_val as f32);
            sysclk = (sysclk_mhz * 1_000_000.0) as u32;


            // Turn PLL off
            rcc.cr.modify(|_, w| w.pllon().off());            
            // Wait till PLL is disabled 
            while ! rcc.cr.read().pllrdy().is_not_ready() {}

            if self.hse.is_some() {  // If HSE is provided  
                // Configure PLL from HSE
                rcc.pllcfgr.write(|w| unsafe {
                    w   
                        .pllsrc().hse()
                        .pllm().bits(pllm as u8)
                        .plln().bits(plln as u16)
                        .pllp().bits(pllp)
                        .pllq().bits(9)
                });
            } else { // If HSE is not provided
                // configure PLL from HSI
                rcc.pllcfgr.modify(|_, w| unsafe {
                    w   
                        .pllsrc().hsi()
                        .pllm().bits(pllm as u8)
                        .plln().bits(plln as u16)
                        .pllp().bits(pllp)
                });
            }

            // Enable PLL
            rcc.cr.modify(|_, w| w.pllon().on());
            // // Wait for PLL to stabilise
            while rcc.cr.read().pllrdy().is_not_ready() {}

            use_pll = true;
        }

        // HCLK. By default, SYSCLK frequence is chosen. Because of the method
        // of clock multiplication and division, even if `sysclk` is set to be
        // the same as `hclk`, it can be slighly inferior to `sysclk` after
        // pllm, pllp... calculations
        let mut hclk: u32 = sysclk; // min(sysclk, self.hclk.unwrap_or(sysclk));

        // Configure HPRE.
        let hpre_val: f32 = (sysclk as f32/ hclk as f32).ceil();
        
        // The real value of hpre is computed to be as near as possible to the
        // desired value, this leads to a quantization error
        let (hpre_val, hpre): (f32, u8) = match hpre_val as u32 {
            0           => unreachable!(),
            1           => (1.0, 0b000),
            2           => (2.0, 0b1000),
            3..=5       => (4.0, 0b1001),
            6..=11      => (8.0, 0b1010),
            12..=39     => (16.0, 0b1011),
            40..=95     => (64.0, 0b1100),
            96..=191    => (128.0, 0b1101),
            192..=383   => (256.0, 0b1110),
            _           => (512.0, 0b1111)
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
            0       => unreachable!(),
            1       => { ppre1_val = 1; 0b000},
            2       => { ppre1_val = 2; 0b100},
            3..=6   => { ppre1_val = 4; 0b101},
            7..=12  => { ppre1_val = 8; 0b110},
            _       => { ppre1_val = 16; 0b111},
        };
        // update pclk1 with the real value
        pclk1 = hclk / ppre1_val;

        // Configure PPRE2
        let mut ppre2_val: u32 = (hclk as f32 / pclk2 as f32).ceil() as u32;
        let ppre2: u32 = match ppre2_val {
            0       => unreachable!(),
            1       => { ppre2_val = 1; 0b000},
            2       => { ppre2_val = 2; 0b100},
            3..=6   => { ppre2_val = 4; 0b101},
            7..=12  => { ppre2_val = 8; 0b110},
            _       => { ppre2_val = 16; 0b111},
        };
        // update pclk2 with the real value
        pclk2 = hclk / ppre2_val;
        
        // Assumes TIMPRE bit of RCC_DCKCFGR1 is reset (0)
        let timclk1 = if ppre1_val == 1 {pclk1} else {2 * pclk1};
        let timclk2 = if ppre2_val == 1 {pclk2} else {2 * pclk2};

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
            while ! rcc.cfgr.read().sws().is_pll() {}
           
        } else if self.hse.is_some() {
            rcc.cfgr.modify(|_, w| w.sw().hse());
            while ! rcc.cfgr.read().sws().is_hse() {}

        } else {
            rcc.cfgr.modify(|_, w| w.sw().hsi());
            while ! rcc.cfgr.read().sws().is_hsi() {}
        }

        // Configure HCLK, PCLK1, PCLK2
        rcc.cfgr.modify(|_, w| unsafe {
            w
                .ppre1().bits(ppre1 as u8)
                .ppre2().bits(ppre2 as u8)
                .hpre().bits(hpre as u8)
        });

        // As requested by user manual we need to wit 16 ticks before the right
        // predivision is applied
        cortex_m::asm::delay(16);

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            sysclk: Hertz(sysclk),
            timclk1: Hertz(timclk1),
            timclk2: Hertz(timclk2),
        }
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
}
