use core::cmp::min;

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
    pub fn freq<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = freq.into().0;

        assert!(4_000_000 <= f && f <= 26_000_000);
        self.freq = f;
        self
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
        let rcc = unsafe { &*RCC::ptr() };

        // If HSE is provided by the user
        let hse_freq: u32 = self.hse.as_ref().map_or(0, |c| c.freq);
        // SYSCLOCK, must be <= 216 Mhz. By default, HSI frequence is chosen
        let mut sysclk = self.sysclk.unwrap_or(HSI);
        let base_clk = match hse_freq {
            0 => HSI,
            _ => hse_freq
        };
        assert!(sysclk <= base_clk);

        if sysclk <= base_clk { // We can use the base clock directly
            match &self.hse {
                // If HSE clock is provided, we use it
                Some(_) => rcc.cfgr.modify(|_, w| w.sw().hse() ),
                // If HSE is not provided, we use HSI
                None => rcc.cfgr.modify(|_, w| w.sw().hsi() )
            };

        } else { // A PLL is needed to multiply the frequence
            // Input divisor from HSI/HSE clock, must result in less than 2MHz, and
            // must be between 2 and 63. In this case, the condition is always
            // respected. We set it at 2MHz as recommanded by the user manual. 
            let pllm = base_clk / 2_000_000;
            let vco_clkin = base_clk / pllm;

            // We can calculate that:
            //  * PLLP = 2 => SYSCLK \in [25*vco_clkin   ; 216*vco_clkin]
            //  * PLLP = 4 => SYSCLK \in [12,5*vco_clkin ; 108*vco_clkin]
            //  * PLLP = 6 => SYSCLK \in [8,33*vco_clkin ; 72*vco_clkin ]
            //  * PLLP = 8 => SYSCLK \in [6,25*vco_clkin ; 54*vco_clkin ] 
            let (plln, pllp) = if sysclk >= 25*vco_clkin {
                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / vco_clkin) * 2;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0b00;

                // Update sysclk with the real value
                sysclk = (vco_clkin * plln) / 2;

                (plln, pllp)
            } else if sysclk >= 13*vco_clkin {
                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / vco_clkin) * 4;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0b01;

                // Update sysclk with the real value
                sysclk = (vco_clkin * plln) / 4;

                (plln, pllp)
            } else if sysclk >= 9*vco_clkin {
                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / vco_clkin) * 6;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0b10;

                // Update sysclk with the real value
                sysclk = (vco_clkin * plln) / 6;

                (plln, pllp)
            } else {
                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / vco_clkin) * 8;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0b11;

                // Update sysclk with the real value
                sysclk = (vco_clkin * plln) / 8;

                (plln, pllp)
            };

            match &self.hse {
                // If HSE is provided
                Some(hse) => {
                    // Configure the HSE mode
                    match hse.mode {
                        HSEClockMode::Bypass => { rcc.cr.modify(
                            |_, w| w.hsebyp().bypassed() 
                        )},
                        HSEClockMode::Oscillator => { rcc.cr.modify(
                            |_, w| w.hsebyp().not_bypassed()
                        )}
                    }
    
                    // Configure PLL from HSI
                    rcc.pllcfgr.write(|w| unsafe {
                        w   
                            .pllsrc().hse()
                            .pllm().bits(pllm as u8)
                            .plln().bits(plln as u16)
                            .pllp().bits(pllp)
                    });
                },
                // If HSE is not provided
                None => {
                    // configure PLL from HSI
                    rcc.pllcfgr.write(|w| unsafe {
                        w   
                            .pllsrc().hsi()
                            .pllm().bits(pllm as u8)
                            .plln().bits(plln as u16)
                            .pllp().bits(pllp)
                    });
                },
            };

            // Enable PLL
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            // Wait for PLL to stabilise
            while rcc.cr.read().pllrdy().is_not_ready() {}

            // Use PLL as SYSCLK
            rcc.cfgr.modify(|_, w| w.sw().pll() );
        }

        // HCLK. By default, SYSCLK frequence is chosen
        let mut hclk: u32 = self.hclk.unwrap_or(sysclk);
        assert!(hclk <= sysclk);  
        // PCLK1 (APB1). Must be <= 54 Mhz. By default, min(hclk, 54Mhz) is
        // chosen
        let mut pclk1: u32 = self.pclk1.unwrap_or(min(54_000_000, hclk));
        // PCLK2 (APB2). Must be <= 108 Mhz. By default, min(hclk, 108Mhz) is
        // chosen
        let mut pclk2: u32 = self.pclk1.unwrap_or(min(108_000_000, hclk));

        // Configure HPRE
        let mut hpre_val: u32 = sysclk / hclk;
        
        // The real value of hpre is computed to be as near as possible to the
        // desired value, this leads to a quantization error
        let hpre = match hpre_val {
            0           => unreachable!(),
            1           => { hpre_val = 1;   0b000},
            2           => { hpre_val = 2;   0b1000},
            3..=5       => { hpre_val = 4;   0b1001},
            6..=11      => { hpre_val = 8;   0b1010},
            12..=39     => { hpre_val = 16;  0b1011},
            40..=95     => { hpre_val = 64;  0b1100},
            96..=191    => { hpre_val = 128; 0b1101},
            192..=383   => { hpre_val = 256; 0b1110},
            _           => { hpre_val = 256; 0b1111},
        };
        // update hclk with the real value
        hclk = sysclk / hpre_val;

        // Configure PPRE1
        let mut ppre1_val: u32 = hclk / pclk1;
        let ppre1: u32 = match ppre1_val {
            0       => unreachable!(),
            1       => { ppre1_val = 1; 0b000},
            2       => { ppre1_val = 2; 0b100},
            3..=6   => { ppre1_val = 4; 0b101},
            7..=12  => { ppre1_val = 8; 0b110},
            _       => { ppre1_val = 16; 0b111},
        };
        // update pclk1 withe the real value
        pclk1 = hclk / ppre1_val;

        // Configure PPRE2
        let mut ppre2_val: u32 = hclk / pclk2;
        let ppre2: u32 = match ppre2_val {
            0       => unreachable!(),
            1       => { ppre2_val = 1; 0b000},
            2       => { ppre2_val = 2; 0b100},
            3..=6   => { ppre2_val = 4; 0b101},
            7..=12  => { ppre2_val = 8; 0b110},
            _       => { ppre2_val = 16; 0b111},
        };
        // update pclk2 withe the real value
        pclk2 = hclk / ppre2_val;
        
        // Configure HCLK, PCLK1, PCLK2
        rcc.cfgr.modify(|_, w| unsafe {
            w
                .ppre1().bits(ppre1 as u8)
                .ppre2().bits(ppre2 as u8)
                .hpre().bits(hpre as u8)
        });

        // Assumes TIMPRE bit of RCC_DCKCFGR1 is reset (0)
        let timclk1 = if ppre1 == 1 {pclk1} else {2 * pclk1};
        let timclk2 = if ppre2 == 1 {pclk2} else {2 * pclk2};

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
