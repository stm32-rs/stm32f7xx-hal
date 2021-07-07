use core::cmp::min;

#[cfg_attr(test, allow(unused_imports))]
use micromath::F32Ext;

use crate::pac::{rcc, FLASH, PWR, RCC};
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
            bdcr: BDCR { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                sysclk: None,
                pclk1: None,
                pclk2: None,
                use_pll: false,
                use_pll48clk: false,
                pllm: 2,
                plln: 50,
                pllp: PLLP::Div2,
                pllq: 2,
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
    /// RCC Backup Domain
    pub bdcr: BDCR,
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

/// Backup Domain Control register (RCC_BDCR)
pub struct BDCR {
    _0: (),
}

impl BDCR {
    pub(crate) fn bdcr(&mut self) -> &rcc::BDCR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).bdcr }
    }
}

/// HSE Clock modes
///     * `Oscillator`: Use of an external crystal/ceramic resonator
///     * `Bypass`: Use of an external user clock
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HSEClockMode {
    Oscillator,
    Bypass,
}

/// HSE Clock
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PLLP {
    Div2 = 0b00,
    Div4 = 0b01,
    Div6 = 0b10,
    Div8 = 0b11,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum VOSscale {
    PwrScale1,
    PwrScale2,
    PwrScale3,
}

impl Default for VOSscale {
    fn default() -> Self {
        VOSscale::PwrScale3
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct CFGR {
    hse: Option<HSEClock>,
    hclk: Option<u32>,
    sysclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    use_pll: bool,
    use_pll48clk: bool,
    pllm: u8,
    plln: u16,
    pllp: PLLP,
    pllq: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
struct InternalRCCConfig {
    hpre: u8,
    ppre1: u8,
    ppre2: u8,
    flash_waitstates: u8,
    overdrive: bool,
    vos_scale: VOSscale,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
struct FreqRequest {
    p: Option<(u32, u32)>,
    q: Option<(u32, u32)>,
}

impl CFGR {
    /// Declare an HSE clock if available.
    pub fn hse(mut self, hse: HSEClock) -> Self {
        assert!(
            hse.mode != HSEClockMode::Bypass || (hse.freq >= 4_000_000 && hse.freq <= 26_000_000)
        );
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

    /// Configure the system clock settings.
    ///
    /// This sets the sysclk frequency and sets up the USB clock if defined.
    /// The provided frequency must be between 12.5 Mhz and 216 Mhz.
    /// 12.5 Mhz is the VCO minimum frequency and sysclk PLLP divider limitation.
    /// If the ethernet peripheral is on, the user should set a frequency higher than 25 Mhz.
    pub fn sysclk<F>(mut self, sysclk: F) -> Self
    where
        F: Into<Hertz>,
    {
        let f: u32 = sysclk.into().0;

        assert!(12_500_000 <= f && f <= 216_000_000);

        self.sysclk = Some(f);
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

    /// Use PLL as clock source
    pub fn use_pll(mut self) -> Self {
        self.use_pll = true;
        self
    }

    /// Use PLL48 as clock source for USB clock
    pub fn use_pll48clk(mut self) -> Self {
        self.use_pll48clk = true;
        self
    }

    /// Set common PLL divider
    pub fn pllm(mut self, pllm: u8) -> Self {
        assert!(pllm >= 2 && pllm <= 63);
        self.pllm = pllm;
        self
    }

    /// Set PLL multiplication
    pub fn plln(mut self, plln: u16) -> Self {
        assert!(plln >= 50 && plln <= 432);
        self.plln = plln;
        self
    }

    /// Set PLL divider
    pub fn pllp(mut self, pllp: PLLP) -> Self {
        self.pllp = pllp;
        self
    }

    /// Set PLL divider for 48 MHz clock
    pub fn pllq(mut self, pllq: u8) -> Self {
        assert!(pllq >= 2 && pllq <= 15);
        self.pllq = pllq;
        self
    }

    /// Output clock calculation
    fn calculate_clocks(&self) -> (Clocks, InternalRCCConfig) {
        let mut config = InternalRCCConfig::default();

        let base_clk = match self.hse.as_ref() {
            Some(hse) => hse.freq,
            None => HSI,
        } as u64;

        let mut sysclk = base_clk;

        let mut pll48clk_valid = false;

        if self.use_pll {
            sysclk = base_clk as u64 * self.plln as u64
                / self.pllm as u64
                / match self.pllp {
                    PLLP::Div2 => 2,
                    PLLP::Div4 => 4,
                    PLLP::Div6 => 6,
                    PLLP::Div8 => 8,
                };
        }

        if self.use_pll48clk {
            pll48clk_valid = {
                let pll48clk =
                    base_clk as u64 * self.plln as u64 / self.pllm as u64 / self.pllq as u64;
                pll48clk >= 48_000_000 - 120_000 && pll48clk <= 48_000_000 + 120_000
            };
        }
        // SYSCLK, must be <= 216 Mhz. By default, HSI/HSE frequency is chosen
        assert!(sysclk <= 216_000_000);
        let sysclk = sysclk as u32;

        // HCLK. By default, SYSCLK frequency is chosen. Because of the method
        // of clock multiplication and division, even if `sysclk` is set to be
        // the same as `hclk`, it can be slightly inferior to `sysclk` after
        // pllm, pllp... calculations
        let mut hclk: u32 = min(sysclk, self.hclk.unwrap_or(sysclk));

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
        config.hpre = hpre;
        // update hclk with the real value
        hclk = (sysclk as f32 / hpre_val).floor() as u32;

        // PCLK1 (APB1). Must be <= 54 Mhz. By default, min(hclk, 54Mhz) is
        // chosen
        // Add limits dependens on OD follows by DS Table 16.
        let max_pclk1 = if sysclk <= 180_000_000 {
            45_000_000
        } else {
            54_000_000
        };
        let mut pclk1: u32 = min(max_pclk1, self.pclk1.unwrap_or(hclk));
        // PCLK2 (APB2). Must be <= 108 Mhz. By default, min(hclk, 108Mhz) is
        // chosen
        // Add limits dependens on OD follows by DS Table 16.
        let max_pclk2 = if sysclk <= 180_000_000 {
            90_000_000
        } else {
            108_000_000
        };
        let mut pclk2: u32 = min(max_pclk2, self.pclk2.unwrap_or(hclk));

        // Configure PPRE1
        let mut ppre1_val: u32 = (hclk as f32 / pclk1 as f32).ceil() as u32;
        config.ppre1 = match ppre1_val {
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
        config.ppre2 = match ppre2_val {
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
        config.flash_waitstates = if sysclk <= 30_000_000 {
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
        };
        // Adjust power state and overdrive mode
        // Configure follows by RM 4.1.4
        // Values getted from DS Table 16. General operating conditions
        config.vos_scale = if sysclk <= 144_000_000 {
            VOSscale::PwrScale3
        } else if sysclk <= 168_000_000 {
            VOSscale::PwrScale2
        } else {
            VOSscale::PwrScale1
        };
        // For every frequency higher than 180 need to enable overdrive
        // Follows by DS Table 16.
        config.overdrive = if sysclk <= 180_000_000 { false } else { true };

        let clocks = Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            sysclk: Hertz(sysclk),
            timclk1: Hertz(timclk1),
            timclk2: Hertz(timclk2),
            pll48clk_valid,
            hse: self.hse.map(|hse| Hertz(hse.freq)),
        };

        (clocks, config)
    }

    /// Calculate the PLL M, N, P and Q values from the provided clock and requested options.
    fn calculate_mnpq(
        f_pll_clock_input: u32,
        freq_req: FreqRequest,
    ) -> Option<(u32, u32, Option<u32>, Option<u32>)> {
        let mut m = 2;
        let mut n = 432;
        let mut p = None;
        let mut q = None;

        if freq_req.p.is_none() && freq_req.q.is_none() {
            return None;
        }

        loop {
            if m > 63 {
                return None;
            }
            let f_vco_input = f_pll_clock_input / m;
            if f_vco_input < 1_000_000 {
                return None;
            }
            if f_vco_input > 2_000_000 || n < 50 {
                m += 1;
                n = 432;
                continue;
            }
            let f_vco_clock = (f_pll_clock_input as u64 * n as u64 / m as u64) as u32;
            if f_vco_clock < 50_000_000 {
                m += 1;
                n = 432;
                continue;
            }
            if f_vco_clock > 432_000_000 {
                n -= 1;
                continue;
            }

            if let Some((p_freq_min, p_freq_max)) = freq_req.p {
                let mut div = None;
                for div_p in &[2, 4, 6, 8] {
                    let f_pll_clock_output = f_vco_clock / div_p;
                    if f_pll_clock_output >= p_freq_min && f_pll_clock_output <= p_freq_max {
                        div = Some(*div_p)
                    }
                }
                if div.is_some() {
                    p = div;
                    if let None = freq_req.q {
                        break;
                    }
                } else {
                    n -= 1;
                    continue;
                }
            }

            if let Some((q_freq_min, q_freq_max)) = freq_req.q {
                let mut div = None;
                for div_q in 2..=15 {
                    let f_usb_clock_output = f_vco_clock / div_q;
                    if f_usb_clock_output >= q_freq_min && f_usb_clock_output <= q_freq_max {
                        div = Some(div_q)
                    }
                }
                if div.is_some() {
                    q = div;
                    break;
                } else {
                    n -= 1;
                    continue;
                }
            }
        }

        Some((m, n, p, q))
    }

    fn pll_configure(&mut self) {
        let base_clk = match self.hse.as_ref() {
            Some(hse) => hse.freq,
            None => HSI,
        };

        let sysclk = if let Some(clk) = self.sysclk {
            clk
        } else {
            base_clk
        };

        let p = if base_clk == sysclk {
            None
        } else {
            Some((sysclk - 1, sysclk + 1))
        };

        let q = if self.use_pll48clk {
            Some((48_000_000 - 120_000, 48_000_000 + 120_000))
        } else {
            None
        };

        if p.is_none() && q.is_none() {
            // We don't need PLL
            self.use_pll = false;
            return;
        }

        // We check if (pllm, plln, pllp) allow to obtain the requested Sysclk,
        // so that we don't have to calculate them
        let p_ok = (sysclk as u64)
            == (base_clk as u64 * self.plln as u64
                / self.pllm as u64
                / match self.pllp {
                    PLLP::Div2 => 2,
                    PLLP::Div4 => 4,
                    PLLP::Div6 => 6,
                    PLLP::Div8 => 8,
                });
        if p_ok && q.is_none() {
            return;
        }

        if let Some((m, n, p, q)) = CFGR::calculate_mnpq(base_clk, FreqRequest { p, q }) {
            self.pllm = m as u8;
            self.plln = n as u16;
            if let Some(p) = p {
                self.use_pll = true;
                self.pllp = match p {
                    2 => PLLP::Div2,
                    4 => PLLP::Div4,
                    6 => PLLP::Div6,
                    8 => PLLP::Div8,
                    _ => unreachable!(),
                };
            }
            if let Some(q) = q {
                self.pllq = q as u8;
            }
        } else {
            panic!("couldn't calculate {} from {}", sysclk, base_clk);
        }
    }

    /// Configure the default clock settings.
    /// Set sysclk as 216 Mhz and setup USB clock if defined.
    pub fn set_defaults(self) -> Self {
        self.sysclk(Hertz(216_000_000))
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
    pub fn freeze(mut self) -> Clocks {
        let flash = unsafe { &(*FLASH::ptr()) };
        let rcc = unsafe { &(*RCC::ptr()) };
        let pwr = unsafe { &(*PWR::ptr()) };

        self.pll_configure();

        let (clocks, config) = self.calculate_clocks();

        // Switch to fail-safe clock settings.
        // This is useful when booting from a bootloader that alters clock tree configuration.
        // Turn on HSI
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsirdy().bit_is_clear() {}
        // Switch to HSI
        rcc.cfgr.modify(|_, w| w.sw().hsi());

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

        // Enable sequence follows by RM 4.1.4 Entering Overdrive mode.
        if self.use_pll || self.use_pll48clk {
            // Disable PLL
            // Since the main-PLL configuration parameters cannot be changed once PLL is enabled, it is
            // recommended to configure PLL before enabling it (selection of the HSI or HSE oscillator as
            // PLL clock source, and configuration of division factors M, N, P, and Q).
            rcc.cr.modify(|_, w| w.pllon().off());

            rcc.pllcfgr.modify(|_, w| unsafe {
                w.pllm().bits(self.pllm);
                w.plln().bits(self.plln);
                w.pllp().bits(self.pllp as u8);
                w.pllq().bits(self.pllq);
                w.pllsrc().bit(self.hse.is_some())
            });

            // Enable PWR domain and setup VOSscale and Overdrive options
            rcc.apb1enr.modify(|_, w| w.pwren().set_bit());

            pwr.cr1.modify(|_, w| match config.vos_scale {
                VOSscale::PwrScale3 => w.vos().scale3(),
                VOSscale::PwrScale2 => w.vos().scale2(),
                VOSscale::PwrScale1 => w.vos().scale1(),
            });

            // Enable PLL
            rcc.cr.modify(|_, w| w.pllon().on());

            // Wait for PLL to stabilise
            while rcc.cr.read().pllrdy().is_not_ready() {}

            //Over-drive
            if config.overdrive {
                // Entering Over-drive mode
                //enable the Over-drive mode
                pwr.cr1.modify(|_, w| w.oden().set_bit());

                //wait for the ODRDY flag to be set
                while !pwr.csr1.read().odrdy().bit_is_set() {}

                //switch the voltage regulator from Normal mode to Over-drive mode
                pwr.cr1.modify(|_, w| w.odswen().set_bit());

                //Wait for the ODSWRDY flag in the PWR_CSR1 to be set.
                while !pwr.csr1.read().odswrdy().bit_is_set() {}
            }
        }

        if self.use_pll48clk {
            // set source clock for 48 MHz to main PLL
            rcc.dckcfgr2.modify(|_, w| w.ck48msel().bit(false));
        }

        flash
            .acr
            .write(|w| w.latency().bits(config.flash_waitstates));

        // Configure HCLK, PCLK1, PCLK2
        rcc.cfgr.modify(|_, w| unsafe {
            w.ppre1()
                .bits(config.ppre1)
                .ppre2()
                .bits(config.ppre2)
                .hpre()
                .bits(config.hpre)
        });

        // Select SYSCLK source
        if self.use_pll {
            rcc.cfgr.modify(|_, w| w.sw().pll());
            while !rcc.cfgr.read().sws().is_pll() {}
        } else if self.hse.is_some() {
            rcc.cfgr.modify(|_, w| w.sw().hse());
            while !rcc.cfgr.read().sws().is_hse() {}
        } else {
            rcc.cfgr.modify(|_, w| w.sw().hsi());
            while !rcc.cfgr.read().sws().is_hsi() {}
        }

        // As requested by user manual we need to wait 16 ticks before the right
        // predivision is applied
        cortex_m::asm::delay(16);

        clocks
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    sysclk: Hertz,
    timclk1: Hertz,
    timclk2: Hertz,
    pll48clk_valid: bool,
    hse: Option<Hertz>,
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

    /// Returns true if the PLL48 clock is within USB
    /// specifications. It is required to use the USB functionality.
    pub fn is_pll48clk_valid(&self) -> bool {
        // USB specification allow +-0.25%
        self.pll48clk_valid
    }

    pub fn hse(&self) -> Option<Hertz> {
        self.hse
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
    SPI4 => (APB2, spi4en, spi4rst),
    SPI5 => (APB2, spi5en, spi5rst),

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

    SYSCFG => (APB2, syscfgen, syscfgrst),
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

#[cfg(feature = "has-can")]
bus! {
    CAN1 => (APB1, can1en, can1rst),
    CAN2 => (APB1, can2en, can2rst),
}

#[cfg(test)]
mod tests {
    use super::{FreqRequest, CFGR};

    fn build_request(sysclk: u32, use_pll48clk: bool) -> FreqRequest {
        let p = Some((sysclk - 1, sysclk + 1));
        let q = if use_pll48clk {
            Some((48_000_000 - 120_000, 48_000_000 + 120_000))
        } else {
            None
        };
        FreqRequest { p, q }
    }

    fn check(hse: u32, sysclk: u32, use_pll48clk: bool) {
        let request = build_request(sysclk, use_pll48clk);
        let (m, n, p, q) =
            CFGR::calculate_mnpq(hse, request).expect("Can't calculate PLL parameters");

        let pll_in = hse;

        if m < 2 || m > 63 {
            panic!("Invalid PLL M value: {}", m);
        }

        let vco_in = pll_in / m;
        if vco_in < 1_000_000 || vco_in > 2_000_000 {
            panic!("Invalid VCO input frequency: {}", vco_in);
        }

        if n < 50 || n > 432 {
            panic!("Invalid PLL N value: {}", n);
        }

        let vco = ((pll_in as u64) * (n as u64) / (m as u64)) as u32;
        if vco < 100_000_000 || vco > 432_000_000 {
            panic!("Invalid VCO frequency: {}", vco);
        }

        let p = p.expect("PLL P value should be defined!");
        if [2, 4, 6, 8].iter().find(|v| **v == p).is_none() {
            panic!("Invalid PLL P value: {}", p);
        }

        let p_freq = vco / p;
        if p_freq > 216_000_000 {
            panic!("Invalid PLL P frequency: {}", p_freq);
        }
        if p_freq < (sysclk - 1) || p_freq > (sysclk + 1) {
            panic!(
                "Invalid PLL P frequency: {} (requested sysclk {})",
                p_freq, sysclk
            );
        }

        if use_pll48clk && q.is_none() {
            panic!("PLL Q value should be defined!");
        }
        if let Some(q) = q {
            if q < 2 || q > 15 {
                panic!("Invalid PLL Q value: {}", q);
            }
            if use_pll48clk {
                let q_freq = vco / q;
                if q_freq < (48_000_000 - 120_000) || q_freq > (48_000_000 + 120_000) {
                    panic!("Invalid PLL Q frequency: {}", q_freq);
                }
            }
        }
    }

    #[test]
    fn test_pll_calc1() {
        check(25_000_000, 48_000_000, false);
    }

    #[test]
    fn test_pll_calc1_usb() {
        check(25_000_000, 48_000_000, true);
    }

    #[test]
    fn test_pll_calc2() {
        check(12_000_000, 48_000_000, false);
    }

    #[test]
    fn test_pll_calc2_usb() {
        check(12_000_000, 48_000_000, true);
    }

    #[test]
    fn test_pll_calc3() {
        check(12_000_000, 216_000_000, false);
    }

    #[test]
    fn test_pll_calc3_usb() {
        check(12_000_000, 216_000_000, true);
    }

    #[test]
    fn test_rcc_calc1() {
        use super::{HSEClock, HSEClockMode, PLLP};
        use crate::time::U32Ext;

        let cfgr = CFGR {
            hse: None,
            hclk: None,
            sysclk: None,
            pclk1: None,
            pclk2: None,
            use_pll: false,
            use_pll48clk: false,
            pllm: 2,
            plln: 50,
            pllp: PLLP::Div2,
            pllq: 2,
        };

        let mut cfgr = cfgr
            .hse(HSEClock::new(25.mhz(), HSEClockMode::Bypass))
            .use_pll()
            .use_pll48clk()
            .sysclk(216.mhz());
        cfgr.pll_configure();

        assert_eq!(cfgr.hse.unwrap().freq, 25_000_000);

        let (clocks, _config) = cfgr.calculate_clocks();
        assert_eq!(clocks.sysclk().0, 216_000_000);
        assert!(clocks.is_pll48clk_valid());
    }

    #[test]
    fn test_rcc_calc2() {
        use super::{HSEClock, HSEClockMode, PLLP};
        use crate::time::U32Ext;

        let cfgr = CFGR {
            hse: None,
            hclk: None,
            sysclk: None,
            pclk1: None,
            pclk2: None,
            use_pll: false,
            use_pll48clk: false,
            pllm: 2,
            plln: 50,
            pllp: PLLP::Div2,
            pllq: 2,
        };

        let mut cfgr = cfgr
            .hse(HSEClock::new(25.mhz(), HSEClockMode::Bypass))
            .use_pll48clk()
            .sysclk(216.mhz());
        cfgr.pll_configure();

        assert_eq!(cfgr.hse.unwrap().freq, 25_000_000);

        let (clocks, _config) = cfgr.calculate_clocks();
        assert_eq!(clocks.sysclk().0, 216_000_000);
        assert!(clocks.is_pll48clk_valid());
    }

    #[test]
    fn test_rcc_calc3() {
        use super::{HSEClock, HSEClockMode, PLLP};
        use crate::time::U32Ext;

        let cfgr = CFGR {
            hse: None,
            hclk: None,
            sysclk: None,
            pclk1: None,
            pclk2: None,
            use_pll: false,
            use_pll48clk: false,
            pllm: 2,
            plln: 50,
            pllp: PLLP::Div2,
            pllq: 2,
        };

        let mut cfgr = cfgr
            .hse(HSEClock::new(25.mhz(), HSEClockMode::Bypass))
            .use_pll48clk()
            .set_defaults();
        cfgr.pll_configure();

        assert_eq!(cfgr.hse.unwrap().freq, 25_000_000);

        let (clocks, _config) = cfgr.calculate_clocks();
        assert_eq!(clocks.sysclk().0, 216_000_000);
        assert!(clocks.is_pll48clk_valid());
    }

    #[test]
    fn test_rcc_default() {
        use super::PLLP;

        let mut cfgr = CFGR {
            hse: None,
            hclk: None,
            sysclk: None,
            pclk1: None,
            pclk2: None,
            use_pll: false,
            use_pll48clk: false,
            pllm: 2,
            plln: 50,
            pllp: PLLP::Div2,
            pllq: 2,
        };

        cfgr.pll_configure();
        assert!(!cfgr.use_pll);
        let (clocks, _config) = cfgr.calculate_clocks();
        assert_eq!(clocks.sysclk().0, 16_000_000);
    }
}
