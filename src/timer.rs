//! Timers

use crate::embedded_time::rate::Hertz;
use crate::hal::timer::{Cancel, CountDown, Periodic};
use crate::pac::{
    TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9,
};
use crate::rcc;
use nb;
use void::Void;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::{DCB, DWT, SYST};

/// Timer wrapper
pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) freq: Hertz,
}

/// Hardware timers
pub struct CountDownTimer<TIM> {
    tim: TIM,
    freq: Hertz,
}

impl<TIM> Timer<TIM> {
    /// Creates CountDownTimer
    pub fn count_down(self) -> CountDownTimer<TIM> {
        let Self { tim, freq } = self;
        CountDownTimer { tim, freq }
    }
}

impl<TIM> Timer<TIM>
where
    CountDownTimer<TIM>: CountDown<Time = Hertz>,
{
    /// Starts timer in count down mode at a given frequency
    pub fn start_count_down<T>(self, timeout: T) -> CountDownTimer<TIM>
    where
        T: Into<Hertz>,
    {
        let mut timer = self.count_down();
        timer.start(timeout);
        timer
    }
}

impl<TIM> Periodic for CountDownTimer<TIM> {}

/// Interrupt events
#[derive(Debug, PartialEq)]
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

/// Timer errors
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Error {
    /// CountDownTimer is disabled
    Disabled,
    /// Auto-reload value is either set to 0 or bigger than the max value
    WrongAutoReload,
}

impl Timer<SYST> {
    /// Initialize timer
    pub fn syst(mut syst: SYST, clocks: &rcc::Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Self {
            tim: syst,
            freq: clocks.sysclk(),
        }
    }

    pub fn release(self) -> SYST {
        self.tim
    }
}

impl CountDownTimer<SYST> {
    /// Starts listening for an `event`
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::TimeOut => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::TimeOut => self.tim.disable_interrupt(),
        }
    }
}

impl CountDown for CountDownTimer<SYST> {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.freq.0 / timeout.into().0 - 1;

        assert!(rvr < (1 << 24));

        self.tim.set_reload(rvr);
        self.tim.clear_current();
        self.tim.enable_counter();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.tim.has_wrapped() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Cancel for CountDownTimer<SYST> {
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        if !self.tim.is_counter_enabled() {
            return Err(Self::Error::Disabled);
        }

        self.tim.disable_counter();
        Ok(())
    }
}

/// A monotonic non-decreasing timer
///
/// This uses the timer in the debug watch trace peripheral. This means, that if the
/// core is stopped, the timer does not count up. This may be relevant if you are using
/// cortex_m_semihosting::hprintln for debugging in which case the timer will be stopped
/// while printing
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, mut dcb: DCB, clocks: &rcc::Clocks) -> Self {
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or reset
        drop(dwt);

        MonoTimer {
            frequency: clocks.hclk(),
        }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(self) -> Instant {
        Instant {
            now: DWT::cycle_count(),
        }
    }
}

/// A measurement of a monotonically non-decreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.now)
    }
}

mod sealed {
    pub trait General {
        type Width: Into<u32>;
        fn max_auto_reload() -> u32;
        fn enable_counter(&mut self);
        fn disable_counter(&mut self);
        fn is_counter_enabled(&self) -> bool;
        fn reset_counter(&mut self);
        fn set_prescaler(&mut self, psc: u16);
        fn set_auto_reload(&mut self, arr: u32) -> Result<(), super::Error>;
        fn trigger_update(&mut self);
        fn clear_update_interrupt_flag(&mut self);
        fn listen_update_interrupt(&mut self, b: bool);
        fn get_update_interrupt_flag(&self) -> bool;
        fn read_count(&self) -> Self::Width;
        fn read_auto_reload(&self) -> Self::Width;
        fn start_one_pulse(&mut self);
        fn cr1_reset(&mut self);
    }
}
pub(crate) use sealed::General;

pub trait Instance:
    crate::Sealed + rcc::Enable + rcc::Reset + rcc::BusTimerClock + General
{
}

impl<TIM> Timer<TIM>
where
    TIM: Instance,
{
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &rcc::Clocks) -> Self {
        unsafe {
            // Enable and reset the timer peripheral
            TIM::enable_unchecked();
            TIM::enable_unchecked();
        }

        Self {
            freq: TIM::timer_clock(clocks),
            tim,
        }
    }
}

macro_rules! hal {
    ($($TIM:ty: $bits:ty,)+) => {
        $(
            impl Instance for $TIM { }

            impl General for $TIM {
                type Width = $bits;

                #[inline(always)]
                fn max_auto_reload() -> u32 {
                    <$bits>::MAX as u32
                }
                #[inline(always)]
                fn enable_counter(&mut self) {
                    self.cr1.modify(|_, w| w.cen().set_bit());
                }
                #[inline(always)]
                fn disable_counter(&mut self) {
                    self.cr1.modify(|_, w| w.cen().clear_bit());
                }
                #[inline(always)]
                fn is_counter_enabled(&self) -> bool {
                    self.cr1.read().cen().is_enabled()
                }
                #[inline(always)]
                fn reset_counter(&mut self) {
                    self.cnt.reset();
                }
                #[inline(always)]
                fn set_prescaler(&mut self, psc: u16) {
                    self.psc.write(|w| w.psc().bits(psc) );
                }
                #[inline(always)]
                fn set_auto_reload(&mut self, arr: u32) -> Result<(), Error> {
                    // Note: Make it impossible to set the ARR value to 0, since this
                    // would cause an infinite loop.
                    if arr > 0 && arr <= Self::max_auto_reload() {
                        Ok(self.arr.write(|w| unsafe { w.bits(arr) }))
                    } else {
                        Err(Error::WrongAutoReload)
                    }
                }
                #[inline(always)]
                fn trigger_update(&mut self) {
                    self.cr1.modify(|_, w| w.urs().set_bit());
                    self.egr.write(|w| w.ug().set_bit());
                    self.cr1.modify(|_, w| w.urs().clear_bit());
                }
                #[inline(always)]
                fn clear_update_interrupt_flag(&mut self) {
                    self.sr.write(|w| w.uif().clear_bit());
                }
                #[inline(always)]
                fn listen_update_interrupt(&mut self, b: bool) {
                    self.dier.write(|w| w.uie().bit(b));
                }
                #[inline(always)]
                fn get_update_interrupt_flag(&self) -> bool {
                    self.sr.read().uif().bit_is_clear()
                }
                #[inline(always)]
                fn read_count(&self) -> Self::Width {
                    self.cnt.read().bits() as Self::Width
                }
                #[inline(always)]
                fn read_auto_reload(&self) -> Self::Width {
                    self.arr.read().bits() as Self::Width
                }
                #[inline(always)]
                fn start_one_pulse(&mut self) {
                    self.cr1.write(|w| unsafe { w.bits(1 << 3) }.cen().set_bit());
                }
                #[inline(always)]
                fn cr1_reset(&mut self) {
                    self.cr1.reset();
                }
            }
        )+
    }
}

impl<TIM> CountDownTimer<TIM>
where
    TIM: General,
{
    /// Starts listening for an `event`
    ///
    /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::TimeOut => {
                // Enable update event interrupt
                self.tim.listen_update_interrupt(true);
            }
        }
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        match event {
            Event::TimeOut => {
                // Clear interrupt flag
                self.tim.clear_update_interrupt_flag();
            }
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::TimeOut => {
                // Disable update event interrupt
                self.tim.listen_update_interrupt(false);
            }
        }
    }

    /// Releases the TIM peripheral
    pub fn release(mut self) -> TIM {
        // pause counter
        self.tim.disable_counter();
        self.tim
    }
}

#[inline(always)]
pub(crate) const fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u32) {
    let ticks = clock / freq;
    let psc_u32 = (ticks - 1) / (1 << 16);
    let psc = if psc_u32 > u16::MAX as u32 {
        panic!();
    } else {
        psc_u32 as u16
    };
    let arr = ticks / (psc_u32 + 1) - 1;
    (psc, arr)
}

impl<TIM> CountDown for CountDownTimer<TIM>
where
    TIM: General,
{
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Self::Time>,
    {
        // pause
        self.tim.disable_counter();
        // reset counter
        self.tim.reset_counter();

        let (psc, arr) = compute_arr_presc(timeout.into().0, self.freq.0);
        self.tim.set_prescaler(psc);
        self.tim.set_auto_reload(arr).unwrap();

        // Trigger update event to load the registers
        self.tim.trigger_update();

        // start counter
        self.tim.enable_counter();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.tim.get_update_interrupt_flag() {
            Err(nb::Error::WouldBlock)
        } else {
            self.tim.clear_update_interrupt_flag();
            Ok(())
        }
    }
}

impl<TIM> Cancel for CountDownTimer<TIM>
where
    TIM: General,
{
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        if !self.tim.is_counter_enabled() {
            return Err(Self::Error::Disabled);
        }

        // disable counter
        self.tim.disable_counter();
        Ok(())
    }
}

hal!(
    TIM1: u16,
    TIM2: u32,
    TIM3: u16,
    TIM4: u16,
    TIM5: u32,
    TIM6: u16,
    TIM7: u16,
    TIM8: u16,
    TIM9: u16,
    TIM10: u16,
    TIM11: u16,
    TIM12: u16,
    TIM13: u16,
    TIM14: u16,
);

#[allow(unused)]
use crate::gpio::{
    gpioa::*, gpiob::*, gpioc::*, gpiod::*, gpioe::*, gpiof::*, gpiog::*, gpioh::*, gpioi::*,
    Alternate,
};

// Output channels markers
pub trait CPin<C, TIM> {}
pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

macro_rules! channel_impl {
    ( $( $TIM:ident, $C:ident, $PINX:ident, $AF:literal; )+ ) => {
        $(
            impl CPin<$C, $TIM> for $PINX<Alternate<$AF>> { }
        )+
    };
}

// All parts have these PWM pins.
channel_impl!(
    TIM1, C1, PA8, 1;
    TIM1, C2, PA9, 1;
    TIM1, C3, PA10, 1;
    TIM1, C4, PA11, 1;

    TIM5, C1, PA0, 2;
    TIM5, C2, PA1, 2;
    TIM5, C3, PA2, 2;
    TIM5, C4, PA3, 2;

    TIM9, C1, PA2, 3;
    TIM9, C2, PA3, 3;

    TIM11, C1, PB9, 3;
);

// TODO: check for other groups of parts
channel_impl!(
    TIM1, C1, PE9, 1;
    TIM1, C2, PE11, 1;
    TIM1, C3, PE13, 1;
    TIM1, C4, PE14, 1;

    TIM2, C1, PA0, 1;
    TIM2, C2, PA1, 1;
    TIM2, C3, PA2, 1;
    TIM2, C4, PA3, 1;

    TIM2, C2, PB3, 1;
    TIM2, C3, PB10, 1;
    TIM2, C4, PB11, 1;

    TIM2, C1, PA5, 1;
    TIM2, C1, PA15, 1;

    TIM3, C1, PA6, 2;
    TIM3, C2, PA7, 2;
    TIM3, C3, PB0, 2;
    TIM3, C4, PB1, 2;

    TIM3, C1, PB4, 2;
    TIM3, C2, PB5, 2;

    TIM3, C1, PC6, 2;
    TIM3, C2, PC7, 2;
    TIM3, C3, PC8, 2;
    TIM3, C4, PC9, 2;

    TIM4, C1, PB6, 2;
    TIM4, C2, PB7, 2;
    TIM4, C3, PB8, 2;
    TIM4, C4, PB9, 2;

    TIM4, C1, PD12, 2;
    TIM4, C2, PD13, 2;
    TIM4, C3, PD14, 2;
    TIM4, C4, PD15, 2;

    TIM10, C1, PB8, 3;
);

channel_impl!(
    TIM9, C1, PE5, 3;
    TIM9, C2, PE6, 3;
);

channel_impl!(
    TIM8, C1, PC6, 3;
    TIM8, C2, PC7, 3;
    TIM8, C3, PC8, 3;
    TIM8, C4, PC9, 3;

    TIM10, C1, PF6, 3;

    TIM11, C1, PF7, 3;

    TIM12, C1, PB14, 9;
    TIM12, C2, PB15, 9;

    TIM13, C1, PA6, 9;
    TIM13, C1, PF8, 9;  // Not a mistake: TIM13 has only one channel.

    TIM14, C1, PA7, 9;
    TIM14, C1, PF9, 9;  // Not a mistake: TIM14 has only one channel.
);

channel_impl!(
    TIM5, C1, PH10, 2;
    TIM5, C2, PH11, 2;
    TIM5, C3, PH12, 2;
    TIM5, C4, PI0, 2;

    TIM8, C1, PI5, 3;
    TIM8, C2, PI6, 3;
    TIM8, C3, PI7, 3;
    TIM8, C4, PI2, 3;

    TIM12, C1, PH6, 9;
    TIM12, C2, PH9, 9;
);
