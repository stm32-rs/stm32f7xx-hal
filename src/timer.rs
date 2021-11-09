//! Timers

use crate::embedded_time::rate::Hertz;
use crate::hal::timer::{Cancel, CountDown, Periodic};
use crate::pac::{
    TIM1, TIM10, TIM11, TIM12, TIM13, TIM14, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9,
};
use crate::rcc::{Clocks, Enable, RccBus, Reset};
use cast::{u16, u32};
use nb;
use void::Void;

/// Hardware timers
pub struct Timer<TIM> {
    clock: Hertz,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
#[derive(Debug, PartialEq)]
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

/// Timer errors
#[derive(Debug, PartialEq)]
pub enum Error {
    /// Timer is disabled.
    Disabled,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $timclk:ident),)+) => {
        $(
            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.disable();

                    self.timeout = timeout.into();
                    let frequency = self.timeout.0;
                    let ticks = self.clock.0 / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());

                    self.enable();
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Cancel for Timer<$TIM> {
                type Error = Error;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    if !self.tim.cr1.read().cen().is_enabled() {
                        return Err(Error::Disabled);
                    }

                    self.disable();

                    Ok(())
                }
            }

            impl Timer<$TIM> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, clocks: Clocks, apb: &mut <$TIM as RccBus>::Bus) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    <$TIM>::enable(apb);
                    <$TIM>::reset(apb);

                    let clock = clocks.$timclk();

                    let mut timer = Timer {
                        clock,
                        tim,
                        timeout: Hertz(0),
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
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
                            self.tim.sr.write(|w| w.uif().clear_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn free(mut self) -> $TIM {
                    self.disable();

                    self.tim
                }

                /// Enables the counter.
                fn enable(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Disables the counter.
                fn disable(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }
            }
        )+
    }
}

hal! {
    TIM2: (tim2, timclk1),
    TIM3: (tim3, timclk1),
    TIM4: (tim4, timclk1),
    TIM5: (tim5, timclk1),
    TIM6: (tim6, timclk1),
    TIM7: (tim7, timclk1),
    TIM12: (tim12, timclk1),
    TIM13: (tim13, timclk1),
    TIM14: (tim14, timclk1),

    TIM1: (tim1, timclk2),
    TIM8: (tim8, timclk2),
    TIM9: (tim9, timclk2),
    TIM10: (tim10, timclk2),
    TIM11: (tim11, timclk2),
}
