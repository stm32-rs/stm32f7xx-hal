//! Quadrature Encoder Interface API

use crate::rcc::{Enable, Reset, APB1};
#[cfg(feature = "stm32f767")]
use stm32f7::stm32f7x7::{TIM2, TIM3, TIM4, TIM5};

#[cfg(feature = "stm32f769")]
use stm32f7::stm32f7x9::{TIM2, TIM3, TIM4, TIM5};

#[derive(Debug)]
pub enum Direction {
    Upcounting,
    Downcounting,
}

/// SMS[3:0] (Slave Mode Selection) register
#[derive(Debug, Clone, Copy)]
pub enum SlaveMode {
    /// Slave mode disabled - if CEN = ‘1’ then the prescaler is clocked directly by the internal
    /// clock.
    Disable = 0b0000,

    /// Counter counts up/down on TI2FP1 edge depending on TI1FP2 level.
    EncoderMode1 = 0b0001,

    /// Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level.
    EncoderMode2 = 0b0010,

    /// Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the
    /// level of the other input.
    EncoderMode3 = 0b0011,

    /// Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and
    /// generates an update of the registers.
    ResetMode = 0b0100,

    /// Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The
    /// counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of
    /// the counter are controlled.
    GatedMode = 0b0101,

    /// Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not
    /// reset). Only the start of the counter is controlled.
    TriggerMode = 0b0110,

    /// External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    ExternalClockMode1 = 0b0111,

    /// Combined reset + trigger mode - Rising edge of the selected trigger input (TRGI)
    /// reinitializes the counter, generates an update of the registers and starts the counter.
    Combined = 0b1000,
}

/// Quadrature Encoder Interface (QEI) options
#[derive(Debug, Clone, Copy)]
pub struct QeiOptions {
    /// Encoder slave mode
    pub slave_mode: SlaveMode,

    /// Autoreload value
    ///
    /// This value allows the maximum count to be configured. Setting a lower value
    /// will overflow the counter to 0 sooner.
    pub auto_reload_value: u32,
}

impl Default for QeiOptions {
    fn default() -> Self {
        Self {
            slave_mode: SlaveMode::EncoderMode3,
            auto_reload_value: core::u32::MAX,
        }
    }
}

///
/// Make sure that pin_ch1 and pin_ch2 are used in the corresponding alternative mode.
/// ----------------------------------
///   TIMx | PIN_CH1  | PIN_CH2  |
/// -------|----------|----------|
///   TIM2 | PA0 \ 1  | PB3 \ 1  |
///   TIM2 | PA0 \ 1  | PA1 \ 1  |
///   TIM2 | PA5 \ 1  | PB3 \ 1  |
///   TIM2 | PA5 \ 1  | PA1 \ 1  |
///   TIM2 | PA15 \ 1 | PB3 \ 1  |
///   TIM2 | PA15 \ 1 | PA1 \ 1  |
///   TIM3 | PA6 \ 2  | PA7 \ 2  |
///   TIM3 | PA6 \ 2  | PB5 \ 2  |
///   TIM3 | PA6 \ 2  | PC7 \ 2  |
///   TIM3 | PB4 \ 2  | PA7 \ 2  |
///   TIM3 | PB4 \ 2  | PB5 \ 2  |
///   TIM3 | PB4 \ 2  | PC7 \ 2  |
///   TIM3 | PC6 \ 2  | PA7 \ 2  |
///   TIM3 | PC6 \ 2  | PB5 \ 2  |
///   TIM3 | PC6 \ 2  | PC7 \ 2  |
///   TIM4 | PB6 \ 2  | PB7 \ 2  |
///   TIM4 | PB6 \ 2  | PD13 \ 2 |
///   TIM4 | PD12 \ 2 | PB7 \ 2  |
///   TIM4 | PD12 \ 2 | PD13 \ 2 |
///   TIM4 | PD12 \ 2 | PD13 \ 2 |
///   TIM5 | PA0 \ 2  | PA1 \ 2  |
pub struct Qei<PIN1, PIN2, TIM> {
    tim: TIM,
    _pin_ch1: PIN1,
    _pin_ch2: PIN2,
}

// General-purpose timers (TIM2/TIM3/TIM4/TIM5) : Up, Down, Up/Down
macro_rules! hal_qei {
    ($fct:ident,$TIMX:ty, $bits:ty) => {
        //, $CH1:ident<$AFCH1:ty>, $CH2:ident<$AFCH2:ty>) => {
        impl<PIN1, PIN2> Qei<PIN1, PIN2, $TIMX> {
            //Qei<$CH1<Alternate<$AFCH1>>, $CH2<Alternate<$AFCH2>>, $TIM> {
            pub fn $fct(
                tim: $TIMX,
                pin_ch1: PIN1, //$CH1<Alternate<$AFCH1>>,
                pin_ch2: PIN2, //$CH2<Alternate<$AFCH2>>,
                apb1: &mut APB1,
                options: QeiOptions,
            ) -> Self {
                // enable and reset peripheral to a clean slate state
                <$TIMX>::enable(apb1);
                <$TIMX>::reset(apb1);

                // Configure TxC1 and TxC2 as captures
                tim.ccmr1_output()
                    .write(|w| unsafe { w.cc1s().bits(0b01).cc2s().bits(0b01) });

                // enable and configure to capture on rising edge
                tim.ccer.write(|w| {
                    w.cc1e()
                        .set_bit()
                        .cc1p()
                        .clear_bit()
                        .cc2e()
                        .set_bit()
                        .cc2p()
                        .clear_bit()
                });

                // configure as quadrature encoder
                tim.smcr.write(|w| w.sms().bits(options.slave_mode as u8));
                tim.arr
                    .write(|w| unsafe { w.bits(options.auto_reload_value) });
                tim.cr1.write(|w| w.cen().set_bit());

                Self {
                    tim,
                    _pin_ch1: pin_ch1,
                    _pin_ch2: pin_ch2,
                }
            }

            pub fn read_count(&self) -> $bits {
                self.tim.cnt.read().bits() as $bits
            }

            pub fn read_direction(&self) -> Direction {
                if self.tim.cr1.read().dir().bit_is_clear() {
                    Direction::Upcounting
                } else {
                    Direction::Downcounting
                }
            }

            pub fn release(self) -> ($TIMX, PIN1, PIN2) {
                (self.tim, self._pin_ch1, self._pin_ch2)
            }
        }
    };
}

hal_qei! {qei_tim2, TIM2, u32}
hal_qei! {qei_tim3, TIM3, u16}
hal_qei! {qei_tim4, TIM4, u16}
hal_qei! {qei_tim5, TIM5, u32}
