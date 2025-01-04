//! Configuration of the Timer/counter 0 as a PWM generator or timebase.

#![allow(clippy::needless_pass_by_ref_mut)] // see comment in `power.rs` for rationale

use avr_device::attiny85::{tc0::tccr0b::CS0_A, PORTB, TC0};

/// Access to the 8bit timer/counter 0 peripheral.
pub struct Timer0(TC0);
impl Timer0 {
    /// Create a new [`Timer0`] from a raw register block. This sets the PWM-pin
    /// PB0 as an output pin as well.
    pub fn new(registers: TC0, portb: &mut PORTB) -> Self {
        portb.ddrb.modify(|_, w| w.pb0().set_bit());

        registers
            .tccr0a
            .write(|w| w.wgm0().pwm_fast().com0a().match_clear());
        registers.tccr0b.write(|w| w.cs0().no_clock());
        Self(registers)
    }

    /// Change the prescaler of the timer to the given value (1, 8, 64, 256 or
    /// 1024).
    pub fn prescale_by<const N: usize>(&mut self) {
        let variant = match N {
            1 => CS0_A::DIRECT,
            8 => CS0_A::PRESCALE_8,
            64 => CS0_A::PRESCALE_64,
            256 => CS0_A::PRESCALE_256,
            1024 => CS0_A::PRESCALE_1024,
            x => panic!("Illegal clock divider `{x}` (use one of 1, 8, 64, 256, 1024)"),
        };
        self.0.tccr0b.modify(|_, w| w.cs0().variant(variant));
    }

    /// Perform a PWM sequence with a delay between each duty cycle.
    pub fn pwm<'a, I: IntoIterator<Item = &'a u8>, F: FnMut()>(
        &mut self,
        duty_cycles: I,
        mut delay: F,
    ) {
        self.prescale_by::<1>();
        self.0
            .tccr0a
            .write(|w| w.wgm0().pwm_fast().com0a().match_clear());
        self.0.tccr0b.write(|w| w.cs0().direct());

        for duty_cycle in duty_cycles {
            self.0.ocr0a.write(|w| w.bits(*duty_cycle));
            delay();
        }
    }

    /// Start the timer (resuming to output the previous PWM value).
    pub fn start(&mut self) {
        self.0.tcnt0.reset();
        self.0.tccr0b.write(|w| w.cs0().direct());
    }

    /// Read the current timer value.
    pub fn current(&self) -> u8 {
        self.0.tcnt0.read().bits()
    }

    /// Stop the timer, preserving the previous last PWM output value.
    pub fn halt(&mut self) {
        self.0.tccr0b.write(|w| w.cs0().no_clock());
    }
}
