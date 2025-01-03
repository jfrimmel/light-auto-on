//! Configuration of the Timer/counter 0 as a PWM generator or timebase.
use avr_device::attiny85::{PORTB, TC0};

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

    /// Perform a PWM sequence with a delay between each duty cycle.
    pub fn pwm<'a, I: IntoIterator<Item = &'a u8>, F: FnMut()>(
        &mut self,
        duty_cycles: I,
        mut delay: F,
    ) {
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
