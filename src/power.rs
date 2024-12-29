//! Power-reduction functionality.
//!
//! The AVR microcontroller enables most of its peripherals by default, which is
//! convenient, but draws a lot of unnecessary power. Therefore this module aims
//! to help with reducing the power consumption by powering down certain unused
//! peripherals or even the whole CPU core.

// The accesses of the registers (reading/writing) is done via a shared (and
// thus immutable) reference, but logically, there are changes performed to
// those registers. Therefore the functions in this module will take mutable
// references, even if they would not need to by the borrow checker.
#![allow(clippy::needless_pass_by_ref_mut)]

use avr_device::attiny85::cpu::clkpr::CLKPS_A;

pub fn divide_system_clock_by<const N: usize>(cpu: &mut avr_device::attiny85::CPU) {
    let prescaler = match N {
        1 => CLKPS_A::PRESCALER_1,
        2 => CLKPS_A::PRESCALER_2,
        4 => CLKPS_A::PRESCALER_4,
        8 => CLKPS_A::PRESCALER_8,
        16 => CLKPS_A::PRESCALER_16,
        32 => CLKPS_A::PRESCALER_32,
        64 => CLKPS_A::PRESCALER_64,
        128 => CLKPS_A::PRESCALER_128,
        256 => CLKPS_A::PRESCALER_256,
        x => panic!("Illegal system clock divider `{x}` (use one of 1, 2, 4, ..., 256)"),
    };

    cpu.clkpr.write(|w| w.clkpce().set_bit());
    cpu.clkpr.write(|w| w.clkps().variant(prescaler));
}

pub fn disable_unused_peripherals(cpu: &mut avr_device::attiny85::CPU) {
    cpu.prr
        .write(|w| w.pradc().set_bit().prusi().set_bit().prtim1().set_bit());
}

pub fn sleep(cpu: &mut avr_device::attiny85::CPU) {
    // enter sleep mode, wake-up is triggered by the next watchdog interrupt
    cpu.mcucr.modify(|_r, w| w.se().set_bit());
    // SAFETY: this function is not called during `interrupt::free`
    unsafe { avr_device::interrupt::enable() };
    avr_device::asm::sleep();
    cpu.mcucr.modify(|_r, w| w.se().clear_bit());
}
