#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod power;

#[avr_device::entry]
fn main() -> ! {
    // SAFETY: this is the first an only time, the peripherals are taken.
    // Normally, this would be done via the safe `take()`-function, but this
    // introduces a "possible panic" into the code with additional code being
    // generated. Therefore, this `unsafe`-function is used.
    let mut peripherals = unsafe { avr_device::attiny85::Peripherals::steal() };

    power::divide_system_clock_by::<256>(&mut peripherals.CPU); // 8MHz/256≈31kHz
    power::disable_unused_peripherals(&mut peripherals.CPU);

    peripherals.PORTB.ddrb.write(|w| w.pb0().set_bit());

    peripherals
        .TC0
        .tccr0a
        .write(|w| w.wgm0().pwm_fast().com0a().match_clear());
    peripherals.TC0.tccr0b.write(|w| w.cs0().direct());

    // https://www.mikrocontroller.net/articles/LED-Fading
    const LINEARIZATION: [u8; 32] = [
        0, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 19, 23, 27, 32, 38, 45, 54, 64, 76, 91,
        108, 128, 152, 181, 215, 255,
    ];

    loop {
        // configure sleep mode to be idle (since PWM is not working in power down).
        peripherals.CPU.mcucr.write(|w| w.sm().idle());
        peripherals.TC0.tccr0b.write(|w| w.cs0().direct());
        for duty_cycle in &LINEARIZATION {
            peripherals.TC0.ocr0a.write(|w| w.bits(*duty_cycle));
            power::sleep_for::<4>(&mut peripherals.CPU, &mut peripherals.WDT);
        }

        peripherals.TC0.tccr0b.write(|w| w.cs0().no_clock());
        peripherals.CPU.mcucr.write(|w| w.sm().pdown());
        power::sleep_for::<256>(&mut peripherals.CPU, &mut peripherals.WDT);

        // configure sleep mode to be idle (since PWM is not working in power down).
        peripherals.CPU.mcucr.write(|w| w.sm().idle());
        peripherals.TC0.tccr0b.write(|w| w.cs0().direct());
        for duty_cycle in LINEARIZATION.iter().rev() {
            peripherals.TC0.ocr0a.write(|w| w.bits(*duty_cycle));
            power::sleep_for::<4>(&mut peripherals.CPU, &mut peripherals.WDT);
        }

        peripherals.TC0.tccr0b.write(|w| w.cs0().no_clock());
        peripherals.CPU.mcucr.write(|w| w.sm().pdown());
        power::sleep_for::<256>(&mut peripherals.CPU, &mut peripherals.WDT);
    }
}

/// The panic handler of the application.
///
/// Normally, one would fill this panic handler with a reset, a flashing LED or
/// similar. But this project aims to avoid using panics altogether. Therefore
/// this panic handler should (in theory) not be necessary as it is never called
/// by Rust. But: the [`core`]-crate requires the function to be provided by a
/// `![no_std]`-binary. Therefore this function contains a body, that will not
/// compile (in reality: will not link) the application due to a missing symbol.
/// If there is any (potentially) reachable panic (i.e. one, that the optimizer
/// could not get rid of), an error like this is shown to the user:
/// ```console
/// src/main.rs:63:(.text.rust_begin_unwind+0x0): undefined reference to `a_panic_is_reachable'
/// ```
/// This indicates, that there is a possibly panicking call and shows even the
/// line number (63 in this case).
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    extern "C" {
        fn a_panic_is_reachable() -> !;
    }
    // SAFETY: if this would be reachable, the code would not compile
    unsafe { a_panic_is_reachable() };
}
