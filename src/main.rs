#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod power;
mod timer0;

#[avr_device::entry]
fn main() -> ! {
    // SAFETY: this is the first an only time, the peripherals are taken.
    // Normally, this would be done via the safe `take()`-function, but this
    // introduces a "possible panic" into the code with additional code being
    // generated. Therefore, this `unsafe`-function is used.
    let mut peripherals = unsafe { avr_device::attiny85::Peripherals::steal() };

    power::divide_system_clock_by::<256>(&mut peripherals.CPU); // 8MHz/256≈31kHz
    power::disable_unused_peripherals(&mut peripherals.CPU);

    // https://www.mikrocontroller.net/articles/LED-Fading
    const LINEARIZATION: [u8; 32] = [
        0, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 19, 23, 27, 32, 38, 45, 54, 64, 76, 91,
        108, 128, 152, 181, 215, 255,
    ];

    let cpu = &mut peripherals.CPU;
    let watchdog = &mut peripherals.WDT;
    let mut timer = timer0::Timer0::new(peripherals.TC0, &mut peripherals.PORTB);

    enum StateMachine {
        Off,
        TurningOn,
        On,
        TurningOff,
    }
    let mut state = StateMachine::Off;
    loop {
        state = match state {
            StateMachine::Off if true => StateMachine::TurningOn,
            StateMachine::TurningOn => {
                cpu.mcucr.write(|w| w.sm().idle());
                timer.pwm(&LINEARIZATION, || power::sleep_for::<4>(cpu, watchdog));
                timer.halt();
                StateMachine::On
            }
            StateMachine::On if true => StateMachine::TurningOff,
            StateMachine::TurningOff => {
                let sequence = LINEARIZATION.iter().rev();
                cpu.mcucr.write(|w| w.sm().idle());
                timer.pwm(sequence, || power::sleep_for::<4>(cpu, watchdog));
                timer.halt();
                StateMachine::Off
            }
            old_state => old_state,
        };

        cpu.mcucr.write(|w| w.sm().pdown());
        power::sleep_for::<256>(cpu, watchdog);
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
