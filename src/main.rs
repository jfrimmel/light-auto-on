#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use avr_device::attiny85::{CPU, EXINT, PORTB, WDT};

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
    let ext = &mut peripherals.EXINT;
    let portb = &mut peripherals.PORTB;
    let watchdog = &mut peripherals.WDT;
    let mut timer = timer0::Timer0::new(peripherals.TC0, portb);

    enum StateMachine {
        Off,
        TurningOn,
        On,
        TurningOff,
    }
    let mut state = StateMachine::Off;
    loop {
        let mut detect = || object_detected(&mut timer, cpu, watchdog, portb, ext);
        state = match state {
            StateMachine::Off if detect() => StateMachine::TurningOn,
            StateMachine::TurningOn => {
                cpu.mcucr.write(|w| w.sm().idle());
                timer.pwm(&LINEARIZATION, || power::sleep_for::<4>(cpu, watchdog));
                timer.halt();
                StateMachine::On
            }
            StateMachine::On if !detect() => StateMachine::TurningOff,
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

/// Check, if the SRF05 sensor detects an objected, i.e. a distance value is
/// below a (hardcoded) threshold.
///
/// The SRF05 sensor is not powered continuously in order to save power in idle
/// mode. This is done by connecting its power pin to PB4 of the µC. Therefore
/// this function will power on the sensor and wait for it to start up (the boot
/// time of the sensor is determined empirically: 64ms is to short, but 96ms
/// always works, so this is used). Afterwards, it will generate a long enough
/// trigger pulse and wait for the resulting pulse-length.
///
/// In order to save power, there is no accurate reading of the pulse length
/// (corresponding to the distance). Instead the timer value (with the prescaled
/// system clock) is compared to a empirically-chosen threshold).
///
/// If the pulse width is below the selected threshold, the function has an
/// object detected.
fn object_detected(
    timer: &mut timer0::Timer0,
    cpu: &mut CPU,
    watchdog: &mut WDT,
    portb: &mut PORTB,
    ext: &mut EXINT,
) -> bool {
    power::divide_system_clock_by::<256>(cpu);
    cpu.mcucr.write(|w| w.sm().pdown());

    ext.pcmsk.write(|w| w.pcint3().set_bit());
    ext.gimsk.write(|w| w.pcie().set_bit());

    // power the sensor on
    portb.ddrb.modify(|_, w| w.pb4().set_bit());
    portb.portb.modify(|_, w| w.pb4().set_bit());
    power::sleep_for::<4>(cpu, watchdog); // roughly ~32ms
    power::sleep_for::<8>(cpu, watchdog); // roughly ~64ms

    // set the trigger/signal pin to output (trigger)
    portb.ddrb.modify(|_, w| w.pb3().set_bit());

    // generate a trigger signal of at least 10µs
    portb.portb.modify(|_, w| w.pb3().set_bit());
    avr_device::asm::nop();
    // no sleep necessary here (core runs slow enough)
    portb.portb.modify(|_, w| w.pb3().clear_bit());

    // set the trigger/signal pin to input (signal)
    portb.ddrb.modify(|_, w| w.pb3().clear_bit());

    // wait for PCINT3 to trigger
    while portb.pinb.read().pb3().bit_is_clear() {
        //power::sleep(cpu);
    }
    timer.start();

    cpu.mcucr.write(|w| w.sm().idle()); // the timer must count in sleep

    let start = timer.current();
    // wait for PCINT3 to trigger
    while portb.pinb.read().pb3().bit_is_set() {
        //power::sleep(cpu);
    }
    timer.halt();

    // measurement done: power the sensor off
    portb.portb.modify(|_, w| w.pb4().clear_bit());

    let duration = timer.current().wrapping_sub(start);

    duration < 50 // empirically chosen
}

#[allow(clippy::missing_const_for_fn)]
#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    // deliberately empty, just used for waking up the device.
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
