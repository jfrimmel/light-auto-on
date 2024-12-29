#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

#[avr_device::entry]
fn main() -> ! {
    // SAFETY: this is the first an only time, the peripherals are taken.
    // Normally, this would be done via the safe `take()`-function, but this
    // introduces a "possible panic" into the code with additional code being
    // generated. Therefore, this `unsafe`-function is used.
    let peripherals = unsafe { avr_device::attiny85::Peripherals::steal() };

    // Divide the system clock by 256, which yields a clock of 8MHz/256≈31kHz
    peripherals.CPU.clkpr.write(|w| w.clkpce().set_bit());
    peripherals.CPU.clkpr.write(|w| w.clkps().prescaler_256());

    peripherals
        .PORTB
        .ddrb
        .modify(|_r, w| w.pb3().set_bit().pb4().set_bit());
    peripherals
        .PORTB
        .portb
        .modify(|_r, w| w.pb3().set_bit().pb4().clear_bit());

    // configure sleep mode to be power down.
    peripherals.CPU.mcucr.write(|w| w.sm().pdown());
    loop {
        peripherals.PORTB.pinb.write(|w| w.pb4().set_bit());

        // configure the watchdog timer to trigger in ~8s
        peripherals
            .WDT
            .wdtcr
            .write(|w| w.wdie().set_bit().wdph().set_bit().wdpl().cycles_4k_1024k());

        // enter sleep mode, wake-up is triggered by the next watchdog interrupt
        peripherals.CPU.mcucr.modify(|_r, w| w.se().set_bit());
        // SAFETY: this function is not called during `interrupt::free`
        unsafe { avr_device::interrupt::enable() };
        avr_device::asm::sleep();
        peripherals.CPU.mcucr.modify(|_r, w| w.se().clear_bit());
    }
}

#[avr_device::interrupt(attiny85)]
fn WDT() {
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
