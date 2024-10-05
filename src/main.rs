#![no_std]
#![no_main]

#[avr_device::entry]
fn main() -> ! {
    let peripherals = unsafe { avr_device::attiny85::Peripherals::steal() };
    peripherals
        .PORTB
        .ddrb
        .modify(|_r, w| w.pb3().set_bit().pb4().set_bit());
    peripherals
        .PORTB
        .portb
        .modify(|_r, w| w.pb3().set_bit().pb4().clear_bit());
    loop {
        peripherals.PORTB.pinb.write(|w| w.pb4().set_bit());
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
