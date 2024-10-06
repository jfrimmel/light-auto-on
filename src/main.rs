#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

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

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
