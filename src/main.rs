//! Automatic light fade-in/fade-oud in distance threshold toggle
//!
//! This little project is a small controller, which sits in between a power
//! source and a stripe of LEDs. The LEDs should only be powered, once something
//! approaches a sensor. This is detected using an ultrasonic distance sensor.
//! The device therefore sits in between the 5V supply and the target LEDs. The
//! device is always powered (unless explicitly powered down by un-plugging it),
//! which implies, that it should be in a low-power/sleep mode most of the time.
//!
//! # Requirements
//! 1. 5V power input (coming from an USB-plug)
//! 2. 5V switchable power output (~160mA: more than a single pin can handle)
//! 3. output optionally may be a PWM output for fading
//! 4. be able to sleep for really long times with minimal power consumption
//! 5. MCU should run at very low speeds
//! 6. project should be realized with the [selected MCU](#mcu-used)
//! 7. sensor readout (and therefore sensor selection) should consume only minor
//!    power
//!
//! # Sensor used
//! This project uses a SRF05 ultrasonic distance sensor. The sensor consumes
//! 4mA on average when powered and provides a output pulse length proportional
//! to the distance to the nearest object.
//!
//! The sensor can be powered off until a readout is performed by connecting the
//! positive power rail to an I/O-pin of the MCU. Once powered up, a trigger
//! signal can be sent to the sensor. Afterwards the MCU can sleep until the
//! rising edge of the sensor, start a timer and sleep again until the falling
//! edge. This way, the timer value at the falling edge is proportional to the
//! distance.
//!
//! # MCU used
//! This project will be implemented using an AVR ATtiny85. This device is a
//! simple-to-use micro-controller able to perform all tasks: it has several PWM
//! outputs (only one is optionally required), a Watchdog-timer, which runs even
//! in sleep mode allowing long sleeps between wake-ups to perform actions, and
//! some pin-change interrupts and normal I/Os for interfacing the SRF05 sensor.
//! ```ascii-drawing
//!                                     ┌───╥───┐
//!        (PCINT5/~RESET/ADC0/dB) PB5 ━┥ 1   8 ┝━ VCC
//! (PCINT3/XTAL1/CLKI/~OC1B/ADC3) PB3 ━┥ 2   7 ┝━ PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2)
//!  (PCINT4/XTAL2/CLKO/OC1B/ADC2) PB4 ━┥ 3   6 ┝━ PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1)
//!                                GND ━┥ 4   5 ┝━ PB0 (MOSI/DI/SDA/AIN0/OC0A/~OC1A/AREF/PCINT0)
//!                                     └───────┘
//! ```
//! `VCC` and `GND` are obviously used. `PB5` is also not available, since the
//! reset-functionality should stay, so that ISP is still possible. This makes
//! a total of 5 pins available to implement the actual functionality. In order
//! to remove the need for an external crystal, the device will be clocked from
//! the internal RC-oscillator, but to save power (by leveraging the low clock
//! speed), the clock source can be divided by up to 256 (yielding a frequency
//! of `8MHz/256≈31kHz`).
//!
//! # Schematic
//! The schematic is rather simple: the device sits the two ends of an USB-cable
//! meaning, that 5V power is available.
//! ```ascii-drawing
//! +5V ╾────────────────────────────────┬───────────╼ +5V (LEDs)
//! GND ╾──┐                             │         ┌─╼ GND (LEDs)
//!     I  ┷                ┌────╥────┐  │ 100n    │ O
//!     N                  ━┥ PB5 VCC ┝━─┴──╫─┐    │ U
//!                  ┌─────━┥ PB3 PB2 ┝━      ┷    │ T
//!          S       │ ┌───━┥ PB4 PB1 ┝━           │
//!          R       │ │ ┌─━┥ GND PB0 ┝━──┐ ┌──────┘
//!          F       │ │ ┷  └─────────┘   │ │
//!          0       │ │             ┌──┐ │ │
//!          5       │ │             ┷  ╽ ╽ ╽ IRZF14
//!      VCC ╾───────┘ │                S G D
//!          ╾         │        (pinout changed for
//! Trig/Sig ╾─────────┘         better schematic)
//!     Mode ╾─┐
//!      GND ╾─┤
//!            ┷
//! ```
//! All the parts were selected because they were available. The MOSFET is way
//! to powerful for such a little application, but it can be driven with the 5V
//! logic level. Any other logic-level MOSFET capable of handling the current
//! for the LEDs can be used.
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use avr_device::attiny85::{CPU, EXINT, PORTB, WDT};

mod power;
mod timer0;

/// Inverse-logarithmic table for perceived linear brightness of LEDs.
///
/// This table was taken from [LED-Fading on microncontroller.net][uC.net], more
/// specifically the values of `pwmtable_8D`. This is the "best" 8bit PWM
/// presented on that page.
///
/// [uC.net]: https://www.mikrocontroller.net/index.php?title=LED-Fading&oldid=106397
const LINEARIZATION: [u8; 32] = [
    0, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 19, 23, 27, 32, 38, 45, 54, 64, 76, 91,
    108, 128, 152, 181, 215, 255,
];

/// The different states of the main-loop.
enum StateMachine {
    /// The LEDs are currently off.
    ///
    /// This state will transition to [`StateMachine::TurningOn`] if the value
    /// of the currently measured distance is below [`DISTANCE_THRESHOLD_CM`].
    Off,
    /// The LEDs are get faded in.
    ///
    /// The next state will always be [`StateMachine::On`].
    TurningOn,
    /// The LEDs are currently on.
    ///
    /// This state will transition to [`StateMachine::TurningOff`] if the value
    /// of the currently measured distance is above [`DISTANCE_THRESHOLD_CM`].
    On,
    /// The LEDs are get faded out.
    ///
    /// The next state will always be [`StateMachine::Off`].
    TurningOff,
}

#[avr_device::entry]
fn main() -> ! {
    // SAFETY: this is the first an only time, the peripherals are taken.
    // Normally, this would be done via the safe `take()`-function, but this
    // introduces a "possible panic" into the code with additional code being
    // generated. Therefore, this `unsafe`-function is used.
    let mut peripherals = unsafe { avr_device::attiny85::Peripherals::steal() };

    power::divide_system_clock_by::<256>(&mut peripherals.CPU); // 8MHz/256≈31kHz
    power::disable_unused_peripherals(&mut peripherals.CPU, &mut peripherals.AC);

    let cpu = &mut peripherals.CPU;
    let ext = &mut peripherals.EXINT;
    let portb = &mut peripherals.PORTB;
    let watchdog = &mut peripherals.WDT;
    let mut timer = timer0::Timer0::new(peripherals.TC0, portb);

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

/// The distance threshold und which the distance sensor has detected an object.
///
/// This is the distance in centimeters, that is used in [`object_detected()`]
/// function to determine, if there is an object in front of the sensor. This
/// should be set to the largest value that reliably does not "detect" the
/// environment around the sensor in order to be sensitive enough.
///
/// Note, that the sensor resolution is limited to ~2cm due to the caveats
/// described in the [`object_detected()`] function.
const DISTANCE_THRESHOLD_CM: f32 = 160.;

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
/// (corresponding to the distance). Instead, a rough estimate is done by having
/// large enough system clock and timer prescaler to have all possible distances
/// inside the measurement window. The controller sleeps most of the time by
/// waiting for watchdog or pin-change interrupts and only performing the most
/// necessary operations.
///
/// If the pulse width is below the selected threshold, the function has an
/// object detected.
#[allow(clippy::needless_pass_by_ref_mut)] // see comment in `power.rs` for rationale
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

    // Speed up the system clock by a bit. If this is not done, the first PCINT
    // interrupt vector fires to late, so that the program misses an edge. As it
    // sleeps almost for the entire time in power down mode, this is fine.
    power::divide_system_clock_by::<64>(cpu);

    // wait for PCINT3 to trigger
    ext.gimsk.write(|w| w.pcie().set_bit());
    while portb.pinb.read().pb3().bit_is_clear() {
        power::sleep(cpu);
    }
    ext.gimsk.write(|w| w.pcie().clear_bit());
    timer.start();

    cpu.mcucr.write(|w| w.sm().idle()); // the timer must count in sleep

    // The SRF05 has a maximum pulse length 0f 30ms. Since timer 0 is an 8bit
    // timer, that value must fit into an 8bit number to prevent overflows (i.e.
    // `(8000000Hz/p)*0.03s < 256` where `p` is the prescaler). So, the total
    // timer prescaler `p` (which itself consists of the system clock prescaler
    // and the timer prescaler) must be chosen large enough to fulfill the in-
    // equality above but as small as possible to still have adequate resolution
    // in the measurement.
    // Furthermore the system clock divider should be large in order to save
    // power.
    // A system prescaler of 128 and a timer prescaler of 8 allow for a maximum
    // pulse length of 32kµs, which is enough. With this setting, a single timer
    // count (i.e. the LSB) is equal to 128µs or roughly 2cm.
    power::divide_system_clock_by::<128>(cpu);
    timer.prescale_by::<8>();
    #[allow(clippy::items_after_statements)] // it makes sense to place it here
    const DISTANCE_PER_TICK: f32 = 1_000_000.0 / (8_000_000.0 / 128.0 / 8.0) / 58.;
    //                             ═════╤═════    ═════╤═════   ══╤══   ═╤═    ═╤═
    //                 #µs in 1s ───────┘              │          │      │      │
    //             #cycles in 1s ──────────────────────┘          │      │      │
    //      system clock divider ─────────────────────────────────┘      │      │
    //           timer prescaler ────────────────────────────────────────┘      │
    //  #µs in 1cm (to & return) ───────────────────────────────────────────────┘

    let start = timer.current();
    // wait for PCINT3 to trigger
    ext.gimsk.write(|w| w.pcie().set_bit());
    while portb.pinb.read().pb3().bit_is_set() {
        power::sleep(cpu);
    }
    ext.gimsk.write(|w| w.pcie().clear_bit());
    timer.halt();

    // measurement done: power the sensor off
    portb.portb.modify(|_, w| w.pb4().clear_bit());

    let duration = timer.current().wrapping_sub(start);

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // user is responsible
    let threshold = (DISTANCE_THRESHOLD_CM / DISTANCE_PER_TICK) as u8;
    duration < threshold
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
