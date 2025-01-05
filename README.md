# Automatic light fade-in/fade-oud in distance threshold toggle

This little project is a small controller, which sits in between a power source and a stripe of LEDs. The LEDs should only be powered, once something approaches a sensor. This is detected using an ultrasonic distance sensor. The device therefore sits in between the 5V supply and the target LEDs. The device is always powered (unless explicitly powered down by un-plugging it), which implies, that it should be in a low-power/sleep mode most of the time.

# Requirements
1. 5V power input (coming from an USB-plug)
2. 5V switchable power output (~160mA: more than a single pin can handle)
3. output optionally may be a PWM output for fading
4. be able to sleep for really long times with minimal power consumption
5. MCU should run at very low speeds
6. project should be realized with the [selected MCU](#mcu-used)
7. sensor readout (and therefore sensor selection) should consume only minor power

# Sensor used
This project uses a SRF05 ultrasonic distance sensor. The sensor consumes 4mA on average when powered and provides a output pulse length proportional to the distance to the nearest object.

The sensor can be powered off until a readout is performed by connecting the positive power rail to an I/O-pin of the MCU. Once powered up, a trigger signal can be sent to the sensor. Afterwards the MCU can sleep until the rising edge of the sensor, start a timer and sleep again until the falling edge. This way, the timer value at the falling edge is proportional to the distance.

# MCU used
This project will be implemented using an AVR ATtiny85. This device is a simple-to-use micro-controller able to perform all tasks: it has several PWM outputs (only one is optionally required), a Watchdog-timer, which runs even in sleep mode allowing long sleeps between wake-ups to perform actions, and some pin-change interrupts and normal I/Os for interfacing the SRF05 sensor.
```ascii-drawing
                                    ┌───╥───┐
       (PCINT5/~RESET/ADC0/dB) PB5 ━┥ 1   8 ┝━ VCC
(PCINT3/XTAL1/CLKI/~OC1B/ADC3) PB3 ━┥ 2   7 ┝━ PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2)
 (PCINT4/XTAL2/CLKO/OC1B/ADC2) PB4 ━┥ 3   6 ┝━ PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1)
                               GND ━┥ 4   5 ┝━ PB0 (MOSI/DI/SDA/AIN0/OC0A/~OC1A/AREF/PCINT0)
                                    └───────┘
```
`VCC` and `GND` are obviously used. `PB5` is also not available, since the reset-functionality should stay, so that ISP is still possible. This makes a total of 5 pins available to implement the actual functionality. In order to remove the need for an external crystal, the device will be clocked from the internal RC-oscillator, but to save power (by leveraging the low clock speed), the clock source can be divided by up to 256 (yielding a frequency of `8MHz/256≈31kHz`).

# Schematic
The schematic is rather simple: the device sits the two ends of an USB-cable meaning, that 5V power is available.
```ascii-drawing
+5V ╾────────────────────────────────┬───────────╼ +5V (LEDs)
GND ╾──┐                             │         ┌─╼ GND (LEDs)
    I  ┷                ┌────╥────┐  │ 100n    │ O
    N                  ━┥ PB5 VCC ┝━─┴──╫─┐    │ U
                 ┌─────━┥ PB3 PB2 ┝━      ┷    │ T
         S       │ ┌───━┥ PB4 PB1 ┝━           │
         R       │ │ ┌─━┥ GND PB0 ┝━──┐ ┌──────┘
         F       │ │ ┷  └─────────┘   │ │
         0       │ │             ┌──┐ │ │
         5       │ │             ┷  ╽ ╽ ╽ IRZF14
     VCC ╾───────┘ │                S G D
         ╾         │        (pinout changed for
Trig/Sig ╾─────────┘         better schematic)
    Mode ╾─┐
     GND ╾─┤
           ┷
```
All the parts were selected because they were available. The MOSFET is way to powerful for such a little application, but it can be driven with the 5V logic level. Any other logic-level MOSFET capable of handling the current for the LEDs can be used.
Please note, that the actual pinout of the MOSFET is different as shown above (the drawing should not contain wire crossings).
