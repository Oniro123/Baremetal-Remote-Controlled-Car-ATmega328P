# Baremetal Remote Controlled Car — ATmega328P

A remote controlled car ported from Arduino to bare-metal C, targeting the *ATmega328P* microcontroller. This project was ported from an Arduino sketch to direct register-level programming.

Built to demonstrate low-level embedded systems knowledge: interrupt-driven I/O, timer management, custom protocol decoding, and hardware peripheral control from scratch.


## Features

- *IR Remote Control* — Decodes NEC infrared protocol entirely in software using a pin-change interrupt and a hardware timer. No IR library used.
- *Dual Motor Control* — Drives two DC motors (left and right), supporting forward, backward, left, and right movement.
- *Ultrasonic Obstacle Detection* — Uses an HC-SR04 sensor to measure distance and trigger automatic avoidance behaviour.
- *Self-Driving Mode* — Toggle between manual IR control and autonomous navigation with a single button press on the remote.
- *UART Debug Output* — Serial logging over UART for real-time diagnostics, implemented without printf or any standard Arduino library.


### Pin Mapping

Motor A IN1 -> AVR Pin: PB1 -> Arduino Pin: D9
Motor A IN2 -> AVR Pin: PD6 -> Arduino Pin: D6
Motor B IN3 -> AVR Pin: PD5 -> Arduino Pin: D5
Motor B IN4 -> AVR Pin: PD3 -> Arduino Pin: D3
Ultrasonic TRIG -> AVR Pin: PD2 -> Arduino Pin: D2
Ultrasonic ECHO -> AVR Pin: PB5 -> Arduino Pin: D13
IR Receiver -> AVR Pin: PB3 -> Arduino Pin: D11

## How It Works

### IR Decoding (NEC Protocol)

The IR receiver triggers a pin-change interrupt (PCINT0) on every signal edge. A state machine measures the elapsed time between edges using Timer 1 (prescaler = 8, giving 0.5 µs per tick) and decodes the NEC frame. 9 ms leader pulse, 4.5 ms space, then 32 bits of address and command data with checksum validation.

### Timer 1

One timer running continously with an overflow interrupt to extend its range beyond 16 bits, allowing accurate timing of both IR pulses and sonar echoes without blocking the CPU.

### Ultrasonic Sensing

The HC-SR04 is triggered with a 10 µs pulse on the TRIG pin. The duration of the ECHO pulse is measured using Timer 1 and converted to centimetres using the speed of sound (343 m/s). 30 ms timeout prevents the car from hanging if no echo is received.

### Motor Control

Motors are driven by directly writing to GPIO registers.

### Self-Driving Mode

When self-driving is active, the car continuously polls the sonar sensor. If an obstacle is detected within 10 cm, it stops and turns right before continuing forward. Toggling between IR remote control and self-driving mode is mapped to a dedicated IR button (code 0x46).

-----

## IR Button Mapping

0x09 -> Forward
0x15 -> Backward
0x40 -> Left
0x43 -> Right
0x45 -> Stop
0x46 -> Toggle modes


## What This Demonstrates

- *Direct register manipulation* — DDRx, PORTx, PINx, TIMSKx, TCCRx, SREG, and more
- *Interrupt-driven design* — ISR for timer overflow and pin-change events, with correct critical section handling (cli/sei, SREG save/restore)
- *Protocol decoding without libraries* — NEC IR protocol implemented as a state machine in an ISR
- *Hardware timer usage* — Free-running timer with overflow tracking for microsecond-accurate timing
- *Custom UART implementation* — Transmit functions written directly against the USART registers
- *Peripheral interfacing* — HC-SR04 ultrasonic sensor, motor driver, IR receiver
- *Porting from Arduino to baremetal* — Understanding what the Arduino framework abstracts away and replacing it with direct AVR C
