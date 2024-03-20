# ElectricBed: Motorized Queen-Sized Bed Project

Welcome to the `ElectricBed` project! This comprehensive guide is designed to assist you in creating a low-speed, electrically powered vehicle, uniquely constructed as a queen-sized bed that can carry two people. It's remotely controlled, featuring advanced functionalities such as telemetry feedback, dynamic control systems, and LED lighting, all managed by an Arduino Mega 2560.

## Overview

The ElectricBed project transforms the concept of mobility and rest into a single, innovative design. Equipped with dual Electronic Speed Controllers (ESCs), input from a FlySky FS-i6X transmitter, LED lighting effects, and a motion sensor for enhanced driving stability, this project invites hobbyists and developers to explore the fusion of comfort and technology.

### Key Features

- Dynamic speed and turn control for smooth maneuverability.
- Telemetry feedback for battery monitoring and RPM-based adjustments, ensuring safe and efficient operation.
- Customizable LED lighting for aesthetic appeal and visibility.
- Motion detection for improved stability and traction control, enhancing passenger comfort.
- Auxiliary controls via relays for headlights, parking lights, turn signals, horn, and reverse light.
- Comprehensive safety features such as an emergency stop mechanism and a 2-step kill switch.

### Required Hardware

- Arduino Mega 2560 for central control.
- Two Electronic Speed Controllers (ESCs) for motor management.
- FlySky FS-i6X remote control transmitter and receiver for remote operation.
- Adafruit NeoPixel LED Strip for customizable lighting.
- MPU6050 Accelerometer and Gyro Sensor for motion detection and stability control.
- Relay module(s) for controlling auxiliary features like headlights, parking lights, turn signals, horn, and reverse light.
- Adequate power sources for the Arduino, ESCs, LEDs, and any other peripherals.

### Wiring Instructions

1. **ESC Connections**: Connect the ESCs to the Arduino's `Serial1` and `Serial2` ports (TX1/RX1 for ESC 1, TX2/RX2 for ESC 2) for motor control.
2. **iBus Receiver**: Connect to `Serial3` (TX3/RX3) on the Arduino to receive input from the FlySky transmitter.
3. **LED Strip**: Attach to pin `6` on the Arduino for lighting effects.
4. **MPU6050 Sensor**: Use the Arduino's I2C ports (SDA and SCL) for connection.
5. **Relay Modules**: Use digital pins `2` through `9` for auxiliary controls such as headlights, parking lights, turn signals, horn, and reverse light, enhancing the functionality and safety of your mobile bed.
6. **Power Connections**: Ensure all components are appropriately powered according to their specifications.

### Program Overview

The provided code orchestrates various functions:

- Initialization of serial ports, ESCs, iBus receiver, relays, LED strip, and I2C devices in the `setup()`.
- Continuous monitoring and processing of control inputs, telemetry data, and safety checks in the `loop()`.
- Customizable control modes (RPM, Duty, Current) for varying operational preferences.
- Relay control for auxiliary functions and LED color customization for personalization and visibility.

### Getting Started

1. Assemble the bed frame and attach the motors, ensuring all hardware is securely mounted.
2. Wire the components according to the instructions, taking care to correctly connect all electronic elements.
3. Upload the program to your Arduino Mega 2560.
4. Configure the FlySky transmitter to communicate with the receiver.
5. Conduct initial tests in a safe, controlled environment to ensure stability and operational safety.

### Customization and Safety

Customize acceleration rates, turn sensitivity, LED behavior, and more to enhance the driving experience and ensure passenger comfort. Prioritize safety by securely fastening all components and conducting thorough testing before use.

Enjoy building and operating your ElectricBed, a unique blend of technology, comfort, and mobility!
