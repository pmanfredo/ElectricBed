# ElectricBed: Motorized Queen-Sized Bed Project

Welcome to the `ElectricBed` project! This comprehensive guide is designed to assist you in creating a low-speed, electrically powered vehicle, uniquely constructed as a queen-sized bed that can carry two people. It's remotely controlled, featuring advanced functionalities such as telemetry feedback, dynamic control systems, and LED lighting, all managed by an Arduino Mega 2560.

**This project is an ongoing endeavor, with new features and enhancements being added and completed over time.**

## Overview

The ElectricBed project transforms the concept of mobility and rest into a single, innovative design. Equipped with dual Electronic Speed Controllers (ESCs), input from a FlySky FS-i6X transmitter, LED lighting effects, and a motion sensor for enhanced driving stability, this project invites hobbyists and developers to explore the fusion of comfort and technology.

### Key Features

- Dynamic speed and turn control for smooth maneuverability.
- Telemetry feedback for battery monitoring and RPM-based adjustments, ensuring safe and efficient operation.
- Customizable LED lighting for aesthetic appeal and visibility.
- Motion detection for improved stability and traction control, enhancing passenger comfort.
- Auxiliary controls via relays for headlights, parking lights, turn signals, horn, reverse light, and volume control.
- Comprehensive safety features such as an emergency stop mechanism and a 2-step kill switch.

### Required Hardware

- Arduino Mega 2560 for central control.
- Two Electronic Speed Controllers (ESCs) for motor management.
- FlySky FS-i6X remote control transmitter and receiver for remote operation.
- HiLetgo X9C104 Digital Potentiometer Module for volume control.
- Adafruit NeoPixel LED Strip for customizable lighting.
- MPU6050 Accelerometer and Gyro Sensor for motion detection and stability control.
- Relay module(s) for controlling auxiliary features like headlights, parking lights, turn signals, horn, and reverse light.
- Adequate power sources for the Arduino, ESCs, LEDs, and any other peripherals.

### Wiring Instructions

1. **ESC Connections**: Connect the ESCs to the Arduino's `Serial1` and `Serial2` ports (TX1/RX1 for ESC 1, TX2/RX2 for ESC 2) for motor control.
2. **iBus Receiver**: Connect to `Serial3` (TX3/RX3) on the Arduino to receive input from the FlySky transmitter.
3. **LED Strip**: Attach to pin `6` on the Arduino for lighting effects.
4. **MPU6050 Sensor**: Use the Arduino's I2C ports (SDA and SCL) for connection.
5. **Digital Potentiometer**: Connect the U/D, INC, and CS pins of the X9C104 to digital pins `10`, `11`, and `12` respectively on the Arduino.
6. **Relay Modules**: Use digital pins `2` through `9` for auxiliary controls such as headlights, parking lights, turn signals, horn, reverse light, and to enable/disable volume, enhancing the functionality and safety of your mobile bed.
7. **Power Connections**: Ensure all components are appropriately powered according to their specifications.

### Program Overview

The provided code orchestrates various functions:

- Initialization of serial ports, ESCs, iBus receiver, relays, LED strip, digital potentiometer, and I2C devices in the `setup()`.
- Continuous monitoring and processing of control inputs, telemetry data, safety checks, and volume adjustments in the `loop()`.
- Customizable control modes (RPM, Duty, Current) for varying operational preferences.
- Relay control for auxiliary functions and LED color customization for personalization and visibility.

### Volume Control Setup

- At startup, the digital potentiometer is reset to its minimum value to establish a reference point.
- Volume adjustments are made relative to this reference point, allowing for precise control over the audio output level.
- The current position of the digital potentiometer's wiper is tracked to facilitate efficient and accurate adjustments.

### Getting Started

1. Assemble the bed frame and attach the motors, ensuring all hardware is securely mounted.
2. Wire the components according to the instructions, taking care to correctly connect all electronic elements.
3. Upload the program to your Arduino Mega 2560.
4. Configure the FlySky transmitter to communicate with the receiver.
5. Conduct initial tests in a safe, controlled environment to ensure stability and operational safety.

### Customization and Safety

Customize acceleration rates, turn sensitivity, LED behavior, and more to enhance the driving experience and ensure passenger comfort. Prioritize safety by securely fastening all components and conducting thorough testing before use.

Enjoy building and operating your ElectricBed, a unique blend of technology, comfort, and mobility!

## Upcoming Features

We're continuously working to enhance the ElectricBed experience with innovative features and improvements. Here are some of the exciting updates you can look forward to as we get them completed:

### Sound System with Remote Volume Control

Enjoy an immersive audio experience with a fully integrated sound system. Future updates will include the ability to control the volume directly from your FlySky FS-i6X remote, allowing for seamless adjustments without the need to stop or reach for a separate controller. Whether you're looking to set the mood with some background music or amp up the energy with your favorite tracks, having control at your fingertips will enhance your ElectricBed experience.

### Electric Bed App for Monitoring and Security Enhancements

Stay connected with your ElectricBed through our upcoming dedicated app. This application will not only allow you to monitor key vehicle metrics in real-time, such as battery levels, speed, and system diagnostics, but it will also introduce new layers of security and convenience:

- **Remote Monitoring**: Check on your ElectricBed's status, ensuring it's always ready for your next adventure.
- **Security Alerts**: Receive instant notifications for unauthorized movements or tampering, offering peace of mind when you're away from your vehicle *when it is on*.
- **Future Updates**: The app will serve as a platform for rolling out future features and enhancements, keeping your ElectricBed at the cutting edge of technology.

Stay tuned for these updates and more as we continue to innovate and expand the capabilities of the ElectricBed project. 

**If you have any features you would like to see added, please make a pull request or add to our discussion!**
