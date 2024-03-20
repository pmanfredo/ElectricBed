# ElectricBed: Remote-Controlled Vehicle Project

Welcome to the `ElectricBed` project! This is a comprehensive guide designed to help you get started with a remote-controlled vehicle equipped with advanced features like telemetry feedback, dynamic control systems, and LED lighting, all controlled by an Arduino Mega 2560.

## Overview

This project utilizes a remote-controlled vehicle framework that includes dual Electronic Speed Controllers (ESCs), input from a FlySky FS-i6X transmitter, LED lighting effects, and a motion sensor for enhanced driving dynamics. It's engineered for hobbyists and developers looking to dive deeper into the world of RC vehicles with custom-built electronics.

### Key Features

- Dynamic speed and turn control for smooth handling.
- Telemetry feedback for battery monitoring and RPM-based adjustments.
- LED lighting control for visual effects.
- Motion detection for stability and traction control.
- Safety features including an emergency stop.

### Required Hardware

- Arduino Mega 2560
- Two Electronic Speed Controllers (ESCs)
- FlySky FS-i6X remote control transmitter and receiver
- Adafruit NeoPixel LED Strip
- MPU6050 Accelerometer and Gyro Sensor
- Relay module(s) for additional control circuits
- Power sources for the Arduino, ESCs, and LEDs

### Wiring Instructions

1. **ESC Connections**: Connect ESC 1 and ESC 2 to the Arduino's `Serial1` and `Serial2` ports respectively (TX1/RX1 for ESC 1 and TX2/RX2 for ESC 2), enabling UART communication for motor control.

2. **iBus Receiver**: Link the iBus receiver to `Serial3` (TX3/RX3) on the Arduino to read signals from the FlySky transmitter.

3. **LED Strip**: Attach the Adafruit NeoPixel LED strip to pin `6` on the Arduino for lighting control.

4. **MPU6050 Sensor**: Connect the MPU6050 accelerometer and gyro sensor via the Arduino's I2C ports (SDA and SCL).

5. **Relay Modules**: Interface the relay modules with digital pins `2` through `9` as needed for additional actuation or control features.

6. **Power Connections**: Ensure the Arduino and all peripherals are powered appropriately, considering their voltage and current requirements.

### Program Overview

The codebase is structured around several key functions, each responsible for a specific aspect of vehicle control:

- `setup()`: Initializes the serial ports, ESCs, iBus receiver, relays, LED strip, and I2C devices.
- `loop()`: Main execution loop handling signal processing, control input reading, telemetry updates, and safety checks.
- Control input processing for speed, turning, and auxiliary functions like lighting or auxiliary circuits.
- Telemetry data handling for real-time feedback on battery voltage and motor RPM.
- Utility functions for relay control, LED color setting, and motion detection.

#### Control Modes

The vehicle supports different control modes (RPM, Duty, Current) selectable through the `controlMode` variable, allowing you to tailor the driving experience to the conditions or preferences.

#### Safety Features

Safety is paramount; the program includes a system for ensuring all switches are off before operation can commence and a 2 step kill switch to prevent unwanted actions.

### Getting Started

1. Assemble the hardware according to the wiring instructions.
2. Upload the provided program to your Arduino Mega 2560.
3. Bind and configure your FlySky FS-i6X transmitter with the receiver.
4. Power on the system and perform initial testing in a safe, controlled environment.

### Customization

The program is designed for easy customization. You can adjust parameters such as acceleration rates, turn sensitivity, and LED behavior to suit your needs. Explore modifying the code to add new features or integrate additional sensors for even more interactive and responsive control.
