# ElectricBed

## Remote-Controlled Vehicle Control Program

This program is designed to control a remote-controlled vehicle equipped with two Electronic Speed Controllers (ESCs), utilizing input from a FlySky FS-i6X remote control transmitter via the iBus protocol. It is intended for use with an Arduino Mega 2560, which communicates with the ESCs to control motor speed and direction, as well as processes telemetry data for features like battery monitoring and RPM-based adjustments for stability and traction control.

### Features

- Smooth acceleration and deceleration to prevent sudden starts or stops.
- Dynamic turning control that adjusts sensitivity based on current speed.
- Telemetry feedback for low battery warning and RPM-based traction and stability adjustments.
- Signal loss detection with automatic motor stop for safety.

### Wiring Instructions

1. **ESC Connections:**  
   ESC 1 and ESC 2 should be connected to the Arduino Mega 2560's Serial1 (pins 18 (TX1) and 19 (RX1)) and Serial2 (pins 16 (TX2) and 17 (RX2)) ports respectively. This allows for UART communication with each ESC to control motor speeds.

2. **iBus Receiver:**  
   The FlySky iBus receiver should be connected to Serial3 (pins 14 (TX3) and 15 (RX3)) of the Arduino Mega 2560. This setup is crucial for reading the control signals sent from the FlySky remote control transmitter.

3. **Power Supply:**  
   Ensure the Arduino Mega 2560 is powered adequately, either via USB or an external power source suitable for its voltage requirements. The ESCs and motors should be powered according to their specifications, typically through a separate battery to handle the higher current demands.

4. **Additional Components:**  
   Depending on the ESCs' telemetry capabilities, additional wiring may be required to enable feedback from the ESCs to the Arduino for processing.

### Usage Notes

- The program is set up to work out of the box with the specified hardware and wiring. It is important to ensure that the iBus receiver and ESCs are correctly bound and configured with the FlySky transmitter before operation.
- The vehicle's behavior (speed, acceleration rates, turning sensitivity) can be adjusted within the code to suit different terrains or driving conditions.
- Safety should be a priority; ensure all connections are secure and the vehicle is in a safe area for operation during testing and use.

This program provides a foundational framework for remote-controlled vehicle operation, with potential for further customization and development to include additional features such as GPS navigation, obstacle avoidance, and more advanced telemetry-based controls.
