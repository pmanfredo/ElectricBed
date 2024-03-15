/**
 * Remote-Controlled Vehicle Control Program
 * ==========================================
 * 
 * This program is designed to control a remote-controlled vehicle equipped with two Electronic Speed Controllers (ESCs),
 * utilizing input from a FlySky FS-i6X remote control transmitter via the iBus protocol. It is intended for use with an Arduino Mega 2560,
 * which communicates with the ESCs to control motor speed and direction, as well as processes telemetry data for features like battery monitoring
 * and RPM-based adjustments for stability and traction control.
 * 
 * Features:
 * ---------
 * - Smooth acceleration and deceleration to prevent sudden starts or stops.
 * - Dynamic turning control that adjusts sensitivity based on current speed.
 * - Telemetry feedback for low battery warning and RPM-based traction and stability adjustments.
 * - Signal loss detection with automatic motor stop for safety.
 * 
 * Wiring Instructions:
 * --------------------
 * 1. ESC Connections:
 *    - ESC 1 and ESC 2 should be connected to the Arduino Mega 2560's Serial1 (pins 18 (TX1) and 19 (RX1)) and Serial2 (pins 16 (TX2) and 17 (RX2)) ports respectively.
 *      This allows for UART communication with each ESC to control motor speeds.
 * 
 * 2. iBus Receiver:
 *    - The FlySky iBus receiver should be connected to Serial3 (pins 14 (TX3) and 15 (RX3)) of the Arduino Mega 2560.
 *      This setup is crucial for reading the control signals sent from the FlySky remote control transmitter.
 * 
 * 3. Power Supply:
 *    - Ensure the Arduino Mega 2560 is powered adequately, either via USB or an external power source suitable for its voltage requirements.
 *    - The ESCs and motors should be powered according to their specifications, typically through a separate battery to handle the higher current demands.
 * 
 * 4. Additional Components:
 *    - Depending on the ESCs' telemetry capabilities, additional wiring may be required to enable feedback from the ESCs to the Arduino for processing.
 * 
 * Usage Notes:
 * ------------
 * - The program is set up to work out of the box with the specified hardware and wiring. It is important to ensure that the iBus receiver and ESCs
 *   are correctly bound and configured with the FlySky transmitter before operation.
 * - The vehicle's behavior (speed, acceleration rates, turning sensitivity) can be adjusted within the code to suit different terrains or driving conditions.
 * - Safety should be a priority; ensure all connections are secure and the vehicle is in a safe area for operation during testing and use.
 * 
 * This program provides a foundational framework for remote-controlled vehicle operation, with potential for further customization and development
 * to include additional features such as GPS navigation, obstacle avoidance, and more advanced telemetry-based controls.
 */

#include <IBusBM.h>
#include <VescUart.h>

VescUart UART1; // VescUart object for commanding the ESC's
VescUart UART2; // VescUart object for commanding the ESC's
IBusBM IBus; // IBus object for receiving signals from transmitter/receiver

unsigned long lastSignalTime = 0; // Tracks the last time the signal was confirmed
const unsigned long signalTimeout = 1000; // Signal timeout in milliseconds
uint16_t lastCntRec = 0; // Last received count for detecting signal loss

// Control variables
int currentSpeed = 127, targetSpeed = 127, currentTurn = 0, targetTurn = 0;
int accelRate = 5, turnRate = 5; // Rate of change for speed and turn

// Telemetry variables
float batteryVoltage1 = 0.0, batteryVoltage2 = 0.0;
int32_t motorRPM1 = 0, motorRPM2 = 0;
const float voltageWarningThreshold = 33.0; // Threshold for 10S battery
const int32_t rpmDifferenceThreshold = 1000; // Threshold for RPM difference
int speedAdjustment = 0; // For adjusting speed difference between two sets of motors

// ibus Channel Assignments
#define RJoyX 1
#define RJoyY 2
#define LJoyY 3
#define LJoyX 4
#define LPot 5
#define RPot 6
#define SWA 7
#define SWB 8
#define SWC 9
#define SWD 10

// Setup function initializes the serial communication ports and prepares the system for operation.
void setup() {
  Serial.begin(9600); // Initializes the Serial port for logging to the Arduino IDE Serial Monitor.
  Serial1.begin(115200); // Initializes Serial1 for communication with the first ESC.
  Serial2.begin(115200); // Initializes Serial2 for communication with the second ESC.
  Serial3.begin(115200); // Initializes Serial3 for communication with the iBus receiver.

  setupESCs(); // Calls the function to setup and initialize the ESCs.
  IBus.begin(Serial3); // Initializes the iBus communication on Serial3.
  waitForReceiver(); // Waits for the receiver to start sending signals before proceeding.
}

// Main loop function checks for signal loss and processes control inputs if signal is present.
void loop() {
  checkSignalLoss(); // Checks if the signal has been lost and stops motors if so.
  if (IBus.cnt_rec != lastCntRec) { // Checks if new iBus messages have been received since the last loop iteration.
    updateTelemetry(); // Updates telemetry data from the ESCs for features like battery monitoring.
    adjustForStraightDrive(); // Adjusts motor power to maintain straight driving based on RPM differences.
    processControlInputs(); // Reads control inputs from the receiver and adjusts motor speeds accordingly.
    lastCntRec = IBus.cnt_rec; // Updates the last iBus message count for the next loop iteration.
    lastSignalTime = millis(); // Updates the last signal time to the current time.
  }
}


// Function to initialize the ESCs (Electronic Speed Controllers) by setting their respective serial ports.
void setupESCs() {
  Serial.println("Initializing ESCs..."); // Log message indicating the start of ESC initialization
  UART1.setSerialPort(&Serial1); // Assigns Serial1 to UART1, linking the ESC to the first serial port
  UART2.setSerialPort(&Serial2); // Assigns Serial2 to UART2, linking the second ESC to another serial port
  Serial.println("ESCs Initialized."); // Log message indicating the ESCs have been successfully initialized
}

// Function to wait for the receiver to start sending data. This ensures the program doesn't proceed without a signal.
void waitForReceiver() {
  Serial.println("Waiting for receiver..."); // Log message indicating the program is waiting for receiver data
  while (IBus.cnt_rec == 0) delay(100); // Waits in a loop, delaying 100ms at a time until data is received from the receiver
  lastCntRec = IBus.cnt_rec; // Stores the count of received messages for later comparison
  lastSignalTime = millis(); // Stores the current time as the last time a signal was received
  Serial.println("Receiver connected."); // Log message indicating a connection to the receiver has been established
}

// Function to check for signal loss and stop the motors if the signal isn't received within the specified timeout.
void checkSignalLoss() {
  if (millis() - lastSignalTime > signalTimeout) { // Checks if the current time minus the last signal time exceeds the timeout
    stopMotors(); // Calls the function to stop the motors due to signal loss
    while (millis() - lastSignalTime > signalTimeout) { // Continues to check for signal loss in a loop
      if (IBus.cnt_rec != lastCntRec) { // If a new signal (message count change) is detected
        lastCntRec = IBus.cnt_rec; // Update the last received message count
        lastSignalTime = millis(); // Update the last signal time to the current time
        Serial.println("Signal regained."); // Log message indicating the signal has been regained
        break; // Breaks out of the loop since signal has been regained
      }
    }
  }
}


// Processes control inputs by reading inputs, making smooth adjustments, and then applying these adjustments to speed and turn.
void processControlInputs() {
  readInputs(); // Reads the control inputs from the receiver
  smoothAdjustments(); // Applies smooth adjustments to speed and turn to avoid abrupt changes
  speedturn(currentSpeed, currentTurn); // Applies the adjusted speed and turn values to the motors
}

// Reads inputs from the remote control receiver and maps them to target speed and turn values.
void readInputs() {
  targetSpeed = IBus.readChannel(LJoyY); // Reads the speed input from Left Joystick Y Axis of the receiver
  // Checks if the SWA input is above 1500 (typically the threshold for forward/reverse switch)
  if (IBus.readChannel(SWA) > 1500) {
    // If true, maps the speed value for reverse direction
    targetSpeed = map(targetSpeed, 1050, 2000, 127, 0); // Maps reverse speed
  } else {
    // Otherwise, maps the speed value for forward direction
    targetSpeed = map(targetSpeed, 1050, 2000, 127, 255); // Maps forward speed
  }
  // Reads the turn input from Right Joystick X Axis and maps it to a target turn value
  targetTurn = (((int)IBus.readChannel(RJoyX) - 1500) * 4) / 10; // Maps turn value based on Right Joystick X Axis input
}

// Applies smooth adjustments to the current speed and turn values to reach the target values without abrupt changes.
void smoothAdjustments() {
  // Adjusts the current speed towards the target speed at a controlled rate (accelRate)
  if (currentSpeed < targetSpeed) currentSpeed += min(accelRate, targetSpeed - currentSpeed); // Increases speed if below target
  else if (currentSpeed > targetSpeed) currentSpeed -= min(accelRate, currentSpeed - targetSpeed); // Decreases speed if above target

  // Adjusts the current turn rate towards the target turn rate at a controlled rate (turnRate)
  if (currentTurn < targetTurn) currentTurn += min(turnRate, targetTurn - currentTurn); // Increases turn rate if below target
  else if (currentTurn > targetTurn) currentTurn -= min(turnRate, currentTurn - targetTurn); // Decreases turn rate if above target
}


// Adjusts and applies the calculated speed and angle to the motors for movement and turning.
void speedturn(int speed, int angle) {
  // Calculate the adjusted speed for each side, incorporating the angle for turning and any speed adjustments for straight driving.
  int leftSpeed = constrain(speed + angle - speedAdjustment, 0, 255); // Adjust left motor speed
  int rightSpeed = constrain(speed - angle + speedAdjustment, 0, 255); // Adjust right motor speed

  // Set the calculated speeds to the ESCs through the VESC UART interface
  UART1.nunchuck.valueY = leftSpeed; // Apply left speed adjustment
  UART2.nunchuck.valueY = rightSpeed; // Apply right speed adjustment

  // Send the updated values to the ESCs to control the motors
  UART1.setNunchuckValues();
  UART2.setNunchuckValues();
}

// Updates telemetry data from the ESCs, including battery voltage and motor RPM, and checks for low battery voltage.
void updateTelemetry() {
  // Retrieve and store telemetry data for each ESC
  if (UART1.getVescValues()) {
    batteryVoltage1 = UART1.data.inpVoltage; // Store battery voltage from ESC 1
    motorRPM1 = UART1.data.rpm; // Store motor RPM from ESC 1
  }
  if (UART2.getVescValues()) {
    batteryVoltage2 = UART2.data.inpVoltage; // Store battery voltage from ESC 2
    motorRPM2 = UART2.data.rpm; // Store motor RPM from ESC 2
  }

  // Check if either battery voltage is below the threshold and warn if so
  if (batteryVoltage1 < voltageWarningThreshold || batteryVoltage2 < voltageWarningThreshold) {
    Serial.println("Warning: Low Battery Voltage!"); // Print a low battery warning
  }
}

// Adjusts the speed adjustment factor to help the vehicle drive straight by correcting for differences in motor RPM.
void adjustForStraightDrive() {
  int rpmDifference = motorRPM1 - motorRPM2; // Calculate the difference in RPM between the two motors
  // If the RPM difference exceeds the threshold, calculate a speed adjustment to compensate
  if (abs(rpmDifference) > rpmDifferenceThreshold) {
    speedAdjustment = rpmDifference / 100; // Determine adjustment factor based on RPM difference
  } else {
    speedAdjustment = 0; // Reset adjustment factor if RPM difference is within the threshold
  }
}

// Stops the motors by setting their speed values to neutral, used in cases of signal loss or as part of a shutdown procedure.
void stopMotors() {
  // Set motors to a neutral state (idle) to safely stop the vehicle
  UART1.nunchuck.valueY = 127; // Neutral speed for left motor
  UART2.nunchuck.valueY = 127; // Neutral speed for right motor

  // Send the neutral speed values to the ESCs to stop the motors
  UART1.setNunchuckValues();
  UART2.setNunchuckValues();

  Serial.println("Motors stopped due to signal loss."); // Log message indicating motors have been stopped
}