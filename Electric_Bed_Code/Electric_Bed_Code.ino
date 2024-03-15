#include <IBusBM.h>
#include <VescUart.h>

VescUart UART1; // VescUart object for commanding the ESC's
VescUart UART2; // VescUart object for commanding the ESC's

IBusBM IBus; // IBus object for receiving signals from transmitter/receiver

unsigned long lastSignalTime = 0; // Tracks the last time the signal was confirmed
unsigned long signalTimeout = 1000; // Signal timeout in milliseconds (1 second)
uint16_t lastCntRec = 0; // Last received count for detecting signal loss
int savespd=0, saveturn=0;

// Global variables for smooth acceleration
int currentSpeed = 127; // Assume starting at neutral
int targetSpeed = 127; // Target speed, initialized to neutral
int accelRate = 5; // Rate of acceleration towards target speed

// Advanced Turning Logic
int currentTurn = 0; // Current turn rate
int targetTurn = 0; // Target turn rate
int turnRate = 5; // Rate of turning towards target turn

// Global telemetry data variables
float batteryVoltage1 = 0.0;
float batteryVoltage2 = 0.0;
int32_t motorRPM1 = 0;
int32_t motorRPM2 = 0;

// Thresholds for functionality
// Adjust the voltage warning threshold for a 10S battery
const float voltageWarningThreshold = 3.3 * 10; // 33V for a 10S battery
const int32_t rpmDifferenceThreshold = 1000; // Example threshold for RPM difference

// Add variables for dynamic adjustment
int speedAdjustment = 0; // This will be used to adjust the speed difference between the two sets of motors


void setup() {
  /** Setup Serial port to enter commands */
  Serial.begin(9600);

  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  Serial.println("Wait for ESC 1");
  while (!Serial1) {;}
  UART1.setSerialPort(&Serial1);

  Serial.println("Wait for ESC 2");
  while (!Serial2) {;}
  UART2.setSerialPort(&Serial2);

  IBus.begin(Serial3);
  Serial.println("Wait for receiver");
  while (IBus.cnt_rec==0) delay(100);

  lastCntRec = IBus.cnt_rec; // Initialize lastCntRec with the first received count
  lastSignalTime = millis(); // Record the time when the signal was last confirmed

  Serial.println("Init done");
}

void loop() {
  // Check for signal loss
  if (IBus.cnt_rec == lastCntRec) {
    // If the count hasn't changed, check the timeout
    if (millis() - lastSignalTime > signalTimeout) {
      // Signal is considered lost, stop motors
      stopMotors(); // Implement this function to stop motors safely
      return; // Skip further processing
    }
  } else {
    // Signal is present, update the last confirmed time and count
    lastCntRec = IBus.cnt_rec;
    lastSignalTime = millis();
    updateTelemetry(); // Update telemetry data
    adjustForTractionAndStability(); // Make adjustments based on telemetry
    processControl(); // Implement this function to handle normal control processing
  }
}

void processControl() {
  // Read the target speed and direction from channels
  targetSpeed = IBus.readChannel(2); // Speed control channel
  if (IBus.readChannel(5) > 1500) {
    // Reverse: Map 1050-2000 to 127-0
    targetSpeed = map(targetSpeed, 1050, 2000, 127, 0);
  } else {
    // Forward: Map 1050-2000 to 127-255
    targetSpeed = map(targetSpeed, 1050, 2000, 127, 255);
  }

  // Smoothly adjust current speed towards target speed
  if (currentSpeed < targetSpeed) {
    currentSpeed += min(accelRate, targetSpeed - currentSpeed);
  } else if (currentSpeed > targetSpeed) {
    currentSpeed -= min(accelRate, currentSpeed - targetSpeed);
  }

  // Read and adjust turn rate based on current speed
  targetTurn = (((int)IBus.readChannel(0) - 1500) * 4) / 10;
  // Adjust turn sensitivity based on speed
  if (abs(currentSpeed - 127) > 60) { // If speed is significantly above neutral
    targetTurn = targetTurn / 2; // Reduce turning sensitivity at higher speeds
  }

  // Smoothly adjust current turn towards target turn
  if (currentTurn < targetTurn) {
    currentTurn += min(turnRate, targetTurn - currentTurn);
  } else if (currentTurn > targetTurn) {
    currentTurn -= min(turnRate, currentTurn - targetTurn);
  }

  // Apply combined speed and turn adjustments
  speedturn(currentSpeed, currentTurn);

  if (savespd != currentSpeed || saveturn != currentTurn) {
    Serial.print("speed="); Serial.print(currentSpeed);
    Serial.print(" turn="); Serial.println(currentTurn);
    savespd = currentSpeed;
    saveturn = currentTurn;
  }
}

void speedturn(int speed, int angle) {
  // Incorporate speedAdjustment to align motor speeds for straight driving
  int leftSpeed = constrain(speed + angle - speedAdjustment, 0, 255);
  int rightSpeed = constrain(speed - angle + speedAdjustment, 0, 255);

  UART1.nunchuck.valueY = leftSpeed;
  UART2.nunchuck.valueY = rightSpeed;

  UART1.setNunchuckValues();
  UART2.setNunchuckValues();
}


void OLDgetTelemetry1() {
  if (UART1.getVescValues()) {
    Serial.println(UART1.data.rpm);
    Serial.println(UART1.data.inpVoltage);
    Serial.println(UART1.data.ampHours);
    Serial.println(UART1.data.tachometerAbs);
  }
}

void OLDgetTelemetry2() {
  if (UART2.getVescValues()) {
    Serial.println(UART2.data.rpm);
    Serial.println(UART2.data.inpVoltage);
    Serial.println(UART2.data.ampHours);
    Serial.println(UART2.data.tachometerAbs);
  }
}

void stopMotors() {
  // Set motors to neutral state
  UART1.nunchuck.valueY = 127;
  UART2.nunchuck.valueY = 127;
  UART1.setNunchuckValues();
  UART2.setNunchuckValues();
  Serial.println("Signal lost, stopping motors.");
}

void updateTelemetry() {
  if (UART1.getVescValues()) {
    batteryVoltage1 = UART1.data.inpVoltage;
    motorRPM1 = UART1.data.rpm;
  }
  if (UART2.getVescValues()) {
    batteryVoltage2 = UART2.data.inpVoltage;
    motorRPM2 = UART2.data.rpm;
  }
  
  // Battery monitoring
  if ((batteryVoltage1 / 10) < voltageWarningThreshold || (batteryVoltage2 / 10) < voltageWarningThreshold) { // Adjusting for 10S battery
    Serial.println("Warning: Low Battery Voltage!");
  }
  
  // No additional logic here for RPM adjustment, we'll handle it directly in the control logic
}

void adjustMotorPowerForStraightDrive() {
  int rpmDifference = motorRPM1 - motorRPM2;

  // If we're not trying to turn (based on a minimal turn input), adjust for straight drive
  if (abs(targetTurn) < 10) { // Assuming a small threshold indicates intention to drive straight
    if (abs(rpmDifference) > rpmDifferenceThreshold) {
      speedAdjustment = rpmDifference / 100; // Calculate adjustment, tune divisor as needed for sensitivity
    } else {
      speedAdjustment = 0; // No significant difference, no adjustment needed
    }
  } else {
    // Clear adjustment during turns to allow for natural handling
    speedAdjustment = 0;
  }
  
  // Apply this adjustment in the speedturn function
}


void adjustForTractionAndStability() {
  // Example: Reduce power if RPM difference indicates potential traction loss
  int rpmDifference = abs(motorRPM1 - motorRPM2);
  if (rpmDifference > rpmDifferenceThreshold) {
    // Example adjustment: Reduce target speed to mitigate traction loss
    targetSpeed -= 5; // Adjust based on your control scheme's scaling
    Serial.println("Adjusting for traction control.");
  }
}
