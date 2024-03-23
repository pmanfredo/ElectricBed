#include <IBusBM.h>
#include <VescUart.h>
#include <Adafruit_NeoPixel.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

VescUart UART1; // VescUart object for commanding the ESC's
VescUart UART2; // VescUart object for commanding the ESC's
IBusBM IBus;    // IBus object for receiving signals from transmitter/receiver

unsigned long lastSignalTime = 0;         // Tracks the last time the signal was confirmed
const unsigned long signalTimeout = 1000; // Signal timeout in milliseconds
uint16_t lastCntRec = 0;                  // Last received count for detecting signal loss

// Control variables
int currentSpeed = 0, targetSpeed = 0, currentTurn = 0, targetTurn = 0;
int accelRate = 50, turnRate = 100; // Rate of change for speed and turn

// Telemetry variables
float batteryVoltage1 = 0.0, batteryVoltage2 = 0.0;
const float voltageWarningThreshold = 33.0;  // Threshold for 10S battery

bool movementEnabled = true; // Global variable to track movement enable state
bool reverse = false;

enum ControlMode { MODE_RPM, MODE_DUTY, MODE_CURRENT };
ControlMode controlMode = MODE_RPM; // Choose control mode: MODE_RPM, MODE_DUTY, or MODE_CURRENT

// ibus Channel Assignments
#define RJoyX 0
#define RJoyY 1
#define LJoyY 2
#define LJoyX 3
#define LPot 4
#define RPot 5
#define SWA 6
#define SWB 7
#define SWC 8
#define SWD 9
#define ON 1000
#define OFF 2000
#define MID 1500

bool startup = true;
bool dangerousTemp = false;
bool dangerousCurrent = false;
bool dangerousDuty = false;

// Temperature thresholds (degrees Celsius)
const float MOSFET_SAFE_TEMP = 80.0; // Maximum safe temperature for the ESC's MOSFETs
const float MOTOR_SAFE_TEMP = 80.0;  // Maximum safe temperature for the motor

// Current limits (Amps)
const float MOTOR_CURRENT_LIMIT = 50.0; // Maximum safe current for the motor

// Duty cycle limit (percentage)
const float DUTY_CYCLE_LIMIT = 95.0; // Maximum safe duty cycle percentage

// Define relay channels to Arduino pin numbers
const int relay1 = 2;
const int relay2 = 3;
const int relay3 = 4;
const int relay4 = 5;
const int relay5 = 6;
const int relay6 = 7;
const int relay7 = 8;
const int relay8 = 9;

// For tracking left joystick x-axis double-tap
unsigned long leftJoystickFirstHitTime = 0;
const unsigned long doubleTapMaxInterval = 500; // Max interval in ms for double-tap detection
bool leftJoystickInRangeLastTime = false;
int leftJoystickHitCount = 0;

const unsigned long hornDuration = 500; // Duration of the horn beep in milliseconds

// Define the pin that the LED strip is connected to
#define LED_PIN 6

// Define the number of LEDs in the strip
#define LED_COUNT 600

unsigned long previousMillis = 0; // will store last time LED was updated
const long interval = 0; // interval at which to blink (milliseconds)
unsigned int i = 0;

// Create an Adafruit_NeoPixel object
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

MPU6050 accelgyro;
const int quickTurnThreshold = 100; // Example threshold, adjust based on testing

// Digipot control pins
const int pinUD = 10; // Up/Down pin
const int pinINC = 11; // Increment pin
const int pinCS = 12; // Chip Select pin

int currentWiperPosition = 0; // Current wiper position, starting at 0 (minimum)

// Setup function initializes the serial communication ports and prepares the system for operation.
void setup(){
    initializeSerial();
    setupESCs();
    IBus.begin(Serial3);
    waitForReceiver();
    //initializeRelays();
    initializeLEDStrip();
    //initializeI2CDevices();
}

// Main loop function checks for signal loss and processes control inputs if signal is present.
void loop(){
  //serialInput();
  //printChannelValues();
  if (IBus.cnt_rec != lastCntRec)
  {                            // Checks if new iBus messages have been received since the last loop iteration.
    updateTelemetry();         // Updates telemetry data from the ESCs for features like battery monitoring.
    processControlInputs();    // Reads control inputs from the receiver and adjusts motor speeds accordingly.
    lastCntRec = IBus.cnt_rec; // Updates the last iBus message count for the next loop iteration.
    lastSignalTime = millis(); // Updates the last signal time to the current time.
  }
  //delay(2000);
}

// Function to initialize the ESCs (Electronic Speed Controllers) by setting their respective serial ports.
void setupESCs(){
  Serial.println("Initializing ESCs..."); // Log message indicating the start of ESC initialization
  UART1.setSerialPort(&Serial1);          // Assigns Serial1 to UART1, linking the ESC to the first serial port
  UART2.setSerialPort(&Serial2);          // Assigns Serial2 to UART2, linking the second ESC to another serial port
  Serial.println("ESCs Initialized.");    // Log message indicating the ESCs have been successfully initialized
}

// Function to wait for the receiver to start sending data. This ensures the program doesn't proceed without a signal.
void waitForReceiver(){
  Serial.println("Waiting for receiver..."); // Log message indicating the program is waiting for receiver data
  while (IBus.cnt_rec == 0)
    delay(100);                          // Waits in a loop, delaying 100ms at a time until data is received from the receiver
  lastCntRec = IBus.cnt_rec;             // Stores the count of received messages for later comparison
  lastSignalTime = millis();             // Stores the current time as the last time a signal was received
  Serial.println("Receiver connected."); // Log message indicating a connection to the receiver has been established
}

void initializeSerial() {
    Serial.begin(9600);
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
}

void initializeRelays() {
    const int relays[] = {relay1, relay2, relay3, relay4, relay5, relay6, relay7, relay8};
    for (int relay : relays) {
        pinMode(relay, OUTPUT);
        digitalWrite(relay, HIGH); // Assuming HIGH means OFF
    }
}

void initializeLEDStrip() {
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}

void initializeI2CDevices() {
    Wire.begin();
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void setupDigipot() {
  pinMode(pinUD, OUTPUT);
  pinMode(pinINC, OUTPUT);
  pinMode(pinCS, OUTPUT);

  digitalWrite(pinCS, HIGH); // Deselect the device by default

  resetDigipotToMinimum();
}

void resetDigipotToMinimum() {
    // Select the digipot
    digitalWrite(pinCS, LOW);

    // Set to decrease wiper position
    digitalWrite(pinUD, LOW);
    for (int i = 0; i < 100; i++) { // Assuming 100 steps for the digipot
        // Pulse the INC pin
        digitalWrite(pinINC, HIGH);
        delay(1); // Short delay for the signal to register
        digitalWrite(pinINC, LOW);
        delay(1);
    }

    // Deselect the digipot
    digitalWrite(pinCS, HIGH);
}

void processControlInputs() {
  readInputs();
  checkTurnSignals();
  checkHornControl();
  checkHeadlights();
  checkHazardLights();
  controlVolumeAndMute();
  controlLEDScenes();

  // Applies smooth adjustments to speed and turn to avoid abrupt changes
  smoothAdjustments();
  
  //Turn system OFF or set current to 0 if input is 1000
  if (targetSpeed <= 1000) {
      UART1.setCurrent(0);
      UART1.setCurrent(0,101);
      UART2.setCurrent(0);
      UART2.setCurrent(0,101);
      //Serial.println("System turned off - Current set to 0.");
      currentSpeed = 1000;
      return;
  }

  // Depending on the control mode, map targetSpeed to the respective range and apply control
  switch (controlMode) {
      case MODE_RPM:
          // Map targetSpeed (1001-2000) to RPM range (0-7000), considering 1000 as OFF
          int rpmValue = map(currentSpeed, 1001, 2000, 1000, 7000);

          if (dangerousTemp || dangerousCurrent || dangerousDuty){
            rpmValue = rpmValue / 2;
          }

          if(reverse){
            rpmValue = -rpmValue;
          }
          applyControlRPM(rpmValue);
          break;
      case MODE_DUTY:
          // Map targetSpeed (1001-2000) to duty cycle range (0-80%)
          float dutyCycle = map(targetSpeed, 1001, 2000, 0, 80);

          if (dangerousTemp || dangerousCurrent || dangerousDuty){
            dutyCycle = dutyCycle / 2;
          }

          if(reverse){
            dutyCycle = -dutyCycle;
          }
          applyControlDuty(dutyCycle);
          break;
      case MODE_CURRENT:
          // Map targetSpeed (1001-2000) to current range (0-40A)
          float current = map(targetSpeed, 1001, 2000, 0, 40);

          if (dangerousTemp || dangerousCurrent || dangerousDuty){
            current = current / 2;
          }

          if(reverse){
            current = -current;
          }
          applyControlCurrent(current);
          break;
  }
}

void checkTurnSignals(){
  // Handling Left Joystick X-Axis for Turn Signals
  int leftJoystickX = IBus.readChannel(LJoyX);
  unsigned long currentMillis = millis();

  // Reset hit count if interval exceeds max double-tap interval
  if (currentMillis - leftJoystickFirstHitTime > doubleTapMaxInterval && leftJoystickHitCount > 0) {
      leftJoystickHitCount = 0;
  }

  if ((leftJoystickX >= 1000 && leftJoystickX <= 1100) || (leftJoystickX >= 1900 && leftJoystickX <= 2000)) {
    if (!leftJoystickInRangeLastTime) {
        leftJoystickInRangeLastTime = true;
        leftJoystickHitCount++;
        leftJoystickFirstHitTime = currentMillis;

        // If first hit, deactivate both relays
        if (leftJoystickHitCount == 1) {
            turnOffRelay(relay1); // Turn off Relay 2
            turnOffRelay(relay2); // Turn off Relay 3
        }

        // If second hit and within interval, activate the appropriate relay and ensure the other is off
        if (leftJoystickHitCount == 2) {
            if (leftJoystickX >= 1000 && leftJoystickX <= 1100) {
                turnOffRelay(relay2); // Ensure Relay 3 is off
                turnOnRelay(relay1); // Turn on Relay 2 for left turn signal
            } else if (leftJoystickX >= 1900 && leftJoystickX <= 2000) {
                turnOffRelay(relay1); // Ensure Relay 2 is off
                turnOnRelay(relay2); // Turn on Relay 3 for right turn signal
            }
            leftJoystickHitCount = 0; // Reset count after activation
        }
    }
  } else {
      leftJoystickInRangeLastTime = false;
    }

}

void checkHornControl() {
    static bool hornBeeped = false; // Tracks if a short beep was already made
    static unsigned long hornStartTime = 0; // Tracks when the horn beep started
    int rightJoystickY = IBus.readChannel(RJoyY); // Reading the Y-axis of the right joystick

    if (rightJoystickY >= 1900 && rightJoystickY <= 2000) {
        turnOnRelay(relay3); // Continuous horn - Joystick pushed upwards
        hornBeeped = false; // Reset beeped flag for next operation
    } else if (rightJoystickY >= 1000 && rightJoystickY <= 1100 && !hornBeeped) {
        // Short beep - Joystick pushed downwards
        if (hornStartTime == 0) { // If the beep hasn't started yet
            turnOnRelay(relay3); // Turn on relay4 for the horn
            hornStartTime = millis(); // Mark the start time of the beep
        }
        if (millis() - hornStartTime > hornDuration) { // If the beep duration has passed
            turnOffRelay(relay3); // Turn off relay4 after the short beep duration
            hornBeeped = true; // Set flag to true to avoid repeating the beep
            hornStartTime = 0; // Reset start time for next beep
        }
    } else if (rightJoystickY > 1100 && rightJoystickY < 1900) {
        // Resets the horn beeped flag when the joystick is neither fully up nor down
        hornBeeped = false;
        hornStartTime = 0; // Reset start time for next beep
        turnOffRelay(relay3); // Ensure the relay is off when not in the beep or continuous horn zones
    }
}

void checkHeadlights() {
    int swcValue = IBus.readChannel(SWC); // Reading the value of the SWC switch

    if (swcValue == OFF) {
        // If SWC is 1000, both relay5 and relay6 are off
        turnOffRelay(relay4);
        turnOffRelay(relay5);
    } else if (swcValue == MID) {
        // If SWC is 1500, relay5 is on and relay6 is off
        turnOnRelay(relay4);
        turnOffRelay(relay5);
    } else if (swcValue == ON) {
        // If SWC is 2000, both relay5 and relay6 are on
        turnOnRelay(relay4);
        turnOnRelay(relay5);
    }
}

void checkHazardLights() {
    int swdValue = IBus.readChannel(SWD); // Reading the value of the SWD switch

    if (swdValue == OFF) {
        // If SWD is 1000, relay7 is off
        turnOffRelay(relay6);
    } else if (swdValue == OFF) {
        // If SWD is 2000, relay7 is on
        turnOnRelay(relay6);
    }
}

// Reads inputs from the remote control receiver and maps them to target speed and turn values.
void readInputs(){
    // Step 1: Ensure all switches are OFF before proceeding
  while (startup) {
    int swaState = IBus.readChannel(SWA);
    int swbState = IBus.readChannel(SWB);
    int swcState = IBus.readChannel(SWC);
    int swdState = IBus.readChannel(SWD);

    if (swaState == OFF && swbState == OFF && swcState == OFF && swdState == OFF) {
      startup = false;
      break; // Exit the loop if all switches are OFF
    }

    Serial.println("Ensure all switches are OFF to proceed.");
    delay(500); // Delay to prevent flooding the serial output
  }

  // Step 2: Proceed with kill switch logic, incorporating the SWB safety feature
  static bool swbSafetyCheckPassed = false; // Static variable to track if the SWB safety check has passed

  while (true)
  {
    int killState = IBus.readChannel(SWA);
    
    // Reset SWB safety check if kill switch is turned OFF
    if (killState == OFF) {
      swbSafetyCheckPassed = false;
      stopMotors();
      movementEnabled = false;
      currentSpeed = 1000;
      Serial.println("Movement Disabled. Kill switch is OFF. SWB safety check reset.");
    }

    // Conduct SWB safety check only if it hasn't been passed yet
    if (!swbSafetyCheckPassed) {
      int swbState = IBus.readChannel(SWB);
      if (killState == ON && swbState == ON) {
        swbSafetyCheckPassed = true;
        Serial.println("SWB safety check passed. System starting in forward.");
      } else {
        Serial.println("Waiting for SWB to be turned ON for safety check.");
        delay(500); // Delay to prevent flooding the serial output
        continue; // Remain in loop until SWB is ON and kill switch is ON
      }
    }

    // Exit loop and allow system to proceed if killState is ON and SWB safety check has been passed
    if (killState == ON && swbSafetyCheckPassed) {
      if (!movementEnabled) {
        movementEnabled = true;
        Serial.println("Movement Enabled. Kill switch is ON.");
      }
      break; // Exit the loop if conditions are met
    }
    
    delay(100); // Brief delay to reduce CPU load during loop
  }

  targetSpeed = IBus.readChannel(LJoyY); // Reads the speed input from Left Joystick Y Axis of the receiver

  int reverseValue = IBus.readChannel(SWB);

  if (reverseValue == OFF && targetSpeed == 1000)
  {
    // If true, maps the speed value for reverse direction
    reverse = true;
  }

  if (reverseValue == ON && targetSpeed == 1000){
    reverse = false;
  }

  // Reads the turn input from Right Joystick X Axis and maps it to a target turn value
  targetTurn = (int)(IBus.readChannel(RJoyX) -1500); // 1000 (left) to 2000 (right), 1500 is neutral, turn -500 to 500
}

// Applies smooth adjustments to the current speed and turn values to reach the target values without abrupt changes.
void smoothAdjustments(){
  checkAndAdjustForQuickTurns();

  // Adjusts the current speed towards the target speed at a controlled rate (accelRate)
  if (currentSpeed < targetSpeed)
    currentSpeed += min(accelRate, targetSpeed - currentSpeed); // Increases speed if below target
  else if (currentSpeed > targetSpeed)
    currentSpeed -= min(accelRate, currentSpeed - targetSpeed); // Decreases speed if above target

  // Adjusts the current turn rate towards the target turn rate at a controlled rate (turnRate)
  if (currentTurn < targetTurn)
    currentTurn += min(turnRate, targetTurn - currentTurn); // Increases turn rate if below target
  else if (currentTurn > targetTurn)
    currentTurn -= min(turnRate, currentTurn - targetTurn); // Decreases turn rate if above target

  //Serial.println(currentSpeed);
}

void checkAndAdjustForQuickTurns() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Assuming gz (rotation around the Z-axis) is your turning axis
    // The sensitivity scale factor for the gyroscope is 131 LSB/degrees/sec
    // according to the MPU-6050 datasheet when using the default sensitivity setting.
    float turnRate = gz / 131.0;

    if (abs(turnRate) > quickTurnThreshold) {
        // Quick turn detected, reduce target speed
        Serial.println("Quick turn detected, reducing speed.");
        targetSpeed = targetSpeed / 2; // Example reduction, adjust as needed
    }
}

void applyControlRPM(int rpm) {
  //Serial.print("Applying RPM");
  // Calculate left and right motor RPM based on target turn and speed adjustments
  int leftRPM = constrain(rpm + (2 * currentTurn), -7000, 7000);
  int rightRPM = constrain(rpm - (2 * currentTurn), -7000, 7000);
  UART1.setRPM(leftRPM);
  UART2.setRPM(leftRPM);
  UART1.setRPM(rightRPM,101);
  UART2.setRPM(rightRPM,101);
  // Serial.print("Left RPM set to: "); Serial.println(leftRPM);
  // Serial.print("Right RPM set to: "); Serial.println(rightRPM);
  // delay(500);
}

void applyControlDuty(float dutyCycle) {
  // Apply duty cycle control to motors
  int leftDutyCycle = constrain(dutyCycle + (0.05 * currentTurn), 0, 80);
  int rightDutyCycle = constrain(dutyCycle - (0.05 * currentTurn), 0, 80);
  UART1.setDuty(leftDutyCycle);

  UART2.setDuty(leftDutyCycle);
  UART1.setDuty(rightDutyCycle, 101);
  UART2.setDuty(rightDutyCycle, 101);
  Serial.print("Left Duty Cycle set to: "); Serial.println(leftDutyCycle);
}

void applyControlCurrent(float current) {
  // Apply current control to motors, adjustments for turning not demonstrated
  int leftCurrent = constrain(current + (0.025 * currentTurn), 0, 40);
  int rightCurrent = constrain(current - (0.025 * currentTurn), 0, 40);
  UART1.setCurrent(leftCurrent);
  UART2.setCurrent(leftCurrent);
  UART1.setCurrent(rightCurrent,101);
  UART2.setCurrent(rightCurrent,101);
  Serial.print("Current set to: "); Serial.println(current);
}

// Updates telemetry data from the ESCs, including battery voltage and motor RPM, and checks for low battery voltage.
void updateTelemetry(){
  // Retrieve and store telemetry data for each ESC
  if (UART1.getVescValues())
  {
    batteryVoltage1 = UART1.data.inpVoltage; // Store battery voltage from ESC 1

    // Serial.print("Battery Voltage: ");
    // Serial.println(batteryVoltage1);
  }
  if (UART2.getVescValues())
  {
    batteryVoltage2 = UART2.data.inpVoltage; // Store battery voltage from ESC 2

    // Serial.print("Battery Voltage: ");
    // Serial.println(batteryVoltage2);
  }

  // Check if either battery voltage is below the threshold and warn if so
  if (batteryVoltage1 < voltageWarningThreshold || batteryVoltage2 < voltageWarningThreshold)
  {
    //Serial.println("Warning: Low Battery Voltage!"); // Print a low battery warning
  }

  checkSystemHealth();
}

void checkSystemHealth() {
  if (UART1.data.tempMosfet > MOSFET_SAFE_TEMP || UART1.data.tempMotor > MOTOR_SAFE_TEMP || UART2.data.tempMosfet > MOSFET_SAFE_TEMP || UART2.data.tempMotor > MOTOR_SAFE_TEMP) {
    Serial.println("High temperature detected. Reducing performance.");
    dangerousTemp = true;
  } else {
    dangerousTemp  = false;
  }

  if (UART1.data.avgMotorCurrent > MOTOR_CURRENT_LIMIT || UART2.data.avgMotorCurrent > MOTOR_CURRENT_LIMIT) {
    Serial.println("High motor current detected. Adjusting control.");
    // Adjust control logic here
    dangerousCurrent = true;
  } else {
    dangerousCurrent  = false;
  }

  if (UART1.data.dutyCycleNow > DUTY_CYCLE_LIMIT || UART2.data.dutyCycleNow > DUTY_CYCLE_LIMIT) {
    Serial.println("High duty cycle detected. Adjusting control.");
    // Reduce demanded RPM or switch control mode
    dangerousDuty = true;
  } else {
    dangerousDuty  = false;
  }
}

// Stops the motors by setting their speed values to neutral, used in cases of signal loss or as part of a shutdown procedure.
void stopMotors(){
  UART1.setBrakeCurrent(5);
  UART1.setBrakeCurrent(5,101);
  UART2.setBrakeCurrent(5);
  UART2.setBrakeCurrent(5,101);
  Serial.println("Braking!");
}

void serialInput(){
  if (Serial.available() > 0) {
    // Read the incoming data as a string until newline is encountered
    String dataString = Serial.readStringUntil('\n');
    // Convert the string to an integer
    int incomingInt = dataString.toInt();
    if (incomingInt == -1) {
      currentSpeed = 0;
      targetSpeed = 0;
      UART1.setBrakeCurrent(5);
      UART1.setCurrent(0,0);
      UART1.setCurrent(0,101);
      Serial.println("Braking!");
    }else{
      targetSpeed = incomingInt;
      Serial.print("Received targetSpeed: ");
      Serial.println(targetSpeed);
    }


  }
}

void turnOnRelay(int relayPin) {
  digitalWrite(relayPin, LOW); // Turn relay on
}

void turnOffRelay(int relayPin) {
  digitalWrite(relayPin, HIGH); // Turn relay off
}

void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

void fillStrip(uint8_t red, uint8_t green, uint8_t blue) {
  for(int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
  }
  strip.show();
}

void adjustVolume(int desiredLevel) {
  if (desiredLevel < 0) desiredLevel = 0;
  if (desiredLevel > 99) desiredLevel = 99; // Ensure desired level is within bounds (0-99 for X9C104)

  // Select the digipot
  digitalWrite(pinCS, LOW);

  if (currentWiperPosition < desiredLevel) {
    digitalWrite(pinUD, HIGH); // Set direction to increase wiper position
    for (int i = currentWiperPosition; i < desiredLevel; i++) {
      // Increment the potentiometer to the desired level
      digitalWrite(pinINC, HIGH);
      delay(1); // Short delay for the signal to register
      digitalWrite(pinINC, LOW);
      delay(1);
    }
  } else if (currentWiperPosition > desiredLevel) {
    digitalWrite(pinUD, LOW); // Set direction to decrease wiper position
    for (int i = currentWiperPosition; i > desiredLevel; i--) {
      // Decrement the potentiometer to the desired level
      digitalWrite(pinINC, HIGH);
      delay(1); // Short delay for the signal to register
      digitalWrite(pinINC, LOW);
      delay(1);
    }
  }

  // Update the current wiper position
  currentWiperPosition = desiredLevel;

  // Deselect the digipot
  digitalWrite(pinCS, HIGH);
}

void controlVolumeAndMute() {
  int lpotValue = IBus.readChannel(LPot);
  if (lpotValue == 1000) {
    turnOffRelay(relay7); // Mute
  } else {
    turnOnRelay(relay7); // Unmute
    // Map the lpotValue (1000-2000) to a desired volume level (e.g., 0-100)
    int volumeLevel = map(lpotValue, 1001, 2000, 0, 100);
    adjustVolume(volumeLevel); // Adjust the volume according to the mapped value
  }
}

void ledScene1() {
  // Placeholder for LED Scene 1
  i = 0;
  strip.fill(strip.Color(255, 0, 0), 0, LED_COUNT); // Fill with red
  strip.show();
}

void ledScene2() {
  // Placeholder for LED Scene 2
  i = 0;
  strip.fill(strip.Color(0, 255, 0), 0, LED_COUNT); // Fill with red
  strip.show();
}

void ledScene3() {
  // Placeholder for LED Scene 3
  i = 0;
  strip.fill(strip.Color(0, 0, 255), 0, LED_COUNT); // Fill with red
  strip.show();
}

void ledScene4() {
  // Placeholder for LED Scene 4
  // for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
  //   strip.setPixelColor(i, strip.Color(255,   0,   0));         //  Set pixel's color (in RAM)
  //   strip.show();                          //  Update strip to match
  //   delay(50);                           //  Pause for a moment
  // }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    strip.setPixelColor(i, strip.Color(255, 0, 0)); // Set pixel's color (in RAM)
    strip.show(); // Update strip to match
    i++;

    // NOTE: This for loop will set the color for one LED and immediately proceed to the next iteration,
    // which might not visually change all LEDs at once in a noticeable way. Adjust logic as needed for your visual effect.
  }
}

void ledScene5() {
  // Placeholder for LED Scene 5
  i = 0;
  strip.fill(strip.Color(127, 127, 127), 0, LED_COUNT); // Fill with red
  strip.show();
}

void controlLEDScenes() {
  int rPotValue = IBus.readChannel(RPot); // Reading the value of RPot

  if (rPotValue <= 1000) {
      // LED off
      strip.clear();
      strip.show();
  } else if (rPotValue > 1000 && rPotValue <= 1200) {
      ledScene1(); // Activate LED Scene 1
  } else if (rPotValue > 1200 && rPotValue <= 1400) {
      ledScene2(); // Activate LED Scene 2
  } else if (rPotValue > 1400 && rPotValue <= 1600) {
      ledScene3(); // Activate LED Scene 3
  } else if (rPotValue > 1600 && rPotValue <= 1800) {
      ledScene4(); // Activate LED Scene 4
  } else if (rPotValue > 1800 && rPotValue <= 2000) {
      ledScene5(); // Activate LED Scene 5
  }
}

void printChannelValues(){
  Serial.print("Channel 0: ");
  Serial.println(IBus.readChannel(0));

  Serial.print("Channel 1: ");
  Serial.println(IBus.readChannel(1));
  
  Serial.print("Channel 2: ");
  Serial.println(IBus.readChannel(2));
  
  Serial.print("Channel 3: ");
  Serial.println(IBus.readChannel(3));
  
  Serial.print("Channel 4: ");
  Serial.println(IBus.readChannel(4));
  
  Serial.print("Channel 5: ");
  Serial.println(IBus.readChannel(5));
  
  Serial.print("Channel 6: ");
  Serial.println(IBus.readChannel(6));
  
  Serial.print("Channel 7: ");
  Serial.println(IBus.readChannel(7));
  
  Serial.print("Channel 8: ");
  Serial.println(IBus.readChannel(8));
  
  Serial.print("Channel 9: ");
  Serial.println(IBus.readChannel(9));

  // Serial.print("IBus Count: ");
  // Serial.println(IBus.cnt_rec);

  Serial.println();
}