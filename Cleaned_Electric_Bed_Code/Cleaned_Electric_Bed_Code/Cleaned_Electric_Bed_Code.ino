#include <IBusBM.h>
#include <VescUart.h>
#include <Adafruit_NeoPixel.h>

VescUart UART1; // VescUart object for commanding the ESC's
VescUart UART2; // VescUart object for commanding the ESC's
IBusBM IBus;    // IBus object for receiving signals from transmitter/receiver

unsigned long lastSignalTime = 0;         // Tracks the last time the signal was confirmed
const unsigned long signalTimeout = 1000; // Signal timeout in milliseconds
uint16_t lastCntRec = 0;                  // Last received count for detecting signal loss

// Control variables
int currentSpeed = 0, targetSpeed = 0, currentTurn = 0, targetTurn = 0;
int accelRate = 100, turnRate = 5; // Rate of change for speed and turn

// Telemetry variables
float batteryVoltage1 = 0.0, batteryVoltage2 = 0.0;
int32_t motorRPM1 = 0, motorRPM2 = 0;
const float voltageWarningThreshold = 33.0;  // Threshold for 10S battery
const int32_t rpmDifferenceThreshold = 1000; // Threshold for RPM difference
int speedAdjustment = 0;                     // For adjusting speed difference between two sets of motors

bool movementEnabled = true; // Global variable to track movement enable state

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
#define ON 2000
#define OFF 1000

// Define relay channels to Arduino pin numbers
const int relay1 = 2;
const int relay2 = 3;
const int relay3 = 4;
const int relay4 = 5;
const int relay5 = 6;
const int relay6 = 7;
const int relay7 = 8;
const int relay8 = 9;

// Define the pin that the LED strip is connected to
#define LED_PIN 6

// Define the number of LEDs in the strip
#define LED_COUNT 300

// Create an Adafruit_NeoPixel object
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Setup function initializes the serial communication ports and prepares the system for operation.
void setup()
{
  Serial.begin(9600);    // Initializes the Serial port for logging to the Arduino IDE Serial Monitor.
  Serial1.begin(115200); // Initializes Serial1 for communication with the first ESC.
  Serial2.begin(115200); // Initializes Serial2 for communication with the second ESC.
  Serial3.begin(115200); // Initializes Serial3 for communication with the iBus receiver.

  setupESCs();         // Calls the function to setup and initialize the ESCs.
  IBus.begin(Serial3); // Initializes the iBus communication on Serial3.
  waitForReceiver();   // Waits for the receiver to start sending signals before proceeding.

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(relay5, OUTPUT);
  pinMode(relay6, OUTPUT);
  pinMode(relay7, OUTPUT);
  pinMode(relay8, OUTPUT);

  // Initialize all relays to OFF position (assuming HIGH means OFF)
  for (int i = relay1; i <= relay8; i++) {
    digitalWrite(i, HIGH);
  }

  strip.begin();           // This initializes the NeoPixel library.
  strip.show();            // Initialize all pixels to 'off'
}

// Main loop function checks for signal loss and processes control inputs if signal is present.
void loop()
{
  serialInput();
  //printChannelValues();
  //if (IBus.cnt_rec != lastCntRec)
  //{                            // Checks if new iBus messages have been received since the last loop iteration.
    updateTelemetry();         // Updates telemetry data from the ESCs for features like battery monitoring.
    processControlInputs();    // Reads control inputs from the receiver and adjusts motor speeds accordingly.
    //lastCntRec = IBus.cnt_rec; // Updates the last iBus message count for the next loop iteration.
    //lastSignalTime = millis(); // Updates the last signal time to the current time.
  //}
  //delay(2000);
}

// Function to initialize the ESCs (Electronic Speed Controllers) by setting their respective serial ports.
void setupESCs()
{
  Serial.println("Initializing ESCs..."); // Log message indicating the start of ESC initialization
  UART1.setSerialPort(&Serial1);          // Assigns Serial1 to UART1, linking the ESC to the first serial port
  UART2.setSerialPort(&Serial2);          // Assigns Serial2 to UART2, linking the second ESC to another serial port
  Serial.println("ESCs Initialized.");    // Log message indicating the ESCs have been successfully initialized
}

// Function to wait for the receiver to start sending data. This ensures the program doesn't proceed without a signal.
void waitForReceiver()
{
  Serial.println("Waiting for receiver..."); // Log message indicating the program is waiting for receiver data
  while (IBus.cnt_rec == 0)
    delay(100);                          // Waits in a loop, delaying 100ms at a time until data is received from the receiver
  lastCntRec = IBus.cnt_rec;             // Stores the count of received messages for later comparison
  lastSignalTime = millis();             // Stores the current time as the last time a signal was received
  Serial.println("Receiver connected."); // Log message indicating a connection to the receiver has been established
}

void processControlInputs() {
  readInputs();

  // Applies smooth adjustments to speed and turn to avoid abrupt changes
  smoothAdjustments();
  
  // Turn system OFF or set current to 0 if input is 1000
  if (targetSpeed <= OFF) {
      UART1.setCurrent(0);
      UART1.setCurrent(0,101);
      UART2.setCurrent(0);
      UART2.setCurrent(0,101);
      Serial.println("System turned off - Current set to 0.");
      return;
  }

  // Depending on the control mode, map targetSpeed to the respective range and apply control
  switch (controlMode) {
      case MODE_RPM:
          // Map targetSpeed (1001-2000) to RPM range (0-7000), considering 1000 as OFF
          int rpmValue = map(currentSpeed, 1001, 2000, 0, 7000);
          applyControlRPM(rpmValue);
          break;
      case MODE_DUTY:
          // Map targetSpeed (1001-2000) to duty cycle range (0-80%)
          float dutyCycle = map(targetSpeed, 1001, 2000, 0, 80) / 100.0;
          applyControlDuty(dutyCycle);
          break;
      case MODE_CURRENT:
          // Map targetSpeed (1001-2000) to current range (0-40A)
          float current = map(targetSpeed, 1001, 2000, 0, 40);
          applyControlCurrent(current);
          break;
  }
}

// Reads inputs from the remote control receiver and maps them to target speed and turn values.
void readInputs()
{
    // Step 1: Ensure all switches are OFF before proceeding
  while (true) {
    int swaState = IBus.readChannel(SWA);
    int swbState = IBus.readChannel(SWB);
    int swcState = IBus.readChannel(SWC);
    int swdState = IBus.readChannel(SWD);

    if (swaState == OFF && swbState == OFF && swcState == OFF && swdState == OFF) {
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
      currentSpeed = 0;
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

  // Checks if the SWA input is above 1000 (typically the threshold for forward/reverse switch)
  if (IBus.readChannel(SWB) == OFF)
  {
    // If true, maps the speed value for reverse direction
    targetSpeed = -targetSpeed;
    Serial.print("Reverse Target Speed: ");
    Serial.println(targetSpeed);
  }
  else
  {
    Serial.print("Target Speed: ");
    Serial.println(targetSpeed);
  }

  // Reads the turn input from Right Joystick X Axis and maps it to a target turn value
  targetTurn = (int)(IBus.readChannel(RJoyX) -1500); // 1000 (left) to 2000 (right), 1500 is neutral, turn -500 to 500
  Serial.print("Target Turn: ");
  Serial.println(targetTurn);
}

// Applies smooth adjustments to the current speed and turn values to reach the target values without abrupt changes.
void smoothAdjustments()
{
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
}

void applyControlRPM(int rpm) {
  // Calculate left and right motor RPM based on target turn and speed adjustments
  int leftRPM = constrain(rpm + (2 * currentTurn), 0, 7000);
  int rightRPM = constrain(rpm - (2 * currentTurn), 0, 7000);
  UART1.setRPM(leftRPM);
  UART2.setRPM(leftRPM);
  UART1.setRPM(rightRPM,101);
  UART2.setRPM(rightRPM,101);
  Serial.print("Left RPM set to: "); Serial.println(leftRPM);
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
void updateTelemetry()
{
  // Retrieve and store telemetry data for each ESC
  if (UART1.getVescValues())
  {
    batteryVoltage1 = UART1.data.inpVoltage; // Store battery voltage from ESC 1
    motorRPM1 = UART1.data.rpm;              // Store motor RPM from ESC 1

    // Serial.print("Battery Voltage: ");
    // Serial.println(batteryVoltage1);
  }
  if (UART2.getVescValues())
  {
    batteryVoltage2 = UART2.data.inpVoltage; // Store battery voltage from ESC 2
    motorRPM2 = UART2.data.rpm;              // Store motor RPM from ESC 2

    // Serial.print("Battery Voltage: ");
    // Serial.println(batteryVoltage2);
  }

  // Check if either battery voltage is below the threshold and warn if so
  if (batteryVoltage1 < voltageWarningThreshold || batteryVoltage2 < voltageWarningThreshold)
  {
    //Serial.println("Warning: Low Battery Voltage!"); // Print a low battery warning
  }
}

// Stops the motors by setting their speed values to neutral, used in cases of signal loss or as part of a shutdown procedure.
void stopMotors()
{
  UART1.setBrakeCurrent(5);
  UART1.setBrakeCurrent(5,101);
  UART1.setCurrent(0);
  UART1.setCurrent(0,101);
  UART2.setBrakeCurrent(5);
  UART2.setBrakeCurrent(5,101);
  UART2.setCurrent(0);
  UART2.setCurrent(0,101);
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

