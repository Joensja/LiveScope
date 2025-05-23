#pragma GCC optimize ("Os")  // OptimIze
// GENERAL INFO
// Forward + Backward is needed for the L298 library, it dont have left/right. 
// ADJUST int baseSpeed = 35; AND int maxSpeed = 50; TO FIT YOU ELECTRIC MOTOR. 

#include <Arduino.h>
#include <L298N.h>  //L298N Andrea Lombardo
#include <EEPROM.h> 
#include <SoftwareSerial.h> 
#include <MotorDriver.h> // Yfrobot Motor driver
#include <Wire.h>
#include <Adafruit_HMC5883_U.h> // Adafruit Unified + Unified Sensor
#include <PID_v1.h> // PID Brett Beareguard

// Initialize HMC5883L magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//---------------------------
// PID CONTROL SETTINGS
//---------------------------
double setHeading = 0;   // Target heading (user sets this)
double output = 0;       // PID output for motor control
double turnThreshold = 4;  // Standardvärde för svänggräns, kan ändras via Bluetooth
double targetHeading = 0; // This is your REAL target
double userSetpoint = 0;      // The heading the user wants (e.g. 33°)
double currentHeading = 0;    // Filtered, current heading (0–360°)
double pidInput = 0;          // What the PID controller reads as "input"
double pidOutput = 0;         // PID controller output to motorf
double pidSetpoint = 0;       // PID setpoint (always zero when using diff!)

// PID tuning constants (adjust as needed)
double Kp = 2.5, Ki = 0.1, Kd = 1.5;
//PID headingPID(&currentHeading, &output, &setHeading, Kp, Ki, Kd, DIRECT);
PID headingPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

//---------------------------
// MAGNETOMETER SETTINGS
//---------------------------
double filteredHeading = 0;       // Filteread heading value
const double alpha = 0.5;         // Filter factor. Higher = Faster
double rawHeading = 0;            // Raw data

// DEFINE STEPPER MOTOR
#define STEPPER_STEP_PIN  30
#define STEPPER_DIR_PIN   31
#define STEPPER_EN_PIN    32

//------------------------------------------
// PIN LAYOUT
//------------------------------------------
// Välj kort
#define BOARD_UNO     0
#define BOARD_D1_R32  1
#define BOARD_MEGA    2

int boardType = 1; // 0 = UNO, 1 = D1 R32, 2 = MEGA

// Globala pinvariabler
int EN, IN1, IN2, posSw, rotSw1, rotSw2, buzzerPin, pedalButton, hallSensorPin, btRxPin, btTxPin;

// BLUETOOTH SETTINGS
#define BTRX_PIN 8
#define BTTX_PIN 13
SoftwareSerial BTserial(BTRX_PIN, BTTX_PIN);

// Byt alla dina andra ställen i koden till att använda t.ex. buzzerPin istället för BUZZER_PIN osv!

void setupPinsByBoard() {
  switch (boardType) {
    case BOARD_UNO:
      EN           = 3;
      IN1          = 12;
      IN2          = 9;
      posSw        = 7;
      rotSw1       = 6;
      rotSw2       = 11;
      buzzerPin    = 10;
      pedalButton  = 5;
      hallSensorPin = A1;
      btRxPin      = 8;
      btTxPin      = 13;
      break;
    case BOARD_D1_R32:
      EN           = 18;
      IN1          = 19;
      IN2          = 21;
      posSw        = 23;
      rotSw1       = 25;
      rotSw2       = 26;
      buzzerPin    = 27;
      pedalButton  = 14;
      hallSensorPin = 32;
      btRxPin      = 16; // (kan ändras efter behov)
      btTxPin      = 17;
      break;
    case BOARD_MEGA:
      EN           = 4;
      IN1          = 5;
      IN2          = 6;
      posSw        = 22;
      rotSw1       = 24;
      rotSw2       = 26;
      buzzerPin    = 28;
      pedalButton  = 30;
      hallSensorPin = 32;
      btRxPin      = 15;
      btTxPin      = 14;
      break;
    default:
      Serial.println("Okänt kort! Kontrollera boardType.");
      while (1);
      break;
  }
}

//-------------------------------------------
// Choose between 1 (L298P) or 2 (L298N);
//-------------------------------------------
int driverMode = 1; 

//-------------------------------------------
// Sensor Mode: 1 = POS-switch, 2 = Hall-sensor
//-------------------------------------------
int sensorMode = 1; 

//-------------------------------------------
// HMC5883L Magnetometer YES/NO
//-------------------------------------------
bool useMagnetometer = true;

// ----------------------------
//   MOTOR CONFIGURATION
// ----------------------------
//int EN, IN1, IN2, posSw, rotSw1, rotSw2, buzzerPin;
L298N* myMotor = nullptr;
enum MotorType { MOTOR_L298, MOTOR_L298N, MOTOR_STEPPER };
MotorType motorType = MOTOR_L298; // byt till MOTOR_STEPPER om du vill

//int pedalButton = 5; // Button on the printed pedal or somewhere that will be used when you don't have the possibility to press both buttons left+right
//int hallSensorPin = A1;  // Digital pin for hallsensor


// ----------------------------
//   GLOBAL VARIABLES
// ----------------------------
bool sweepModeActive   = false;
bool compassModeActive = false;
bool bothPressing = false;
bool pressingsweep = false;
bool countingDoublePress = false;
bool useHallSensor = false;
bool stepperZeroed = false;  // om nollning är gjord efter start
bool waitingForSecondPress = false;

//--------------------------
// SWEEP SETTINGS
//--------------------------
int  sweep_angle   = 5;    
int manualSpeed = 125;  // Default speed for manual mode
int sweepSpeed = 125;    // Default speed for sweep mode
int motorDirection; // Direction of the motor compared to the gyro
int lastPwmValue = 0;


//STEPPER SETTINGS
int stepperPos = 0; // aktuell position (steg från noll)
const unsigned long doublePressTimeout = 2000; // 2 sek max mellan trycken
int stepsPerVarv = 200; // Nema 23

float stepsPerGrad = stepsPerVarv / 360.0; // ca 0.5555

unsigned long lastStopPrintTime = 0;
unsigned long lastPosSwPress = 0;
unsigned long posSwDebounceTime = 400;  
unsigned long lastPressTime = 0;
unsigned long doublePressStartTime = 0;
unsigned long pressTimesweep = 0;
unsigned long bothPressStart = 0;
unsigned long lastDebugPrintTime = 0;  // Timer för debug writing
unsigned long lastPedalPress = 0;
unsigned long firstPedalPress = 0;

// ----------------------------
//   FUNCTION DECLARATIONS
// ----------------------------
void checkBothLongestPress(bool sw1, bool sw2);
void runsweepmaticMode();
void runManualMode(bool sw1, bool sw2);
void stopAllMotors();
void beepMultiple(int n);
void setSweepAngle();
void updateDebounceTime(); 
void beepQuickDouble();
void beepLong();
void beepMinMaxAlert();
void checkModeExit();
void handleSerialCommand(char command);

double shortestAngle(double target, double current) {
    double diff = fmod((target - current + 540), 360) - 180;
    return diff;
}


void saveSpeed(int address, int &currentSpeed, int newSpeed) {
  if (currentSpeed != newSpeed) {  // Only write if different
    currentSpeed = newSpeed;       // Update variable
    EEPROM.write(address, newSpeed);  // Save to EEPROM
  }
}

void savePIDValues() {
    EEPROM.put(0, Kp);
    EEPROM.put(4, Ki);
    EEPROM.put(8, Kd);
}

void loadPIDValues() {
    EEPROM.get(0, Kp);
    EEPROM.get(4, Ki);
    EEPROM.get(8, Kd);

    // ✅ Prevent NaN or zero values
  if (Kp == 0 || Ki == 0 || Kd == 0 || isnan(Kp) || isnan(Ki) || isnan(Kd)) {
      Kp = 2.5; Ki = 0.1; Kd = 0.5;
      savePIDValues(); // Saves default values
    }

    headingPID.SetTunings(Kp, Ki, Kd);
}

// CHECK IF BOTH BUTTONS ARE PRESSED AT THE SAME TIME
bool checkHoldToExit(unsigned long &holdStart, unsigned long exitHoldTime = 3000) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);
    bool pedal = isPedalPressed();  

    bool exitPressed = (sw1 && sw2) || pedal;

    if (exitPressed) {
        if (holdStart == 0) holdStart = millis();

        if (millis() - holdStart >= exitHoldTime) {
            beepMultiple(2);  
            return true;     
        }
    } else {
        holdStart = 0; 
    }

    return false; 
}

//----------------------------------
// HEADING UPDATE
//----------------------------------
void updateHeading() {
    if (!useMagnetometer) return;

    sensors_event_t event;
    static int failCount = 0;
    static double filteredHeading = 0;   // Intern variable for filter
    const double alpha = 0.3;           // Filter factor higher = faster
    double rawHeading = 0;

    if (!mag.getEvent(&event)) {
        failCount++;
        if (failCount > 5) {
            Serial.println(F("Magnetometer failed repeatedly. Disabling..."));
            useMagnetometer = false;
            failCount = 0;
            return;
        }
        return;
    }
    failCount = 0;

    // calculate new data from magnetometer (in degrees, 0-360)
    rawHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (rawHeading < 0) rawHeading += 360;

    // WRAP-FILTER 0/360
    double delta = rawHeading - filteredHeading;
    if (delta > 180) delta -= 360;
    else if (delta < -180) delta += 360;

    filteredHeading += alpha * delta;

    // Wrap if filter value  [0,360]
    if (filteredHeading < 0) filteredHeading += 360;
    if (filteredHeading >= 360) filteredHeading -= 360;

    currentHeading = filteredHeading;

    // --- PRINT DEBUG ONLY EVERY 500ms ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 750) {
        Serial.print(F("Newheading (raw): "));
        Serial.print(rawHeading, 2);
        Serial.print(F(" | Filtered: "));
        Serial.println(filteredHeading, 2);
        lastPrint = millis();
    }
}

// ----------------------------
//   PEDAL BUTTON FUNCTION TO MINIMIZE DEBOUNCE
// ----------------------------
bool isPedalPressed() {
  static unsigned long lastDebounceTime = 0;
  static bool lastState = HIGH;
  bool currentState = digitalRead(pedalButton) == LOW;

  if (currentState != lastState) {
      lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {  // 🟢 50ms debounce
      if (currentState) {
          lastState = currentState;
          return true;
      }
  }

  lastState = currentState;
  return false;
}

void loadSettings() {
    EEPROM.get(12, turnThreshold);

    if (isnan(turnThreshold) || turnThreshold < 1 || turnThreshold > 20) {
        turnThreshold = 5;  // Reset if saved value is crap
        EEPROM.put(12, turnThreshold);
    }
}

void stepMotor(int steps, int dir, int speedDelay) {
  digitalWrite(STEPPER_DIR_PIN, dir > 0 ? HIGH : LOW);
  digitalWrite(STEPPER_EN_PIN, LOW); // Enable
  for (int i = 0; i < abs(steps); i++) {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(speedDelay);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(speedDelay);
  }
  // Uppdatera stegräknaren
  stepperPos += steps * dir;
}

void setup() {
    Wire.begin();  // start the I2C-bus
    mag.begin();
    Serial.begin(9600);
    BTserial.begin(9600);
    updateHeading();
    loadSettings();
    loadPIDValues();
    setupPinsByBoard();

    setupPinsByBoard();
    //BTserial = new SoftwareSerial(btRxPin, btTxPin);
    //BTserial->begin(9600);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, LOW); // Enable motorn direkt
    stepperPos = 0;
    stepperZeroed = false;

    EEPROM.get(20, motorDirection); // Load motor direction from calibration saved in EEPROM
    if (motorDirection != 1 && motorDirection != -1) {
        motorDirection = 1; // Default if invalid data
    }

    Serial.print(F("M.Dir "));
    Serial.println(motorDirection);

    // Initiera 5883
    if (useMagnetometer) {
    if (!mag.begin()) {
        Serial.println(F("Could not find a valid HMC5883L sensor, check wiring!"));
        useMagnetometer = false;
    } else {
        Serial.println(F("HMC5883L found and responsive."));
        Serial.print(F("Heading:"));
        Serial.println(currentHeading);
    }
    } else {
    Serial.println(F("Magnetometer disabled by config."));
    }


  // HALL SENSOR 
  if (sensorMode == 1) {
    useHallSensor = false;
    Serial.println(F("Sensor: POS-switch aktiv"));
    pinMode(posSw, INPUT_PULLUP);
  } else if (sensorMode == 2) {
    useHallSensor = true;
    Serial.println(F("Sensor: Hall-sensor aktiv"));
    pinMode(hallSensorPin, INPUT_PULLUP);
  }

  // Initialize PID controller
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-255, 255); // Motor PWM range

    if (driverMode == 2) { // External board L298N 
    Serial.println(F("L298N"));
    EN  = 10;
    IN1 = 9;
    IN2 = 8;
    posSw   = 4;
    rotSw1  = 3;
    rotSw2  = 2;
    buzzerPin = 7;
    myMotor = new L298N(EN, IN1, IN2); // Create motor driver
    }

    // Configure pins as output or input
    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(posSw, INPUT_PULLUP);
    pinMode(rotSw1, INPUT_PULLUP);
    pinMode(rotSw2, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    pinMode(pedalButton, INPUT_PULLUP);

    digitalWrite(EN, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

  // Load last saved speeds from EEPROM
  int savedManualSpeed = EEPROM.read(0);  
  int savedsweepSpeed = EEPROM.read(1);

  if (savedManualSpeed >= 50 && savedManualSpeed <= 250) {
      manualSpeed = savedManualSpeed;
  } else {
      manualSpeed = 125;
  }

  if (savedsweepSpeed >= 50 && savedsweepSpeed <= 250) {
      sweepSpeed = savedsweepSpeed;
  } else {
      sweepSpeed = 150;
  }

  if (myMotor) {
      myMotor->setSpeed(manualSpeed);
      myMotor->stop();
  }

  updateDebounceTime();
}

void loop() {
  if (Serial.available()) {
    char data = Serial.read();
    BTserial.write(data);
    handleSerialCommand(data); 
  }
  if (BTserial.available()) {
    char data = BTserial.read();
    handleBluetoothCommand(data);
  }

  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

    checkBothLongestPress(sw1, sw2);
    checkModeExit();
    checkPedalDoublePress();
    if (!sweepModeActive && !compassModeActive) {
        runManualMode(digitalRead(rotSw1) == LOW, digitalRead(rotSw2) == LOW);
    }

      static unsigned long lastHeadingCheck = millis();
      static double previousHeading = 0;
      static int headingStuckCount = 0;

      //Check only if in compass or search mode and motor is on:
      if ((sweepModeActive || compassModeActive) && lastPwmValue > 0 && useMagnetometer && millis() - lastHeadingCheck > 5000) {
          if (abs(currentHeading - previousHeading) < 0.1) {
              headingStuckCount++;
              Serial.print(F("Heading still stuck! Retry #"));
              Serial.println(headingStuckCount);
              Serial.print(F("Heading:"));
              Serial.println(currentHeading);
              Serial.print(F("previousHeading:"));
              Serial.println(previousHeading);

              if (headingStuckCount >= 3) {
                  Serial.println(F("Too many heading stalls. Disabling magnetometer."));
                  useMagnetometer = false;
                  headingStuckCount = 0;
              }
             
          } else {
              headingStuckCount = 0; // Reset if it moved
          }
          previousHeading = currentHeading;
          lastHeadingCheck = millis();
        }
    if (useMagnetometer) updateHeading();
    delay(200); 
    }

//---------------------
// SPEED SETTINGS
//---------------------
void adjustSpeed(bool increase, bool issweep) {
  int &speed = issweep ? sweepSpeed : manualSpeed;
  int address = issweep ? 1 : 0;
  int change = increase ? 25 : -25;
  
  int newSpeed = constrain(speed + change, 50, 250);
  if (newSpeed != speed) {
      speed = newSpeed;
      EEPROM.write(address, speed);
      Serial.print(issweep ? "New sweep Mode Speed: " : "New Manual Mode Speed: ");
      Serial.println(speed);
  } else {
      Serial.println(issweep ? "sweep Mode Speed limit reached" : "Manual Mode Speed limit reached");
      beepMinMaxAlert();
  }
}

// ----------------------------
//   MOTOR CONTROL
// ----------------------------
void controlMotor(int direction, int speed) { // 1 = Right/Forward -1 = Left/Backward
    lastPwmValue = speed; // Saves PWM-value
    if (driverMode == 1) { // L298P motor driver
        if (direction == 1) { // Forward (right)
            digitalWrite(IN1, HIGH);
        } else if (direction == -1) { // Backward (left)
            digitalWrite(IN1, LOW);
        }
        analogWrite(EN, speed); // Set motor speed using PWM
    } else if (driverMode == 2 && myMotor) { // L298N motor driver
        myMotor->setSpeed(speed);
        if (direction == 1) {
            myMotor->forward();
        } else if (direction == -1) {
            myMotor->backward();
        }
    }
}

// ----------------------------
//   SWEEP ANGLE
// ----------------------------
void setSweepAngle() {
  Serial.println(F("** setSweepAngle MENU **"));
  stopAllMotors();

  bool exitMenu = false;
  unsigned long lastButtonPress = 0;
  const unsigned long buttonDebounce = 300;

  while (!exitMenu) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);
    unsigned long holdStart = 0; 
    bool pedal = isPedalPressed();
    bool exitPressed = (sw1 && sw2) || pedal;

    if (checkHoldToExit(holdStart)) {
      Serial.println(F("Exiting Sweep Angle Menu"));
      delay(1000);
      break;
    }

    // Öka sweep_angle
    if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      sweep_angle = (sweep_angle >= 8) ? 2 : sweep_angle + 1;
      //Serial.print(F("New Sweep Angle: "));
      Serial.println(sweep_angle);
      beepMultiple(1);
    }

    // Minska sweep_angle
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      sweep_angle = (sweep_angle <= 2) ? 8 : sweep_angle - 1;
      //Serial.print(F("New Sweep Angle: "));
      Serial.println(sweep_angle);
      beepMultiple(1);
      }
    }
}

//-----------------------------
// BLUETOOTH COMMANDS
// L - LEFT
// R - RIGHT
// + - INCRESE MANUAL SPEED
// - - DECREASE MANUAL SPEED
// U . INCREASE SWEEP SPEED
// H - DECREASE SWEEP SPEED
// N - INCREASE SWEEP ANGLE
// n - DECREASE SWEEP ANGLE
// C - ACTIVATE COMPASS MODE
// A - ACTIVATE SEARCH MODE
// S - STOP ALL FUNCTIONS
// P - ADJUST KP = "P3.5" → Sets Kp to 3.5
// I - ADJUST I = "I0.15" → Sets Ki to 0.15
// D - ADJUST D = "D0.7" → Sets Kd to 0.7
// T - CHANGE THREASHHOLD FOR COMPASS
// h - SET HEADING "H90.0" FOR 90°
// X - ACTIVATE/DEACTIVATE COMPASS
// Q - COMPASS CALIBRATION
// O - GET ALL SETTINGS
// ----------------------------
// ADD THESE
// NOTHING TO ADD
//-----------------------------
void handleBluetoothCommand(char command) {
  switch (command) {
    case 'L':  // Turn Left
      if (driverMode == 1) {
          digitalWrite(IN1, LOW);
          analogWrite(EN, manualSpeed);
      } else if (driverMode == 2 && myMotor) {
          myMotor->setSpeed(manualSpeed);
          myMotor->backward();
      }
      break;

    case 'R': //  Turn right
      if (driverMode == 1) {
          digitalWrite(IN1, HIGH);
          analogWrite(EN, manualSpeed);
      } else if (driverMode == 2 && myMotor) {
          myMotor->setSpeed(manualSpeed);
          myMotor->forward();
      }
      break;

    case '+':  // Increase Manual Speed
      saveSpeed(0, manualSpeed, min(manualSpeed + 25, 250));  // Prevent exceeding 250
      myMotor->setSpeed(manualSpeed);
      BTserial.print(F("Manual Speed: "));
      BTserial.println(manualSpeed);      
      break;

    case '-':  // Decrease Manual Speed
      saveSpeed(0, manualSpeed, max(manualSpeed - 25, 25));  // Prevent going below 25
      myMotor->setSpeed(manualSpeed);
      BTserial.print(F("Manual Speed: "));
      BTserial.println(manualSpeed); 
      break;

    case 'U':  // Increase sweep Speed
      saveSpeed(1, sweepSpeed, min(sweepSpeed + 25, 250));  // Prevent exceeding 250
      BTserial.print(F("Sweep Speed: "));
      BTserial.println(sweepSpeed);
      break;

    case 'H':  // Decrease sweep Speed
      saveSpeed(1, sweepSpeed, max(sweepSpeed - 25, 25));  // Prevent going below 25
      BTserial.print(F("Sweep Speed: "));
      BTserial.println(sweepSpeed);
      break;
      
    case 'N':  // INCREASE SWEEP ANGLE
        sweep_angle = min(sweep_angle + 1, 8);  // max 8 eller vad du vill
        BTserial.print(F("Sweep Angle: "));
        BTserial.println(sweep_angle);
        break;

    case 'n':  // DECREASE SWEEP ANGLE
        sweep_angle = max(sweep_angle - 1, 2);  // min 2 eller vad du vill
        BTserial.print(F("Sweep Angle: "));
        BTserial.println(sweep_angle);
        break;

    case 'C':  // ACTIVATE COMPASS MODE
      compassModeActive = true;
      BTserial.println(F("Compass Mode activated"));
      break;

    case 'A':  // ACTIVATE SEARCH MODE
      sweepModeActive = true;
      BTserial.println(F("Sweep Mode Activated"));
      break;

    /*case 'M':  
      sweepModeActive = false;
      stopAllMotors();
      BTserial.println(F("Sweep Mode Deactivated"));

      break;*/

    case 'S':  // STOP ALL FUNCTIONS
        sweepModeActive = false;    // Deactivate Sweep Mode
        compassModeActive = false;  // Deactivate Compass Mode
        setHeading = 0;             // Reset setpoint
        stopAllMotors();
        Serial.println(F("Sweep OR Compass deactivated"));
        break;

    case 'P':  // Adjust Kp "P3.5" → Sets Kp to 3.5
      if (BTserial.available()) {
          Kp = BTserial.parseFloat();
          headingPID.SetTunings(Kp, Ki, Kd);
          savePIDValues();  // Save new Kd to EEPROM
        }
      break;

    case 'I':  // Adjust Ki "I0.15" → Sets Ki to 0.15
      if (BTserial.available()) {
          Ki = BTserial.parseFloat();
          headingPID.SetTunings(Kp, Ki, Kd);
          savePIDValues();  // Save new Kd to EEPROM
        }
      break;

    case 'D':  // Adjust Kd "D0.7" → Sets Kd to 0.7
      if (BTserial.available()) {
          Kd = BTserial.parseFloat();
          headingPID.SetTunings(Kp, Ki, Kd);
          savePIDValues();  // Save new Kd to EEPROM
        }
      break;

      case 'T':  // Ändra tröskelvärdet för styrning
          if (BTserial.available()) {
              turnThreshold = BTserial.parseFloat();
              turnThreshold = constrain(turnThreshold, 1, 20);  // Begränsa mellan 1-20
              EEPROM.put(12, turnThreshold);  // Spara i EEPROM
          }
          break;

        case 'h':  // SET HEADING (example: H90.0 for 90 degrees)
            headingPID.SetMode(MANUAL);  // Pause PID
            delay(50);  // Give Serial time to receive full value
            if (Serial.available()) {
                userSetpoint = Serial.parseFloat(); 
                Serial.print(F("New Heading Setpoint: "));
                Serial.println(userSetpoint);
            }
            headingPID.SetMode(AUTOMATIC);  // Reactivate PID
            break;

        case 'X':  // ACTIVATE/DEACTIVATE COMPASS 
            useMagnetometer = !useMagnetometer;
            Serial.print(F("Magnetometer is now "));
            Serial.println(useMagnetometer ? "ENABLED" : "DISABLED");
            BTserial.println(useMagnetometer ? "X1" : "X0");
            break;

        case 'Q':
            calibrateCompassMotorDirection(); // New command to calibrate direction
            break;

        case 'O':  // Request all current settings
          BTserial.println(F("=== Current System Status ==="));
          BTserial.print(F("Driver Mode: "));
          BTserial.println(driverMode == 1 ? "L298P" : "L298N");

          BTserial.print(F("Manual Speed: "));
          BTserial.println(manualSpeed);

          BTserial.print(F("Sweep Speed: "));
          BTserial.println(sweepSpeed);

          BTserial.print(F("Sweep Mode: "));
          BTserial.println(sweepModeActive ? "Active" : "Inactive");

          BTserial.print(F("Current Heading: "));
          BTserial.println(currentHeading);

          BTserial.print(F("Target Heading: "));
          BTserial.println(setHeading);

          BTserial.print(F("Kp: "));
          BTserial.println(Kp);

          BTserial.print(F("Ki: "));
          BTserial.println(Ki);

          BTserial.print(F("Kd: "));
          BTserial.println(Kd);

          BTserial.println(F("=== End of Report ==="));
          Serial.println(F("[BT] Sent system status report"));
        break;

      
    default:
      //Serial.println(F("[BT] Unknown Command"));
      //BTserial.print(Fln(F("Unknown Command"));
      break;
  }
}

//-----------------------------
// SERIAL COMMANDS
// C - ACTIVATE COMPASS MODE
// H - SET HEADING "H90.0" FOR 90°
// P - SET Kp "P4.5" FOR 4.5
// I - SET Ki "I0.2" FOR 0.2
// D - SET Kd "D1.0" FOR 1.0
// S - STOP ALL FUNCTIONS
// O - SHOW HEADING STATUS
// Q - COMPASS CALIBRATION
// X - ACTIVATE/DEACTIVATE COMPASS 
//-----------------------------
void handleSerialCommand(char command) {
  switch (command) {
    case 'C':  // ACTIVATE COMPASS MODE
      delay(50);  // Ge Serial lite tid att läsa hela värdet
      runCompassMode();
      Serial.println(F("Compass Mode Activated"));
      break;

  case 'H':  // SET HEADING (example: H90.0 for 90 degrees)
      headingPID.SetMode(MANUAL);  // Pause PID
      delay(50);  // Give Serial time to receive full value
      if (Serial.available()) {
          userSetpoint = Serial.parseFloat();  // <-- use userSetpoint everywhere!
          Serial.print(F("New Heading Setpoint: "));
          Serial.println(userSetpoint);
      }
      headingPID.SetMode(AUTOMATIC);  // Reactivate PID
      break;

  case 'P':  // ADJUST Kp (ex: P4.5)
    if (Serial.available()) {
      Kp = Serial.parseFloat();
      headingPID.SetTunings(Kp, Ki, Kd);
      savePIDValues();
      Serial.print(F("Kp set to: "));
      Serial.println(Kp);
    }
    break;

  case 'I':  // ADJUST Ki (ex: I0.2)
    if (Serial.available()) {
      Ki = Serial.parseFloat();
      headingPID.SetTunings(Kp, Ki, Kd);
      savePIDValues();
      Serial.print(F("Ki set to: "));
      Serial.println(Ki);
    }
    break;

  case 'D':  // ADJUST Kd (ex: D1.0)
    if (Serial.available()) {
      Kd = Serial.parseFloat();
      headingPID.SetTunings(Kp, Ki, Kd);
      savePIDValues();
      Serial.print(F("Kd set to: "));
      Serial.println(Kd);
    }
    break;

  case 'S':  // STOP ALL FUNCTIONS
      sweepModeActive = false;    // Deactivate Sweep Mode
      compassModeActive = false;  // Deactivate Compass Mode
      stopAllMotors();
      Serial.println(F("Sweep + Compass deactivated"));
      break;

  case 'O':  // SHOW STATUS
    updateHeading();
    Serial.print(F("Current Heading: "));
    Serial.println(currentHeading);
    Serial.print(F("Target Heading: "));
    Serial.println(userSetpoint);
    break;

  case 'Q':
      calibrateCompassMotorDirection(); // New command to calibrate direction
      break;

  while (Serial.available()) Serial.read();  // Clear buffer
  Serial.println(F("Unknown Command"));
  break;

  case 'X':  // ACTIVATE/DEACTIVATE COMPASS 
    useMagnetometer = !useMagnetometer;
    Serial.print(F("Magnetometer is now "));
    Serial.println(useMagnetometer ? "ENABLED" : "DISABLED");
    break;
  }
}

// ----------------------------
//   COMPASS CALIBRATION
// ----------------------------
// Function to calibrate motor direction for compass mode
// This function runs a short motor movement and checks which direction reduces the error to the set heading.
void calibrateCompassMotorDirection() {
    int testDirection = 1; // Default value
    Serial.println(F("=== Calibrating motor direction (compass) ==="));
    updateHeading();
    double startHeading = currentHeading;
    Serial.print(F("Start Heading: "));
    Serial.println(startHeading, 2);

    // Move motor forward (direction 1)
    unsigned long startTime = millis();
    while (millis() - startTime < 4000) {  // 4 seconds of movement
        controlMotor(1, 50);
        delay(5);  // Small delay to not block I2C bus
    }

    stopAllMotors();
    delay(500);  // Let the sensor stabilize
    updateHeading();
    double endHeading = currentHeading;
    Serial.print(F("End Heading: "));
    Serial.println(endHeading, 2);

    // Calculate heading delta (wrap-safe)
    double delta = shortestAngle(endHeading, startHeading);
    Serial.print(F("Delta heading: "));
    Serial.println(delta, 2);

    // Decide direction
    if (delta > 10) {
        testDirection = 1; // Motor direction 1 = RIGHT (increases heading)
    } else if (delta < -10) {
        testDirection = -1; // Motor direction 1 = LEFT (decreases heading)
    } else {
        Serial.println(F("Calibration failed! Not enough movement."));
        return;
    }

    // Move back to starting position (optional but nice)
    unsigned long returnTime = millis();
    while (millis() - returnTime < 4000) {
        controlMotor(-1, 50);
        delay(5);
    }
    stopAllMotors();

    // Save to EEPROM
    EEPROM.put(20, testDirection);
    Serial.print(F("Motor Direction Calibrated: "));
    Serial.println(testDirection);

    if (testDirection == 1) {
        Serial.println(F("Motor 1 = RIGHT (increases heading)"));
    } else {
        Serial.println(F("Motor 1 = LEFT (decreases heading)"));
    }
}


// ----------------------------
//   SOMETHING FOR THE FUTURE OF DEBOUNCE
// ----------------------------
void updateDebounceTime() {
  unsigned long newDebounceTime = 450;
  // Only print if debounce time changes
  static unsigned long lastDebounceTime = 0;
  if (newDebounceTime != lastDebounceTime) {
    lastDebounceTime = newDebounceTime;  // Store last debounce value
  }

  posSwDebounceTime = newDebounceTime;  // Apply new debounce time
}

// ----------------------------
//   SWEEP MODE
// ----------------------------
/*void runsweepmaticMode() {
    sweepModeActive = true;
    beepMultiple(0); // Initial beep signal to indicate sweep mode start
    stopAllMotors(); // Ensure motors are stopped before starting the sweep
    delay(1000); // Short delay for stabilization

    // Move motor backward until position switch (posSw) is triggered, indicating starting point
    controlMotor(-1, sweepSpeed); // Move motor backward at sweep speed
    unsigned long sweepStart = millis();
    while (isSensorActive() == HIGH) {
        if (millis() - sweepStart > 2000) { // -Stop motor if no trigger in 3s
            Serial.println(F("Sweep init 2s timeout."));
            sweepModeActive = false;
            stopAllMotors();
            return;
        }
        checkModeExit();
        if (!sweepModeActive) return;
    }


    stopAllMotors(); // Stop motors after reaching start position
    int posCount = 0; // Initialize position counter
    delay(1000); // Delay to ensure motor is fully stopped

    while (sweepModeActive) { // Main loop for sweeping
        checkModeExit(); // Check for exit command each loop iteration
        if (!sweepModeActive) break; // Exit loop if sweep mode is deactivated

        // Move motor forward towards the sweep angle limit
        controlMotor(1, sweepSpeed); // Move motor forward at sweep speed

        // Increment position counter until the desired sweep angle is reached
        while (posCount < sweep_angle) {
            checkModeExit(); // Check for exit command
            if (!sweepModeActive) return; // Exit if sweep mode is deactivated

          if (isSensorActive() && millis() - lastPosSwPress > posSwDebounceTime) {
              lastPosSwPress = millis();
              posCount++;
              Serial.print(F("Sensor registrerad ("));
              Serial.print(useHallSensor ? "Hall" : "POS");
              Serial.print(F(") - steg framåt: "));
              Serial.println(posCount);
          }
        }

          stopAllMotors(); // Stop motors after reaching sweep limit
          delay(1000); // Delay to stabilize before returning

          // Move motor backward to return to the start position
          controlMotor(-1, sweepSpeed); // Move motor backward at sweep speed

          // Decrement position counter until the start position is reached
          while (posCount > 0) {
              checkModeExit(); // Check for exit command
              if (!sweepModeActive) return; // Exit if sweep mode is deactivated

              if (isSensorActive() && millis() - lastPosSwPress > posSwDebounceTime) {
                  lastPosSwPress = millis();
                  posCount--;
                  Serial.print(F("Sensor registrerad ("));
                  Serial.print(useHallSensor ? "Hall" : "POS");
                  Serial.print(F(") - steg bakåt: "));
                  Serial.println(posCount);
              }
          }

        stopAllMotors(); // Stop motors after returning to start
        delay(1000); // Delay to ensure motor is fully stopped before next cycle
    }
}*/

// ----------------------------
//   SWEEP MODE 2.0
// ----------------------------
void runsweepmaticMode() {
    sweepModeActive = true;
    beepMultiple(0);  // Initial beep signal to indicate sweep mode start
    stopAllMotors();  // Ensure motors are stopped before starting the sweep
    delay(1000);      // Short delay for stabilization

    const unsigned long timeoutLimit = 10000;  // Timeout in milliseconds for any sensor wait
    unsigned long timeoutStart = millis();

    // *** För stegmotor ***
    if (motorType == MOTOR_STEPPER) {
        if (!stepperZeroed) {
            Serial.println(F("Nollställ stegmotorn först (dubbeltryck på pedal)!"));
            beepMultiple(3); // Exempel på varningssignal
            sweepModeActive = false;
            return;
        }

        // Börja från stepperPos = 0
        int stepsPerVarv = 200; // din motor
        float stepsPerGrad = stepsPerVarv / 360.0;
        int sweepSteps = round(sweep_angle * stepsPerGrad);

        while (sweepModeActive) {
            // Svep höger
            stepMotor(sweepSteps, 1, 500); // 1 = höger, justera delay för fart
            delay(500); // paus

            // Svep vänster
            stepMotor(sweepSteps, -1, 500); // -1 = vänster
            delay(500); // paus

            // Lägg till avbrottskoll om du vill kunna avbryta med knapp/pedal
            checkModeExit();
            if (!sweepModeActive) break;
        }
        stopAllMotors();
        return;
    }

    // Move motor left to reach starting position (sensor trigger)
    controlMotor(1, sweepSpeed);
    while (!isSensorActive()) {
        if (millis() - timeoutStart > timeoutLimit) {
            Serial.println(F("[SWEEP] Timeout while seeking start position."));
            sweepModeActive = false;
            stopAllMotors();
            return;
        }
        checkModeExit();
        if (!sweepModeActive) return;
    }

    stopAllMotors();
    int posCount = 0;
    delay(1000);  // Ensure motor is fully stopped

    while (sweepModeActive) {
        checkModeExit();
        if (!sweepModeActive) break;

        // Move right
        controlMotor(-1, sweepSpeed);
        timeoutStart = millis();

        while (posCount < sweep_angle) {
            checkModeExit();
            if (!sweepModeActive) return;

            if (isSensorActive() && millis() - lastPosSwPress > posSwDebounceTime) {
                lastPosSwPress = millis();
                posCount++;
                Serial.print(F("[SWEEP] Sensor triggered - step forward: "));
                Serial.println(posCount);
                timeoutStart = millis();  // Reset timeout after successful trigger

                // Wait for sensor to be released before accepting next trigger
                while (isSensorActive()) {
                    checkModeExit();
                    if (!sweepModeActive) return;
                    delay(10);
                }
            }

            if (millis() - timeoutStart > timeoutLimit) {
                Serial.println(F("[SWEEP] Timeout while sweeping forward."));
                sweepModeActive = false;
                stopAllMotors();
                return;
            }
        }

        stopAllMotors();
        delay(1000);

        // Move left
        controlMotor(1, sweepSpeed);
        timeoutStart = millis();

        while (posCount > 0) {
            checkModeExit();
            if (!sweepModeActive) return;

            if (isSensorActive() && millis() - lastPosSwPress > posSwDebounceTime) {
                lastPosSwPress = millis();
                posCount--;
                Serial.print(F("[SWEEP] Sensor triggered - step backward: "));
                Serial.println(posCount);
                timeoutStart = millis();  // Reset timeout after successful trigger

                // Wait for sensor to be released before accepting next trigger
                while (isSensorActive()) {
                    checkModeExit();
                    if (!sweepModeActive) return;
                    delay(10);
                }
            }

            if (millis() - timeoutStart > timeoutLimit) {
                Serial.println(F("[SWEEP] Timeout while sweeping backward."));
                sweepModeActive = false;
                stopAllMotors();
                return;
            }
        }

        stopAllMotors();
        delay(1000);  // Pause before starting next sweep cycle
    }
}



// ----------------------------
//   Compass Mode (NOT ACTUALLY A COMPASS BUT I'M TO LAZY TO CHANGE THE NAME)
// ----------------------------
//NEW STEPPER MOTOR VERSION
// ---------------------------------------------
// Compass Mode - Stepper and DC Motor support
// This function will run compass/heading control
// - Uses a magnetometer (HMC5883L)
// - Supports both DC motors (with L298) and stepper motor (manual zeroing)
// - Stepper zero position set by double-press on pedal
// - Extensive comments for clarity
// ---------------------------------------------

void runCompassMode() {
    // --- Safety: If magnetometer is not enabled, skip ---
    if (!useMagnetometer) {
        Serial.println(F("Compass mode skipped: Magnetometer disabled."));
        return;
    }

    compassModeActive = true;  // Flag for active compass mode
    Serial.println(F("Compass mode started."));
    EEPROM.get(20, motorDirection); // Load motor direction from EEPROM (set by calibration)

    beepMultiple(2);        // Two beeps to indicate mode start
    stopAllMotors();        // Make sure motors are not moving

    // Optional: Print initial target heading to both serial and Bluetooth
    if (millis() - lastDebugPrintTime >= 2000) {
        Serial.print(F("Initial User Setpoint: "));
        Serial.println(userSetpoint);
        BTserial.print(F("Initial User Setpoint: "));
        BTserial.println(userSetpoint);
        lastDebugPrintTime = millis();
    }

    // --- Main compass control loop ---
    while (compassModeActive) {
        checkModeExit();  // Allow user to exit with switches/pedal

        // --- Serial: Update heading setpoint if requested (Hxxx.x) ---
        if (Serial.available()) {
            char command = Serial.read();
            if (command == 'H') {
                headingPID.SetMode(MANUAL); // Pause PID while updating
                delay(50);
                if (Serial.available()) {
                    userSetpoint = Serial.parseFloat();
                    Serial.print(F("New Heading Setpoint: "));
                    Serial.println(userSetpoint);
                }
                headingPID.SetMode(AUTOMATIC);
            } else {
                handleSerialCommand(command); // Handle any other command
            }
        }

        // --- Get latest heading from magnetometer (filtered) ---
        updateHeading(); // Updates currentHeading variable

        // --- Calculate the shortest angle between current and setpoint ---
        double diff = shortestAngle(userSetpoint, currentHeading);

        // --- PID setup: We want to reduce diff to zero ---
        pidInput = diff;
        pidSetpoint = 0;
        headingPID.Compute();  // Updates pidOutput

        // --- Debug print: Print once per second ---
        if (millis() - lastDebugPrintTime >= 1000) {
            Serial.print(F("[DEBUG] Target Heading: "));
            Serial.print(userSetpoint, 2);
            Serial.print(F(" | True Heading: "));
            Serial.print(currentHeading, 2);
            Serial.print(F(" | Diff: "));
            Serial.print(diff, 2);
            Serial.print(F(" | PID Output: "));
            Serial.println(pidOutput, 2);
            lastDebugPrintTime = millis();
        }

        // --- SAFETY: Stop everything if angle difference is huge (sensor error?) ---
        if (abs(diff) > 120) {
            stopAllMotors();
            Serial.println(F("[COMPASS] Diff > 120°, motor stopped for safety!"));
            continue; // Wait until error is fixed
        }

        // ---------------------------------------------
        //         STEPPER MOTOR MODE 
        // ---------------------------------------------
        if (motorType == MOTOR_STEPPER) {
            // --- Ensure stepper has been zeroed (double pedal press) ---
            if (!stepperZeroed) {
                Serial.println(F("Stepper not zeroed! Double-press pedal first."));
                compassModeActive = false;
                stopAllMotors();
                return;
            }

            // --- Timing for stepper moves (don't step too fast!) ---
            static unsigned long lastStepperMove = 0;
            const unsigned long stepperInterval = 80; // Minimum ms between steps
            static unsigned long lastOnTargetPrint = 0;
            const int deadZone = 5;   // How close to target is "good enough"

            // --- Only move stepper every stepperInterval ms ---
            if (millis() - lastStepperMove >= stepperInterval) {
                lastStepperMove = millis();

                // --- If within deadZone, don't move at all ---
                if (abs(diff) <= deadZone) {
                    // Only print message once per second
                    if (millis() - lastOnTargetPrint > 1000) {
                        Serial.println(F("Stepper ON TARGET (within deadZone) - STOP"));
                        lastOnTargetPrint = millis();
                    }
                    // Do not call stepMotor!
                } 
                // --- If too much to the right, step right (positive direction) ---
                else if (diff > turnThreshold) {
                    // One microstep right, delay controls speed (smaller=faster)
                    stepMotor(1, 1, 400);  // 1 step, dir=1, 400us pulse (tune for your motor)
                    Serial.println(F("Stepper Turning RIGHT"));
                } 
                // --- If too much to the left, step left (negative direction) ---
                else if (diff < -turnThreshold) {
                    stepMotor(1, -1, 400); // 1 step, dir=-1
                    Serial.println(F("Stepper Turning LEFT"));
                }
            }

            delay(30); // Don't run too tight a loop (smoothness & CPU)
            continue;  // Skip rest of loop (DC-motor section)
        }

        // ---------------------------------------------
        //         DC MOTOR (L298) MODE
        // ---------------------------------------------

        // --- Only update motor every 200ms (for stability) ---
        static unsigned long lastMotorUpdate = 0;
        const unsigned long motorUpdateInterval = 200;
        const int deadZone = 5;
        static unsigned long lastOnTargetPrint = 0;

        if (millis() - lastMotorUpdate >= motorUpdateInterval) {
            lastMotorUpdate = millis();

            // --- Calculate smooth speed ramp ---
            int baseSpeed = 35;     // Minimum PWM value (won't stall)
            int maxSpeed = 50;      // Maximum PWM value (tune for your motor)
            static int lastSpeed = 0;
            int targetSpeed = constrain(abs(pidOutput), baseSpeed, maxSpeed);
            int speed = lastSpeed + ((targetSpeed - lastSpeed) / 4); // Soft acceleration
            lastSpeed = speed;

            // --- If within deadZone, stop and print once/sec ---
            if (abs(diff) <= deadZone) {
                stopAllMotors();
                if (millis() - lastOnTargetPrint > 1000) {
                    Serial.println(F("ON TARGET (within deadZone) - STOPPING"));
                    lastOnTargetPrint = millis();
                }
                continue;
            }

            // --- Motor direction logic based on heading difference ---
            if (diff > turnThreshold) {
                Serial.print(F("Turning RIGHT | Speed: "));
                Serial.println(speed);
                controlMotor(motorDirection == 1 ? 1 : -1, speed); // Right = 1
            } else if (diff < -turnThreshold) {
                Serial.print(F("Turning LEFT | Speed: "));
                Serial.println(speed);
                controlMotor(motorDirection == 1 ? -1 : 1, speed); // Left = -1
            } else {
                stopAllMotors();
            }
        }

        delay(40); // Extra delay for stability (DC motor only)
    }
}


/*
void runCompassMode() {
    if (!useMagnetometer) {
        Serial.println(F("Compass mode skipped: Magnetometer disabled."));
        return;
    }

    compassModeActive = true;
    Serial.println(F("Compass mode started."));
    EEPROM.get(20, motorDirection);

    beepMultiple(2);
    stopAllMotors();

    if (millis() - lastDebugPrintTime >= 2000) {
        Serial.print(F("Initial User Setpoint: "));
        Serial.println(userSetpoint);
        BTserial.print(F("Initial User Setpoint: "));
        BTserial.println(userSetpoint);
        lastDebugPrintTime = millis();
    }

    while (compassModeActive) {
        checkModeExit();

        // Handle serial "set heading"
        if (Serial.available()) {
            char command = Serial.read();
            if (command == 'H') {
                headingPID.SetMode(MANUAL);
                delay(50);
                if (Serial.available()) {
                    userSetpoint = Serial.parseFloat();
                    Serial.print(F("New Heading Setpoint: "));
                    Serial.println(userSetpoint);
                }
                headingPID.SetMode(AUTOMATIC);
            } else {
                handleSerialCommand(command); // Any other commands
            }
        }

        updateHeading(); // This should update currentHeading

        // Calculate shortest way to target heading
        double diff = shortestAngle(userSetpoint, currentHeading);

        // PID control: Always try to make diff = 0
        pidInput = diff;
        pidSetpoint = 0;
        headingPID.Compute();

        // DEBUG print every second
        if (millis() - lastDebugPrintTime >= 1000) {
            Serial.print(F("[DEBUG] Target Heading: "));
            Serial.print(userSetpoint, 2);
            Serial.print(F(" | True Heading: "));
            Serial.print(currentHeading, 2);
            Serial.print(F(" | Diff: "));
            Serial.print(diff, 2);
            Serial.print(F(" | PID Output: "));
            Serial.println(pidOutput, 2);
            lastDebugPrintTime = millis();
        }

        // Safety: If too far off, stop
        if (abs(diff) > 120) {
            stopAllMotors();
            Serial.println(F("[COMPASS] Diff > 120°, motor stopped for safety!"));
            continue;
        }

        // Motor update every 200ms
        static unsigned long lastMotorUpdate = 0;
        const unsigned long motorUpdateInterval = 200;
        const int deadZone = 5;
        static unsigned long lastOnTargetPrint = 0;

        if (millis() - lastMotorUpdate >= motorUpdateInterval) {
            lastMotorUpdate = millis();

            int baseSpeed = 35;
            int maxSpeed = 50;
            static int lastSpeed = 0;
            int targetSpeed = constrain(abs(pidOutput), baseSpeed, maxSpeed);
            int speed = lastSpeed + ((targetSpeed - lastSpeed) / 4);
            lastSpeed = speed;

            // Deadzone – print ON TARGET every second only
            if (abs(diff) <= deadZone) {
                stopAllMotors();
                if (millis() - lastOnTargetPrint > 1000) {
                    Serial.println(F("ON TARGET (within deadZone) - STOPPING"));
                    lastOnTargetPrint = millis();
                }
                continue;
            }

            // Motor direction logic
            if (diff > turnThreshold) {
                Serial.print(F("Turning RIGHT | Speed: "));
                Serial.println(speed);
                controlMotor(motorDirection == 1 ? 1 : -1, speed); // Right = 1
            } else if (diff < -turnThreshold) {
                Serial.print(F("Turning LEFT | Speed: "));
                Serial.println(speed);
                controlMotor(motorDirection == 1 ? -1 : 1, speed); // Left = -1
            } else {
                stopAllMotors();
            }
        }

        delay(40);
    }
}*/

//---------------------
// MANUAL MODE
//---------------------
void runManualMode(bool sw1, bool sw2) {
    static int lastState = -1;  // 0 = STOP, 1 = FORWARD, 2 = BACKWARD
    static unsigned long lastPrintTime = 0;  // Timestamp to limit serial printing

    // STEGMOTOR
    if (motorType == MOTOR_STEPPER) {
        if (!stepperZeroed) {
            // Säg till om nollning krävs
            if (lastState != 99) {
                Serial.println(F("Stegmotor ej nollad! Dubbeltryck pedal först."));
                lastState = 99;
            }
            return;
        }

        int stepDelay = 400;  // microsekunder, kan göras justerbar via meny

        if (sw1 && !sw2) {  // höger
            stepMotor(1, 1, stepDelay);
            if (lastState != 1 && millis() - lastPrintTime >= 1000) {
                Serial.println(F("Stepper Right"));
                lastState = 1;
                lastPrintTime = millis();
            }
        } else if (!sw1 && sw2) { // vänster
            stepMotor(1, -1, stepDelay);
            if (lastState != 2 && millis() - lastPrintTime >= 1000) {
                Serial.println(F("Stepper Left"));
                lastState = 2;
                lastPrintTime = millis();
            }
        } else {
            // inget, ingen rörelse
            if (lastState != 0 && millis() - lastPrintTime >= 1000) {
                lastState = 0;
                lastPrintTime = millis();
            }
        }
        return; // Hoppa över vanlig motorkod!
    }

    // DC-motor (vanliga L298-koden)
    if (sw1 && !sw2) {
        controlMotor(1, manualSpeed);
        if (lastState != 1 && millis() - lastPrintTime >= 1000) {
            Serial.println(F("M Right"));
            lastState = 1;
            lastPrintTime = millis();
        }
    } else if (!sw1 && sw2) {
        controlMotor(-1, manualSpeed);
        if (lastState != 2 && millis() - lastPrintTime >= 1000) {
            Serial.println(F("M Left"));
            lastState = 2;
            lastPrintTime = millis();
        }
    } else {
        stopAllMotors();
        if (lastState != 0 && millis() - lastPrintTime >= 1000) {
            lastState = 0;
            lastPrintTime = millis();
        }
    }
}


// ----------------------------
//   CHECK BUTTON LONG PRESS
// ----------------------------
// Detects if both rotation switches are held for a long duration.
// Different actions are triggered based on how long both buttons are held.
void checkBothLongestPress(bool sw1, bool sw2) {
    static unsigned long lastUpdate = 0;
    static unsigned long holdStartTime = 0;
    static unsigned long lastBeepTime = 0;

    static bool releaseLogged = false;
    static bool longPressTriggered = false;
    static unsigned long firstPressTime = 0;
    static bool waitingForSecondPress = false;

    bool pedalButtonPressed = (digitalRead(pedalButton) == LOW);
    //bool activationPressed = (sw1 && sw2) || pedalButtonPressed;  // Either both buttons OR pedal button
    bool activationPressed = (sw1 && sw2) || isPedalPressed();  // 

    if (!bothPressing && activationPressed) {  
        bothPressing = true;
        holdStartTime = millis();
        lastUpdate = millis();
        lastBeepTime = millis();
        releaseLogged = false;
        longPressTriggered = false;
        waitingForSecondPress = false;
    } 
    else if (bothPressing && !activationPressed) {  
        if (!releaseLogged && !longPressTriggered) {  
            unsigned long held = millis() - holdStartTime;
            Serial.println(F("[DEBUG] Buttons released. Timer reset."));
            releaseLogged = true;
            delay(50);

            if (held >= 8000) {  
                Serial.println(F("[WARNING] 8s"));
                tone(buzzerPin, 1000);
                delay(5000);
                noTone(buzzerPin);
                longPressTriggered = true;
            }
            else if (held >= 5000) {  
                Serial.println(F("[sweep] 5s Speed"));
                beepMultipleDuration(3, 100);
                setMotorSpeed();  
            } 
            else if (held >= 3000) {  
                Serial.println(F("[Compass Mode] 3s Compass"));
                beepMultiple(2);
                runCompassMode();  
            } 
            else if (held >= 300) {  
                Serial.println(F("[sweep] 300ms sweep"));
                beepMultipleDuration(1, 300);
                runsweepmaticMode();                
            }
        }
        bothPressing = false;

          while (digitalRead(rotSw1) == LOW || digitalRead(rotSw2) == LOW || isPedalPressed()) {
              delay(10);
          }

    } 
    else if (bothPressing) {  
        unsigned long heldTime = millis() - holdStartTime;

        if (heldTime < 8000 && millis() - lastBeepTime >= 1000) {  
            tone(buzzerPin, 1000);
            delay(100);
            noTone(buzzerPin);
            lastBeepTime = millis();
        }

        if (millis() - lastUpdate >= 500) {  
            Serial.print(F("[DEBUG] Held Time: "));
            Serial.print(heldTime / 1000.0, 1);
            Serial.println(F("s"));
            lastUpdate = millis();
        }
    }
}

//----------------------------------
// CHECK PEDAL BUTTON DOUBLE PRESS
//----------------------------------
void checkPedalDoublePress() {
  static bool pedalPressedLast = false;
  bool pedalNow = (digitalRead(pedalButton) == LOW);

  if (pedalNow && !pedalPressedLast) {  // Nytt tryck
    unsigned long now = millis();
    if (!waitingForSecondPress) {
      // Första trycket
      firstPedalPress = now;
      waitingForSecondPress = true;
      // Kanske: blink eller beep 1 gång?
    } else {
      // Andra trycket
      if (now - firstPedalPress <= doublePressTimeout) {
        // Dubbeltryck godkänd! Nollställ
        stepperPos = 0;
        stepperZeroed = true;
        Serial.println(F("Stegmotor nollställd!"));
        // Beep eller annan feedback här!
      }
      waitingForSecondPress = false;
    }
    lastPedalPress = now;
  }

  // Timeout? Släpp "dubbeltryck" om det gått för länge
  if (waitingForSecondPress && (millis() - firstPedalPress > doublePressTimeout)) {
    waitingForSecondPress = false;
  }

  pedalPressedLast = pedalNow;
}

// ----------------------------
//   SET MOTOR SPEED (FOR MANUAL AND SWEEP MODE) ADDED PEDALBUTTON FUNKTION.
// ----------------------------
void setMotorSpeed() {
    Serial.println(F("** Speed Adjustment  **"));
    stopAllMotors();

    bool exitMenu = false;
    bool adjustingSweepSpeed = false;  // True = Adjusting sweep speed, False = Adjusting manual speed
    bool adjustingManualSpeed = true;  // Default to manual speed adjustment
    unsigned long lastButtonPress = 0;
    unsigned long holdStart = 0;
    unsigned long pedalHoldStart = 0; // Track pedal button hold time
    unsigned long modeSwitchHoldStart = 0; // Track hold time for mode switch using buttons
    const unsigned long buttonDebounce = 300;
    const unsigned long exitHoldTime = 3000;  // Hold 3 seconds to exit
    const unsigned long switchModeHoldTime = 1000; // Hold 1 second to switch mode
    const int minSpeed = 50;
    const int maxSpeed = 250;

    Serial.println(F("MANUAL SPEED"));

    while (!exitMenu) {
      bool sw1 = (digitalRead(rotSw1) == LOW);
      bool sw2 = (digitalRead(rotSw2) == LOW);
      bool pedalPressed = isPedalPressed();
      bool switchModePressed = (sw1 && sw2) || pedalPressed;

      if (checkHoldToExit(holdStart)) {
        Serial.println(F("Exiting Speed Menu"));
        delay(1000);
        break;
      }
        // **Switch Between Manual Mode & Sweep Mode (Hold Pedal OR Both Buttons for 1 Second)**
        if (switchModePressed) {
            if (modeSwitchHoldStart == 0) {
                modeSwitchHoldStart = millis();
            } else if (millis() - modeSwitchHoldStart >= switchModeHoldTime) {
                adjustingSweepSpeed = !adjustingSweepSpeed;
                adjustingManualSpeed = !adjustingSweepSpeed;

                Serial.println(adjustingSweepSpeed ? "Switched to adjusting: SWEEP MODE SPEED" : "Switched to adjusting: MANUAL MODE SPEED");
                beepTriple();  // Three quick beeps for mode switch
                delay(500);  // Prevent accidental double-switch
                modeSwitchHoldStart = 0; // Reset mode switch hold start
            }
        } else {
            modeSwitchHoldStart = 0; // Reset when released
        }

        // **Increase Speed** (Stops at max value)
        if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
            lastButtonPress = millis();

            if (adjustingSweepSpeed) {
                if (sweepSpeed < maxSpeed) {
                    sweepSpeed += 25;
                    myMotor->setSpeed(sweepSpeed);
                    EEPROM.write(1, sweepSpeed);
                    Serial.println(sweepSpeed);
                    beepQuickDouble();
                } else {
                    Serial.println(F("MAX (250)"));
                    beepMinMaxAlert();
                }
            } else if (adjustingManualSpeed) {
                if (manualSpeed < maxSpeed) {
                    manualSpeed += 25;
                    myMotor->setSpeed(manualSpeed);
                    EEPROM.write(0, manualSpeed);
                    Serial.println(manualSpeed);
                    beepLong();
                } else {
                    Serial.println(F("MAX (250)"));
                    beepMinMaxAlert();
                }
            }
        }

        // **Decrease Speed** (Stops at min value)
        if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
            lastButtonPress = millis();

            if (adjustingSweepSpeed) {
                if (sweepSpeed > minSpeed) {
                    sweepSpeed -= 25;
                    myMotor->setSpeed(sweepSpeed);
                    EEPROM.write(1, sweepSpeed);
                    Serial.println(sweepSpeed);
                    beepQuickDouble();
                } else {
                    Serial.println(F("MIN (50)"));
                    beepMinMaxAlert();
                }
            } else if (adjustingManualSpeed) {
                if (manualSpeed > minSpeed) {
                    manualSpeed -= 25;
                    myMotor->setSpeed(manualSpeed);
                    EEPROM.write(0, manualSpeed);
                    Serial.println(manualSpeed);
                    beepLong();
                } else {
                    Serial.println(F("MIN (50)"));
                    beepMinMaxAlert();
                }
            }
        }
        delay(200);  // Prevent sensitive navigation
    }
}

// ----------------------------
//   STOP MOTORS
// ----------------------------
void stopAllMotors() {
      lastPwmValue = 0; // Reset Value
    if (driverMode == 1) { // L298P
        analogWrite(EN, 0);
    } else if (driverMode == 2 && myMotor) { // L298N
        myMotor->stop();
    }
    //Serial.println(F("[System] Motors Stopped."));
}

// ----------------------------
//   HALL SENSOR 
// ----------------------------
bool isSensorActive() {
  if (useHallSensor) {
    return digitalRead(hallSensorPin) == LOW;
  } else {
    return digitalRead(posSw) == LOW;
  }
}

// ----------------------------
//   Check Exit Mode
// ----------------------------
void checkModeExit() {
    bool sw1 = (digitalRead(rotSw1) == LOW); // Read first switch state
    bool sw2 = (digitalRead(rotSw2) == LOW); // Read second switch state
    bool pedal = (digitalRead(pedalButton) == LOW); // Read pedal state

    if (sw1 || sw2 || pedal) {
        static unsigned long pressStart = 0;
        if (pressStart == 0) pressStart = millis(); // Start timing when button is pressed

        if (millis() - pressStart >= 300) { // If held for 300ms, exit mode
            if (compassModeActive) { 
                compassModeActive = false;
                setHeading = 0; // Reset heading setpoint
                headingPID.SetMode(MANUAL); // Disable PID
                Serial.println(F("Compass Mode Deactivated"));
            }
            if (sweepModeActive) {
                sweepModeActive = false;
                Serial.println(F("Sweep Mode Deactivated"));
            }
            stopAllMotors(); // Stop the motors
            pressStart = 0; // Reset timer
        }
    } else {
        static unsigned long pressStart = 0;
        pressStart = 0; // Reset timer if no buttons are pressed
    }
}

// ----------------------------
//   BEEP FUNCTIONS
// ----------------------------
void beepMultiple(int n) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000);
    delay(250);
    noTone(buzzerPin);
    delay(250);
  }
}
// Beep function for short/long signals
void beepMultipleDuration(int n, int duration) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000);
    delay(duration);
    noTone(buzzerPin);
    delay(200);
  }
}
// sweep Mode: Two quick beeps
void beepQuickDouble() {
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
}
// Manual Mode: One long beep
void beepLong() {
  tone(buzzerPin, 1000);
  delay(400);
  noTone(buzzerPin);
}
// Alert when reaching MIN or MAX speed (2 quick + 2 long beeps)
void beepMinMaxAlert() {
  // Two quick beeps
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  delay(200); // Small pause

  // Two long beeps
  tone(buzzerPin, 1000);
  delay(400);
  noTone(buzzerPin);
  delay(200);
  tone(buzzerPin, 1000);
  delay(400);
  noTone(buzzerPin);
}
// Three quick beeps when switching mode
void beepTriple() {
  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, 1000);
    delay(100);
    noTone(buzzerPin);
    delay(100);
  }
}