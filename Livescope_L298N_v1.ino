#pragma GCC optimize ("Os")  // Optimerar för minsta möjliga storlek
// GENERAL INFO
// Forward + Backward is needed for the L298 library, it dont have left/right. 


#include <Arduino.h>
#include <L298N.h>  
#include <EEPROM.h>
#include <SoftwareSerial.h> // Används för att hantera seriell kommunikation
#include <MotorDriver.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <PID_v1.h>

// Initialize HMC5883L magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Variables for PID control
double setHeading = 0;   // Target heading (user sets this)
double currentHeading = 0; // Current heading
double output = 0;       // PID output for motor control
double turnThreshold = 4;  // Standardvärde för svänggräns, kan ändras via Bluetooth

// PID tuning constants (adjust as needed)
double Kp = 6.0, Ki = 0.1, Kd = 0.5;
PID headingPID(&currentHeading, &output, &setHeading, Kp, Ki, Kd, DIRECT);

// GYRO Variables
double filteredHeading = 0;       // Håller det filtrerade värdet
const double alpha = 0.5;         // Filterfaktor (0.0-1.0) – högre = snabbare respons
double rawHeading = 0;            // Lagrar råvärdet


//SoftwareSerial BTserial(3, 2); // HC-05: TX på 3, RX på 4
//SoftwareSerial BTserial(2, 3); // Now pin 2 and pin 3 of Arduino are Serial Rx & Tx pin Respectively
SoftwareSerial BTserial(3, 2); // RX | TX

int driverMode = 1; // Choose between 1 (L298P) or 2 (L298N);

// ----------------------------
//   PIN CONFIGURATION
// ----------------------------
int EN, IN1, IN2, posSw, rotSw1, rotSw2, buzzerPin;
L298N* myMotor = nullptr;

int pedalButton = 5; // Button on the printed pedal or somewhere that will be used when you don't have the possibility to press both buttons left+right

const long baudRate = 38400;
char c = ' ';
boolean NL = true;

// ----------------------------
//   GLOBAL VARIABLES
// ----------------------------
bool sweepModeActive   = false;
bool compassModeActive = false;
bool bothPressing = false;
bool pressingsweep = false;
bool countingDoublePress = false;

int  sweep_angle   = 7;    
int manualSpeed = 125;  // Default speed for manual mode
int sweepSpeed = 100;    // Default speed for sweep mode
int motorDirection; // Direction of the motor compared to the gyro

unsigned long lastStopPrintTime = 0;
unsigned long lastPosSwPress = 0;
unsigned long posSwDebounceTime = 400;  
unsigned long lastPressTime = 0;
unsigned long doublePressStartTime = 0;
unsigned long pressTimesweep = 0;
unsigned long bothPressStart = 0;
unsigned long lastDebugPrintTime = 0;  // Timer för debugutskrifter


//myMotor = new L298N(EN, IN1, IN2);

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
      savePIDValues(); // Sparar defaultvärden
    }

    headingPID.SetTunings(Kp, Ki, Kd);
}

//----------------------------------
// GYRO FUNCTIONS
//----------------------------------
/*void stabilizeCompass(int samples = 5, int delayMs = 200) {
    double sum = 0;
    for (int i = 0; i < samples; i++) {
        updateHeading();
        sum += currentHeading;
        delay(delayMs);  // Vänta mellan mätningarna
    }
    currentHeading = sum / samples;  // Använd snittet som startvärde
}*/

// Function to update compass heading using both X and Y axes
void updateHeading() {
    sensors_event_t event;
    if (!mag.getEvent(&event)) {  // Om sensorn inte svarar
        Serial.println("Magnetometer error! Reinitializing...");
        mag.begin();  // Starta om magnetometern
        return;
    }

    double newHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (newHeading < 0) newHeading += 360;

    // Kolla om värdet faktiskt förändrats
    if (abs(newHeading - currentHeading) > 0.01) {
        currentHeading = newHeading;
        Serial.print("Updated Heading: ");
        Serial.println(currentHeading);
    }
}



/*void updateHeading() {
    sensors_event_t event;
    mag.getEvent(&event);

    double newHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (newHeading < 0) newHeading += 360;

    rawHeading = newHeading;  // Sparar det råa värdet globalt

    const double alpha = 0.8;  // Snabbt filter
    currentHeading = (alpha * newHeading) + ((1 - alpha) * currentHeading);
}*/

/*void waitForStableHeading(int attempts = 10, int delayMs = 100) {
    double lastHeading = 0;
    for (int i = 0; i < attempts; i++) {
        updateHeading();
        if (abs(currentHeading - lastHeading) < 1.0) {  // Om förändringen är mindre än 1 grad
            break;
        }
        lastHeading = currentHeading;
        delay(delayMs);  // Vänta innan nästa avläsning
    }
}*/

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

void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  loadSettings();
  loadPIDValues();

  EEPROM.get(20, motorDirection); // Load motor direction from calibration saved in EEPROM
  if (motorDirection != 1 && motorDirection != -1) {
      motorDirection = 1; // Default if invalid data
  }
  Serial.print("M.Dir ");
  Serial.println(motorDirection);

  // Initialize magnetometer
  if (!mag.begin()) {
      Serial.println("HMC5883L na");
      while (1);
  }
  Serial.println("HMC5883L ok");

  // Initialize PID controller
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-255, 255); // Motor PWM range

  // Select motor driver and corresponding I/O configuration, different depending on the motor driver
  // Motor Driver is set above at "int driverMode = 1;"
  if (driverMode == 1) { // Shield board "Aliexpress board" L298NH/L298P
    Serial.println("L298P");
    IN1 = 12;  // Direction (HIGH = Forward, LOW = Backward)
    EN  = 3;   // PWM control for speed
    IN2 = 9;   // Brake
    posSw   = 7; // Rotation position pin
    rotSw1  = 6; // Turn right
    rotSw2  = 11; // Turn left
    buzzerPin = 10;
  } else if (driverMode == 2) { // External boarn L298H 
      Serial.println("L298N");
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
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  checkBothLongestPress(sw1, sw2);
  checkModeExit();

    if (!sweepModeActive && !compassModeActive) {
        runManualMode(digitalRead(rotSw1) == LOW, digitalRead(rotSw2) == LOW);
    }

        static unsigned long lastHeadingCheck = millis();
        static double previousHeading = 0;

      if (millis() - lastHeadingCheck > 5000) {  // Kollar var 5:e sekund
          if (abs(currentHeading - previousHeading) < 0.1) {  // Ingen märkbar förändring
              Serial.println("Heading still stuck! Reinitializing magnetometer...");
              mag.begin();
          }
          previousHeading = currentHeading;
          lastHeadingCheck = millis();
      }

  if (Serial.available()) {
    char command = Serial.read();
    handleSerialCommand(command);
  }
  delay(200);  // För att undvika att spamma seriell monitor
}

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
//   SWEEPING DEGREE
// ----------------------------
void setSweepAngle() {
  Serial.println("** setSweepAngle MENU **");
  //Serial.print("SweepAngle: ");
  //Serial.println(sweep_angle);
  stopAllMotors();

  bool exitMenu = false;
  unsigned long lastButtonPress = 0;
  const unsigned long buttonDebounce = 300;

  while (!exitMenu) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);
    // Vänta kort innan enkeltryck registreras, ifall det är ett dubbeltryck
    if (sw1 || sw2) {
      unsigned long pressStart = millis();
      delay(150);  // Vänta 150ms för att se om en andra knapp också trycks
      sw1 = (digitalRead(rotSw1) == LOW);
      sw2 = (digitalRead(rotSw2) == LOW);
    }

    // Öka sweep_angle
    if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      sweep_angle = (sweep_angle >= 8) ? 2 : sweep_angle + 1;
      //Serial.print("New Sweep Angle: ");
      Serial.println(sweep_angle);
      beepMultiple(1);
    }

    // Minska sweep_angle
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      sweep_angle = (sweep_angle <= 2) ? 8 : sweep_angle - 1;
      //Serial.print("New Sweep Angle: ");
      Serial.println(sweep_angle);
      beepMultiple(1);
    }

    // Lämna menyn genom att hålla båda knapparna i 1 sekund
    if (sw1 && sw2) {
      unsigned long pressStart = millis();
      while (sw1 && sw2) {
        if (millis() - pressStart >= 1000) {
          //Serial.println("Exiting setSweepAngle...");
          beepMultiple(2);  // Exit confirmation
          delay(2000);  // ✅ Add delay to prevent unintended single-clicks after exit
          // ✅ Wait until both buttons are fully released
          while ((digitalRead(rotSw1) == LOW) || (digitalRead(rotSw2) == LOW)) {
            // Do nothing, just wait
          }
          exitMenu = true;
          break;
        }
    sw1 = (digitalRead(rotSw1) == LOW);
    sw2 = (digitalRead(rotSw2) == LOW);
      }
    }
  }
}

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
      break;

    case '-':  // Decrease Manual Speed
      saveSpeed(0, manualSpeed, max(manualSpeed - 25, 25));  // Prevent going below 25
      myMotor->setSpeed(manualSpeed);
      break;

    case 'U':  // Increase sweep Speed
      saveSpeed(1, sweepSpeed, min(sweepSpeed + 25, 250));  // Prevent exceeding 250
      break;

    case 'H':  // Decrease sweep Speed
      saveSpeed(1, sweepSpeed, max(sweepSpeed - 25, 25));  // Prevent going below 25
      break;

    case 'C':  // Deactivate Compass Mode
        compassModeActive = false;
        stopAllMotors();
        break;

    case 'A':  
      sweepModeActive = true;
      break;

    case 'M':  
      sweepModeActive = false;
      stopAllMotors();
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

    /*case 'O':  // Request all current settings
      BTserial.println("=== Current System Status ===");
      BTserial.print("Driver Mode: ");
      BTserial.println(driverMode == 1 ? "L298P" : "L298N");
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      BTserial.print("Sweep Speed: ");
      BTserial.println(sweepSpeed);
      BTserial.print("Sweep Mode: ");
      BTserial.println(sweepModeActive ? "Active" : "Inactive");
      BTserial.print("Current Heading: ");
      BTserial.println(currentHeading);
      BTserial.print("Target Heading: ");
      BTserial.println(setHeading);
      BTserial.print("Kp: ");
      BTserial.print(Kp);
      BTserial.print(", Ki: ");
      BTserial.print(Ki);
      BTserial.print(", Kd: ");
      BTserial.println(Kd);
      BTserial.println("=== End of Report ===");
      Serial.println("[BT] Sent system status report");
    break;
      
    default:
      //Serial.println("[BT] Unknown Command");
      //BTserial.println("Unknown Command");
      break;*/
  }
}
/*=== BLUETOOTH -- Current System Status ===
Driver Mode: L298N
Manual Speed: 125
Sweep Speed: 100
Sweep Mode: Inactive
Current Heading: 135.7
Target Heading: 180.0
Kp: 2.5, Ki: 0.1, Kd: 0.5
=== End of Report ===*/

void handleSerialCommand(char command) {
  switch (command) {
    case 'C':  // Aktivera Compass Mode
      delay(50);  // Ge Serial lite tid att läsa hela värdet
      runCompassMode();
      Serial.println("Compass Mode Activated");
      break;

      case 'H':  // Sätt Heading (ex: H90.0 för 90 grader)
        headingPID.SetMode(MANUAL);  // Pausa PID
        delay(50);  // Låt Serial hinna läsa
        if (Serial.available()) {
          setHeading = Serial.parseFloat();
          Serial.print("New Heading Setpoint: ");
          Serial.println(setHeading);
        }
        headingPID.SetMode(AUTOMATIC);  // Återaktivera PID
        break;

    case 'P':  // Justera Kp (ex: P4.5)
      if (Serial.available()) {
        Kp = Serial.parseFloat();
        headingPID.SetTunings(Kp, Ki, Kd);
        savePIDValues();
        Serial.print("Kp set to: ");
        Serial.println(Kp);
      }
      break;

    case 'I':  // Justera Ki (ex: I0.2)
      if (Serial.available()) {
        Ki = Serial.parseFloat();
        headingPID.SetTunings(Kp, Ki, Kd);
        savePIDValues();
        Serial.print("Ki set to: ");
        Serial.println(Ki);
      }
      break;

    case 'D':  // Justera Kd (ex: D1.0)
      if (Serial.available()) {
        Kd = Serial.parseFloat();
        headingPID.SetTunings(Kp, Ki, Kd);
        savePIDValues();
        Serial.print("Kd set to: ");
        Serial.println(Kd);
      }
      break;

    case 'S':  // STOP ALL FUNCTIONS
        sweepModeActive = false;    // Deactivate Sweep Mode
        compassModeActive = false;  // Deactivate Compass Mode
        setHeading = 0;             // Reset setpoint
        stopAllMotors();
        Serial.println("Sweep + Compass deactivated");
        break;


    case 'O':  // Visa status
      updateHeading();
      Serial.print("Current Heading: ");
      Serial.println(currentHeading);
      Serial.print("Target Heading: ");
      Serial.println(setHeading);
      break;

    case 'Q':
        calibrateCompassMotorDirection(); // New command to calibrate direction
        break;

    while (Serial.available()) Serial.read();  // Clear buffer
    Serial.println("Unknown Command");
    break;
  }
}

// ----------------------------
//   COMPASS CALIBRATION
// ----------------------------
// Function to calibrate motor direction for compass mode
// This function runs a short motor movement and checks which direction reduces the error to the set heading.
void calibrateCompassMotorDirection() {
    int testDirection = 1; // Default test direction

    Serial.println("Cal. Dir");
    updateHeading();
    double startHeading = currentHeading;
    Serial.print("startHeading: ");
    Serial.println(startHeading);

    unsigned long startTime = millis();
    while (millis() - startTime < 6000) {
    controlMotor(1, 50);  // Move motor forward at speed 50
    }   

    //delay(6000); // Let motor turn
    updateHeading();
    double endHeading = currentHeading;

    stopAllMotors();
    Serial.print("startHeading: ");
    Serial.println(startHeading);
    Serial.print("endHeadingd: ");
    Serial.println(endHeading);

    // Determine motor direction based on heading change
    if (endHeading < startHeading) { // SETS WHEN LEFT IS LEFT AND RIGHT IS RIGHT
        testDirection = 1;
    } else {
        testDirection = -1;
    }
    
    Serial.print("Move back");
    // Move motor back to starting position
    unsigned long returnstartTime = millis();
    while (millis() - returnstartTime < 6000) {
    controlMotor(-1, 50);  // Move motor forward at speed 50
    }

    //delay(6000); // Move back for the same duration
    stopAllMotors();

    EEPROM.put(20, testDirection); // Store direction in EEPROM
    Serial.print("Motor Direction Calibrated: ");
    Serial.println(testDirection);
    if (testDirection == 1) {
      Serial.print("Left is decrease degrees");
       } else {
        Serial.print("Right is decrease degrees");
      }  
}

// ----------------------------
//   SOMETHING FO THE FUTURE OF DEBOUNCE
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
void runsweepmaticMode() {
    sweepModeActive = true;
    beepMultiple(0); // Initial beep signal to indicate sweep mode start
    stopAllMotors(); // Ensure motors are stopped before starting the sweep
    delay(1000); // Short delay for stabilization

    // Move motor backward until position switch (posSw) is triggered, indicating starting point
    controlMotor(-1, sweepSpeed); // Move motor backward at sweep speed
    while (digitalRead(posSw) == HIGH) { // Wait until position switch is pressed
        checkModeExit(); // Continuously check if the user wants to exit
        if (!sweepModeActive) return; // Exit sweep mode if deactivated
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

            if (digitalRead(posSw) == LOW && millis() - lastPosSwPress > posSwDebounceTime) { // Detect position switch activation
                lastPosSwPress = millis(); // Update last press time to avoid double counting
                posCount++; // Increase position counter
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

              if (digitalRead(posSw) == LOW && millis() - lastPosSwPress > posSwDebounceTime) { // Detect position switch activation
                  lastPosSwPress = millis(); // Update last press time
                  posCount--; // Decrease position counter
              }
          }

        stopAllMotors(); // Stop motors after returning to start
        delay(1000); // Delay to ensure motor is fully stopped before next cycle
    }
}

// ----------------------------
//   Compass Mode 
// ----------------------------
void runCompassMode() {
    compassModeActive = true;
    Serial.print("compass Mode");
    EEPROM.get(20, motorDirection); // Load motor direction from EEPROM

    beepMultiple(2);
    stopAllMotors();
    
   /* //waitForStableHeading(10, 100);  // Vänta tills heading är stabil
    setHeading = currentHeading;    // Använd stabilt värde som setpoint
    Serial.print("Initial Stable Heading: ");
    Serial.println(setHeading);

    //stabilizeCompass(5, 200);  // Läs in 5 stabila värden med 200ms mellanrum
    setHeading = currentHeading;
    Serial.print("Initial Stable Heading: ");
    Serial.println(setHeading);*/

      if (millis() - lastDebugPrintTime >= 2000) {
        Serial.print("Initial Stable Heading: ");
        Serial.println(setHeading);
        BTserial.print("Initial Stable Heading: ");
        BTserial.println(setHeading);    
      lastDebugPrintTime = millis();
  }

    bool pressingExit = false;
    unsigned long pressStartTime = 0;
    unsigned long lastPrintTime = 0;  // Timer for limiting serial output

    while (compassModeActive) {
    checkModeExit(); // Check for exit command with buttons
    
    if (Serial.available()) {
        char command = Serial.read();
        handleSerialCommand(command);  // Hanterar alla kommandon, inklusive 'S'
    }

    if (!compassModeActive) break; // Exit loop if deactivated

    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'H') {
            headingPID.SetMode(MANUAL);
            delay(50);
            if (Serial.available()) {
                setHeading = Serial.parseFloat();
                Serial.print("New Heading Setpoint: ");
                Serial.println(setHeading);
            }
            headingPID.SetMode(AUTOMATIC);
        }
    }

      updateHeading();
      headingPID.Compute();  // Compute PID correction

      delay(300);  // Small delay for stability

      // Timer för att skicka motoruppdateringar
      static unsigned long lastMotorUpdate = 0;
      const unsigned long motorUpdateInterval = 300; // Uppdatera var 500ms
      const int deadZone = 2; // If the error is within ±8 degrees, don't move

  if (millis() - lastMotorUpdate >= motorUpdateInterval) {
      lastMotorUpdate = millis();  // Uppdatera timer
      
      if (millis() - lastDebugPrintTime >= 1000) {  
            Serial.print("Setpoint: ");
            Serial.print(setHeading);
            Serial.print(" | Current: ");
            Serial.print(currentHeading);
            Serial.print(" | Raw: ");
            Serial.print(rawHeading);  // Skriv ut råvärdet
            Serial.print(" | OUTPUT to motor: ");
            Serial.println(output); 
        lastDebugPrintTime = millis();
      }

      int baseSpeed = 45;  // Lägsta hastighet
      int maxSpeed =100;   // Högsta hastighet
      //int speed = constrain(abs(output), baseSpeed, maxSpeed); // Begränsar hastighet
      static int lastSpeed = 0;
      int targetSpeed = constrain(abs(output), baseSpeed, maxSpeed);
      int speed = lastSpeed + ((targetSpeed - lastSpeed) / 2);  // Mjuk ökning
      lastSpeed = speed;

      // If within deadzone nothing = Wait
      if (abs(setHeading - currentHeading) <= deadZone) {
          if (millis() - lastDebugPrintTime >= 1000) {
          Serial.println("ON TARGET (within deadZone) - STOPPING");
          lastDebugPrintTime = millis();
          }
          stopAllMotors();
          continue; // Hoppa över resten av loopen för denna iteration
         }

         if (output > turnThreshold || output < -turnThreshold) {

              // Drive motor based on PID output and calibrated direction
              if (output > 0) {  // Turn right
                  Serial.print("Counting Pos down | Speed: ");
                  Serial.println(speed);
                  controlMotor(motorDirection == 1 ? 1 : -1, speed);  // Forward if direction is 1, else backward
              } else if (output < 0) {  // Turn left
                  Serial.print("Counting Pos Up | Speed: ");
                  Serial.println(speed);
                  controlMotor(motorDirection == 1 ? -1 : 1, speed);  // Backward if direction is 1, else forward
              }
            }
           } else {
            stopAllMotors();  // Stop motors if no significant output
            Serial.println("[COMPASS] Output Zero - Motors Stopped");
        }
      }
}

// ----------------------------
//   Manual Mode 
// ----------------------------
void runManualMode(bool sw1, bool sw2) {
    static int lastState = -1;  // 0 = STOP, 1 = FORWARD, 2 = BACKWARD
    static unsigned long lastPrintTime = 0;  // Timestamp to limit serial printing

    if (sw1 && !sw2) {  // Move forward
        controlMotor(1, manualSpeed);  // Forward direction
        if (lastState != 1 && millis() - lastPrintTime >= 1000) { 
            Serial.println("M Right");
            lastState = 1;
            lastPrintTime = millis();
        }

    } else if (!sw1 && sw2) {  // Move backward
        controlMotor(-1, manualSpeed);  // Backward direction
        if (lastState != 2 && millis() - lastPrintTime >= 1000) { 
            Serial.println("M Left");
            lastState = 2;
            lastPrintTime = millis();
        }

    } else {  // Stop motor
        stopAllMotors();  // Stop using the existing stop function
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
            Serial.println("[DEBUG] Buttons released. Timer reset.");
            releaseLogged = true;
            delay(50);

            if (held >= 8000) {  
                Serial.println("[WARNING] 8s");
                tone(buzzerPin, 1000);
                delay(5000);
                noTone(buzzerPin);
                longPressTriggered = true;
            }
            else if (held >= 5000) {  
                Serial.println("[sweep] 5s Speed");
                beepMultipleDuration(3, 100);
                setMotorSpeed();  
            } 
            else if (held >= 3000) {  
                Serial.println("[Compass Mode] 3s Compass");
                beepMultiple(2);
                runCompassMode();  
            } 
            else if (held >= 300) {  
                Serial.println("[sweep] 300ms sweep");
                beepMultipleDuration(1, 300);
                runsweepmaticMode();                
            }
        }
        bothPressing = false;

        while (digitalRead(rotSw1) == LOW || digitalRead(rotSw2) == LOW || pedalButtonPressed) {
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
            Serial.print("[DEBUG] Held Time: ");
            Serial.print(heldTime / 1000.0, 1);
            Serial.println("s");
            lastUpdate = millis();
        }
    }
}

// ----------------------------
//   SET MOTOR SPEED (FOR MANUAL AND SWEEP MODE) ADDED PEDALBUTTON FUNKTION.
// ----------------------------
void setMotorSpeed() {
    Serial.println("** Speed Adjustment  **");
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

    Serial.println("MANUAL SPEED");

    while (!exitMenu) {
        bool sw1 = (digitalRead(rotSw1) == LOW);
        bool sw2 = (digitalRead(rotSw2) == LOW);
        bool pedalPressed = (digitalRead(pedalButton) == LOW);  // Read pedal button state
        bool exitPressed = (sw1 && sw2) || pedalPressed;  // Exit trigger: Either both buttons OR pedal
        bool switchModePressed = (sw1 && sw2) || pedalPressed; // Mode switch trigger

        // **Exit Menu (Hold Pedal OR Both Buttons for 3 Seconds)**
        if (exitPressed) {
            if (holdStart == 0) {
                holdStart = millis();
            } else if (millis() - holdStart >= exitHoldTime) {
                Serial.println("Exiting");
                beepMultiple(2);  // Exit confirmation beep
                exitMenu = true;
                break;
            }
        } else {
            holdStart = 0;  // Reset hold tracking when buttons are released
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
                    Serial.println("MAX (250)");
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
                    Serial.println("MAX (250)");
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
                    Serial.println("MIN (50)");
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
                    Serial.println("MIN (50)");
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
    if (driverMode == 1) { // L298P
        analogWrite(EN, 0);
    } else if (driverMode == 2 && myMotor) { // L298N
        myMotor->stop();
    }
    //Serial.println("[System] Motors Stopped.");
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
                Serial.println("Compass Mode Deactivated");
            }
            if (sweepModeActive) {
                sweepModeActive = false;
                Serial.println("Sweep Mode Deactivated");
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
