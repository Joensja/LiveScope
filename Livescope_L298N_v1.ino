/*
===============================================================================
Project: Motorized Control System for a live sonar fish finder with L298N/L298P, Compass, and Bluetooth
Author: [Joens]
Date: [Early 2025]
===============================================================================

Description:
This Arduino-based system controls a **motorized pole for live transducers** using an **L298N** or **L298P** motor driver. 
It supports **manual operation**, **automatic sweeping mode**, and a **compass-guided mode(Heading)** using an 
**HMC5883L magnetometer**. Bluetooth commands enable remote adjustments, and a **dedicated pedal button** 
enhances usability.
Made for a pedal with two buttons(boght) that can be pressed at the same time or a pedal with one button at the time and a additional third button(pedalButton, 3D-printed)

Whats needed to build this? 
https://cults3d.com/en/3d-model/gadget/electric-live-sonar-pole-auto-sweep-arduino-control
===============================================================================
Key Features:
-------------------------------------------------------------------------------
✔ **Motor Control** (L298N/L298P) 
   - Right & Left movement (Called forward and backwards due to driver logic. Forwards is left.)
   - PWM speed control stored after reboot.
   - Supports manual, Sweep and Compass mode.

✔ **Compass Mode** (HMC5883L Magnetometer)
   - Uses a **PID controller** to hold a specific heading. (NOT TUNED)
   - Changes the speed depending on the PID fault. (NOT TUNED)
   - Adjustable **PID parameters** in code or Bluetooth(`Kp`, `Ki`, `Kd`).
   - Activated via pedal press or **Bluetooth command (`C`)**.

✔ **Sweep Mode**
   - Automated Sweeping between set angles.
   - Adjustable sweep angle & sweep speed.
   - Starts to turn to the left until first DIP-Switch activation and will after that start to turn to the right. You start your sweeping from where you are when sweeping is activated. . 

✔ **Manual Control**
   - Two buttons (`rotSw1`, `rotSw2`) for navigation.
   - The two buttons are the two pedal buttons.These are also used to activate, deactivate and do settings.
   - Pedal button (`pedalButton`)** for double-press actions for when you cant press both the pedals at the same time. Pressing this pedalButton is like pressing both the pedals.

✔ **Settings**
   - Two buttons `rotSw1`, `rotSw2`are the two pedal buttons.These are used to activate, deactivate and do settings.
   - Press and hold "more than" -300ms = Sweep mode active. 3000ms = Activate compass mode. 5000ms = Set sweep angle. 8000ms = Set motor speed manual/sweep(Hold down both to switch)
   - Pedal button (`pedalButton`)** for double-press actions for when you cant press both the pedals at the same time. Pressing this pedalButton is like pressing both the pedals.

✔ **Bluetooth Remote Control**
   - Send commands (`L`, `R`, `S`, `A`, `M`, etc.).
   - Adjust speeds (`+`, `-`, `U`, `H` for sweep speed).
   - **Live system status request (`O`)**.

✔ **Memory & Configuration**
   - **Stores speed & PID settings in EEPROM**.
   - Improved **debounce handling** for input stability.
   - **Optimized memory usage**, but close to full capacity.

===============================================================================
Notes:
- **Modify `driverMode` to 1 (L298P) or 2 (L298N)** as per hardware.
- Default speeds: **Manual (125), Sweep (100)**.
- Keep an eye on **EEPROM usage**, as memory is near full.
===============================================================================
To be added or changed: 
- Manual relay board configuration. So it can be used without a motor driver. 
- DONE! Memory usage fixes (94% now)
- Remove Debug comments in serial manager
- Clean up and sort the code properly
===============================================================================
Testing needed for: 
- DONE! Gyro. Note tested at all, no prio due to bad weather.
- TEST GYRO ON A MOTOR AND SEE HOW IT REACTS. TRY TO SET THE REGULATOR
- Bluetooth. Not tested at all due to hardware failure. 
- DONE! Deeper general testing due to... it's boring and I need to sleep.
===============================================================================
*/

// GENERAL INFO
// Forward + Backward is needed for the L298 library, it dont have left/right. 


#include <Arduino.h>
#include <L298N.h>  
#include <EEPROM.h>
#include <SoftwareSerial.h> // Används för att hantera seriell kommunikation
#include <MotorDriver.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <PID_v1.h>

// Initialize HMC5883L magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Variables for PID control
double setHeading = 0;   // Target heading (user sets this)
double currentHeading = 0; // Current heading
double output = 0;       // PID output for motor control
double turnThreshold = 10;  // Standardvärde för svänggräns, kan ändras via Bluetooth

// PID tuning constants (adjust as needed)
double Kp = 7.5, Ki = 0.1, Kd = 0.5;
PID headingPID(&currentHeading, &output, &setHeading, Kp, Ki, Kd, DIRECT);


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
int  sweep_angle   = 7;    
// int  currentSpeed     = 125;   // OLD default for speed
int manualSpeed = 125;  // Default speed for manual mode
int sweepSpeed = 100;    // Default speed for sweep mode
bool bothPressing = false;
unsigned long bothPressStart = 0;
bool pressingsweep = false;
unsigned long pressTimesweep = 0;
bool compassModeActive = false;  //
bool motorIsStopped = false;  // Global flag to track motor state


// Debounce och dubbeltryck-räknare
unsigned long lastStopPrintTime = 0;
unsigned long lastPosSwPress = 0;
unsigned long posSwDebounceTime = 400;  
unsigned long lastPressTime = 0;
unsigned long doublePressStartTime = 0;
bool countingDoublePress = false;

//myMotor = new L298N(EN, IN1, IN2);

// ----------------------------
//   FUNCTION DECLARATIONS
// ----------------------------
void checkBothLongestPress(bool sw1, bool sw2);
bool handlesweepButtons();
void runsweepmaticMode();
void runManualMode(bool sw1, bool sw2);
void stopAllMotors();
void beepMultiple(int n);
void setSweepAngle();  // Nu deklarerad korrekt
void updateDebounceTime(); 
void beepQuickDouble();
void beepLong();
void beepMinMaxAlert();


//bool bleConnected = false;  // Global variabel för att hålla koll på Bluetooth-anslutning

void saveSpeed(int address, int &currentSpeed, int newSpeed) {
  if (currentSpeed != newSpeed) {  // Only write if different
    currentSpeed = newSpeed;       // Update variable
    EEPROM.write(address, newSpeed);  // Save to EEPROM
//    Serial.print("[EEPROM] Speed Updated at address ");
//    Serial.print(address);  
//    Serial.print(": ");
//    Serial.println(newSpeed);
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
      Serial.println("[WARNING] Invalid PID values from EEPROM. Resetting...");
      Kp = 2.5; Ki = 0.1; Kd = 0.5;
      savePIDValues(); // Sparar defaultvärden
    }

    headingPID.SetTunings(Kp, Ki, Kd);
}


// Function to read heading from HMC5883L
/*void updateHeading() {
    sensors_event_t event;
    mag.getEvent(&event);

    // Beräkna heading i grader
    currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (currentHeading < 0) {
        currentHeading += 360;  // Se till att värdet är mellan 0–360°
    }

    Serial.print("[DEBUG] Updated Heading: ");
    Serial.println(currentHeading);
}*/

void updateHeading() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate heading in degrees
  currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (currentHeading < 0) {
      currentHeading += 360; // Ensure values stay within 0–360°
    //Serial.print("[DEBUG] Updated Heading: ");
    //Serial.println(currentHeading);
  }
}

/*void updateHeading() {
    sensors_event_t event;
    mag.getEvent(&event);

    Serial.print("[Compass] Raw Data: X=");
    Serial.print(event.magnetic.x);
    Serial.print(" | Y=");
    Serial.print(event.magnetic.y);
    Serial.print(" | Z=");
    Serial.println(event.magnetic.z);

    currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (currentHeading < 0) {
        currentHeading += 360;  
    }

    // ✅ Check if heading is valid
    if (isnan(currentHeading) || currentHeading < 0 || currentHeading > 360) {
        Serial.println("[ERROR] Compass returned invalid heading! Resetting...");
        currentHeading = 0;  // Set a fallback value
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
        turnThreshold = 5;  // Återställ till standard om EEPROM har skräpdata
        EEPROM.put(12, turnThreshold);
    }
}

void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  loadSettings();  // 🛠 Anropa funktionen här
  loadPIDValues();  // Load stored PID values from EEPROM

  // Initialize magnetometer
  if (!mag.begin()) {
      Serial.println("HMC5883L not found");
      while (1);
  }
  Serial.println("HMC5883L ok");

  /*if (headingPID.GetMode() == AUTOMATIC) {
      Serial.println("✅ PID is ACTIVE");
  } else {
      Serial.println("🛑 PID is NOT active!");
  }*/

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
  } else if (driverMode == 2) { // Externa boarn L298H 
      Serial.println("L298N");
      EN  = 10;
      IN1 = 9;
      IN2 = 8;
      posSw   = 4;
      rotSw1  = 3;
      rotSw2  = 2;
      buzzerPin = 7;

      // Create motor driver
      myMotor = new L298N(EN, IN1, IN2);
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

  if (sweepModeActive) {
    runsweepmaticMode();
  } else {
    runManualMode(sw1, sw2);
  }

  delay(200);  // För att undvika att spamma seriell monitor
}
  // FUNCTION TO TEST THE COMPASS UNIT
  /*testCompass();  // Testa kompassen
  void testCompass() {
      sensors_event_t event;
      mag.getEvent(&event);  // Läs in data från kompassen

      // Räkna ut heading
      float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
      if (heading < 0) heading += 360;  // Se till att vi har 0–360 grader

      // Debugutskrift
      Serial.print("📡 Compass Reading | Heading: ");
      Serial.print(heading);
      Serial.print("° | X: ");
      Serial.print(event.magnetic.x);
      Serial.print(" Y: ");
      Serial.print(event.magnetic.y);
      Serial.print(" Z: ");
      Serial.println(event.magnetic.z);
      
      delay(1000); // Vänta en sekund mellan mätningarna
    } */


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
    case 'L':  
      if (driverMode == 1) {
          digitalWrite(IN1, LOW);
          analogWrite(EN, manualSpeed);
      } else if (driverMode == 2 && myMotor) {
          myMotor->setSpeed(manualSpeed);
          myMotor->backward();
      }
      Serial.println("[BT] Rotating Left");
      break;

    case 'R':  
      if (driverMode == 1) {
          digitalWrite(IN1, HIGH);
          analogWrite(EN, manualSpeed);
      } else if (driverMode == 2 && myMotor) {
          myMotor->setSpeed(manualSpeed);
          myMotor->forward();
      }
      Serial.println("[BT] Rotating Right");
      break;

    case 'S':  
      stopAllMotors();
      Serial.println("[BT] Stopping Motor");
      BTserial.println("Stopping Motor");
      break;

    case '+':  // ⏫ Increase Manual Speed
      saveSpeed(0, manualSpeed, min(manualSpeed + 25, 250));  // Prevent exceeding 250
      myMotor->setSpeed(manualSpeed);
      //Serial.print("[BT] Increased Manual Speed: ");
      //Serial.println(manualSpeed);
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      break;

    case '-':  // ⏬ Decrease Manual Speed
      saveSpeed(0, manualSpeed, max(manualSpeed - 25, 25));  // Prevent going below 25
      myMotor->setSpeed(manualSpeed);
      //Serial.print("[BT] Decreased Manual Speed: ");
      //Serial.println(manualSpeed);
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      break;

    case 'U':  // ⏫ Increase sweep Speed
      saveSpeed(1, sweepSpeed, min(sweepSpeed + 25, 250));  // Prevent exceeding 250
      //Serial.print("[BT] Increased sweep Speed: ");
      //Serial.println(sweepSpeed);
      BTserial.print("sweep Speed: ");
      BTserial.println(sweepSpeed);
      break;

    case 'H':  // ⏬ Decrease sweep Speed
      saveSpeed(1, sweepSpeed, max(sweepSpeed - 25, 25));  // Prevent going below 25
      //Serial.print("[BT] Decreased sweep Speed: ");
      //Serial.println(sweepSpeed);
      //BTserial.print("sweep Speed: ");
      //BTserial.println(sweepSpeed);
      break;

    case 'C':  // Deactivate Compass Mode
        compassModeActive = false;
        Serial.println("[BT] Compass Deactivated");
        BTserial.println("Compass Deactivated");
        stopAllMotors();
        break;

    case 'A':  
      sweepModeActive = true;
      //Serial.println("[BT] sweep Mode Activated");
      //BTserial.println("sweep Mode Activated");
      break;

    case 'M':  
      sweepModeActive = false;
      stopAllMotors();
      //Serial.println("[BT] sweep Mode Deactivated");
      //BTserial.println("sweep Mode Deactivated");
      break;

    case 'P':  // Adjust Kp "P3.5" → Sets Kp to 3.5
      if (BTserial.available()) {
          Kp = BTserial.parseFloat();
          headingPID.SetTunings(Kp, Ki, Kd);
          savePIDValues();  // Save new Kd to EEPROM
          Serial.print("[BT] Updated Kp: ");
          Serial.println(Kp);
          BTserial.print("Kp Set to: ");
          BTserial.println(Kp);
        }
      break;

    case 'I':  // Adjust Ki "I0.15" → Sets Ki to 0.15
      if (BTserial.available()) {
          Ki = BTserial.parseFloat();
          headingPID.SetTunings(Kp, Ki, Kd);
          savePIDValues();  // Save new Kd to EEPROM
          Serial.print("[BT] Updated Ki: ");
          Serial.println(Ki);
          BTserial.print("Ki Set to: ");
          BTserial.println(Ki);
        }
      break;

    case 'D':  // Adjust Kd "D0.7" → Sets Kd to 0.7
      if (BTserial.available()) {
          Kd = BTserial.parseFloat();
          headingPID.SetTunings(Kp, Ki, Kd);
          savePIDValues();  // Save new Kd to EEPROM
          Serial.print("[BT] Updated Kd: ");
          Serial.println(Kd);
          BTserial.print("Kd Set to: ");
          BTserial.println(Kd);
        }
      break;

      case 'T':  // Ändra tröskelvärdet för styrning
          if (BTserial.available()) {
              turnThreshold = BTserial.parseFloat();
              turnThreshold = constrain(turnThreshold, 1, 20);  // Begränsa mellan 1-20
              EEPROM.put(12, turnThreshold);  // Spara i EEPROM
              Serial.print("[BT] Updated turnThreshold: ");
              Serial.println(turnThreshold);
              BTserial.print("New Turn Threshold: ");
              BTserial.println(turnThreshold);
          }
          break;

    case 'O':  // Request all current settings
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
      break;
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


// ----------------------------
//   SOMETHING FO THE FUTURE OF DEBOUNCE
// ----------------------------
void updateDebounceTime() {
  unsigned long newDebounceTime = 450;
  // Only print if debounce time changes
  static unsigned long lastDebounceTime = 0;
  if (newDebounceTime != lastDebounceTime) {
    //Serial.print("[System] Updated debounce time: ");
    //Serial.print(newDebounceTime);
    //Serial.println("ms");
    lastDebounceTime = newDebounceTime;  // Store last debounce value
  }

  posSwDebounceTime = newDebounceTime;  // Apply new debounce time
}

// ----------------------------
//   SWEEP MODE
// ----------------------------
void runsweepmaticMode() {
    //Serial.println("[sweep] Starting...");
    beepMultiple(0);

    stopAllMotors();
    delay(1000);

    //Serial.println("Going backward until posSw=LOW...");

    if (driverMode == 1) {  // L298P motorstyrning
        digitalWrite(IN1, LOW);  // Sätt riktning bakåt
        analogWrite(EN, sweepSpeed);  // Sätt hastighet

        while (digitalRead(posSw) == HIGH) {
            if (!handlesweepButtons()) return;
        }
    } else if (driverMode == 2 && myMotor) {  // L298N motorstyrning
        myMotor->setSpeed(sweepSpeed);
        myMotor->backward();

        while (digitalRead(posSw) == HIGH) {
            if (!handlesweepButtons()) return;
        }
    }

  stopAllMotors();
  int posCount = 0;
  delay(1000);

  while (sweepModeActive) {
    Serial.println("UP");
    
    if (driverMode == 1) {  // L298P
      digitalWrite(IN1, HIGH);  // Framåt
      analogWrite(EN, sweepSpeed);
    } else if (driverMode == 2 && myMotor) {  // L298N
      myMotor->setSpeed(sweepSpeed);
      myMotor->forward();
    }

    while (posCount < sweep_angle) {
      if (!handlesweepButtons()) return;

      if (digitalRead(posSw) == LOW && millis() - lastPosSwPress > posSwDebounceTime) {
        lastPosSwPress = millis();
        //Serial.println("[DEBUG] posSw CLICKED - UP");
        posCount++;
        Serial.print("posCount (UP) = ");
        Serial.println(posCount);
      }
    }

    stopAllMotors();
    delay(1000);

    Serial.println("DOWN");
    
    if (driverMode == 1) {  // L298P
        digitalWrite(IN1, LOW);  // Bakåt
        analogWrite(EN, sweepSpeed);
     } else if (driverMode == 2 && myMotor) {  // L298N
        myMotor->setSpeed(sweepSpeed);
        myMotor->backward();
    }

    while (posCount > 0) {
        if (!handlesweepButtons()) return;

        if (digitalRead(posSw) == LOW && millis() - lastPosSwPress > posSwDebounceTime) {
          lastPosSwPress = millis();
          //Serial.println("[DEBUG] posSw CLICKED - DOWN");
          posCount--;
          Serial.print("posCount (DOWN) = ");
          Serial.println(posCount);
        }
      }

    stopAllMotors();
    delay(1000);
    }
}

// ----------------------------
//   Compass Mode 
// ----------------------------
void runCompassMode() {
    compassModeActive = true;
    Serial.print("compassMode = ");
    Serial.println(compassModeActive);

    beepMultiple(2);
    stopAllMotors();
    motorIsStopped = true;  // Initially, motor is stopped

    /*// 🟢 Läs in aktuell heading flera gånger innan PID startar
    for (int i = 0; i < 3; i++) {  
        updateHeading();
        delay(200);  // Vänta lite för att stabilisera
    }*/

    // Get current heading and set it as target
    updateHeading();
    setHeading = currentHeading;
    Serial.print("Holding Heading: ");
    Serial.println(setHeading);
    BTserial.print("Holding Heading: ");
    BTserial.println(setHeading);
 
    bool pressingExit = false;
    unsigned long pressStartTime = 0;
    unsigned long lastPrintTime = 0;  // Timer for limiting serial output

    // 🛑 PAUSA PID FÖR ATT FÖRHINDRA INITIALA RYCK
    //Serial.println("[Compass Mode] Stabilizing...");
    //delay(1500);  // Ge systemet 1.5s att stabilisera sig innan styrning

    while (compassModeActive) {
        updateHeading();
        headingPID.Compute();  // Compute PID correction
        Serial.println("PID Compute...");
        headingPID.Compute();
        Serial.println("PID ok");

        //DEBUG
        Serial.print("Before PID | Setpoint: ");
        Serial.print(setHeading);
        Serial.print(" | Current: ");
        Serial.print(currentHeading);
        Serial.print(" |  BEFORE Compute(): ");
        Serial.println(output); // 🔴 Se vad output är innan PID körs
        Serial.print("After PID | Output AFTER Compute(): ");
        Serial.println(output);

        delay(500);  // Small delay for stability

      // Timer för att skicka motoruppdateringar
      static unsigned long lastMotorUpdate = 0;
      const unsigned long motorUpdateInterval = 300; // Uppdatera var 500ms
      const int deadZone = 8; // If the error is within ±8 degrees, don't move

if (millis() - lastMotorUpdate >= motorUpdateInterval) {
      lastMotorUpdate = millis();  // Uppdatera timer
      Serial.print("[DEBUG] Motor CMD: ");

      int baseSpeed = 35;  // Lägsta hastighet
      int maxSpeed = 200;   // Högsta hastighet
      //int speed = constrain(abs(output), baseSpeed, maxSpeed); // Begränsar hastighet
      static int lastSpeed = 0;
      int targetSpeed = constrain(abs(output), baseSpeed, maxSpeed);
      int speed = lastSpeed + ((targetSpeed - lastSpeed) / 2);  // Mjuk ökning
      lastSpeed = speed;

      // ✅ Dödzon: Om felet är mindre än x° ska motorerna stanna
      if (abs(setHeading - currentHeading) <= deadZone) {
          Serial.println("ON TARGET (within deadZone) - STOPPING");
          stopAllMotors();
          continue; // Hoppa över resten av loopen för denna iteration
      }

      if (output > turnThreshold) {
          Serial.print("RIGHT | Speed: ");
          Serial.println(speed);

          if (driverMode == 1) {  // L298P
              digitalWrite(IN1, HIGH);
              analogWrite(EN, speed);
          } else if (driverMode == 2 && myMotor) {  // L298N
              myMotor->setSpeed(speed);
              myMotor->forward();
          }
      } 
      else if (output < -turnThreshold) {
          Serial.print("LEFT | Speed: ");
          Serial.println(speed);

          if (driverMode == 1) {  // L298P
              digitalWrite(IN1, LOW);
              analogWrite(EN, speed);
          } else if (driverMode == 2 && myMotor) {  // L298N
              myMotor->setSpeed(speed);
              myMotor->backward();
          }
      } 
      else {
          Serial.println("STOP");
          stopAllMotors();
      }
  }



        // **Print Status Once Per Second**
        /*if (millis() - lastPrintTime >= 3000) {  // Every 1000ms (1s)
            lastPrintTime = millis();
            Serial.print("[Compass] Current Heading: ");
            Serial.print(currentHeading);
            Serial.print(" | Target: ");
            Serial.print(setHeading);
            Serial.print(" | Output: ");
            Serial.println(output);
        }*/

        // **Exit condition: Press ANY button for 300ms to exit**
        bool sw1 = (digitalRead(rotSw1) == LOW);
        bool sw2 = (digitalRead(rotSw2) == LOW);
        bool pedal = (digitalRead(pedalButton) == LOW);

        if (sw1 || sw2 || pedal) { 
            if (!pressingExit) { 
                pressingExit = true;
                pressStartTime = millis();
            } else if (millis() - pressStartTime >= 300) {  // Hold any button for 300ms to exit
                Serial.println("[Compass Mode] Off");
                beepMultiple(2);
                stopAllMotors();
                compassModeActive = false;
                return;
            }
        } else {
            pressingExit = false; // Reset if button is released
        }

        delay(50);  // Small delay for stability
    }
}


// ----------------------------
//   Manual Mode 
// ----------------------------
void runManualMode(bool sw1, bool sw2) {
    static int lastState = -1;  // 0 = STOP, 1 = FORWARD, 2 = BACKWARD
    static unsigned long lastPrintTime = 0;  // Timestamp to limit serial printing

  if (sw1 && !sw2) {  // Kör framåt
    if (driverMode == 1) { // L298P
      digitalWrite(IN1, HIGH);  // Riktning framåt
      analogWrite(EN, manualSpeed);
    } else if (driverMode == 2 && myMotor) { // L298N
      myMotor->setSpeed(manualSpeed);
      myMotor->forward();
    }

  if (lastState != 1 && millis() - lastPrintTime >= 1000) { 
      Serial.println("M Right");
      lastState = 1;
      lastPrintTime = millis();
  }

  } else if (!sw1 && sw2) {  // Kör bakåt
      if (driverMode == 1) { // L298P
        digitalWrite(IN1, LOW);  // Riktning bakåt
        analogWrite(EN, manualSpeed);
      } else if (driverMode == 2 && myMotor) { // L298N
        myMotor->setSpeed(manualSpeed);
        myMotor->backward();
      }

  if (lastState != 2 && millis() - lastPrintTime >= 1000) { 
    Serial.println("M Left");
    lastState = 2;
    lastPrintTime = millis();
  }

} else {  // Stanna motorn
    if (driverMode == 1) { // L298P
        analogWrite(EN, 0);  // Stänger av PWM
    } else if (driverMode == 2 && myMotor) { // L298N
        myMotor->stop();
    }

    if (lastState != 0 && millis() - lastPrintTime >= 1000) {  
        //Serial.println("[Manual] Motor B => STOP");
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

//UTESTED SO THIS WILL PROBABLY NOT WORK PROPERLY!!!
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
                sweepModeActive = true;
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

/* void checkBothLongestPress(bool sw1, bool sw2) {
  static unsigned long lastUpdate = 0;  // Last log update timestamp
  static unsigned long holdStartTime = 0;  // When the buttons were first pressed
  static unsigned long lastBeepTime = 0;   // Last beep timestamp

  static bool releaseLogged = false;  // Prevents duplicate log messages
  static bool longPressTriggered = false; // Prevents multiple activations for 8s hold
  
  static unsigned long firstPressTime = 0; // Timestamp for the first button press
  static bool waitingForSecondPress = false; // Indicates if waiting for second button press
  

  if (!bothPressing && (sw1 || sw2)) {  
    // First button press detected, start timing
    firstPressTime = millis();
    waitingForSecondPress = true;
  }

  if (waitingForSecondPress && (sw1 && sw2)) {
    // If the second button is pressed within 150ms, prevent single press registration
    if (millis() - firstPressTime < 150) {
      Serial.println("[INFO] Single press ignored due to quick second press.");
      waitingForSecondPress = false;
    }
  }

  if (!bothPressing && sw1 && sw2) {  
    // If both buttons are pressed and were not already held
    bothPressing = true;
    holdStartTime = millis();  // Store press start time
    lastUpdate = millis();  // Reset last log timestamp
    lastBeepTime = millis(); // Start beep timer
    releaseLogged = false;  // Reset flag
    longPressTriggered = false; // Reset long hold flag
    waitingForSecondPress = false; // Reset waiting flag
  } 
  else if (bothPressing && (!sw1 || !sw2)) {  // If one button is released after being held down
    if (!releaseLogged && !longPressTriggered) {  
      unsigned long held = millis() - holdStartTime;  // Calculate hold duration
      Serial.println("[DEBUG] Both buttons released. Timer reset.");
      releaseLogged = true;  // Prevent duplication
      delay(50);  // Small delay to avoid bouncing

      // ✅ 8-second hold => 5-second continuous beep, no action on release
      if (held >= 8000) {  
        Serial.println("[WARNING] 8s Hold Detected! 5s Continuous Beep!");
        tone(buzzerPin, 1000);  // Start continuous beep for 5s
        delay(5000);
        noTone(buzzerPin);  // Stop beep
        longPressTriggered = true; // Prevent further actions
      }
      else if (held >= 8000) {  
        Serial.println("[sweep] 5s => Entering Motor Speed Adjustment Mode (3 fast beeps)");
        beepMultipleDuration(3, 100); // Three short beeps
        setMotorSpeed();  
      } 
      else if (held >= 5000) {  
        Serial.println("[sweep] 3s => Set Sweep Angle (2 beeps)");
        beepMultipleDuration(2, 200); // Two beeps
        setSweepAngle();
      } 
      else if (held >= 3000) {  
        Serial.println("[Compass Mode] 3s Hold Detected - Activating Compass Mode");
        beepMultiple(2);
        runCompassMode();  
      } 
      else if (held >= 300) {  
        Serial.println("[sweep] 300ms => Activating sweep Mode (1 beep)");
        beepMultipleDuration(1, 300); // One beep
        sweepModeActive = true;
      }
    }
    bothPressing = false;  // Reset flag

    // ✅ Ensure buttons are fully released before continuing
    while (digitalRead(rotSw1) == LOW || digitalRead(rotSw2) == LOW) {
      delay(10);
    }
  } 
  else if (bothPressing) {  
    // If buttons are held down, continue checking time
    unsigned long heldTime = millis() - holdStartTime;
    
    // 📢 Add a beep every second unless it's in the 8s hold state
    if (heldTime < 8000 && millis() - lastBeepTime >= 1000) {  
      tone(buzzerPin, 1000);
      delay(100);
      noTone(buzzerPin);
      lastBeepTime = millis();
    }

    // Print held time in seconds
    if (millis() - lastUpdate >= 500) {  
      Serial.print("[DEBUG] Held Time: ");
      Serial.print(heldTime / 1000.0, 1);  // Convert to seconds with 1 decimal
      Serial.println("s");
      lastUpdate = millis();
    }
  }
}*/

// ----------------------------
//   SET MOTOR SPEED (FOR MANUAL AND SWEEP MODE) ADDED PEDALBUTTON FUNKTION. UNTESTED!!!
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


/*void setMotorSpeed() {
    Serial.println("** Entering Motor Speed Adjustment Mode **");
    stopAllMotors();

    bool exitMenu = false;
    bool adjustingSweepSpeed = false;  // True = Adjusting sweep speed, False = Adjusting manual speed
    bool adjustingManualSpeed = true;  // Default to manual speed adjustment
    unsigned long lastButtonPress = 0;
    unsigned long holdStart = 0;
    unsigned long pedalHoldStart = 0; // Track pedal button hold time
    const unsigned long buttonDebounce = 300;
    const unsigned long exitHoldTime = 3000;  // Hold 3 seconds to exit
    const int minSpeed = 50;
    const int maxSpeed = 250;

    Serial.println("Now adjusting: MANUAL MODE SPEED");

    while (!exitMenu) {
      bool sw1 = (digitalRead(rotSw1) == LOW);
      bool sw2 = (digitalRead(rotSw2) == LOW);

      // Detect exit command (hold both buttons for 3 seconds)
      if (sw1 && sw2) {
          if (holdStart == 0) {
              holdStart = millis();
          } else if (millis() - holdStart >= exitHoldTime) {
              Serial.println("Exiting Motor Speed Adjustment Mode...");
              beepMultiple(2);  // Exit confirmation beep
              exitMenu = true;
              break;
          }
      } else {
          holdStart = 0;  // Reset hold tracking when buttons are released
      }

      // Toggle between adjusting sweep and manual mode (tap both buttons quickly)
      if (sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
          lastButtonPress = millis();
          adjustingSweepSpeed = !adjustingSweepSpeed;
          adjustingManualSpeed = !adjustingSweepSpeed;

          Serial.println(adjustingSweepSpeed ? "Switched to adjusting: SWEEP MODE SPEED" : "Switched to adjusting: MANUAL MODE SPEED");
          beepTriple();  // Three quick beeps for switching mode
          delay(500);  // Prevent accidental double-switch
      }

      // **Increase Speed** (Stops at max value)
      if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
          lastButtonPress = millis();

          if (adjustingSweepSpeed) {
              if (sweepSpeed < maxSpeed) {
                  sweepSpeed += 25;
                  myMotor->setSpeed(sweepSpeed);
                  EEPROM.write(1, sweepSpeed);
                  //Serial.print("New SWEEP MODE Speed: ");
                  Serial.println(sweepSpeed);
                  beepQuickDouble();
              } else {
                  Serial.println("SWEEP MODE Speed is at MAX (250)");
                  beepMinMaxAlert();
              }
          } else if (adjustingManualSpeed) {
              if (manualSpeed < maxSpeed) {
                  manualSpeed += 25;
                  myMotor->setSpeed(manualSpeed);
                  EEPROM.write(0, manualSpeed);
                  //Serial.print("New MANUAL MODE Speed: ");
                  Serial.println(manualSpeed);
                  beepLong();
              } else {
                  Serial.println("MANUAL MODE Speed is at MAX (250)");
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
                  //Serial.print("New SWEEP MODE Speed: ");
                  Serial.println(sweepSpeed);
                  beepQuickDouble();
              } else {
                  Serial.println("SWEEP MODE Speed is at MIN (50)");
                  beepMinMaxAlert();
              }
          } else if (adjustingManualSpeed) {
              if (manualSpeed > minSpeed) {
                  manualSpeed -= 25;
                  myMotor->setSpeed(manualSpeed);
                  EEPROM.write(0, manualSpeed);
                  //Serial.print("New MANUAL MODE Speed: ");
                  Serial.println(manualSpeed);
                  beepLong();
              } else {
                  Serial.println("MANUAL MODE Speed is at MIN (50)");
                  beepMinMaxAlert();
              }
          }
      }
      delay(200);  // Prevent sensitive navigation
    }
}*/



// ----------------------------
//   HANDLE SWEEP MODE EXIT BUTTONS
// ----------------------------
bool handlesweepButtons() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  if ((sw1 && sw2) || (!sw1 && !sw2)) {
    pressingsweep = false;
    return true;
  }

  if (!pressingsweep) {
    pressingsweep = true;
    pressTimesweep = millis();
  } else if (millis() - pressTimesweep >= 300) {  // Press 300ms to exit sweepmode, any button
    //Serial.println("ABORT sweep-läge (knapp >=300ms).");
    beepMultiple(1);
    stopAllMotors();

    //Serial.println("[System] Waits 1s to activate manual mode");
    delay(1000);
    sweepModeActive = false;
    pressingsweep = false;
    return false;
  }
  return true;
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
//   BEEP FUNCTION
// ----------------------------
void beepMultiple(int n) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000);
    delay(250);
    noTone(buzzerPin);
    delay(250);
  }
}

// ✅ Improved beep function for short/long signals
void beepMultipleDuration(int n, int duration) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000);
    delay(duration);
    noTone(buzzerPin);
    delay(200);
  }
}
// 🔊 sweep Mode: Two quick beeps
void beepQuickDouble() {
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
}

// 🔊 Manual Mode: One long beep
void beepLong() {
  tone(buzzerPin, 1000);
  delay(400);
  noTone(buzzerPin);
}

// 🔊 Alert when reaching MIN or MAX speed (2 quick + 2 long beeps)
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

// 🔊 Three quick beeps when switching mode
void beepTriple() {
  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, 1000);
    delay(100);
    noTone(buzzerPin);
    delay(100);
  }
}
