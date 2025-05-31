/*
---------------------------------------------------------
  Arduino Motor Control with L298N, Sweep Mode, and Bluetooth Speed Adjustment
---------------------------------------------------------
  This code controls a DC motor via the L298P H-bridge driver, using an Arduino UNO
  The motor can be operated in two modes: manual mode (with two rotation buttons) or automatic sweep mode.
  
  Main features:
    - Supports both a position switch and a hall sensor as limit switches for sweep mode.
    - Sweep mode: Motor sweeps back and forth between two sensor triggers, changing direction each time.
    - Manual mode: Motor is controlled directly by left/right button presses.
    - A single adjustable speed is used for both sweep and manual mode.
    - Speed can be set and saved via:
         - Bluetooth commands (‘+’ to increase, ‘-’ to decrease speed)
         - Button menu (hold both rotation buttons or pedal >5s to enter speed setting)
    - Long/short button presses control mode switching:
         - Short press (>0.3s): Start sweep mode.
         - Long press (>3s): Enter speed adjustment menu.
    - Sweep mode and menus can be exited by holding any button or the pedal for 3 seconds.
    - Uses EEPROM to store last set speed across power cycles.
    - Includes buzzer/beep feedback for menu actions and limits.

 The code uses SoftwareSerial for Bluetooth communication, but note that some boards may require adjustments.
  
  NOTE: 
    - To use hall sensor as the sweep trigger, set 'useHallSensor' to true.
    - To enable manual mode (button left/right control), call runManualMode() in the main loop if needed.

  Written for clear extension and troubleshooting. All logic is structured to allow adding new features like compass mode or PID control in the future.

  (c) Joens / Live Transducer Pole, 2024
---------------------------------------------------------
*/
/*
  -----------------------------------------------
  I/O OVERVIEW – ARDUINO MOTOR CONTROL / L298N
  -----------------------------------------------

  OUTPUTS:
    EN         - Motor driver PWM (speed control)
    IN1        - Motor driver direction 1
    IN2        - Motor driver direction 2
    buzzerPin  - Buzzer for sound feedback

  INPUTS:
    posSw         - Position switch (limit/end stop)
    rotSw1        - Rotation button 1 (right)
    rotSw2        - Rotation button 2 (left)
    pedalButton   - Pedal button (same function as both rotation buttons)
    hallSensorPin - Hall sensor for alternative end stop detection

  BLUETOOTH:
    btRxPin - Bluetooth RX (receiver)
    btTxPin - Bluetooth TX (transmitter)

  NOTES:
    - All inputs are active LOW.
    - PinMode for all I/O is set in setup().
    - Pin numbers are assigned automatically by boardType (see setupPinsByBoard()).
*/


#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>   // For Bluetooth HC-05 etc

// --- Pin Definitions ---
// Motor A (L298P)
// IN1: Direction (HIGH = Forward, LOW = Backward)
const int IN1 = 12;
// EN:  PWM (speed)
const int EN = 3;
// IN2: Brake (LOW = drive, HIGH = brake/stop)
const int IN2 = 9;

// IO for buttons/sensors
const int posSw = 7;          // Position switch (limit switch)
const int rotSw1 = 6;         // Right rotation button
const int rotSw2 = 11;        // Left rotation button
const int buzzerPin = 10;     // Piezo buzzer
const int pedalButton = 5;    // Pedal button (alternative to rotation buttons)
const int hallSensorPin = A1; // Hall-effect sensor (optional as limit switch)

// Bluetooth pins (UNO)
#define BTRX_PIN 8
#define BTTX_PIN 13
SoftwareSerial BTserial(BTRX_PIN, BTTX_PIN);

// --- Global variables ---
bool sweepModeActive = false;    // True: sweep mode is running
bool bothPressing = false;       // True: both rotation buttons or pedal held
bool useHallSensor = false;      // If true, hall sensor is used as endstop instead of posSw
bool sweepTimeoutAlarm = false;  // True if sweep timeout occurred

int motorSpeed = 100;            // Motor speed (50-250 typical, 0-255 possible)
int lastDirection = 1;           // Remember last direction (1 = right, -1 = left)

unsigned long lastPosSwPress = 0;      // For sensor debounce
unsigned long posSwDebounceTime = 400; // ms, debounce time for limit switch/hall
unsigned long sweepTimeoutMs = 6000;   // ms, max sweep time before alarm

// --------------------- Motor control functions ----------------------

// Run motor forward (right)
void motorForward(int speed) {
  digitalWrite(IN1, HIGH);    // Set direction forward
  digitalWrite(IN2, LOW);     // Brake off
  analogWrite(EN, speed);     // Set speed (0-255)
}

// Run motor backward (left)
void motorBackward(int speed) {
  digitalWrite(IN1, LOW);     // Set direction backward
  digitalWrite(IN2, LOW);     // Brake off
  analogWrite(EN, speed);     // Set speed (0-255)
}

// Stop the motor (release)
void motorStop() {
  analogWrite(EN, 0);         // No speed (coast/stop)
  digitalWrite(IN2, LOW);     // Brake off
}

// (Optional) active brake
void motorBrake() {
  analogWrite(EN, 0);         // No PWM
  digitalWrite(IN2, HIGH);    // Brake ON
}

// --------------------- Buzzer feedback helpers ----------------------

// Beep the buzzer n times with 250ms beeps
void beepMultiple(int n) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000); delay(250); noTone(buzzerPin); delay(250);
  }
}

// Beep n times with custom beep duration (duration in ms)
void beepMultipleDuration(int n, int duration) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000); delay(duration); noTone(buzzerPin); delay(200);
  }
}

// Single long beep for feedback
void beepLong() {
  tone(buzzerPin, 1000); delay(400); noTone(buzzerPin);
}

// Special beep pattern for min/max speed alert
void beepMinMaxAlert() {
  tone(buzzerPin, 1000); delay(100); noTone(buzzerPin); delay(100);
  tone(buzzerPin, 1000); delay(100); noTone(buzzerPin); delay(200);
  tone(buzzerPin, 1000); delay(400); noTone(buzzerPin); delay(200);
  tone(buzzerPin, 1000); delay(400); noTone(buzzerPin);
}

// --------------------- Pedal Debounce Function ----------------------

// Check if pedal is pressed (with debounce)
bool isPedalPressed() {
  static unsigned long lastDebounceTime = 0;
  static bool lastState = HIGH;
  bool currentState = digitalRead(pedalButton) == LOW; // active low
  if (currentState != lastState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > 50) {
    if (currentState) {
      lastState = currentState;
      return true;
    }
  }
  lastState = currentState;
  return false;
}

// --------------------- Limit Sensor Function ------------------------

// Return true if currently active (limit switch or hall sensor triggered)
bool isSensorActive() {
  if (useHallSensor) return digitalRead(hallSensorPin) == LOW;
  else return digitalRead(posSw) == LOW;
}

// Stop all motors (here only M1 is used)
void stopAllMotors() {
  motorStop();
}

// Save new speed to EEPROM if changed
void saveSpeed(int newSpeed) {
  if (motorSpeed != newSpeed) {
    motorSpeed = newSpeed;
    EEPROM.write(0, newSpeed); // Save to address 0
  }
}

// --------------------------- Arduino SETUP --------------------------
void setup() {
  Serial.begin(9600);           // Debug/monitor serial
  BTserial.begin(9600);         // Bluetooth serial

  // Set input/output pins
  pinMode(IN1, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(posSw, INPUT_PULLUP);      // Endstop
  pinMode(rotSw1, INPUT_PULLUP);     // Button right
  pinMode(rotSw2, INPUT_PULLUP);     // Button left
  pinMode(buzzerPin, OUTPUT);        // Buzzer
  pinMode(pedalButton, INPUT_PULLUP);// Pedal
  if (useHallSensor) pinMode(hallSensorPin, INPUT_PULLUP);

  motorStop(); // Ensure stopped on startup

  // Load last speed from EEPROM, or set default
  int savedSpeed = EEPROM.read(0);
  if (savedSpeed >= 50 && savedSpeed <= 250) motorSpeed = savedSpeed;
  else motorSpeed = 125;
}

// --------------------------- Arduino MAIN LOOP ----------------------
void loop() {
  // Forward serial to Bluetooth if needed (not essential for core logic)
  if (Serial.available()) {
    char data = Serial.read();
    BTserial.write(data);
  }
  // Handle incoming Bluetooth commands
  if (BTserial.available()) {
    char data = BTserial.read();
    switch (data) {
      case 'T': { // Set sweep timeout via Bluetooth
        delay(50);
        String numStr = "";
        while (BTserial.available()) {
          char c = BTserial.read();
          if (isDigit(c)) numStr += c;
          else break;
        }
        if (numStr.length() > 0) {
          unsigned long newTimeout = numStr.toInt();
          if (newTimeout >= 2 && newTimeout <= 60) {
            sweepTimeoutMs = newTimeout * 1000UL;
            BTserial.print(F("Sweep timeout set to: "));
            BTserial.print(newTimeout);
            BTserial.println(F(" s"));
          } else {
            BTserial.println(F("Timeout must be 2–60 seconds"));
          }
        }
        break;
      }
      case '+': // Increase speed
        saveSpeed(min(motorSpeed + 25, 250));
        BTserial.print(F("Speed: ")); BTserial.println(motorSpeed); break;
      case '-': // Decrease speed
        saveSpeed(max(motorSpeed - 25, 50));
        BTserial.print(F("Speed: ")); BTserial.println(motorSpeed); break;
      case 'O': // Output current system status
        BTserial.println(F("=== Current System Status ==="));
        BTserial.print(F("Motor Speed: ")); BTserial.println(motorSpeed);
        BTserial.print(F("Sweep Mode: ")); BTserial.println(sweepModeActive ? "Active" : "Inactive");
        BTserial.print(F("Sweep Timeout (s): ")); BTserial.println(sweepTimeoutMs / 1000);
        BTserial.print(F("Timeout Alarm: ")); BTserial.println(sweepTimeoutAlarm ? "YES" : "NO");
        BTserial.println(F("=== End of Report ==="));
        Serial.println(F("[BT] Sent system status report"));
        break;
    }
  }

  // Read button states (active LOW)
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  // Menu logic: Hold both buttons or pedal for menu/sweep
  checkBothLongestPress(sw1, sw2);

  // Emergency stop: hold any button
  checkModeExit();

  // Manual motor control: Only run when not in sweep/menu
  runManualMode(sw1, sw2);

  delay(20); // Short delay for CPU relief
}

// ----------- MENU EXIT (hold both buttons/pedal for exit) --------------

// Return true if both rotation buttons or pedal held for a period (default 3s)
bool checkHoldToExit(unsigned long &holdStart, unsigned long exitHoldTime = 3000) {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);
  bool pedal = isPedalPressed();
  bool exitPressed = (sw1 && sw2) || pedal;
  if (exitPressed) {
    if (holdStart == 0) holdStart = millis();
    if (millis() - holdStart >= exitHoldTime) {
      beepMultiple(2); // Double beep for exit
      return true;
    }
  } else {
    holdStart = 0;
  }
  return false;
}

// ----------- MENU LOGIC: Handle short/long press of both buttons --------

// Handle both-buttons/ pedal for menu and sweep activation
void checkBothLongestPress(bool sw1, bool sw2) {
    static unsigned long lastUpdate = 0;
    static unsigned long holdStartTime = 0;
    static unsigned long lastBeepTime = 0;
    static bool releaseLogged = false;
    static bool longPressTriggered = false;

    // Activation is either both rotation buttons or pedal held down
    bool activationPressed = (sw1 && sw2) || isPedalPressed();

    if (!bothPressing && activationPressed) {  // Start of hold
        bothPressing = true;
        holdStartTime = millis();
        lastUpdate = millis();
        lastBeepTime = millis();
        releaseLogged = false;
        longPressTriggered = false;
    } 
    else if (bothPressing && !activationPressed) {  // Released
        if (!releaseLogged && !longPressTriggered) {  
            unsigned long held = millis() - holdStartTime;
            releaseLogged = true;
            delay(50); // Button release debounce

            if (held >= 3000) {   // Hold >5 seconds = enter speed menu
                beepMultipleDuration(3, 100);
                setMotorSpeed();
            } 
            else if (held >= 300) {  // Hold >0.3s = start sweep mode
                beepMultipleDuration(1, 300);
                runsweepmaticMode();                
            }
        }
        bothPressing = false;
        // Wait for all to be released before accepting new press
        while (digitalRead(rotSw1) == LOW || digitalRead(rotSw2) == LOW || isPedalPressed()) {
            delay(10);
        }
    } 
    else if (bothPressing) {  // Holding, give feedback
        unsigned long heldTime = millis() - holdStartTime;
        if (heldTime < 8000 && millis() - lastBeepTime >= 1000) {  
            tone(buzzerPin, 1000); delay(100); noTone(buzzerPin);
            lastBeepTime = millis();
        }
        // Print hold time for debugging
        if (millis() - lastUpdate >= 500) {  
            Serial.print(F("[DEBUG] Held Time: "));
            Serial.print(heldTime / 1000.0, 1);
            Serial.println(F("s"));
            lastUpdate = millis();
        }
    }
}

// ------------------- SWEEP-MATIC MODE ------------------------------------

// Run motor back and forth between two sensor triggers (limit or hall)
// Changes direction each time, stops if button/pedal held
void runsweepmaticMode() {
  sweepModeActive = true;        // Mark sweep as active
  sweepTimeoutAlarm = false;     // Reset timeout alarm
  beepMultiple(1);               // One beep on start
  stopAllMotors();
  delay(500);

  int sweepDirection = lastDirection; // Use last used direction

  while (sweepModeActive) {
    checkModeExit();     // Allow emergency exit at any time
    if (!sweepModeActive) break;

    // Drive in current direction (positive=forward, negative=backward)
    if (sweepDirection == 1) motorForward(motorSpeed);
    else motorBackward(motorSpeed);

    unsigned long sweepStartTime = millis();
    bool triggered = false;

    // Wait for sensor to trigger or timeout
    while (!triggered && sweepModeActive) {
      checkModeExit();
      if (!sweepModeActive) break;
      // Triggered by limit or hall (with debounce)
      if (isSensorActive() && millis() - lastPosSwPress > posSwDebounceTime) {
        lastPosSwPress = millis();
        triggered = true;
        break;
      }
      // Timeout protection: stop sweep if stuck
      if (millis() - sweepStartTime > sweepTimeoutMs) {
        stopAllMotors();
        beepMultiple(5);
        Serial.println(F("[ALARM] Timeout! No trigger within max time. Sweep mode stopped."));
        BTserial.println(F("[ALARM] Timeout! No trigger within max time. Sweep mode stopped."));
        sweepTimeoutAlarm = true;
        sweepModeActive = false;
        delay(1000);
        return;
      }
      delay(10); // Reduce CPU load
    }
    stopAllMotors(); // Pause at end
    delay(500);
    // Reverse direction for next sweep
    sweepDirection = -sweepDirection;
    lastDirection = sweepDirection;
  }
  stopAllMotors();
}

// ------------------- MANUAL MODE -----------------------------------------

// Manual motor control with rotation buttons (left/right)
// Right = forward, left = backward, none = stop
void runManualMode(bool sw1, bool sw2) {
  static int lastState = -1;
  static unsigned long lastPrintTime = 0;
  if (sw1 && !sw2) {
    motorForward(motorSpeed); lastDirection = 1;
    if (lastState != 1 && millis() - lastPrintTime >= 1000) {
      Serial.println(F("M Right")); lastState = 1; lastPrintTime = millis();
    }
  } else if (!sw1 && sw2) {
    motorBackward(motorSpeed); lastDirection = -1;
    if (lastState != 2 && millis() - lastPrintTime >= 1000) {
      Serial.println(F("M Left")); lastState = 2; lastPrintTime = millis();
    }
  } else {
    stopAllMotors();
    if (lastState != 0 && millis() - lastPrintTime >= 1000) {
      lastState = 0; lastPrintTime = millis();
    }
  }
}

// ------------------- SPEED MENU ------------------------------------------

// Speed adjustment menu, entered by holding both buttons/pedal >5s
void setMotorSpeed() {
  Serial.println(F("** Speed Adjustment  **"));
  stopAllMotors();

  unsigned long lastButtonPress = 0;
  unsigned long holdStart = 0;
  const unsigned long buttonDebounce = 300;
  const unsigned long exitHoldTime = 3000;
  const int minSpeed = 75;
  const int maxSpeed = 250;

  while (true) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);

    // Exit menu if both buttons or pedal held for 3s
    if (checkHoldToExit(holdStart)) {
      Serial.println(F("Exiting Speed Menu"));
      delay(1000);
      break;
    }
    // Increase speed
    if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed < maxSpeed) {
        saveSpeed(motorSpeed + 25);
        Serial.print(F("Speed: ")); Serial.println(motorSpeed);
        beepLong();
      } else {
        Serial.println(F("MAX (250)"));
        beepMinMaxAlert();
      }
    }
    // Decrease speed
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed > minSpeed) {
        saveSpeed(motorSpeed - 25);
        Serial.print(F("Speed: ")); Serial.println(motorSpeed);
        beepLong();
      } else {
        Serial.println(F("MIN (50)"));
        beepMinMaxAlert();
      }
    }
    delay(200); // Debounce and CPU relief
  }
}

// ------------------- EXIT/STOP CURRENT MODE ------------------------------

// Cancel any mode if button/pedal is held >300ms (emergency stop/exit)
void checkModeExit() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);
  bool pedal = (digitalRead(pedalButton) == LOW);
  static unsigned long pressStart = 0;
  if (sw1 || sw2 || pedal) {
    if (pressStart == 0) pressStart = millis();
    if (millis() - pressStart >= 300) {
      if (sweepModeActive) {
        sweepModeActive = false;
        Serial.println(F("Sweep Mode Deactivated"));
      }
      stopAllMotors();
      pressStart = 0;
    }
  } else {
    pressStart = 0;
  }
}
