/*
---------------------------------------------------------
  Arduino Motor Control with L298N, Sweep Mode, and Bluetooth Speed Adjustment
---------------------------------------------------------
  This code controls a DC motor via the L298N H-bridge driver, using either an Arduino UNO or D1 R32 board.
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
         - Long press (>5s): Enter speed adjustment menu.
    - Sweep mode and menus can be exited by holding any button or the pedal for 3 seconds.
    - Uses EEPROM to store last set speed across power cycles.
    - Includes buzzer/beep feedback for menu actions and limits.

  Wiring/pins are selectable for UNO or D1 R32 via the boardType variable.
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

  PINOUT TABLE (Default Assignments):

    | Function         | Arduino UNO | D1 R32 (ESP32) |
    |------------------|:-----------:|:--------------:|
    | EN (PWM)         |     3       |     18         |
    | IN1              |    12       |     19         |
    | IN2              |     9       |     21         |
    | posSw            |     7       |     23         |
    | rotSw1           |     6       |     25         |
    | rotSw2           |    11       |     26         |
    | buzzerPin        |    10       |     27         |
    | pedalButton      |     5       |     14         |
    | hallSensorPin    |    A1       |     32         |
    | btRxPin          |     8       |     16         |
    | btTxPin          |    13       |     17         |

  -----------------------------------------------
*/


#pragma GCC optimize ("Os")
#include <Arduino.h>
#include <L298N.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//---------------------------------------------------------
// BOARD AND PIN SETTINGS
//---------------------------------------------------------
#define BOARD_UNO     0
#define BOARD_D1_R32  1

int boardType = 1; // 0 = UNO, 1 = D1 R32

// Global pin variables (will be set in setupPinsByBoard)
int EN, IN1, IN2, posSw, rotSw1, rotSw2, buzzerPin, pedalButton, hallSensorPin, btRxPin, btTxPin;

// Bluetooth serial (can be re-routed per board)
#define BTRX_PIN 8
#define BTTX_PIN 13
SoftwareSerial BTserial(BTRX_PIN, BTTX_PIN);

//---------------------------------------------------------
// GLOBALS
//---------------------------------------------------------
L298N* myMotor = nullptr;         // Motor driver object
bool sweepModeActive = false;     // True if sweep mode is active
bool bothPressing = false;        // True if both buttons are being pressed
bool useHallSensor = false;       // If true, use hall sensor; else use position switch
bool sweepTimeoutAlarm = false;      // Timeout alarm flag

int motorSpeed = 100;
int lastDirection = 1;

unsigned long lastPosSwPress = 0;      // Debounce for sensor
unsigned long posSwDebounceTime = 400; // Debounce time for sensor
unsigned long sweepTimeoutMs = 6000; // Default timeout 6 s

//---------------------------------------------------------
// FORWARD DECLARATIONS
//---------------------------------------------------------
void setupPinsByBoard();
void checkBothLongestPress(bool sw1, bool sw2);
void runsweepmaticMode();
void runManualMode(bool sw1, bool sw2);
void stopAllMotors();
void beepMultiple(int n);
void beepMultipleDuration(int n, int duration);
void beepLong();
void beepMinMaxAlert();
void setMotorSpeed();
void updateDebounceTime();
void checkModeExit();
void handleSerialCommand(char command);
bool isSensorActive();
bool isPedalPressed();

//---------------------------------------------------------
// SETUP PIN LAYOUT FOR EACH BOARD
//---------------------------------------------------------
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
      btRxPin      = 16;
      btTxPin      = 17;
      break;
    default:
      Serial.println("Unknown board! Check boardType variable.");
      while (1);
  }
}

//---------------------------------------------------------
// SAVE SPEED TO EEPROM IF CHANGED
//---------------------------------------------------------
void saveSpeed(int newSpeed) {
  if (motorSpeed != newSpeed) {
    motorSpeed = newSpeed;
    EEPROM.write(0, newSpeed);
    if (myMotor) myMotor->setSpeed(motorSpeed);
  }
}

//---------------------------------------------------------
// PEDAL BUTTON WITH DEBOUNCE
//---------------------------------------------------------
bool isPedalPressed() {
  static unsigned long lastDebounceTime = 0;
  static bool lastState = HIGH;
  bool currentState = digitalRead(pedalButton) == LOW;

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

//---------------------------------------------------------
// CHECK IF SENSOR (HALL OR POS) IS TRIGGERED
//---------------------------------------------------------
bool isSensorActive() {
  if (useHallSensor) return digitalRead(hallSensorPin) == LOW;
  else return digitalRead(posSw) == LOW;
}

//---------------------------------------------------------
// ARDUINO SETUP
//---------------------------------------------------------
void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  setupPinsByBoard();

  // Set all pin modes
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

  // Create motor driver object for L298N
  myMotor = new L298N(EN, IN1, IN2);

  // Set up sensor type (default: posSw)
  if (useHallSensor) pinMode(hallSensorPin, INPUT_PULLUP);

  // Load speed from EEPROM or set default
  int savedSpeed = EEPROM.read(0);
  if (savedSpeed >= 50 && savedSpeed <= 250) motorSpeed = savedSpeed;
  else motorSpeed = 125;

  myMotor->setSpeed(motorSpeed);
  myMotor->stop();

  updateDebounceTime();
}

//---------------------------------------------------------
// ARDUINO MAIN LOOP
//---------------------------------------------------------
void loop() {
  if (Serial.available()) {
    char data = Serial.read();
    BTserial.write(data);
    handleSerialCommand(data); 
  }

  if (BTserial.available()) {
  char data = BTserial.read();
  // Bluetooth: + = faster, - = slower, T = set timeout
  switch (data) {
    // ... your existing cases
    case 'T': { // Set sweep timeout in seconds, e.g. "T12" for 12s
      delay(50); // Wait for number
      String numStr = "";
      while (BTserial.available()) {
        char c = BTserial.read();
        if (isDigit(c)) numStr += c;
        else break;
      }
      if (numStr.length() > 0) {
        unsigned long newTimeout = numStr.toInt();
        if (newTimeout >= 2 && newTimeout <= 60) { // 2–60 seconds allowed
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
      case '+': saveSpeed(min(motorSpeed + 25, 250)); BTserial.print(F("Speed: ")); BTserial.println(motorSpeed); break;
      case '-': saveSpeed(max(motorSpeed - 25, 50));  BTserial.print(F("Speed: ")); BTserial.println(motorSpeed); break;
      case 'O':  // Request all current settings
        BTserial.println(F("=== Current System Status ==="));

        BTserial.print(F("Motor Speed: "));
        BTserial.println(motorSpeed);

        BTserial.print(F("Sweep Mode: "));
        BTserial.println(sweepModeActive ? "Active" : "Inactive");

        BTserial.print(F("Board type: "));
        BTserial.println(boardType);

        BTserial.print(F("Sweep Timeout (s): "));
        BTserial.println(sweepTimeoutMs / 1000);

        BTserial.print(F("Timeout Alarm: "));
        BTserial.println(sweepTimeoutAlarm ? "YES" : "NO");

        BTserial.println(F("=== End of Report ==="));
        Serial.println(F("[BT] Sent system status report"));
      break;
  }
}

  // Read button states
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  // Handle menus and sweepmode via long/short press
  checkBothLongestPress(sw1, sw2);

  // Emergency stop & exit
  checkModeExit();

  // Manual mode is NOT run automatically here, you may want to add:
  // runManualMode(sw1, sw2);
  delay(20);
}

// Returns true if both rotation buttons OR pedal are held long enough to exit menus
bool checkHoldToExit(unsigned long &holdStart, unsigned long exitHoldTime = 3000) {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);
  bool pedal = isPedalPressed();
  bool exitPressed = (sw1 && sw2) || pedal;

  if (exitPressed) {
    if (holdStart == 0) holdStart = millis();
    if (millis() - holdStart >= exitHoldTime) {
      beepMultiple(2); // Double beep on exit
      return true;
    }
  } else {
    holdStart = 0;
  }
  return false;
}

//---------------------------------------------------------
// BUTTON LONG PRESS FUNCTION (Menu logic!)
//---------------------------------------------------------
void checkBothLongestPress(bool sw1, bool sw2) {
    static unsigned long lastUpdate = 0;
    static unsigned long holdStartTime = 0;
    static unsigned long lastBeepTime = 0;
    static bool releaseLogged = false;
    static bool longPressTriggered = false;

    // Activation: both buttons or pedal pressed
    bool activationPressed = (sw1 && sw2) || isPedalPressed();

    if (!bothPressing && activationPressed) {  
        bothPressing = true;
        holdStartTime = millis();
        lastUpdate = millis();
        lastBeepTime = millis();
        releaseLogged = false;
        longPressTriggered = false;
    } 
    else if (bothPressing && !activationPressed) {  
        if (!releaseLogged && !longPressTriggered) {  
            unsigned long held = millis() - holdStartTime;
            releaseLogged = true;
            delay(50);

            if (held >= 5000) {   // Hold > 5 seconds = open speed menu
                beepMultipleDuration(3, 100);
                setMotorSpeed();
            } 
            else if (held >= 300) {  // Hold > 0.3 sec = start sweep mode
                beepMultipleDuration(1, 300);
                runsweepmaticMode();                
            }
        }
        bothPressing = false;

        // Wait for all to be released
        while (digitalRead(rotSw1) == LOW || digitalRead(rotSw2) == LOW || isPedalPressed()) {
            delay(10);
        }
    } 
    else if (bothPressing) {  
        unsigned long heldTime = millis() - holdStartTime;

        // Beep every second while holding (max 8s)
        if (heldTime < 8000 && millis() - lastBeepTime >= 1000) {  
            tone(buzzerPin, 1000);
            delay(100);
            noTone(buzzerPin);
            lastBeepTime = millis();
        }

        // Print how long pressed for debugging
        if (millis() - lastUpdate >= 500) {  
            Serial.print(F("[DEBUG] Held Time: "));
            Serial.print(heldTime / 1000.0, 1);
            Serial.println(F("s"));
            lastUpdate = millis();
        }
    }
}

//---------------------------------------------------------
// SWEEP MODE: Go to trigger, reverse, repeat until cancelled
//---------------------------------------------------------
void runsweepmaticMode() {
  sweepModeActive = true;
  sweepTimeoutAlarm = false; // Reset alarm status on entry
  beepMultiple(1);    // Single beep on start
  stopAllMotors();
  delay(500);

  int sweepDirection = lastDirection; // Use previous direction

  while (sweepModeActive) {
    checkModeExit();
    if (!sweepModeActive) break;

    myMotor->setSpeed(motorSpeed);
    if (sweepDirection == 1) myMotor->forward();
    else myMotor->backward();

    // --- Timeout protection: start timing for this sweep
    unsigned long sweepStartTime = millis();
    bool triggered = false;

    // Wait until trigger or timeout occurs
    while (!triggered && sweepModeActive) {
      checkModeExit();
      if (!sweepModeActive) break;

      // If limit switch or hall sensor is activated (with debounce)
      if (isSensorActive() && millis() - lastPosSwPress > posSwDebounceTime) {
        lastPosSwPress = millis();
        triggered = true;
        break;
      }

      // Timeout: if max allowed time is exceeded without a trigger
      if (millis() - sweepStartTime > sweepTimeoutMs) {
        stopAllMotors();
        beepMultiple(5); // Sound alarm: 5 beeps
        Serial.println(F("[ALARM] Timeout! No trigger within max time. Sweep mode stopped."));
        BTserial.println(F("[ALARM] Timeout! No trigger within max time. Sweep mode stopped."));
        sweepTimeoutAlarm = true; // Set alarm flag
        sweepModeActive = false;  // Stop sweep mode
        delay(1000);
        return;
      }
      delay(10); // Short delay to reduce CPU usage
    }
    stopAllMotors();
    delay(500);

    // Reverse sweep direction after each successful trigger
    sweepDirection = -sweepDirection;
    lastDirection = sweepDirection;
  }
  stopAllMotors();
}

//---------------------------------------------------------
// MANUAL MODE: Drive right/left using buttons (add to loop if you want always-on)
//---------------------------------------------------------
void runManualMode(bool sw1, bool sw2) {
  static int lastState = -1;
  static unsigned long lastPrintTime = 0;
  if (sw1 && !sw2) {
    myMotor->setSpeed(motorSpeed); myMotor->forward(); lastDirection = 1;
    if (lastState != 1 && millis() - lastPrintTime >= 1000) {
      Serial.println(F("M Right")); lastState = 1; lastPrintTime = millis();
    }
  } else if (!sw1 && sw2) {
    myMotor->setSpeed(motorSpeed); myMotor->backward(); lastDirection = -1;
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

//---------------------------------------------------------
// SPEED MENU (Hold both buttons >5s to enter)
//---------------------------------------------------------
void setMotorSpeed() {
  Serial.println(F("** Speed Adjustment  **"));
  stopAllMotors();

  bool exitMenu = false;
  unsigned long lastButtonPress = 0;
  unsigned long holdStart = 0;
  const unsigned long buttonDebounce = 300;
  const unsigned long exitHoldTime = 3000;
  const int minSpeed = 75;
  const int maxSpeed = 250;

  while (!exitMenu) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);

    // Exit speed menu if both buttons or pedal held for 3s
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
    delay(200);
  }
}

//---------------------------------------------------------
// STOP ALL MOTORS
//---------------------------------------------------------
void stopAllMotors() {
  if (myMotor) myMotor->stop();
}

//---------------------------------------------------------
// EXIT/CANCEL CURRENT MODE IF ANY BUTTON IS HELD >300ms
//---------------------------------------------------------
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

//---------------------------------------------------------
// SERIALL FEEDBACK
//---------------------------------------------------------
void handleSerialCommand(char command) {
  // For now, just clear serial buffer and print unknown
  while (Serial.available()) Serial.read();  // Clear buffer
  Serial.println(F("Unknown Command"));
}


//---------------------------------------------------------
// BEEP FUNCTIONS FOR FEEDBACK
//---------------------------------------------------------
void beepMultiple(int n) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000); delay(250); noTone(buzzerPin); delay(250);
  }
}
void beepMultipleDuration(int n, int duration) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000); delay(duration); noTone(buzzerPin); delay(200);
  }
}
void beepLong() {
  tone(buzzerPin, 1000); delay(400); noTone(buzzerPin);
}
void beepMinMaxAlert() {
  tone(buzzerPin, 1000); delay(100); noTone(buzzerPin); delay(100);
  tone(buzzerPin, 1000); delay(100); noTone(buzzerPin); delay(200);
  tone(buzzerPin, 1000); delay(400); noTone(buzzerPin); delay(200);
  tone(buzzerPin, 1000); delay(400); noTone(buzzerPin);
}

//---------------------------------------------------------
// UPDATE DEBOUNCE TIME (OPTIONAL)
//---------------------------------------------------------
void updateDebounceTime() {
  unsigned long newDebounceTime = 450;
  static unsigned long lastDebounceTime = 0;
  if (newDebounceTime != lastDebounceTime) lastDebounceTime = newDebounceTime;
  posSwDebounceTime = newDebounceTime;
}
