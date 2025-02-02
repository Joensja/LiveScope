/*
=================================================================
                         TABLE OF CONTENTS
=================================================================
1️⃣  INCLUDES & GLOBAL VARIABLES
    - Libraries (L298N, EEPROM, SoftwareSerial)
    - Pin configuration
    - Global variables (speed settings, mode states, debounce timers, etc.)

2️⃣  SETUP FUNCTION
    - Initializes serial communication
    - Loads speeds from EEPROM (manualSpeed & autoSpeed)
    - Configures pin modes (motors, buttons, buzzer)
    - Sets initial motor state
    - Checks Bluetooth connection with HC-05

3️⃣  MAIN LOOP (⚡ Core Execution)
    - Reads button states (rotSw1, rotSw2)
    - Prevents false single-clicks if double-click follows quickly
    - Detects long-presses (checkBothLongestPress())
    - Switches between Manual & Auto mode based on button input
    - Runs the active mode:
      - Auto Mode: `runAutomaticMode()`
      - Manual Mode: `runManualMode()`
    - Reads Bluetooth commands (`handleBluetoothCommand()`)
    - Adds a **200ms delay** to avoid unnecessary processing load

4️⃣  AUTO MODE FUNCTIONS (🔄 Automated Motor Movement)
    - `runAutomaticMode()`       → Moves motor according to `targetPosition`
    - `handleAutoButtons()`      → Detects if user wants to exit auto mode

5️⃣  MANUAL MODE FUNCTIONS (🎛️ Manual Control)
    - `runManualMode(sw1, sw2)`  → Moves motor manually
    - `stopAllMotors()`          → Stops motor completely

6️⃣  SPEED CONTROL FUNCTIONS (⚡ Adjusting Speeds)
    - `setMotorSpeed()`          → Adjusts speed for **manual & auto** mode
    - `saveSpeed()`              → **(EEPROM Optimization)** Saves only when changed
    - `updateDebounceTime()`     → Adjusts button debounce time based on speed

7️⃣  BUTTON PRESS FUNCTIONS (🎮 Button Press Detection)
    - `checkBothLongestPress()`  → Detects long button holds
      - 1s Hold → Switch to Auto Mode
      - 3s Hold → Enter Target Position Menu
      - 5s Hold → Enter Speed Adjustment Mode
      - 8s Hold → **5s Continuous Beep & No Action**
    - `setTargetPosition()`      → Adjusts auto mode sweep range

8️⃣  BLUETOOTH FUNCTIONS (📡 Communication)
    - `handleBluetoothCommand()` → Interprets HC-05 commands:
      - 'L' → Rotate Left
      - 'R' → Rotate Right
      - 'S' → Stop Motor
      - '+' / '-' → Change Manual Speed
      - 'U' / 'D' → Change Auto Speed
      - 'A' → Activate Auto Mode
      - 'M' → Deactivate Auto Mode
    - Bluetooth Initialization (`BTserial.begin()` and HC-05 checks)

9️⃣  BEEP FUNCTIONS (🔊 Sound Alerts & Feedback)
    - `beepMultiple()`           → Standard beeping function
    - `beepMultipleDuration()`   → Custom duration for each beep
    - `beepQuickDouble()`        → Two quick beeps (auto mode speed change)
    - `beepLong()`               → One long beep (manual mode speed change)
    - `beepMinMaxAlert()`        → **(NEW)** Beeps when speed reaches min/max
    - `beepTriple()`             → Three quick beeps when switching mode

=================================================================
*/


#include <Arduino.h>
#include <L298N.h>  
#include <EEPROM.h>
#include <SoftwareSerial.h> // Används för att hantera seriell kommunikation

//SoftwareSerial BTserial(3, 2); // HC-05: TX på 3, RX på 4
//SoftwareSerial BTserial(2, 3); // Now pin 2 and pin 3 of Arduino are Serial Rx & Tx pin Respectively
SoftwareSerial BTserial(3, 2); // RX | TX
// ----------------------------
//   PIN CONFIGURATION
// ----------------------------
const int motorPin1 = 1;   
const int motorPin2 = 1;   

const int EN  = 9;   
const int IN1 = 8;   
const int IN2 = 7;   

const int posSw   = 5;
const int rotSw1  = 6;   
const int rotSw2  = 11;  

const int buzzerPin = 10;
const long baudRate = 38400;
char c = ' ';
boolean NL = true;

// ----------------------------
//   GLOBAL VARIABLES
// ----------------------------
bool autoModeActive   = false;
int  targetPosition   = 7;    
// int  currentSpeed     = 125;   // OLD default for speed
int manualSpeed = 125;  // Default speed for manual mode
int autoSpeed = 100;    // Default speed for auto mode

bool bothPressing = false;
unsigned long bothPressStart = 0;

bool pressingAuto = false;
unsigned long pressTimeAuto = 0;

// Debounce och dubbeltryck-räknare
unsigned long lastStopPrintTime = 0;
unsigned long lastPosSwPress = 0;
unsigned long posSwDebounceTime = 400;  
unsigned long lastPressTime = 0;
unsigned long doublePressStartTime = 0;
bool countingDoublePress = false;

L298N myMotor(EN, IN1, IN2);

// ----------------------------
//   FUNCTION DECLARATIONS
// ----------------------------
void checkBothLongestPress(bool sw1, bool sw2);
bool handleAutoButtons();
void runAutomaticMode();
void runManualMode(bool sw1, bool sw2);
void stopAllMotors();
void beepMultiple(int n);
void setTargetPosition();  // Nu deklarerad korrekt
void updateDebounceTime(); 
//bool bleConnected = false;  // Global variabel för att hålla koll på Bluetooth-anslutning

void saveSpeed(int address, int &currentSpeed, int newSpeed) {
  if (currentSpeed != newSpeed) {  // Only write if different
    currentSpeed = newSpeed;       // Update variable
    EEPROM.write(address, newSpeed);  // Save to EEPROM
    Serial.print("[EEPROM] Speed Updated at address ");
    Serial.print(address);  
    Serial.print(": ");
    Serial.println(newSpeed);
  }
}

void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(posSw, INPUT_PULLUP);
  pinMode(rotSw1, INPUT_PULLUP);
  pinMode(rotSw2, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  // Load last saved speeds from EEPROM
  int savedManualSpeed = EEPROM.read(0);  
  int savedAutoSpeed = EEPROM.read(1);

  if (savedManualSpeed >= 25 && savedManualSpeed <= 250) {
      manualSpeed = savedManualSpeed;
  } else {
      manualSpeed = 125;
  }

  if (savedAutoSpeed >= 25 && savedAutoSpeed <= 250) {
      autoSpeed = savedAutoSpeed;
  } else {
      autoSpeed = 100;
  }

  myMotor.setSpeed(manualSpeed);
  myMotor.stop();

  updateDebounceTime();

  //Serial.println("Bluetooth Ready...");
  BTserial.println("Bluetooth Ready...");
  Serial.print("Target position: ");
  Serial.println(targetPosition);
  Serial.print("Manual Saved speed: ");
  Serial.println(manualSpeed);
  Serial.print("Auto Saved speed: ");
  Serial.println(autoSpeed);  
}

void loop() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  checkBothLongestPress(sw1, sw2);


  if (autoModeActive) {
    runAutomaticMode();
  } else {
    runManualMode(sw1, sw2);
  }

  delay(200);  // För att undvika att spamma seriell monitor
}

// ----------------------------
//   SWEEPING DEGREE
// ----------------------------
void setTargetPosition() {
  Serial.println("** setTargetPosition MENU **");
  Serial.print("Target Position: ");
  Serial.println(targetPosition);
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

    // Öka targetPosition
    if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      targetPosition = (targetPosition >= 8) ? 2 : targetPosition + 1;
      Serial.print("New Target Position: ");
      Serial.println(targetPosition);
      beepMultiple(1);
    }

    // Minska targetPosition
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      targetPosition = (targetPosition <= 2) ? 8 : targetPosition - 1;
      Serial.print("New Target Position: ");
      Serial.println(targetPosition);
      beepMultiple(1);
    }

    // Lämna menyn genom att hålla båda knapparna i 1 sekund
    if (sw1 && sw2) {
      unsigned long pressStart = millis();
      while (sw1 && sw2) {
        if (millis() - pressStart >= 1000) {
          Serial.println("Exiting setTargetPosition...");
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
      myMotor.setSpeed(manualSpeed);
      myMotor.backward();
      Serial.println("[BT] Rotating Left");
      BTserial.println("Rotating Left");
      break;

    case 'R':  
      myMotor.setSpeed(manualSpeed);
      myMotor.forward();
      Serial.println("[BT] Rotating Right");
      BTserial.println("Rotating Right");
      break;

    case 'S':  
      myMotor.stop();
      Serial.println("[BT] Stopping Motor");
      BTserial.println("Stopping Motor");
      break;

    case '+':  // ⏫ Increase Manual Speed
      saveSpeed(0, manualSpeed, min(manualSpeed + 25, 250));  // Prevent exceeding 250
      myMotor.setSpeed(manualSpeed);
      Serial.print("[BT] Increased Manual Speed: ");
      Serial.println(manualSpeed);
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      break;

    case '-':  // ⏬ Decrease Manual Speed
      saveSpeed(0, manualSpeed, max(manualSpeed - 25, 25));  // Prevent going below 25
      myMotor.setSpeed(manualSpeed);
      Serial.print("[BT] Decreased Manual Speed: ");
      Serial.println(manualSpeed);
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      break;

    case 'U':  // ⏫ Increase Auto Speed
      saveSpeed(1, autoSpeed, min(autoSpeed + 25, 250));  // Prevent exceeding 250
      Serial.print("[BT] Increased Auto Speed: ");
      Serial.println(autoSpeed);
      BTserial.print("Auto Speed: ");
      BTserial.println(autoSpeed);
      break;

    case 'D':  // ⏬ Decrease Auto Speed
      saveSpeed(1, autoSpeed, max(autoSpeed - 25, 25));  // Prevent going below 25
      Serial.print("[BT] Decreased Auto Speed: ");
      Serial.println(autoSpeed);
      BTserial.print("Auto Speed: ");
      BTserial.println(autoSpeed);
      break;


    case 'A':  
      autoModeActive = true;
      Serial.println("[BT] Auto Mode Activated");
      BTserial.println("Auto Mode Activated");
      break;

    case 'M':  
      autoModeActive = false;
      stopAllMotors();
      Serial.println("[BT] Auto Mode Deactivated");
      BTserial.println("Auto Mode Deactivated");
      break;

    default:
      Serial.println("[BT] Unknown Command");
      BTserial.println("Unknown Command");
      break;
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
    Serial.print("[System] Updated debounce time: ");
    Serial.print(newDebounceTime);
    Serial.println("ms");
    lastDebounceTime = newDebounceTime;  // Store last debounce value
  }

  posSwDebounceTime = newDebounceTime;  // Apply new debounce time
}

// ----------------------------
//   AUTO SWEEP MODE
// ----------------------------
void runAutomaticMode() {
  Serial.println("[Auto] Starting...");
  beepMultiple(0);

  stopAllMotors();
  delay(1000);

  Serial.println("Going backward until posSw=LOW...");
  myMotor.setSpeed(autoSpeed);
  myMotor.backward();

  while (digitalRead(posSw) == HIGH) {
    if (!handleAutoButtons()) return;
  }
  stopAllMotors();
  int posCount = 0;
  delay(1000);

  while (autoModeActive) {
    Serial.println("Pendling UP...");
    myMotor.setSpeed(autoSpeed);
    myMotor.forward();

    while (posCount < targetPosition) {
      if (!handleAutoButtons()) return;

      if (digitalRead(posSw) == LOW && millis() - lastPosSwPress > posSwDebounceTime) {
        lastPosSwPress = millis();
        Serial.println("[DEBUG] posSw CLICKED - UP");
        posCount++;
        Serial.print("posCount (UP) = ");
        Serial.println(posCount);
      }
    }
    stopAllMotors();
    delay(1000);

    Serial.println("Pendling DOWN...");
    myMotor.setSpeed(autoSpeed);
    myMotor.backward();

    while (posCount > 0) {
      if (!handleAutoButtons()) return;

      if (digitalRead(posSw) == LOW && millis() - lastPosSwPress > posSwDebounceTime) {
        lastPosSwPress = millis();
        Serial.println("[DEBUG] posSw CLICKED - DOWN");
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
//   Manual Mode 
// ----------------------------
void runManualMode(bool sw1, bool sw2) {
  static int lastState = -1;  // Store last motor state (0 = STOP, 1 = FORWARD, 2 = BACKWARD)
  static unsigned long lastPrintTime = 0;  // Timestamp to limit printing frequency

  if (sw1 && !sw2) {
    myMotor.setSpeed(manualSpeed);
    myMotor.forward();

    if (lastState != 1 && millis() - lastPrintTime >= 1000) {  // Only print every 1s if state changed
      Serial.println("[Manual] Motor B => FORWARD");
      lastState = 1;
      lastPrintTime = millis();
    }
    
    //Dont think this is needed, leaves here for now
    //updateDebounceTime();  // Ensure debounce updates if speed changes

  } else if (!sw1 && sw2) {
    myMotor.setSpeed(manualSpeed);
    myMotor.backward();

    if (lastState != 2 && millis() - lastPrintTime >= 1000) {  // Only print every 1s if state changed
      Serial.println("[Manual] Motor B => BACKWARD");
      lastState = 2;
      lastPrintTime = millis();
    }

    updateDebounceTime();

  } else {
    myMotor.stop();

    if (lastState != 0 && millis() - lastPrintTime >= 1000) {  // Only print every 1s if state changed
      Serial.println("[Manual] Motor B => STOP");
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
  else if (bothPressing && (!sw1 || !sw2)) {  
    // If one button is released after being held down
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
      else if (held >= 5000) {  
        Serial.println("[Auto] 5s => Entering Motor Speed Adjustment Mode (3 fast beeps)");
        beepMultipleDuration(3, 100); // Three short beeps
        setMotorSpeed();  
      } 
      else if (held >= 3000) {  
        Serial.println("[Auto] 3s => Set Target Position (2 beeps)");
        beepMultipleDuration(2, 200); // Two beeps
        setTargetPosition();
      } 
      else if (held >= 1000) {  
        Serial.println("[Auto] 1s => Activating Auto Mode (1 beep)");
        beepMultipleDuration(1, 300); // One beep
        autoModeActive = true;
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
}

// ----------------------------
//   SET MOTOR SPEED (FOR MANUAL AND AUTO MODE)
// ----------------------------
void setMotorSpeed() {
  Serial.println("** Entering Motor Speed Adjustment Mode **");
  stopAllMotors();

  bool exitMenu = false;
  bool adjustingAutoSpeed = false;  // Tracks whether user is adjusting auto or manual speed
  unsigned long lastButtonPress = 0;
  unsigned long holdStart = 0;  // To track how long both buttons are held
  const unsigned long buttonDebounce = 300;
  const unsigned long exitHoldTime = 3000;  // 3s hold time to exit
  const int minSpeed = 25;   // Minimum speed
  const int maxSpeed = 250;  // Maximum speed

  Serial.println("Now adjusting: MANUAL MODE SPEED");

  while (!exitMenu) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);

    // ADD A SMALL DELAY INSIDE MENU TO PREVENT SENSITIVE NAVIGATION
    delay(200);  

    // **Increase Speed** (Stops at max value)
    if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (adjustingAutoSpeed) {
        if (autoSpeed < maxSpeed) {
          autoSpeed += 25;
          myMotor.setSpeed(autoSpeed);
          EEPROM.write(1, autoSpeed);
          Serial.print("New Auto Mode Speed: ");
          Serial.println(autoSpeed);
          beepQuickDouble(); // Auto Mode: Two quick beeps
        } else {
          Serial.println("Auto Mode Speed is at MAX (250)");
          beepMinMaxAlert(); // Beep alert for reaching MAX
        }
      } else {
        if (manualSpeed < maxSpeed) {
          saveSpeed(0, manualSpeed, min(manualSpeed + 25, 250));
          myMotor.setSpeed(manualSpeed);
          EEPROM.write(0, manualSpeed);
          Serial.print("New Manual Mode Speed: ");
          Serial.println(manualSpeed);
          beepLong(); // Manual Mode: One long beep
        } else {
          Serial.println("Manual Mode Speed is at MAX (250)");
          beepMinMaxAlert(); // Beep alert for reaching MAX
        }
      }
    }

    // **Decrease Speed** (Stops at min value)
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (adjustingAutoSpeed) {
        if (autoSpeed > minSpeed) {
          autoSpeed -= 25;
          myMotor.setSpeed(autoSpeed);
          EEPROM.write(1, autoSpeed);
          Serial.print("New Auto Mode Speed: ");
          Serial.println(autoSpeed);
          beepQuickDouble(); // Auto Mode: Two quick beeps
        } else {
          Serial.println("Auto Mode Speed is at MIN (25)");
          beepMinMaxAlert(); // Beep alert for reaching MIN
        }
      } else {
        if (manualSpeed > minSpeed) {
          saveSpeed(0, manualSpeed, max(manualSpeed - 25, 25));
          myMotor.setSpeed(manualSpeed);
          EEPROM.write(0, manualSpeed);
          Serial.print("New Manual Mode Speed: ");
          Serial.println(manualSpeed);
          beepLong(); // Manual Mode: One long beep
        } else {
          Serial.println("Manual Mode Speed is at MIN (25)");
          beepMinMaxAlert(); // Beep alert for reaching MIN
        }
      }
    }

    // **Switch between Manual Mode Speed and Auto Mode Speed**
    if (sw1 && sw2) {
      if (holdStart == 0) {  // Start tracking hold time
        holdStart = millis();
      }

      if (millis() - holdStart >= 1000) {  // Hold 1s to switch mode
        adjustingAutoSpeed = !adjustingAutoSpeed;  // Toggle between manual and auto mode

        if (adjustingAutoSpeed) {
          Serial.println("Switched to adjusting: AUTO MODE SPEED");
        } else {
          Serial.println("Switched to adjusting: MANUAL MODE SPEED");
        }

        beepTriple();  // Three short beeps for switching mode
        delay(1000);  // Prevent accidental double-switch
        holdStart = 0;
      }

      // **Exit the menu if held for 3 seconds**
      if (millis() - holdStart >= exitHoldTime) {
        Serial.println("Exiting Motor Speed Adjustment Mode...");
        beepLong();  // Long beep for exit confirmation
        exitMenu = true;
        break;
      }
    } else {
      holdStart = 0;  // Reset hold timer if buttons are released
    }
  }
}


// ----------------------------
//   HANDLE AUTO MODE EXIT BUTTONS
// ----------------------------
bool handleAutoButtons() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  if ((sw1 && sw2) || (!sw1 && !sw2)) {
    pressingAuto = false;
    return true;
  }

  if (!pressingAuto) {
    pressingAuto = true;
    pressTimeAuto = millis();
  } else if (millis() - pressTimeAuto >= 1000) {  // 1 sek hållning av en knapp
    Serial.println("ABORT auto-läge (knapp >=1s).");
    beepMultiple(1);
    stopAllMotors();

    Serial.println("[System] Väntar 1 sekund innan återgång till manuellt läge...");
    delay(1000);
    autoModeActive = false;
    pressingAuto = false;
    return false;
  }
  return true;
}

// ----------------------------
//   STOP MOTORS
// ----------------------------
void stopAllMotors() {
  myMotor.stop();
  Serial.println("[System] Motors Stopped.");
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
// 🔊 Auto Mode: Two quick beeps
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

