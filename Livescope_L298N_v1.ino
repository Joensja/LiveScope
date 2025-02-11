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
#include <MotorDriver.h>

//SoftwareSerial BTserial(3, 2); // HC-05: TX på 3, RX på 4
//SoftwareSerial BTserial(2, 3); // Now pin 2 and pin 3 of Arduino are Serial Rx & Tx pin Respectively
SoftwareSerial BTserial(3, 2); // RX | TX

int driverMode = 1; // Choose between 1 (L298P) or 2 (L298N);

// ----------------------------
//   PIN CONFIGURATION
// ----------------------------
int EN, IN1, IN2, posSw, rotSw1, rotSw2, buzzerPin;
L298N* myMotor = nullptr;

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
    Serial.print("[EEPROM] Speed Updated at address ");
    Serial.print(address);  
    Serial.print(": ");
    Serial.println(newSpeed);
  }
}

void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);

  // Select motor driver and corresponding I/O configuration, different depending on the motor driver
// Motor Driver is set above at "int driverMode = 1;"
if (driverMode == 1) { // Shield board "Aliexpress board" L298NH/L298P
    Serial.println("Using L298P Motor Driver");
    IN1 = 12;  // Direction (HIGH = Forward, LOW = Backward)
    EN  = 3;   // PWM control for speed
    //IN2 = 9;   // Brake
    posSw   = 7; // Rotation position pin
    rotSw1  = 6; // Turn right
    rotSw2  = 11; // Turn left
    buzzerPin = 10;
  } else if (driverMode == 2) { // Externa boarn L298H 
      Serial.println("Using L298N Motor Driver");
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

  //Serial.println("Bluetooth Ready...");
  BTserial.println("Bluetooth Ready...");
  Serial.print("Sweep Angle: ");
  Serial.println(sweep_angle);
  Serial.print("Manual speed: ");
  Serial.println(manualSpeed);
  Serial.print("sweep Speed: ");
  Serial.println(sweepSpeed);  
}

void loop() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  checkBothLongestPress(sw1, sw2);

  if (sweepModeActive) {
    runsweepmaticMode();
  } else {
    runManualMode(sw1, sw2);
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
//   SWEEPING DEGREE
// ----------------------------
void setSweepAngle() {
  Serial.println("** setSweepAngle MENU **");
  Serial.print("SweepAngle: ");
  Serial.println(sweep_angle);
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
      Serial.print("New Sweep Angle: ");
      Serial.println(sweep_angle);
      beepMultiple(1);
    }

    // Minska sweep_angle
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      sweep_angle = (sweep_angle <= 2) ? 8 : sweep_angle - 1;
      Serial.print("New Sweep Angle: ");
      Serial.println(sweep_angle);
      beepMultiple(1);
    }

    // Lämna menyn genom att hålla båda knapparna i 1 sekund
    if (sw1 && sw2) {
      unsigned long pressStart = millis();
      while (sw1 && sw2) {
        if (millis() - pressStart >= 1000) {
          Serial.println("Exiting setSweepAngle...");
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
      Serial.print("[BT] Increased Manual Speed: ");
      Serial.println(manualSpeed);
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      break;

    case '-':  // ⏬ Decrease Manual Speed
      saveSpeed(0, manualSpeed, max(manualSpeed - 25, 25));  // Prevent going below 25
      myMotor->setSpeed(manualSpeed);
      Serial.print("[BT] Decreased Manual Speed: ");
      Serial.println(manualSpeed);
      BTserial.print("Manual Speed: ");
      BTserial.println(manualSpeed);
      break;

    case 'U':  // ⏫ Increase sweep Speed
      saveSpeed(1, sweepSpeed, min(sweepSpeed + 25, 250));  // Prevent exceeding 250
      Serial.print("[BT] Increased sweep Speed: ");
      Serial.println(sweepSpeed);
      BTserial.print("sweep Speed: ");
      BTserial.println(sweepSpeed);
      break;

    case 'D':  // ⏬ Decrease sweep Speed
      saveSpeed(1, sweepSpeed, max(sweepSpeed - 25, 25));  // Prevent going below 25
      Serial.print("[BT] Decreased sweep Speed: ");
      Serial.println(sweepSpeed);
      BTserial.print("sweep Speed: ");
      BTserial.println(sweepSpeed);
      break;


    case 'A':  
      sweepModeActive = true;
      Serial.println("[BT] sweep Mode Activated");
      BTserial.println("sweep Mode Activated");
      break;

    case 'M':  
      sweepModeActive = false;
      stopAllMotors();
      Serial.println("[BT] sweep Mode Deactivated");
      BTserial.println("sweep Mode Deactivated");
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
//   SWEEP MODE
// ----------------------------
void runsweepmaticMode() {
    Serial.println("[sweep] Starting...");
    beepMultiple(0);

    stopAllMotors();
    delay(1000);

    Serial.println("Going backward until posSw=LOW...");

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
        Serial.println("Pendling UP...");
        
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
                Serial.println("[DEBUG] posSw CLICKED - UP");
                posCount++;
                Serial.print("posCount (UP) = ");
                Serial.println(posCount);
            }
        }

        stopAllMotors();
        delay(1000);

        Serial.println("Pendling DOWN...");
        
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
            Serial.println("[Manual] Motor B => FORWARD");
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
            Serial.println("[Manual] Motor B => BACKWARD");
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
        Serial.println("[sweep] 5s => Entering Motor Speed Adjustment Mode (3 fast beeps)");
        beepMultipleDuration(3, 100); // Three short beeps
        setMotorSpeed();  
      } 
      else if (held >= 3000) {  
        Serial.println("[sweep] 3s => Set Sweep Angle (2 beeps)");
        beepMultipleDuration(2, 200); // Two beeps
        setSweepAngle();
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
}

// ----------------------------
//   SET MOTOR SPEED (FOR MANUAL AND SWEEP MODE)
// ----------------------------
void setMotorSpeed() {
    Serial.println("** Entering Motor Speed Adjustment Mode **");
    stopAllMotors();

    bool exitMenu = false;
    bool adjustingSweepSpeed = false;  // True = Adjusting sweep speed, False = Adjusting manual speed
    bool adjustingManualSpeed = true;  // Default to manual speed adjustment
    unsigned long lastButtonPress = 0;
    unsigned long holdStart = 0;
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
                    Serial.print("New SWEEP MODE Speed: ");
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
                    Serial.print("New MANUAL MODE Speed: ");
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
                    Serial.print("New SWEEP MODE Speed: ");
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
                    Serial.print("New MANUAL MODE Speed: ");
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
}



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
    Serial.println("ABORT sweep-läge (knapp >=300ms).");
    beepMultiple(1);
    stopAllMotors();

    Serial.println("[System] Waits 1s to activate manual mode");
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
