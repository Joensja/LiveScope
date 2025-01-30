#include <Arduino.h>
#include <L298N.h>  

// ----------------------------
//   PIN CONFIGURATION
// ----------------------------

const int motorPin1 = 2;   
const int motorPin2 = 4;   

const int EN  = 9;   
const int IN1 = 8;   
const int IN2 = 7;   

const int posSw   = 5;
const int rotSw1  = 6;   
const int rotSw2  = 11;  

const int buzzerPin = 10;

// ----------------------------
//   GLOBAL VARIABLES
// ----------------------------
bool autoModeActive   = false;
int  targetPosition   = 7;    
int  currentSpeed     = 125;   

bool bothPressing = false;
unsigned long bothPressStart = 0;

bool pressingAuto = false;
unsigned long pressTimeAuto = 0;

// Variabel för att reglera debug-utskrift av stoppad motor
unsigned long lastStopPrintTime = 0;

L298N myMotor(EN, IN1, IN2);

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(posSw,   INPUT_PULLUP);
  pinMode(rotSw1,  INPUT_PULLUP);
  pinMode(rotSw2,  INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  myMotor.setSpeed(currentSpeed);
  myMotor.stop();

  Serial.begin(9600);
  Serial.println("System startup. Manual mode by default.");
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
}

// ----------------------------
//   MANUAL MODE
// ----------------------------
void runManualMode(bool sw1, bool sw2) {
  if (sw1 && !sw2) {
    myMotor.setSpeed(currentSpeed);
    myMotor.forward();
    Serial.println("[Manual] Motor B => FORWARD");
  } else if (!sw1 && sw2) {
    myMotor.setSpeed(currentSpeed);
    myMotor.backward();
    Serial.println("[Manual] Motor B => BACKWARD");
  } else {
    myMotor.stop();
    
    if (millis() - lastStopPrintTime >= 1000) {
      Serial.println("[Manual] Motor B => STOP");
      lastStopPrintTime = millis();
    }
  }
}

// ----------------------------
//   AUTO MODE
// ----------------------------
void runAutomaticMode() {
  Serial.println("[Auto] Starting...");
  beepMultiple(3);

  stopAllMotors();
  delay(1000);

  Serial.println("Going backward until posSw=LOW...");
  myMotor.setSpeed(currentSpeed);
  myMotor.backward();

  while (digitalRead(posSw) == HIGH) {
    if (!handleAutoButtons()) return;
  }
  stopAllMotors();
  int posCount = 0;
  delay(1000);

  while (autoModeActive) {
    Serial.println("Pendling UP...");
    myMotor.setSpeed(currentSpeed);
    myMotor.forward();

    while (posCount < targetPosition) {
      if (!handleAutoButtons()) return;

      if (digitalRead(posSw) == LOW) {
        delay(150);  
        if (digitalRead(posSw) == LOW) {  
          posCount++;
          Serial.print("posCount (UP) = ");
          Serial.println(posCount);
          Serial.println("[DEBUG] posSw CLICKED - UP");
        }
      }
    }
    stopAllMotors();
    delay(1000);

    Serial.println("Pendling DOWN...");
    myMotor.setSpeed(currentSpeed);
    myMotor.backward();

    while (posCount > 0) {
      if (!handleAutoButtons()) return;

      if (digitalRead(posSw) == LOW) {
        delay(150);  
        if (digitalRead(posSw) == LOW) {  
          posCount--;
          Serial.print("posCount (DOWN) = ");
          Serial.println(posCount);
          Serial.println("[DEBUG] posSw CLICKED - DOWN");
        }
      }
    }
    stopAllMotors();
    delay(1000);
  }
}

// ----------------------------
//   CHANGELOG
// ----------------------------
/*
# Changelog

## Version: 1.1
### Changes since the timing adjustments for auto mode, target position, and set speed:

- **Debounce added to position switch** (`posSw`) to prevent double counting of position steps.
- **Debug log added** to show when the position switch is triggered:  
  - `"posSw CLICKED - UP"`  
  - `"posSw CLICKED - DOWN"`
- **Auto mode exit delay reduced** from **2000ms to 500ms**.
- **Auto mode activation changed** from **3s hold to 1s double press**.
- **Target position menu activation changed** from **5s to 3s hold**.
- **Motor speed menu activation changed** from **7s to 5s hold**.
- **Manual mode "STOP" debug message limited** to **once per second** to reduce spam.
*/

bool handleAutoButtons() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  if ((sw1 && sw2) || (!sw1 && !sw2)) return true;
  if (!pressingAuto) {
    pressingAuto = true;
    pressTimeAuto = millis();
  } else if ((millis() - pressTimeAuto) >= 500) {  
    Serial.println("ABORT auto-läge (knapp >=500ms).");
    stopAllMotors();
    autoModeActive = false;
    pressingAuto = false;
    return false;
  }
  return true;
}

void stopAllMotors() {
  myMotor.stop();
}

void checkBothLongestPress(bool sw1, bool sw2) {
  if (!bothPressing && sw1 && sw2) {
    bothPressing = true;
    bothPressStart = millis();
  } else if (bothPressing && (!sw1 || !sw2)) {
    unsigned long held = millis() - bothPressStart;
    bothPressing = false;

    if (held >= 5000) {  
      Serial.println("[Auto] 5s => setMotorSpeed (3 beeps)");
      beepMultiple(3);
      setMotorSpeed();
    } else if (held >= 3000) {  
      Serial.println("[Auto] 3s => setTargetPosition (5 beeps)");
      beepMultiple(5);
      setTargetPosition();
    } else if (held >= 1000) {  
      Serial.println("[Auto] 1s => Aktiverar auto-läge (1 beep)");
      beepMultiple(1);
      autoModeActive = true;
    }
  }
}

void beepMultiple(int n) {
  for (int i = 0; i < n; i++) {
    tone(buzzerPin, 1000);
    delay(250);
    noTone(buzzerPin);
    delay(250);
  }
}
