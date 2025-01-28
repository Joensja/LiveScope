// ----------------------------------------------------------
//   PIN CONFIGURATION
// ----------------------------------------------------------
const int motorPin1 = 3;
const int motorPin2 = 4;
const int posSw     = 5;
const int rotSw1    = 6;
const int rotSw2    = 7;
const int buzzerPin = 8;

// ----------------------------------------------------------
//   GLOBAL VARIABLES
// ----------------------------------------------------------
bool autoModeActive = false;
int  targetPosition = 3;  // This will be set to 4 in auto mode below
unsigned long buttonPressTime = 0;  // For activating auto mode (3s)

int currentDirection = 0; // +1 = forward, -1 = backward, 0 = off

// ----------------------------------------------------------
//   setMotorDirection(dir)
//   dir = +1 (forward), -1 (backward), 0 (stop)
// ----------------------------------------------------------
void setMotorDirection(int dir) {
  currentDirection = dir;
  switch (dir) {
    case +1:  // forward
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      break;
    case -1:  // backward
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      break;
    default:  // stop
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      break;
  }
}

// ----------------------------------------------------------
//   setup()
// ----------------------------------------------------------
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(posSw,     INPUT_PULLUP);
  pinMode(rotSw1,    INPUT_PULLUP);
  pinMode(rotSw2,    INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  setMotorDirection(0);  // motor off initially
  noTone(buzzerPin);

  Serial.begin(9600);
  Serial.println("System startup. Manual mode by default.");
}

// ----------------------------------------------------------
//   loop()
// ----------------------------------------------------------
void loop() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  // Press both buttons >= 3s => activate auto mode
  if (sw1 && sw2) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
    } 
    else if ((millis() - buttonPressTime >= 3000) && !autoModeActive) {
      // After 3s => auto mode
      tone(buzzerPin, 1000);
      delay(1000);  // 1s beep
      noTone(buzzerPin);

      autoModeActive = true;
      targetPosition = 4;  // <--- Default changed to 4

      // Wait until both buttons are released before entering menu
      while ((digitalRead(rotSw1) == LOW) && (digitalRead(rotSw2) == LOW)) {
        // do nothing
      }

      setTargetPosition();  // let user adjust targetPosition if desired
      buttonPressTime = 0;
    }
  } 
  else {
    buttonPressTime = 0;
  }

  // Choose mode
  if (autoModeActive) {
    runAutomaticMode();
  } else {
    runManualMode();
  }
}

// ----------------------------------------------------------
//   runManualMode()
// ----------------------------------------------------------
void runManualMode() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  if (sw1 && sw2) {
    setMotorDirection(0);
  }
  else if (sw1) {
    setMotorDirection(+1); // forward
  }
  else if (sw2) {
    setMotorDirection(-1); // backward
  }
  else {
    setMotorDirection(0);  // off
  }
}

// ----------------------------------------------------------
//   runAutomaticMode()
//   - 3 quick beeps
//   - 1s off before start
//   - move backward while posSw == HIGH, stop when LOW => posCount=0
//   - wait 1s after zero-locate
//   - pendulum up/down with 1s wait when changing direction
//   - single short press does nothing
//   - single long press (>=2s) => abort auto mode
// ----------------------------------------------------------
void runAutomaticMode() {
  Serial.println("\n=== AUTOMATIC MODE ACTIVATED ===");

  // 1) Three quick beeps
  beepMultiple(3);

  // 2) 1s motor off
  setMotorDirection(0);
  delay(1000);
  Serial.println("Motor off 1s. Going backward...");

  // 3) move backward while posSw == HIGH
  setMotorDirection(-1);

  while (digitalRead(posSw) == HIGH) {
    if (!handleAutoButtons()) {
      // auto mode aborted
      return;
    }
  }

  // posSw == LOW => stop => posCount=0
  setMotorDirection(0);
  int posCount = 0;
  Serial.println("Zero-locate (posSw=LOW), motor stopped, posCount=0.");

  // wait 1s before pendulum
  delay(1000);
  Serial.println("Now starting pendulum cycle...");

  // 4) Pendulum
  while (autoModeActive) {
    // UP cycle (0->targetPosition)
    setMotorDirection(+1); // forward
    Serial.println("Pendulum UP (0->targetPosition).");

    while (posCount < targetPosition) {
      if (!handleAutoButtons()) return;
      if (digitalRead(posSw) == LOW) {
        posCount++;
        delay(300);
        Serial.print("posCount (up) = ");
        Serial.println(posCount);
      }
    }

    // stop
    setMotorDirection(0);
    Serial.print("Reached targetPosition (");
    Serial.print(targetPosition);
    Serial.println("). Motor off.");

    // wait 1s before changing direction
    delay(1000);

    // DOWN cycle (targetPosition->0)
    setMotorDirection(-1);
    Serial.println("Pendulum DOWN (targetPosition->0).");

    while (posCount > 0) {
      if (!handleAutoButtons()) return;
      if (digitalRead(posSw) == LOW) {
        posCount--;
        delay(300);
        Serial.print("posCount (down) = ");
        Serial.println(posCount);
      }
    }

    // stop
    setMotorDirection(0);
    Serial.println("Back at posCount=0, motor off.");

    // wait 1s before changing direction
    delay(1000);
  }

  Serial.println("=== EXITING AUTOMATIC MODE ===");
}

// ----------------------------------------------------------
//   handleAutoButtons()
//   - Do nothing for short press (<2s)
//   - If single button pressed >=2s => abort auto mode
//   - Both or none => reset
// ----------------------------------------------------------
bool handleAutoButtons() {
  static unsigned long pressTime = 0;
  static bool pressing = false;

  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  // A) If both or none => reset pressing
  if ((sw1 && sw2) || (!sw1 && !sw2)) {
    pressing = false;
    pressTime = 0;
    return true;
  }

  // B) Exactly one button
  if (!pressing) {
    // first time we see a single button
    pressing = true;
    pressTime = millis();
    // do not stop the motor here (no short press function)
  }
  else {
    // pressing == true
    unsigned long held = millis() - pressTime;
    if (held >= 2000) {
      // abort auto mode
      Serial.println("Auto mode aborted (button held >=2s).");
      tone(buzzerPin, 1000);
      delay(1000);
      noTone(buzzerPin);
      setMotorDirection(0);
      delay(1000);

      autoModeActive = false;
      pressing = false;
      pressTime = 0;
      return false;
    }
  }
  return true;
}

// ----------------------------------------------------------
//   setTargetPosition()
//   - user can set targetPosition (1..5) before auto mode
//   - hold one button >=1s => +1/-1
//   - both => exit
// ----------------------------------------------------------
void setTargetPosition() {
  while (true) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);

    // both => exit menu
    if (sw1 && sw2) {
      delay(1000);
      if ((digitalRead(rotSw1) == LOW) && (digitalRead(rotSw2) == LOW)) {
        Serial.println("Exiting target position menu.");
        break;
      }
    }

    // Increase
    if (sw1 && !sw2) {
      unsigned long start = millis();
      while (digitalRead(rotSw1) == LOW) {
        if (digitalRead(rotSw2) == LOW) break; // both => abort
        if (millis() - start >= 1000) {
          if (targetPosition < 5) targetPosition++;
          beepMultiple(targetPosition);
          Serial.print("Target position increased to: ");
          Serial.println(targetPosition);

          // wait until button is released
          while (digitalRead(rotSw1) == LOW) {}
          delay(250);
          break;
        }
      }
    }

    // Decrease
    if (sw2 && !sw1) {
      unsigned long start = millis();
      while (digitalRead(rotSw2) == LOW) {
        if (digitalRead(rotSw1) == LOW) break; // both => abort
        if (millis() - start >= 1000) {
          if (targetPosition > 1) targetPosition--;
          beepMultiple(targetPosition);
          Serial.print("Target position decreased to: ");
          Serial.println(targetPosition);

          // wait until button is released
          while (digitalRead(rotSw2) == LOW) {}
          delay(250);
          break;
        }
      }
    }
  }
}

// ----------------------------------------------------------
//   beepMultiple(count)
//   - plays "count" pulses: 250ms tone + 250ms silence
// ----------------------------------------------------------
void beepMultiple(int count) {
  for (int i = 0; i < count; i++) {
    tone(buzzerPin, 1000);
    delay(250);
    noTone(buzzerPin);
    delay(250);
  }
}
