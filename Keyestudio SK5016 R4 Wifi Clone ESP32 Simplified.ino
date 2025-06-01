/*
  ------------------------------------------------------------------------------
  ESP32 Motor Control with WiFi and SpeedMenu
  ------------------------------------------------------------------------------
  NOTE!!!!
  I NEVER GOT THE KY003 TO WORK ON THE KEYESTUDIOS SK5016 ESP32-WROOM-32 UNIT WITH THE L298NH MOTOR SHIELD. 
  I USED A MAGNETIC SWITCH INSTEAD ON THE INPUT FOR POSSW INSTEAD WORKS PERFECTLY!!
  //-----------------------------------------------------------------------------

  This code controls a motor using an ESP32, with input from physical buttons and a web interface.
  It supports manual operation, automatic "sweep" motion, and a menu for adjusting speed.

  Main Features:
  - WiFi Access Point: Sets up a wireless network and provides a simple web page for status/info.
  - Manual Mode: Control the motor directly with two buttons (right/left).
  - Sweep Mode: Automatic motor movement, either time-based or using a limit switch/Hall sensor.
  - Speed Menu: Enter by holding the pedal button for 3 seconds to adjust motor speed.
      + Increase speed: Press and release SW1 (right button).
      + Decrease speed: Press and release SW2 (left button).
      + Exit menu: Hold both buttons or the pedal for >1 second.

  Key Functions:
  - Button debounce logic to avoid false readings due to button bounce.
  - Buzzer for sound feedback when changing menu or on alerts.
  - EEPROM saves the motor speed between restarts.
  - Double-tap the pedal button (within 2 seconds) to start sweep mode.

  ESP32 Core v3.x Adaptations:
  - Uses ledcAttach(pin, freq, resolution) for PWM, instead of ledcSetup/ledcAttachPin.
  - PWM output is used for motor speed control.

  User Tips:
  - Hold the pedal for 3 seconds to enter SpeedMenu (3 beeps confirm entry).
  - Exit SpeedMenu by holding either the pedal or both control buttons for 1 second.
  - Adjust speed in the menu using SW1 (up) and SW2 (down).
  - Double-tap the pedal to start sweep mode.

  See the code below for details!
*/

#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// =================== WIFI SETTINGS ===================
const char* ssid = "WLi-Transducer_Pole";
const char* password = "123456789";
WebServer server(80);

// =================== PIN DEFINITIONS ===================
const int IN1 = 19;
const int EN = 25;
const int posSw = 14;
const int rotSw1 = 16;
const int rotSw2 = 27;
const int buzzerPin = 26;
const int pedalButton = 5;
const int hallSensorPin = 33;

// =================== GLOBAL VARIABLES ===================
const int referencePWM = 250;
int motorSpeed = 125;
int lastDirection = 1;
bool useHallSensor = false;
unsigned long sweepTimeoutMs = 6000;
unsigned long sweepTimeMs = 4000;
const int pwmFreq = 1000;
const int pwmResolution = 8; // 8-bit (0-255)

// =================== STATE MACHINE ===================
enum MainState { MANUAL_MODE, SWEEP_MODE, SPEED_MENU };
MainState mainState = MANUAL_MODE;

enum SweepModeType { SWEEP_SWITCH, SWEEP_TIME };
SweepModeType sweepModeType = SWEEP_SWITCH;
bool sweepTimeoutAlarm = false;

// =================== BUTTON DEBOUNCE ===================
// Debounce config
const unsigned long DEBOUNCE_MS = 40;
struct Button {
  const int pin;
  bool state;
  bool lastRaw;
  unsigned long lastChange;
};

Button btnSw1   = { rotSw1, HIGH, HIGH, 0 };
Button btnSw2   = { rotSw2, HIGH, HIGH, 0 };
Button btnPedal = { pedalButton, HIGH, HIGH, 0 };

// Debounce loop for all buttons
void updateButton(Button &btn) {
  bool raw = digitalRead(btn.pin);
  if (raw != btn.lastRaw) {
    btn.lastChange = millis();
    btn.lastRaw = raw;
  }
  if ((millis() - btn.lastChange) > DEBOUNCE_MS) {
    btn.state = raw;
  }
}

// Knapplogik för att använda i resten av koden
bool isSw1Pressed()    { return btnSw1.state == LOW; }
bool isSw2Pressed()    { return btnSw2.state == LOW; }
bool isPedalPressed()  { return btnPedal.state == LOW; }
bool isAnyButton()     { return isSw1Pressed() || isSw2Pressed() || isPedalPressed(); }

// =================== MOTOR CONTROL ===================
void motorForward(int speed)   { digitalWrite(IN1, HIGH); ledcWrite(EN, speed); }
void motorBackward(int speed)  { digitalWrite(IN1, LOW);  ledcWrite(EN, speed); }
void motorStop()               { ledcWrite(EN, 0); digitalWrite(IN1, LOW); }
void stopAllMotors()           { motorStop(); }

// =================== BUZZER ===================
void beep(int duration = 250) {
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
}

void beepMultiple(int n) {
  for (int i = 0; i < n; i++) {
    beep(250);
    delay(250);
  }
}

void beepMultipleDuration(int n, int duration) {
  for (int i = 0; i < n; i++) {
    beep(duration);
    delay(200);
  }
}

void beepLong() {
  beep(400);
}

void beepMinMaxAlert() {
  beepMultiple(2);
  beepLong();
}

// =================== EEPROM ===================
void saveSpeed(int newSpeed) {
  newSpeed = constrain(newSpeed, 75, 250);
  if (motorSpeed != newSpeed) {
    motorSpeed = newSpeed;
    EEPROM.write(0, newSpeed);
    EEPROM.commit();
  }
}

// =================== LIMIT SENSOR ===================
bool isSensorActive() {
  if (useHallSensor) return digitalRead(hallSensorPin) == LOW;
  else               return digitalRead(posSw) == LOW;
}

// =================== SWEEP MODE ===================
void runSweepMode() {
  beepMultiple(1);
  sweepTimeoutAlarm = false;
  bool sweepActive = true;
  int sweepDirection = (sweepModeType == SWEEP_TIME) ? 1 : lastDirection;
  unsigned long ignoreUntil = 0;

  Serial.println("[SWEEP-DBG] === Sweepmode startar ===");

  while (sweepActive) {
    server.handleClient();
    updateButton(btnSw1);
    updateButton(btnSw2);

    // Avbryt sweep med sw1 eller sw2 1s
    static unsigned long swHoldStart = 0;
    if (isSw1Pressed() || isSw2Pressed() || isPedalPressed()) {
      if (swHoldStart == 0) swHoldStart = millis();
      if (millis() - swHoldStart > 500) {
        sweepActive = false;
        beepMultiple(2);
        Serial.println("[SWEEP-DBG] Avbryter sweepmode via sw1/sw2/pedal");
        break;
      }
    } else {
      swHoldStart = 0;
    }


    // Kör ett svep: Stoppa alltid innan nytt move
    stopAllMotors(); delay(50);

    Serial.printf("[SWEEP-DBG] Motor igång: %s | motorSpeed=%d\n",
      sweepDirection == 1 ? "Forward" : "Backward", motorSpeed);

    if (sweepDirection == 1) {
      motorForward(motorSpeed);
    } else {
      motorBackward(motorSpeed);
    }

    unsigned long actionStart = millis();

    if (sweepModeType == SWEEP_SWITCH) {
  static bool lastSensor = false; // Lagrar föregående sensorstatus

  while (true) {
    server.handleClient();
    updateButton(btnSw1);
    updateButton(btnSw2);

    // --- Edge-detection: Skriv bara ut när sensorn TRIGGAS (går från 0 till 1)
    bool currentSensor = isSensorActive();
    if (currentSensor && !lastSensor) {
      Serial.printf("[SWEEP-DBG] LIMIT SWITCH TRIGGERED! (sensorActive=1) | t=%lu ms\n", millis());
    }
    lastSensor = currentSensor;
    // ---

    // Samma exit även i denna loop
    static unsigned long innerSwHoldStart = 0;
    if (isSw1Pressed() || isSw2Pressed()) {
      if (innerSwHoldStart == 0) innerSwHoldStart = millis();
      if (millis() - innerSwHoldStart > 500) {
        sweepActive = false;
        beepMultiple(2);
        Serial.println("[SWEEP-DBG] Avbryter sweepmode via sw1/sw2 (inner loop)");
        break;
      }
    } else {
      innerSwHoldStart = 0;
    }

    if (!sweepActive) break;

    if (millis() > ignoreUntil && isSensorActive()) {
      stopAllMotors(); delay(500);
      sweepDirection = -sweepDirection;
      lastDirection = sweepDirection;
      ignoreUntil = millis() + 2000;
      Serial.printf("[SWEEP-DBG] Riktning byts! Ny sweepDirection=%d\n", sweepDirection);
      break;
    }
    if (millis() - actionStart > sweepTimeoutMs) {
      stopAllMotors();
      beepMultiple(5);
      Serial.println("[SWEEP-DBG] [ALARM] Timeout! Ingen trigger – sweepmode stoppad.");
      sweepTimeoutAlarm = true;
      sweepActive = false;
      break;
    }
    delay(10);
  }
}

    else if (sweepModeType == SWEEP_TIME) {
      unsigned long effectiveSweepTime = sweepTimeMs * referencePWM / max(motorSpeed, 1);
      unsigned long startTime = millis();
      Serial.printf("[SWEEP-DBG] [TIME] EffectiveSweepTime=%lu ms (speed=%d)\n", effectiveSweepTime, motorSpeed);

      while ((millis() - startTime < effectiveSweepTime) && sweepActive) {
        server.handleClient();
        updateButton(btnSw1);
        updateButton(btnSw2);

        static unsigned long swHoldTime = 0;
        if (isSw1Pressed() || isSw2Pressed()) {
          if (swHoldTime == 0) swHoldTime = millis();
          if (millis() - swHoldTime > 1000) {
            sweepActive = false;
            beepMultiple(2);
            Serial.println("[SWEEP-DBG] Avbryter sweepmode via sw1/sw2 (TIME)");
            break;
          }
        } else {
          swHoldTime = 0;
        }

        if (!sweepActive) break;
        delay(10);
      }
      stopAllMotors(); delay(500);
      sweepDirection = -sweepDirection;
      lastDirection = sweepDirection;
      Serial.printf("[SWEEP-DBG] [TIME] Byter riktning: Ny sweepDirection=%d\n", sweepDirection);
    }
  }
  stopAllMotors();
  delay(200);

  Serial.println("[SWEEP-DBG] === Sweepmode avslutad ===");
}

// =================== MANUAL MODE ===================
void runManualMode() {
  static int lastState = -1;
  if (isSw1Pressed() && !isSw2Pressed()) {
    motorForward(motorSpeed); lastDirection = 1;
    if (lastState != 1) { Serial.println("M Right"); lastState = 1; }
  } else if (!isSw1Pressed() && isSw2Pressed()) {
    motorBackward(motorSpeed); lastDirection = -1;
    if (lastState != 2) { Serial.println("M Left"); lastState = 2; }
  } else {
    motorStop();
    if (lastState != 0) { Serial.println("M Stop"); lastState = 0; }
  }
    // HALLSENSOR DEBUG (lägg till nedan)
  static unsigned long lastHallDebug = 0;
  if (millis() - lastHallDebug > 500) {  // Visa var 500 ms
    lastHallDebug = millis();
    int hallRaw = digitalRead(hallSensorPin);
    Serial.printf("[HALL-DEBUG] ManualMode: HallSensorPin=%d, value=%d\n", hallSensorPin, hallRaw);
  }
}

// =================== SPEED MENU ===================
void runSpeedMenu() {
  stopAllMotors();
  unsigned long lastButtonPress = 0;
  unsigned long holdStart = 0;
  const unsigned long buttonDebounce = 300;
  const int minSpeed = 75;
  const int maxSpeed = 250;
  Serial.println("[SWEEP-DBG] SpeedMenu START");

  // Spara tidigare status på knapparna för edge-detection
  bool prevSw1 = isSw1Pressed();
  bool prevSw2 = isSw2Pressed();

  while (true) {
    server.handleClient();
    updateButton(btnSw1);
    updateButton(btnSw2);
    updateButton(btnPedal);
    bool sw1 = isSw1Pressed();
    bool sw2 = isSw2Pressed();
    bool pedal = isPedalPressed();

    // Debug
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 300) {
      lastDebug = millis();
      Serial.printf("[SWEEP-DBG] SpeedMenu | speed=%d | sw1=%d sw2=%d pedal=%d\n", motorSpeed, sw1, sw2, pedal);
    }

    // Exit på båda switchar eller pedal långtryck
    if ((sw1 && sw2) || pedal) {
      if (holdStart == 0) holdStart = millis();
      if (millis() - holdStart > 1500) {
        Serial.println("[SWEEP-DBG] Exit SpeedMenu (via pedal eller båda switchar)");
        beepMultiple(2);
        while (isPedalPressed() || (isSw1Pressed() && isSw2Pressed())) {
          updateButton(btnPedal); updateButton(btnSw1); updateButton(btnSw2); delay(5);
        }
        break;
      }
    } else {
      holdStart = 0;
    }

    // ÄNDRA: edge-detection istället för direkt respons!
    // Öka hastighet när Sw1 SLÄPPS (går från LOW till HIGH) och Sw2 är INTE nedtryckt
    if (!sw1 && prevSw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed < maxSpeed) {
        saveSpeed(motorSpeed + 25);
        Serial.printf("[SWEEP-DBG] +Speed -> %d\n", motorSpeed);
        beepLong();
      } else {
        Serial.println("[SWEEP-DBG] SpeedMenu: MaxSpeedAlert");
        beepMinMaxAlert();
      }
    }

    // Minska hastighet när Sw2 SLÄPPS (går från LOW till HIGH) och Sw1 är INTE nedtryckt
    if (!sw2 && prevSw2 && !sw1 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed > minSpeed) {
        saveSpeed(motorSpeed - 25);
        Serial.printf("[SWEEP-DBG] -Speed -> %d\n", motorSpeed);
        beepLong();
      } else {
        Serial.println("[SWEEP-DBG] SpeedMenu: MinSpeedAlert");
        beepMinMaxAlert();
      }
    }

    // Spara föregående status för nästa varv
    prevSw1 = sw1;
    prevSw2 = sw2;

    delay(50); // gärna lite snabbare respons nu när vi kör edge
  }
  Serial.println("[SWEEP-DBG] SpeedMenu END");
  delay(300);
}




// =================== MAIN LOOP ===================
void loop() {
  updateButton(btnSw1);
  updateButton(btnSw2);
  updateButton(btnPedal);

  server.handleClient();

  bool sw1 = isSw1Pressed();
  bool sw2 = isSw2Pressed();
  bool pedal = isPedalPressed();

  static unsigned long pedalHoldStart = 0;
  static bool wasInSpeedMenu = false;

  // Gå in i speedmenu om pedal hålls in i 3 sekunder
  if (mainState == MANUAL_MODE) {
    if (pedal) {
      if (pedalHoldStart == 0) pedalHoldStart = millis();
      if ((millis() - pedalHoldStart > 3000) && !wasInSpeedMenu) {
        beepMultiple(3); // 3 pip när du går in
        // Vänta tills pedalen släpps
        while (isPedalPressed()) { updateButton(btnPedal); delay(5); }
        mainState = SPEED_MENU;
        wasInSpeedMenu = true;
      }
    } else {
      pedalHoldStart = 0;
      wasInSpeedMenu = false;
    }

    // -- Fortsatt gammal sweepmode-logik på dubbeltryck --
    static int pedalClickCount = 0;
    static unsigned long firstClickTime = 0;
    static bool lastPedalState = false;

    // Endast sweepmode på 2 klick
    if (pedal && !lastPedalState) {
      if (pedalClickCount == 0) firstClickTime = millis();
      pedalClickCount++;
    }
    lastPedalState = pedal;
    if (pedalClickCount > 0 && millis() - firstClickTime > 2000) {
      pedalClickCount = 0; firstClickTime = 0;
    }
    if (pedalClickCount == 2 && (millis() - firstClickTime < 2000)) {
      beepMultipleDuration(1, 150);
      while (isPedalPressed()) { updateButton(btnPedal); delay(5); }
      mainState = SWEEP_MODE;
      pedalClickCount = 0; firstClickTime = 0;
    }

    runManualMode();
  }
  else if (mainState == SPEED_MENU) {
    static unsigned long pedalHoldStart = 0;
    if (pedal) {
      if (pedalHoldStart == 0) pedalHoldStart = millis();
      if (millis() - pedalHoldStart > 500) {
        beepMultiple(2);
        mainState = MANUAL_MODE;
        pedalHoldStart = 0;
      }
    } else {
      pedalHoldStart = 0;
    }
    runSpeedMenu();
  }
  else if (mainState == SWEEP_MODE) {
    while (isPedalPressed()) { updateButton(btnPedal); delay(5); }
    runSweepMode();
    mainState = MANUAL_MODE;
  }

  // Debug
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.printf("[state] MANUAL=%d SWEEP=%d SPEEDMENU=%d | speed=%d | sw1=%d sw2=%d pedal=%d\n",
      mainState==MANUAL_MODE, mainState==SWEEP_MODE, mainState==SPEED_MENU,
      motorSpeed, sw1, sw2, pedal);
  }
  delay(10);
}


// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(16);
  pinMode(IN1, OUTPUT);
  pinMode(posSw, INPUT_PULLUP);
  pinMode(rotSw1, INPUT_PULLUP);
  pinMode(rotSw2, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(pedalButton, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT_PULLUP);

  pinMode(hallSensorPin, INPUT_PULLUP);
  for (int i = 0; i < 20; i++) {
    int hallRaw = digitalRead(hallSensorPin);
    Serial.printf("KY-003 Startup check #%d: %d\n", i, hallRaw);
    delay(250);
  }

  // PWM-setup: EN (PIN) direkt!
  ledcAttach(EN, pwmFreq, pwmResolution);

  motorStop();

  int savedSpeed = EEPROM.read(0);
  if (savedSpeed >= 75 && savedSpeed <= 250) motorSpeed = savedSpeed;
  else motorSpeed = 125;

  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point started! IP: "); Serial.println(IP);

  server.on("/", []() {
    String html = "<html><body>";
    html += "<h3>ESP32 Motor Control</h3>";
    html += "<p>Speed: " + String(motorSpeed) + "<br>";
    html += "Sweep mode: " + String((sweepModeType == SWEEP_TIME) ? "TIME" : "SWITCH") + "<br>";
    html += "Sweep time: " + String(sweepTimeMs / 1000) + "<br>";
    html += "IP-address: " + WiFi.softAPIP().toString() + "</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });
  server.begin();
}
