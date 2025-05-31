#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// ----------- WIFI SETTINGS -----------
const char* ssid = "WLi";
const char* password = "WLi";
WebServer server(80);

// ----------- PIN DEFINITIONS -----------
const int IN1 = 12;
const int EN = 3;
const int IN2 = 9;
const int posSw = 7;
const int rotSw1 = 6;
const int rotSw2 = 11;
const int buzzerPin = 10;
const int pedalButton = 5;
const int hallSensorPin = 1; // OBS! ESP32 har ibland inte "A1" – använd GPIO-numret

const int referencePWM = 250; // Reference for full speed

// ----------- GLOBAL VARIABLES -----------
bool sweepModeActive = false;
bool bothPressing = false;
bool useHallSensor = false;
bool sweepTimeoutAlarm = false;

int motorSpeed = 125; // Motor speed (50–250 typical)
int lastDirection = 1; // 1 = right, -1 = left

unsigned long lastPosSwPress = 0;
unsigned long posSwDebounceTime = 400;
unsigned long sweepTimeoutMs = 6000;
unsigned long sweepTimeMs = 2000;

enum SweepModeType { SWEEP_SWITCH, SWEEP_TIME };
SweepModeType sweepModeType = SWEEP_TIME;

// ----------- MOTOR CONTROL FUNCTIONS -----------
void motorForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(0, speed); // PWM på EN
}
void motorBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(0, speed);
}
void motorStop() {
  ledcWrite(0, 0);
  digitalWrite(IN2, LOW);
}
void motorBrake() {
  ledcWrite(0, 0);
  digitalWrite(IN2, HIGH);
}
void stopAllMotors() {
  motorStop();
}

// ----------- BUZZER FEEDBACK -----------
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

// ----------- PEDAL BUTTON DEBOUNCE -----------
bool isPedalPressed() {
  static unsigned long lastDebounceTime = 0;
  static bool lastState = HIGH;
  bool currentState = digitalRead(pedalButton) == LOW; // Active LOW
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

// ----------- LIMIT SENSOR FUNCTION -----------
bool isSensorActive() {
  if (useHallSensor) return digitalRead(hallSensorPin) == LOW;
  else return digitalRead(posSw) == LOW;
}

// ----------- EEPROM SPEED SAVE/LOAD -----------
void saveSpeed(int newSpeed) {
  if (motorSpeed != newSpeed) {
    motorSpeed = newSpeed;
    EEPROM.write(0, newSpeed);
    EEPROM.commit();
  }
}

// ----------- MENU: HOLD BOTH BUTTONS/PEDAL TO EXIT -----------
bool checkHoldToExit(unsigned long &holdStart, unsigned long exitHoldTime = 500) {
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

// ----------- MENU BUTTON PRESS LOGIC -----------
void checkBothLongestPress(bool sw1, bool sw2) {
  static unsigned long lastUpdate = 0;
  static unsigned long holdStartTime = 0;
  static unsigned long lastBeepTime = 0;
  static bool releaseLogged = false;
  static bool longPressTriggered = false;

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
      if (held >= 3000) {
        beepMultipleDuration(3, 100);
        setMotorSpeed();
      } 
      else if (held >= 300) {
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
      tone(buzzerPin, 1000); delay(100); noTone(buzzerPin);
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

// ----------- SWEEP-MATIC MODE -----------
void runsweepmaticMode() {
  sweepModeActive = true;
  sweepTimeoutAlarm = false;
  beepMultiple(1);
  stopAllMotors();
  delay(300);

  int sweepDirection = (sweepModeType == SWEEP_TIME) ? 1 : lastDirection;
  unsigned long ignoreUntil = 0;

  while (sweepModeActive) {
    checkModeExit();
    if (!sweepModeActive) break;

    if (sweepDirection == 1) motorForward(motorSpeed);
    else motorBackward(motorSpeed);

    unsigned long actionStart = millis();

    if (sweepModeType == SWEEP_SWITCH) {
      while (sweepModeActive) {
        checkModeExit();
        if (!sweepModeActive) break;
        if (millis() > ignoreUntil) {
          if (isSensorActive()) {
            stopAllMotors();
            delay(500);
            sweepDirection = -sweepDirection;
            lastDirection = sweepDirection;
            ignoreUntil = millis() + 2000;
            break;
          }
        }
        if (millis() - actionStart > sweepTimeoutMs) {
          stopAllMotors();
          beepMultiple(5);
          Serial.println(F("[ALARM] Timeout! No trigger within max time. Sweep mode stopped."));
          sweepTimeoutAlarm = true;
          sweepModeActive = false;
          delay(1000);
          return;
        }
        delay(10);
      }
    } else if (sweepModeType == SWEEP_TIME) {
      unsigned long effectiveSweepTime = sweepTimeMs;
      if (motorSpeed > 0) {
        effectiveSweepTime = sweepTimeMs * referencePWM / motorSpeed;
      }
      unsigned long startTime = millis();
      while ((millis() - startTime < effectiveSweepTime) && sweepModeActive) {
        checkModeExit();
        if (!sweepModeActive) break;
        delay(10);
      }
      stopAllMotors();
      delay(500);
      sweepDirection = -sweepDirection;
      lastDirection = sweepDirection;
    }
  }
  stopAllMotors();
}

// ----------- MANUAL MODE (BUTTONS) -----------
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

// ----------- SPEED ADJUSTMENT MENU -----------
void setMotorSpeed() {
  Serial.println(F("** Speed Adjustment Menu **"));
  stopAllMotors();

  unsigned long lastButtonPress = 0;
  unsigned long holdStart = 0;
  const unsigned long buttonDebounce = 300;
  const unsigned long exitHoldTime = 500;
  const int minSpeed = 75;
  const int maxSpeed = 250;

  while (true) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);

    if (checkHoldToExit(holdStart)) {
      Serial.println(F("Exiting Speed Menu"));
      delay(500);
      break;
    }
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
    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed > minSpeed) {
        saveSpeed(motorSpeed - 25);
        Serial.print(F("Speed: ")); Serial.println(motorSpeed);
        beepLong();
      } else {
        Serial.println(F("MIN (75)"));
        beepMinMaxAlert();
      }
    }
    delay(200);
  }
}

// ----------- MODE EXIT: BUTTON/PEDAL HELD -----------
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

// ----------- COMMANDS: WIFI / SERIAL -----------
void processCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (cmd == "RIGHT") {
    motorForward(motorSpeed);
    beepLong();
  } else if (cmd == "LEFT") {
    motorBackward(motorSpeed);
    beepLong();
  } else if (cmd == "STOP") {
    stopAllMotors();
    beepLong();
  } else if (cmd.startsWith("SPEED=")) {
    int val = cmd.substring(6).toInt();
    if (val >= 50 && val <= 250) {
      saveSpeed(val);
      beepLong();
    }
  } else if (cmd == "SWEEP") {
    runsweepmaticMode();
  } else if (cmd == "+") {
    saveSpeed(min(motorSpeed + 25, 250));
    beepLong();
  } else if (cmd == "-") {
    saveSpeed(max(motorSpeed - 25, 50));
    beepLong();
  } else if (cmd == "M") {
    sweepModeType = (sweepModeType == SWEEP_SWITCH) ? SWEEP_TIME : SWEEP_SWITCH;
    beepMultiple(2);
  } else if (cmd.startsWith("TIMEOUT=")) {
    int t = cmd.substring(8).toInt();
    if (t >= 2 && t <= 60) sweepTimeoutMs = t * 1000UL;
  } else if (cmd.startsWith("SWEEPTIME=")) {
    int t = cmd.substring(10).toInt();
    if (t >= 2 && t <= 60) sweepTimeMs = t * 1000UL;
  }
}

// ----------- WIFI SERVER HANDLERS -----------
void handleRoot() {
  String html = "<html><body>";
  html += "<h3>ESP32 Motor Control</h3>";
  html += "<p>Last command: " + String(server.uri()) + "<br>";
  html += "Speed: " + String(motorSpeed) + "<br>";
  html += "Sweep mode: " + String((sweepModeType == SWEEP_TIME) ? "TIME" : "SWITCH") + "<br>";
  html += "IP-adress: " + WiFi.softAPIP().toString() + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleCommand() {
  String cmd = server.uri();
  if (cmd.startsWith("/RIGHT")) processCommand("RIGHT");
  else if (cmd.startsWith("/LEFT")) processCommand("LEFT");
  else if (cmd.startsWith("/STOP")) processCommand("STOP");
  else if (cmd.startsWith("/SWEEP")) processCommand("SWEEP");
  else if (cmd.startsWith("/SPEED=")) processCommand("SPEED=" + cmd.substring(7));
  else if (cmd.startsWith("/SWEEPTIME=")) processCommand("SWEEPTIME=" + cmd.substring(11));
  else if (cmd.startsWith("/TIMEOUT=")) processCommand("TIMEOUT=" + cmd.substring(9));
  handleRoot();
}

// ----------- SETUP -----------
void setup() {
  Serial.begin(115200);
  EEPROM.begin(16);

  pinMode(IN1, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(posSw, INPUT_PULLUP);
  pinMode(rotSw1, INPUT_PULLUP);
  pinMode(rotSw2, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(pedalButton, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT_PULLUP);

  // PWM setup för motor-EN-pinnen
  ledcSetup(0, 1000, 8); // channel 0, 1 kHz, 8 bitars upplösning
  ledcAttachPin(EN, 0);

  motorStop();

  int savedSpeed = EEPROM.read(0);
  if (savedSpeed >= 75 && savedSpeed <= 250) motorSpeed = savedSpeed;
  else motorSpeed = 125;

  Serial.println("Startar ESP32 Access Point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point startad! IP: ");
  Serial.println(IP);

  server.on("/", handleRoot);
  server.onNotFound(handleCommand); // alla andra GET-url:er hanteras här
  server.begin();
}

// ----------- MAIN LOOP -----------
void loop() {
  // Webserver
  server.handleClient();

  // Manuella knappar/switchar
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  checkBothLongestPress(sw1, sw2);
  checkModeExit();
  runManualMode(sw1, sw2);

  delay(20);
}
