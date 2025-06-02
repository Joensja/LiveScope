/******************************************************
  Universal L298N Motor Control for ESP32 & Arduino Uno
  -----------------------------------------------------
  - Sweep mode (number of sweeps/angle selection)
  - Compass (PID, HMC5883L support)
  - Speed menu (pedal button hold)
  - Cross-platform: WiFi/App for ESP32, HC05 Bluetooth for Uno
  - All features identical on both platforms
  - All comments in English
******************************************************/

// ========= PLATFORM SELECTION =========
#if defined(ESP32)
  #define IS_ESP32 1
#else
  #define IS_ESP32 0
#endif

// ========= LIBRARIES =========
#if IS_ESP32
  #include <WiFi.h>
  #include <WebServer.h>
  #include <EEPROM.h>
  #include <Wire.h>
  #include <Adafruit_HMC5883_U.h>
#else
  #include <EEPROM.h>
  #include <Wire.h>
  #include <Adafruit_HMC5883_U.h>
  #include <SoftwareSerial.h>
#endif
#include <PID_v1.h>

// ========= PIN DEFINITIONS =========
// Pins are selected to match Arduino Uno shield form factor as closely as possible
#if IS_ESP32
  // ESP32 D1 R32 / similar UNO-style ESP32 board
  #define PIN_IN1        19
  #define PIN_IN2        21
  #define PIN_EN         25
  #define PIN_POSSW      14  // Limit switch/magnetic/hall input
  #define PIN_ROTSW1     16  // Right button
  #define PIN_ROTSW2     27  // Left button
  #define PIN_BUZZER     26
  #define PIN_PEDAL      5
  #define PIN_HALL       33
  #define PWM_CHANNEL    0
  #define BUZZER_CHANNEL 1
  #define BT_RX_PIN      32   // Optional, for HW Serial2 if using BT on ESP32
  #define BT_TX_PIN      33
#else
  // Arduino Uno (classic shield)
  #define PIN_IN1        8
  #define PIN_IN2        7
  #define PIN_EN         6
  #define PIN_POSSW      4
  #define PIN_ROTSW1     2
  #define PIN_ROTSW2     3
  #define PIN_BUZZER     5
  #define PIN_PEDAL      9
  #define PIN_HALL       10
  #define BT_RX_PIN      11
  #define BT_TX_PIN      12
#endif

// ========= GLOBALS =========
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
bool useMagnetometer = true;

int motorSpeed = 125;
int sweepSpeed = 125;
int sweepAngle = 5; // Number of "sweeps" or sweep range, default 5

// PID control for compass mode
double setHeading = 0;
double output = 0;
double turnThreshold = 4;
double targetHeading = 0;
double userSetpoint = 0;
double currentHeading = 0;
double pidInput = 0;
double pidOutput = 0;
double pidSetpoint = 0;

double Kp = 2.5, Ki = 0.1, Kd = 1.5;
PID headingPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

bool sweepModeActive = false;
bool compassModeActive = false;

bool useHallSensor = false; // Use hall or limit switch
bool sweepTimeoutAlarm = false;

unsigned long sweepTimeoutMs = 6000; // Timeout for sweep
unsigned long sweepTimeMs = 4000;    // For time-based sweep

enum MainState { MANUAL_MODE, SWEEP_MODE, COMPASS_MODE, SPEED_MENU };
MainState mainState = MANUAL_MODE;

// ========= COMMUNICATION =========
#if IS_ESP32
  WebServer server(80);
#else
  SoftwareSerial BTserial(BT_RX_PIN, BT_TX_PIN); // HC-05 Bluetooth
#endif

// ========= BUTTON DEBOUNCE STRUCTURE =========
const unsigned long DEBOUNCE_MS = 40;
struct Button {
  const int pin;
  bool state;
  bool lastRaw;
  unsigned long lastChange;
};

Button btnSw1   = { PIN_ROTSW1, HIGH, HIGH, 0 };
Button btnSw2   = { PIN_ROTSW2, HIGH, HIGH, 0 };
Button btnPedal = { PIN_PEDAL,  HIGH, HIGH, 0 };

// Update the state of a button with debounce
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

bool isSw1Pressed()   { return btnSw1.state == LOW; }
bool isSw2Pressed()   { return btnSw2.state == LOW; }
bool isPedalPressed() { return btnPedal.state == LOW; }

// ========= MOTOR DRIVER (L298N, PWM abstraction) =========
void motorForward(int speed) {
#if IS_ESP32
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PWM_CHANNEL, speed);
#else
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_EN, speed);
#endif
}
void motorBackward(int speed) {
#if IS_ESP32
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  ledcWrite(PWM_CHANNEL, speed);
#else
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  analogWrite(PIN_EN, speed);
#endif
}
void motorStop() {
#if IS_ESP32
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
#else
  analogWrite(PIN_EN, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
#endif
}
void stopAllMotors() { motorStop(); }

// ========= BUZZER ABSTRACTION =========
void beep(int duration = 250) {
#if IS_ESP32
  digitalWrite(PIN_BUZZER, HIGH);
  delay(duration);
  digitalWrite(PIN_BUZZER, LOW);
#else
  tone(PIN_BUZZER, 1000, duration);
  delay(duration);
  noTone(PIN_BUZZER);
#endif
}
void beepMultiple(int n) {
  for (int i = 0; i < n; i++) { beep(200); delay(200); }
}
void beepMultipleDuration(int n, int duration) {
  for (int i = 0; i < n; i++) { beep(duration); delay(100); }
}
void beepLong() { beep(400); }
void beepMinMaxAlert() { beepMultiple(2); beepLong(); }

// ========= EEPROM (cross-platform abstraction) =========
void saveSpeed(int newSpeed) {
  newSpeed = constrain(newSpeed, 75, 250);
  if (motorSpeed != newSpeed) {
    motorSpeed = newSpeed;
    EEPROM.write(0, newSpeed);
#if IS_ESP32
    EEPROM.commit();
#endif
  }
}
void savePIDValues() {
  EEPROM.put(1, Kp);
  EEPROM.put(5, Ki);
  EEPROM.put(9, Kd);
#if IS_ESP32
  EEPROM.commit();
#endif
}
void loadPIDValues() {
  EEPROM.get(1, Kp);
  EEPROM.get(5, Ki);
  EEPROM.get(9, Kd);
  if (Kp <= 0 || Kd < 0 || isnan(Kp) || isnan(Ki) || isnan(Kd)) {
    Kp = 2.5; Ki = 0.1; Kd = 1.5;
    savePIDValues();
  }
  headingPID.SetTunings(Kp, Ki, Kd);
}

// ========= SENSOR (limit/hall) =========
bool isSensorActive() {
  if (useHallSensor)
    return digitalRead(PIN_HALL) == LOW;
  else
    return digitalRead(PIN_POSSW) == LOW;
}

// ========= SHORT ANGLE HELPER =========
double shortestAngle(double target, double current) {
  double diff = fmod((target - current + 540), 360) - 180;
  return diff;
}

// ========= COMPASS/MAGNETOMETER =========
void updateHeading() {
  if (!useMagnetometer) return;
  sensors_event_t event;
  static int failCount = 0;
  static double filteredHeading = 0;
  const double alpha = 0.3;
  double rawHeading = 0;

  if (!mag.getEvent(&event)) {
    failCount++;
    if (failCount > 5) {
      useMagnetometer = false;
      failCount = 0;
      return;
    }
    return;
  }
  failCount = 0;
  rawHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (rawHeading < 0) rawHeading += 360;

  double delta = rawHeading - filteredHeading;
  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;
  filteredHeading += alpha * delta;
  if (filteredHeading < 0) filteredHeading += 360;
  if (filteredHeading >= 360) filteredHeading -= 360;

  currentHeading = filteredHeading;
}

// ========= SWEEP MODE =========
void runSweepMode() {
  beepMultiple(1);
  sweepTimeoutAlarm = false;
  bool sweepActive = true;
  int sweepDirection = 1;
  unsigned long ignoreUntil = 0;
  int sweepCount = 0;
  Serial.println("[SWEEP] === Sweep mode started ===");
  while (sweepActive && sweepCount < sweepAngle) {
    stopAllMotors(); delay(50);
    if (sweepDirection == 1) { motorForward(sweepSpeed); }
    else { motorBackward(sweepSpeed); }
    unsigned long actionStart = millis();

    static bool lastSensor = false;
    while (true) {
      updateButton(btnSw1); updateButton(btnSw2);
      bool currentSensor = isSensorActive();
      if (currentSensor && !lastSensor) {
        Serial.println("[SWEEP] Limit sensor triggered");
      }
      lastSensor = currentSensor;

      // Button exit
      static unsigned long innerSwHoldStart = 0;
      if (isSw1Pressed() || isSw2Pressed()) {
        if (innerSwHoldStart == 0) innerSwHoldStart = millis();
        if (millis() - innerSwHoldStart > 500) {
          sweepActive = false; beepMultiple(2);
          Serial.println("[SWEEP] Stopping sweep via button");
          break;
        }
      } else { innerSwHoldStart = 0; }

      if (!sweepActive) break;

      if (millis() > ignoreUntil && isSensorActive()) {
        stopAllMotors(); delay(500);
        sweepDirection = -sweepDirection;
        ignoreUntil = millis() + 2000;
        sweepCount++;
        Serial.print("[SWEEP] Direction reversed. Sweep count: "); Serial.println(sweepCount);
        break;
      }
      if (millis() - actionStart > sweepTimeoutMs) {
        stopAllMotors();
        beepMultiple(5);
        Serial.println("[SWEEP] [ALARM] Timeout! No sensor trigger - sweep stopped.");
        sweepTimeoutAlarm = true;
        sweepActive = false;
        break;
      }
      delay(10);
    }
  }
  stopAllMotors(); delay(200);
  Serial.println("[SWEEP] === Sweep mode ended ===");
}

// ========= COMPASS MODE =========
void runCompassMode() {
  if (!useMagnetometer) {
    Serial.println("[COMPASS] Skipped: Magnetometer disabled.");
    return;
  }
  compassModeActive = true;
  beepMultiple(2); stopAllMotors();

  while (compassModeActive) {
    updateButton(btnSw1); updateButton(btnSw2); updateButton(btnPedal);
    if (isSw1Pressed() && isSw2Pressed() || isPedalPressed()) {
      compassModeActive = false; stopAllMotors();
      Serial.println("[COMPASS] Exiting via button/pedal");
      break;
    }

    updateHeading();
    double diff = shortestAngle(userSetpoint, currentHeading);
    pidInput = diff;
    pidSetpoint = 0;
    headingPID.Compute();

    // Safety: big error, stop!
    if (abs(diff) > 120) { stopAllMotors(); continue; }
    // Deadzone
    const int deadZone = 5;
    if (abs(diff) <= deadZone) {
      stopAllMotors();
      continue;
    }
    int speed = constrain(abs(pidOutput), 35, 50);

    if (diff > turnThreshold) { motorForward(speed); }
    else if (diff < -turnThreshold) { motorBackward(speed); }
    else { stopAllMotors(); }
    delay(40);
  }
  stopAllMotors();
}

// ========= SPEED MENU =========
void runSpeedMenu() {
  stopAllMotors();
  unsigned long lastButtonPress = 0;
  unsigned long holdStart = 0;
  const unsigned long buttonDebounce = 300;
  const int minSpeed = 75, maxSpeed = 250;
  bool prevSw1 = isSw1Pressed(), prevSw2 = isSw2Pressed();

  while (true) {
    updateButton(btnSw1); updateButton(btnSw2); updateButton(btnPedal);
    bool sw1 = isSw1Pressed(), sw2 = isSw2Pressed(), pedal = isPedalPressed();

    // Exit: both switches or pedal long press
    if ((sw1 && sw2) || pedal) {
      if (holdStart == 0) holdStart = millis();
      if (millis() - holdStart > 1500) {
        beepMultiple(2);
        while (isPedalPressed() || (isSw1Pressed() && isSw2Pressed()))
          { updateButton(btnPedal); updateButton(btnSw1); updateButton(btnSw2); delay(5); }
        break;
      }
    } else { holdStart = 0; }

    // Increase speed on Sw1 release
    if (!sw1 && prevSw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed < maxSpeed) {
        saveSpeed(motorSpeed + 25);
        beepLong();
      } else { beepMinMaxAlert(); }
    }
    // Decrease speed on Sw2 release
    if (!sw2 && prevSw2 && !sw1 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      if (motorSpeed > minSpeed) {
        saveSpeed(motorSpeed - 25);
        beepLong();
      } else { beepMinMaxAlert(); }
    }
    prevSw1 = sw1; prevSw2 = sw2;
    delay(50);
  }
  delay(300);
}

// ========= MANUAL MODE =========
void runManualMode() {
  static int lastState = -1;
  if (isSw1Pressed() && !isSw2Pressed()) {
    motorForward(motorSpeed);
    if (lastState != 1) { lastState = 1; }
  } else if (!isSw1Pressed() && isSw2Pressed()) {
    motorBackward(motorSpeed);
    if (lastState != 2) { lastState = 2; }
  } else {
    motorStop();
    if (lastState != 0) { lastState = 0; }
  }
}

// ========= COMMAND HANDLING (BLUETOOTH / APP / WIFI) =========
void handleAppCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (cmd == "RIGHT")      { motorForward(motorSpeed); }
  else if (cmd == "LEFT")  { motorBackward(motorSpeed); }
  else if (cmd == "STOP")  { motorStop(); }
  else if (cmd.startsWith("SPEED=")) {
    int s = cmd.substring(6).toInt();
    saveSpeed(s);
  }
  else if (cmd.startsWith("SWEEP=")) {
    int n = cmd.substring(6).toInt();
    sweepAngle = constrain(n, 1, 12);
  }
  else if (cmd == "SWEEPMODE") {
    mainState = SWEEP_MODE;
  }
  else if (cmd == "COMPASSMODE") {
    mainState = COMPASS_MODE;
  }
  else if (cmd == "SPEEDMENU") {
    mainState = SPEED_MENU;
  }
  else if (cmd.startsWith("KP=")) { Kp = cmd.substring(3).toFloat(); savePIDValues(); }
  else if (cmd.startsWith("KI=")) { Ki = cmd.substring(3).toFloat(); savePIDValues(); }
  else if (cmd.startsWith("KD=")) { Kd = cmd.substring(3).toFloat(); savePIDValues(); }
  // Add more commands as needed
}

// ========= SETUP =========
void setup() {
  Serial.begin(115200);
#if IS_ESP32
  EEPROM.begin(32);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_POSSW, INPUT_PULLUP); pinMode(PIN_ROTSW1, INPUT_PULLUP);
  pinMode(PIN_ROTSW2, INPUT_PULLUP); pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_PEDAL, INPUT_PULLUP); pinMode(PIN_HALL, INPUT_PULLUP);
  ledcSetup(PWM_CHANNEL, 1000, 8);
  ledcAttachPin(PIN_EN, PWM_CHANNEL);
  WiFi.softAP("TransducerPole", "12345678");
  server.on("/", []() {
    String html = "<h3>ESP32 Motor Control</h3>";
    html += "<p>Speed: " + String(motorSpeed) + "<br>";
    html += "Sweep angle: " + String(sweepAngle) + "<br>";
    html += "IP: " + WiFi.softAPIP().toString() + "</p>";
    server.send(200, "text/html", html);
  });
  server.onNotFound([]() {
    String uri = server.uri();
    if (uri.startsWith("/CMD=")) handleAppCommand(uri.substring(5));
    server.send(200, "text/plain", "OK");
  });
  server.begin();
#else
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_POSSW, INPUT_PULLUP); pinMode(PIN_ROTSW1, INPUT_PULLUP);
  pinMode(PIN_ROTSW2, INPUT_PULLUP); pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_PEDAL, INPUT_PULLUP); pinMode(PIN_HALL, INPUT_PULLUP);
  analogWrite(PIN_EN, 0); // Start stopped
  BTserial.begin(9600);
#endif

  int savedSpeed = EEPROM.read(0);
  if (savedSpeed >= 75 && savedSpeed <= 250) motorSpeed = savedSpeed;
  else motorSpeed = 125;
  loadPIDValues();
  mag.begin();
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-255, 255);

  Serial.println("Motor control ready.");
}

// ========= LOOP =========
void loop() {
  updateButton(btnSw1); updateButton(btnSw2); updateButton(btnPedal);

#if IS_ESP32
  server.handleClient();
#else
  if (BTserial.available()) {
    String cmd = BTserial.readStringUntil('\n');
    handleAppCommand(cmd);
  }
#endif

  static unsigned long pedalHoldStart = 0;
  static bool wasInSpeedMenu = false;

  // Enter speed menu by holding pedal
  if (mainState == MANUAL_MODE) {
    if (isPedalPressed()) {
      if (pedalHoldStart == 0) pedalHoldStart = millis();
      if ((millis() - pedalHoldStart > 2000) && !wasInSpeedMenu) {
        beepMultiple(3);
        while (isPedalPressed()) { updateButton(btnPedal); delay(5); }
        mainState = SPEED_MENU;
        wasInSpeedMenu = true;
      }
    } else { pedalHoldStart = 0; wasInSpeedMenu = false; }
    // Double-click pedal for sweep
    static int pedalClickCount = 0;
    static unsigned long firstClickTime = 0;
    static bool lastPedalState = false;
    if (isPedalPressed() && !lastPedalState) {
      if (pedalClickCount == 0) firstClickTime = millis();
      pedalClickCount++;
    }
    lastPedalState = isPedalPressed();
    if (pedalClickCount > 0 && millis() - firstClickTime > 2000) { pedalClickCount = 0; firstClickTime = 0; }
    if (pedalClickCount == 2 && (millis() - firstClickTime < 2000)) {
      beepMultipleDuration(1, 150);
      while (isPedalPressed()) { updateButton(btnPedal); delay(5); }
      mainState = SWEEP_MODE;
      pedalClickCount = 0; firstClickTime = 0;
    }
    runManualMode();
  }
  else if (mainState == SPEED_MENU) {
    runSpeedMenu();
    mainState = MANUAL_MODE;
  }
  else if (mainState == SWEEP_MODE) {
    while (isPedalPressed()) { updateButton(btnPedal); delay(5); }
    runSweepMode();
    mainState = MANUAL_MODE;
  }
  else if (mainState == COMPASS_MODE) {
    runCompassMode();
    mainState = MANUAL_MODE;
  }
  delay(10);
}
