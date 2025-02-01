#include <Arduino.h>
#include <L298N.h>  
#include <EEPROM.h>
#include <SoftwareSerial.h> // Används för att hantera seriell kommunikation

SoftwareSerial BTSerial(3, 4); // HC-05: TX på 3, RX på 4


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
bool bleConnected = false;  // Global variabel för att hålla koll på Bluetooth-anslutning


void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600);

    Serial.println("🔍 Startar HC-05 kontroll...");
    BTSerial.println("HC-05 Check");

    delay(1000);  // Vänta en kort stund för att HC-05 ska kunna svara

    bool bleConnected = false;  // Först antar vi att den inte är ansluten
    unsigned long startTime = millis();  // Starttid för timeout

    while (millis() - startTime < 1000) {  // Vänta max 3 sekunder på svar
        if (BTSerial.available()) {  // Om vi får respons från HC-05
            Serial.println("HC-05 svarar!");
            bleConnected = true;
            break;  // Avsluta loopen
        }
    }

    if (!bleConnected) {
        Serial.println("Ingen respons från HC-05!");
    }

    Serial.println("Fortsätter programkörning...");

    // Fortsätt programmet oavsett HC-05-status
    Serial.println("Fortsätter");
    Serial.println("Resume");

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(posSw, INPUT_PULLUP);
    pinMode(rotSw1, INPUT_PULLUP);
    pinMode(rotSw2, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);

    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);

    int savedSpeed = EEPROM.read(0);
    if (savedSpeed >= 25 && savedSpeed <= 250) {
        currentSpeed = savedSpeed;
    } else {
        currentSpeed = 125;
    }

    myMotor.setSpeed(currentSpeed);
    myMotor.stop();

    updateDebounceTime();

    Serial.println("Bluetooth Ready...");
    BTSerial.println("Bluetooth Ready...");
    Serial.print("Target position: ");
    Serial.println(targetPosition);
    Serial.print("Motor Saved speed: ");
    Serial.println(currentSpeed);
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

/*Serial.print("rotSw1: ");
Serial.print(digitalRead(rotSw1));
Serial.print("  rotSw2: ");
Serial.println(digitalRead(rotSw2));
*/
delay(200);  // För att undvika att spamma seriell monitor

  // ✅ Läser Bluetooth-data
  if (BTSerial.available()) {
    char command = BTSerial.read();
    handleBluetoothCommand(command);
    Serial.println("128");
  }
    // ✅ Kontrollera inkommande Bluetooth-data
  if (BTSerial.available()) {
    char command = BTSerial.read();
    Serial.print("📶 Data från HC-05: ");
    Serial.println(command);  // Skriv ut vilken knapp som trycktes från appen
    handleBluetoothCommand(command);
  }
}

// ----------------------------
//   FIXAD SETTARGETPOSITION()
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
    case 'L':  // 🔄 Vänster rotation
      myMotor.setSpeed(currentSpeed);
      myMotor.backward();
      Serial.println("[BT] Rotating Left");
      BTSerial.println("Rotating Left");
      break;

    case 'R':  // 🔄 Höger rotation
      myMotor.setSpeed(currentSpeed);
      myMotor.forward();
      Serial.println("[BT] Rotating Right");
      BTSerial.println("Rotating Right");
      break;

    case 'S':  // ⏹️ Stoppa motor
      myMotor.stop();
      Serial.println("[BT] Stopping Motor");
      BTSerial.println("Stopping Motor");
      break;

    case '+':  // ⏫ Öka hastighet
      currentSpeed = (currentSpeed >= 250) ? 250 : currentSpeed + 25;
      myMotor.setSpeed(currentSpeed);
      EEPROM.write(0, currentSpeed);
      Serial.print("[BT] Increased Speed: ");
      Serial.println(currentSpeed);
      BTSerial.print("Speed: ");
      BTSerial.println(currentSpeed);
      break;

    case '-':  // ⏬ Minska hastighet
      currentSpeed = (currentSpeed <= 25) ? 25 : currentSpeed - 25;
      myMotor.setSpeed(currentSpeed);
      EEPROM.write(0, currentSpeed);
      Serial.print("[BT] Decreased Speed: ");
      Serial.println(currentSpeed);
      BTSerial.print("Speed: ");
      BTSerial.println(currentSpeed);
      break;

    case 'A':  // 🔄 Aktivera Auto Mode
      autoModeActive = true;
      Serial.println("[BT] Auto Mode Activated");
      BTSerial.println("Auto Mode Activated");
      break;

    case 'M':  // ❌ Avbryt Auto Mode
      autoModeActive = false;
      stopAllMotors();
      Serial.println("[BT] Auto Mode Deactivated");
      BTSerial.println("Auto Mode Deactivated");
      break;

    default:
      Serial.println("[BT] Unknown Command");
      BTSerial.println("Unknown Command");
      break;
  }
}

// ----------------------------
//   UPPDATERA DEBOUNCE BASERAT PÅ HASTIGHET (STEGVIS ÄNDRING)
// ----------------------------
void updateDebounceTime() {
  // Linear equation: posSwDebounceTime = a * currentSpeed + b
  /*float a = (200.0 - 600.0) / (250.0 - 25.0);  // a = -400 / 225
  float b = 600.0 - a * 25.0;                 // b = 600 - (-1.777 * 25)

  unsigned long newDebounceTime = a * currentSpeed + b;  // Calculate new debounce
*/
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

/*void updateDebounceTime() {
  // Linjär ekvation: posSwDebounceTime = a * currentSpeed + b
  // Vid speed = 25 -> debounce = 600ms
  // Vid speed = 250 -> debounce = 200ms
  float a = (200.0 - 600.0) / (250.0 - 25.0);  // a = -400 / 225
  float b = 600.0 - a * 25.0;                 // b = 600 - (-1.777 * 25)

  posSwDebounceTime = a * currentSpeed + b;  // Linjär formel

  Serial.print("[System] Updated debounce time: ");
  Serial.print(posSwDebounceTime);
  Serial.println("ms");
}*/

// ----------------------------
//   AUTO MODE (LAGT TILL)
// ----------------------------
void runAutomaticMode() {
  Serial.println("[Auto] Starting...");
  beepMultiple(0);

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
    myMotor.setSpeed(currentSpeed);
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
    myMotor.setSpeed(currentSpeed);
    myMotor.forward();

    if (lastState != 1 && millis() - lastPrintTime >= 1000) {  // Only print every 1s if state changed
      Serial.println("[Manual] Motor B => FORWARD");
      lastState = 1;
      lastPrintTime = millis();
    }
    
    updateDebounceTime();  // Ensure debounce updates if speed changes

  } else if (!sw1 && sw2) {
    myMotor.setSpeed(currentSpeed);
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

/*void runManualMode(bool sw1, bool sw2) {
  if (sw1 && !sw2) {
    myMotor.setSpeed(currentSpeed);
    myMotor.forward();
    Serial.println("[Manual] Motor B => FORWARD");
    updateDebounceTime();  // Se till att debounce uppdateras om hastigheten ändras
  } else if (!sw1 && sw2) {
    myMotor.setSpeed(currentSpeed);
    myMotor.backward();
    Serial.println("[Manual] Motor B => BACKWARD");
    updateDebounceTime();
  } else {
    myMotor.stop();
    
    if (millis() - lastStopPrintTime >= 1000) {
      Serial.println("[Manual] Motor B => STOP");
      lastStopPrintTime = millis();
    }
  }
}
*/
// ----------------------------
//   CHECK BUTTON LONG PRESS
// ----------------------------
// Detects if both rotation switches are held for a long duration.
// Different actions are triggered based on how long both buttons are held.

// TESTAR EN NY FUNKTION
void checkBothLongestPress(bool sw1, bool sw2) {
  static unsigned long lastUpdate = 0;  // Tidpunkt för senaste loggning
  static unsigned long holdStartTime = 0;  // När knapparna först trycktes
  static unsigned long lastBeepTime = 0;   // Senaste tidpunkt för pip
  static bool releaseLogged = false;  // Förhindrar duplicerade meddelanden

  if (!bothPressing && sw1 && sw2) {  
    // Om båda knapparna trycks ner och inte redan hålls
    bothPressing = true;
    holdStartTime = millis();  // Starttid för tryck
    lastUpdate = millis();  // Återställ senaste loggtid
    lastBeepTime = millis(); // Starta pip-timer
    releaseLogged = false;  // Återställ flagga
  } 
  else if (bothPressing && (!sw1 || !sw2)) {  
    // Om en av knapparna släpps efter att ha hållits nere
    if (!releaseLogged) {  
      // Förhindrar att loggen skrivs flera gånger
      unsigned long held = millis() - holdStartTime;  // Beräkna hålltid
      Serial.println("[DEBUG] Both buttons released. Timer reset.");
      releaseLogged = true;  // Förhindra duplicering
      delay(50);  // Små fördröjningar för att undvika studsar

      // Hantera åtgärder beroende på hur länge knappen hölls nere
      if (held >= 5000) {  
        Serial.println("[Auto] 5s => Entering Motor Speed Adjustment Mode (3 beeps)");
        beepMultiple(3);
        setMotorSpeed();  
      } 
      else if (held >= 3000) {  
        Serial.println("[Auto] 3s => Set Target Position (2 beeps)");
        beepMultiple(2);
        setTargetPosition();
      } 
      else if (held >= 1000) {  
        Serial.println("[Auto] 1s => Activating Auto Mode (1 beep)");
        beepMultiple(1);
        autoModeActive = true;
      }
    }
    bothPressing = false;  // Återställ flaggan
  } 
  else if (bothPressing) {  
    // Om knapparna hålls in fortsätter vi att kolla tiden
    unsigned long heldTime = millis() - holdStartTime;
    
    // 📢 Lägger till ett pip varje sekund
    if (millis() - lastBeepTime >= 1000) {  
      tone(buzzerPin, 1000);
      delay(100);
      noTone(buzzerPin);
      lastBeepTime = millis(); // Uppdatera senaste pip-tid
    }

    // Skriver ut hålltid i sekunder
    if (millis() - lastUpdate >= 500) {  
      Serial.print("[DEBUG] Held Time: ");
      Serial.print(heldTime / 1000.0, 1);  // Konvertera till sekunder med 1 decimal
      Serial.println("s");
      lastUpdate = millis();  // Uppdatera senaste utskriftstid
    }
  }
}



/*void checkBothLongestPress(bool sw1, bool sw2) {
  if (!bothPressing && sw1 && sw2) {  
    // If both buttons are pressed and were not already held
    bothPressing = true;
    bothPressStart = millis();  // Store the time the press started
  } 
  else if (bothPressing && (!sw1 || !sw2)) {  
    // If one or both buttons are released after being held
    unsigned long held = millis() - bothPressStart;  // Calculate hold duration
    bothPressing = false;  // Reset flag

    // 5s hold -> Open Motor Speed Adjustment Menu
    if (held >= 5000) {  
      Serial.println("[Auto] 5s => Entering Motor Speed Adjustment Mode (3 beeps)");
      beepMultiple(3);  // 3 beeps for confirmation
      setMotorSpeed();  // Call new function
    } 

    // 3s hold -> Open Set Target Position Menu
    else if (held >= 3000) {  
      Serial.println("[Auto] 3s => Set Target Position (5 beeps)");
      beepMultiple(5);  // 5 beeps for confirmation
      setTargetPosition();
    } 
    
    // 1s hold -> Activate Auto Mode
    else if (held >= 1000) {  
      Serial.println("[Auto] 1s => Activating Auto Mode (1 beep)");
      beepMultiple(1);  // 1 beep for confirmation
      autoModeActive = true;
    }
  }
}*/
// ----------------------------
//   SET MOTOR SPEED
// ----------------------------
void setMotorSpeed() {
  Serial.println("** Entering Motor Speed Adjustment Mode **");
  stopAllMotors();

  bool exitMenu = false;
  unsigned long lastButtonPress = 0;
  const unsigned long buttonDebounce = 300;

  while (!exitMenu) {
    bool sw1 = (digitalRead(rotSw1) == LOW);
    bool sw2 = (digitalRead(rotSw2) == LOW);

    // ✅ ADD A SMALL DELAY INSIDE MENU TO PREVENT SENSITIVE NAVIGATION
    delay(200);  // Prevents rapid toggling between speeds

    if (sw1 && !sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      currentSpeed = (currentSpeed >= 250) ? 25 : currentSpeed + 25;
      myMotor.setSpeed(currentSpeed);
      EEPROM.write(0, currentSpeed);  // Save new speed in EEPROM
      Serial.print("New Motor Speed: ");
      Serial.println(currentSpeed);
      beepMultiple(1);
    }

    if (!sw1 && sw2 && millis() - lastButtonPress > buttonDebounce) {
      lastButtonPress = millis();
      currentSpeed = (currentSpeed <= 25) ? 250 : currentSpeed - 25;
      myMotor.setSpeed(currentSpeed);
      EEPROM.write(0, currentSpeed);  // Save new speed in EEPROM
      Serial.print("New Motor Speed: ");
      Serial.println(currentSpeed);
      beepMultiple(1);
    }

    // ✅ Prevent accidental single-click detection before exit
    if (sw1 && sw2) {
      unsigned long pressStart = millis();
      
      while (sw1 && sw2) {
        if (millis() - pressStart >= 1000) {  // Require 1s hold to exit
          Serial.println("Exiting Motor Speed Adjustment Mode...");
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


// ----------------------------
//   HANDLE AUTO BUTTONS (FIXAD)
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
        // ⏳ NYTT: Vänta 1 sekund innan den går tillbaka till manuellt läge
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
