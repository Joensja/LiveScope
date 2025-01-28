#include <Arduino.h>
#include <L298N.h>  // Adjust to your actual library name

// ----------------------------
//   PIN CONFIGURATION
// ----------------------------

// Motor A (simple digital pins)
const int motorPin1 = 2;   
const int motorPin2 = 4;   

// Motor B (L298N library)
const int EN  = 9;   
const int IN1 = 8;   
const int IN2 = 7;   

// Position & Buttons
const int posSw   = 5;
const int rotSw1  = 6;   // forward-knapp
const int rotSw2  = 11;  // backward-knapp

// Buzzer
const int buzzerPin = 10;

// ----------------------------
//   GLOBAL VARIABLES
// ----------------------------
bool autoModeActive   = false;
int  targetPosition   = 5;     // default 5, range 1..7
int  currentSpeed     = 125;   // default, range 25..250 in steps of 25

// "Largest time wins" measurement for both buttons
bool bothPressing     = false;
unsigned long bothPressStart = 0;

// Avbryt auto-läge (2s en knapp)
bool pressingAuto     = false;
unsigned long pressTimeAuto = 0;

// Create L298N object for motor B
L298N myMotor(EN, IN1, IN2);

// FORWARD DECL
void runManualMode(bool sw1, bool sw2);
void runAutomaticMode();
void stopAllMotors();
bool handleAutoButtons();
void checkBothLongestPress(bool sw1, bool sw2);

// --------------------------------------------------------------------
//   setup()
// --------------------------------------------------------------------
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  pinMode(posSw,   INPUT_PULLUP);
  pinMode(rotSw1,  INPUT_PULLUP);
  pinMode(rotSw2,  INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  // Initialize motors
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  myMotor.setSpeed(currentSpeed); 
  myMotor.stop();

  Serial.begin(9600);
  Serial.println("System startup. Manual mode by default.");
}

// --------------------------------------------------------------------
//   loop()
// --------------------------------------------------------------------
void loop() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  // "Largest time wins" for both pressed
  checkBothLongestPress(sw1, sw2);

  // Choose mode
  if (autoModeActive) {
    runAutomaticMode();
  } else {
    runManualMode(sw1, sw2);
  }
}

// --------------------------------------------------------------------
//   checkBothLongestPress(sw1, sw2)
//   - On release, measure time:
//       >=10s => setTargetPosition() (5 beeps)
//       >=7s  => setMotorSpeed() (3 beeps)
//       >=3s  => auto mode (1 beep)
//   - If auto-mode was active, we stop motors during the menu and resume
// --------------------------------------------------------------------
void checkBothLongestPress(bool sw1, bool sw2) {
  static bool wasAutoWhenPressed = false;

  if (!bothPressing) {
    if (sw1 && sw2) {
      bothPressing = true;
      bothPressStart = millis();
      wasAutoWhenPressed = autoModeActive;
      if (autoModeActive) {
        Serial.println("[checkBoth] Stopping motors while measuring...");
        stopAllMotors();
      }
    }
  } else {
    // already measuring
    if (!sw1 || !sw2) {
      // released => measure total time
      unsigned long held = millis() - bothPressStart;
      bothPressing = false;
      bothPressStart = 0;

      if (held >= 10000) {
        // 10s => setTargetPosition
        Serial.println("[checkBoth] 10s => setTargetPosition (5 beeps)");
        beepNtimes(5, 1000, 1000);
        bool prevAuto = wasAutoWhenPressed;
        autoModeActive = false;
        setTargetPosition();
        autoModeActive = prevAuto;
        if (autoModeActive) {
          Serial.println("[checkBoth] Resuming auto-läge after targetPosition.");
        }
      }
      else if (held >= 7000) {
        // 7s => setMotorSpeed
        Serial.println("[checkBoth] 7s => setMotorSpeed (3 beeps)");
        beepNtimes(3, 1000, 1000);
        bool prevAuto = wasAutoWhenPressed;
        autoModeActive = false;
        setMotorSpeed();
        autoModeActive = prevAuto;
        if (autoModeActive) {
          Serial.println("[checkBoth] Resuming auto-läge after speed menu.");
        }
      }
      else if (held >= 3000) {
        // 3s => auto-läge
        Serial.println("[checkBoth] 3s => auto-läge (1 beep)");
        tone(buzzerPin, 1000);
        delay(1000);
        noTone(buzzerPin);

        currentSpeed   = 125;
        targetPosition = 5;
        myMotor.setSpeed(currentSpeed);
        autoModeActive = true;
      } 
      else {
        Serial.println("[checkBoth] <3s => do nothing");
      }
    }
    else {
      // still pressed => wait
    }
  }
}

// --------------------------------------------------------------------
//   runManualMode(sw1, sw2)
//   - Motor A: digitalWrite
//   - Motor B: myMotor.setSpeed(...); forward/backward/stop
//   - Add debug for Motor B direction + speed
// --------------------------------------------------------------------
void runManualMode(bool sw1, bool sw2) {
  // Motor A
  if (sw1 && !sw2) {
    // forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  else if (!sw1 && sw2) {
    // backward
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
  else {
    // none or both => stop
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }

  // Motor B
  if (sw1 && !sw2) {
    myMotor.setSpeed(currentSpeed);
    myMotor.forward();
    Serial.print("[Manual] Motor B => FORWARD, speed=");
    Serial.println(currentSpeed);
  }
  else if (!sw1 && sw2) {
    myMotor.setSpeed(currentSpeed);
    myMotor.backward();
    Serial.print("[Manual] Motor B => BACKWARD, speed=");
    Serial.println(currentSpeed);
  }
  else {
    myMotor.stop();
    Serial.println("[Manual] Motor B => STOP");
  }
}

// --------------------------------------------------------------------
//   runAutomaticMode()
//   - pendling 0..targetPosition
//   - add debug for direction + speed for Motor B
// --------------------------------------------------------------------
void runAutomaticMode() {
  Serial.println("[runAutomaticMode] Starting...");

  // triple beep
  beepMultiple(3);

  // 1s off
  stopAllMotors();
  delay(1000);

  // backward until posSw=LOW => zero-locate
  Serial.println("Going backward until posSw=LOW...");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  myMotor.setSpeed(currentSpeed);
  myMotor.backward();
  Serial.print("[Auto] Motor B => BACKWARD, speed=");
  Serial.println(currentSpeed);

  while (digitalRead(posSw)==HIGH) {
    if (!handleAutoButtons()) return;
  }
  stopAllMotors();
  int posCount=0;
  Serial.println("posCount=0, wait 1s...");
  delay(1000);

  while (autoModeActive) {
    // UP
    Serial.println("Pendling UP...");
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    myMotor.setSpeed(currentSpeed);
    myMotor.forward();
    Serial.print("[Auto] Motor B => FORWARD, speed=");
    Serial.println(currentSpeed);

    while(posCount<targetPosition){
      if(!handleAutoButtons()) return;
      if(digitalRead(posSw)==LOW){
        posCount++;
        delay(300);
        Serial.print("posCount(up)=");
        Serial.println(posCount);
      }
    }
    stopAllMotors();
    Serial.println("Reached targetPosition, pause 1s...");
    delay(1000);

    // DOWN
    Serial.println("Pendling DOWN...");
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    myMotor.setSpeed(currentSpeed);
    myMotor.backward();
    Serial.print("[Auto] Motor B => BACKWARD, speed=");
    Serial.println(currentSpeed);

    while(posCount>0){
      if(!handleAutoButtons()) return;
      if(digitalRead(posSw)==LOW){
        posCount--;
        delay(300);
        Serial.print("posCount(down)=");
        Serial.println(posCount);
      }
    }
    stopAllMotors();
    Serial.println("posCount=0, pause 1s...");
    delay(1000);
  }
  Serial.println("Exiting auto-läge...");
}

// --------------------------------------------------------------------
//   handleAutoButtons()
//   - single button >=2s => abort auto-läge
//   - both or none => reset
// --------------------------------------------------------------------
bool handleAutoButtons() {
  bool sw1 = (digitalRead(rotSw1)==LOW);
  bool sw2 = (digitalRead(rotSw2)==LOW);

  if((sw1 && sw2) || (!sw1 && !sw2)){
    pressingAuto=false;
    pressTimeAuto=0;
    return true;
  }
  if(!pressingAuto){
    pressingAuto=true;
    pressTimeAuto=millis();
  }
  else{
    unsigned long held=millis()-pressTimeAuto;
    if(held>=2000){
      Serial.println("ABORT auto-läge (knapp >=2s).");
      tone(buzzerPin,1000);
      delay(1000);
      noTone(buzzerPin);
      stopAllMotors();
      delay(1000);

      autoModeActive=false;
      pressingAuto=false;
      pressTimeAuto=0;
      return false;
    }
  }
  return true;
}

// --------------------------------------------------------------------
//   stopAllMotors()
//   - stops Motor A & B
// --------------------------------------------------------------------
void stopAllMotors(){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  myMotor.stop();
}

// --------------------------------------------------------------------
//   setTargetPosition()
//   - 1..7 (wrap), beep for each step with beepMultiple500()
// --------------------------------------------------------------------
void setTargetPosition(){
  Serial.println("** setTargetPosition MENU **");
  stopAllMotors();
  while(true){
    bool sw1=(digitalRead(rotSw1)==LOW);
    bool sw2=(digitalRead(rotSw2)==LOW);

    // both => exit
    if(sw1&&sw2){
      delay(1000);
      if(digitalRead(rotSw1)==LOW && digitalRead(rotSw2)==LOW){
        Serial.println("Leaving setTargetPosition...");
        return;
      }
    }
    // Increase
    if(sw1&&!sw2){
      unsigned long start=millis();
      while(digitalRead(rotSw1)==LOW){
        if(digitalRead(rotSw2)==LOW) break;
        if((millis()-start)>=1000){
          if(targetPosition<7) targetPosition++;
          else targetPosition=1; 
          beepMultiple500(targetPosition); 
          Serial.print("// Debug: targetPosition => ");
          Serial.println(targetPosition);

          while(digitalRead(rotSw1)==LOW){}
          delay(250);
          break;
        }
      }
    }
    // Decrease
    if(sw2&&!sw1){
      unsigned long start=millis();
      while(digitalRead(rotSw2)==LOW){
        if(digitalRead(rotSw1)==LOW) break;
        if((millis()-start)>=1000){
          if(targetPosition>1) targetPosition--;
          else targetPosition=7;
          beepMultiple500(targetPosition);
          Serial.print("// Debug: targetPosition => ");
          Serial.println(targetPosition);

          while(digitalRead(rotSw2)==LOW){}
          delay(250);
          break;
        }
      }
    }
  }
}

// --------------------------------------------------------------------
//   setMotorSpeed()
//   - circular 25..250 in increments of 25
//   - beep stepIndex times with beepMultiple500()
//   - debug prints speed
// --------------------------------------------------------------------
void setMotorSpeed(){
  Serial.println("** setMotorSpeed MENU **");
  stopAllMotors();
  while(true){
    bool sw1=(digitalRead(rotSw1)==LOW);
    bool sw2=(digitalRead(rotSw2)==LOW);

    // both => exit
    if(sw1&&sw2){
      delay(1000);
      if(digitalRead(rotSw1)==LOW && digitalRead(rotSw2)==LOW){
        Serial.println("Leaving setMotorSpeed...");
        return;
      }
    }

    // Increase
    if(sw1 && !sw2){
      unsigned long start=millis();
      while(digitalRead(rotSw1)==LOW){
        if(digitalRead(rotSw2)==LOW) break;
        if((millis()-start)>=1000){
          int newSpeed=currentSpeed+25;
          if(newSpeed>250) newSpeed=25; 
          currentSpeed=newSpeed;

          int stepIndex=currentSpeed/25; 
          beepMultiple500(stepIndex);
          Serial.print("// Debug: motorB speed => ");
          Serial.println(currentSpeed);

          myMotor.setSpeed(currentSpeed);
          while(digitalRead(rotSw1)==LOW){}
          delay(250);
          break;
        }
      }
    }
    // Decrease
    if(sw2 && !sw1){
      unsigned long start=millis();
      while(digitalRead(rotSw2)==LOW){
        if(digitalRead(rotSw1)==LOW) break;
        if((millis()-start)>=1000){
          int newSpeed=currentSpeed-25;
          if(newSpeed<25) newSpeed=250;
          currentSpeed=newSpeed;

          int stepIndex=currentSpeed/25;
          beepMultiple500(stepIndex);
          Serial.print("// Debug: motorB speed => ");
          Serial.println(currentSpeed);

          myMotor.setSpeed(currentSpeed);
          while(digitalRead(rotSw2)==LOW){}
          delay(250);
          break;
        }
      }
    }
  }
}

// --------------------------------------------------------------------
//   beepMultiple(n) => each beep=250ms ON, 250ms OFF
// --------------------------------------------------------------------
void beepMultiple(int n){
  for(int i=0;i<n;i++){
    tone(buzzerPin,1000);
    delay(250);
    noTone(buzzerPin);
    delay(250);
  }
}

// --------------------------------------------------------------------
//   beepNtimes(n, toneMS, gapMS)
//   e.g. beepNtimes(5, 1000, 1000) => 5 times, each 1s tone + 1s gap
// --------------------------------------------------------------------
void beepNtimes(int n, int toneMS, int gapMS){
  for(int i=0;i<n;i++){
    tone(buzzerPin,1000);
    delay(toneMS);
    noTone(buzzerPin);
    if(i<n-1) delay(gapMS);
  }
}

// --------------------------------------------------------------------
//   beepMultiple500(n) => each beep=500ms ON + 500ms OFF
// --------------------------------------------------------------------
void beepMultiple500(int n){
  for(int i=0;i<n;i++){
    tone(buzzerPin,1000);
    delay(500);
    noTone(buzzerPin);
    delay(500);
  }
}
