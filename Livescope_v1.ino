// ----------------------------------------------------------
//   KONFIGURATION AV PINS
// ----------------------------------------------------------
const int motorPin1 = 3;    // Motor-styrning
const int motorPin2 = 4;    // Motor-styrning
const int posSw = 5;        // Positionsswitch
const int rotSw1 = 6;       // Knappar
const int rotSw2 = 7;
const int buzzerPin = 8;    // Summer

// ----------------------------------------------------------
//   GLOBALA VARIABLER
// ----------------------------------------------------------
bool autoModeActive = false;     // Om auto-läget är aktivt
int  targetPosition = 1;         // Antal "stängningar" av posSw (1–5)
unsigned long buttonPressTime = 0;  // Används för att aktivera auto-läge

// ----------------------------------------------------------
//   INSTÄLLNINGAR (SETUP)
// ----------------------------------------------------------
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(posSw, INPUT_PULLUP);
  pinMode(rotSw1, INPUT_PULLUP);
  pinMode(rotSw2, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  noTone(buzzerPin);

  Serial.begin(9600);
  Serial.println("System startup. Manual mode by default.");
}

// ----------------------------------------------------------
//   LOOP
// ----------------------------------------------------------
void loop() {
  // Håll in båda knapparna (rotSw1 & rotSw2) >= 3s => aktivera auto-läge
  if (digitalRead(rotSw1) == LOW && digitalRead(rotSw2) == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
    } 
    else if ((millis() - buttonPressTime >= 3000) && !autoModeActive) {
      // Aktivera auto-läge efter 3 sek
      tone(buzzerPin, 1000, 500);  // Kort pip
      delay(500);
      noTone(buzzerPin);

      buttonPressTime = 0;
      autoModeActive = true;
      targetPosition = 1;  
      setTargetPosition(); // Låt användaren välja hur många stängningar som ska uppnås
    }
  } 
  else {
    buttonPressTime = 0; 
  }

  // Välj läge
  if (autoModeActive) {
    runAutomaticMode();
  } else {
    runManualMode();
  }
}

// ----------------------------------------------------------
//   MANUELLT LÄGE
// ----------------------------------------------------------
void runManualMode() {
  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);

  if (sw1 && sw2) {
    // Båda knappar => motor stopp
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    //Serial.println("Manual: Both buttons pressed => motor off.");
  }
  else if (sw1) {
    // Bara sw1 => motor framåt
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    //Serial.println("Manual: Motor forward.");
  }
  else if (sw2) {
    // Bara sw2 => motor bakåt
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    //Serial.println("Manual: Motor backward.");
  }
  else {
    // Ingen knapp => motor av
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}

// ----------------------------------------------------------
//   AUTOMATISKT LÄGE
// ----------------------------------------------------------
void runAutomaticMode() {
  int posCount = 0;

  Serial.println("\n=== AUTOMATISK DRIFT AKTIVERAD ===");

  // 1) Vänta 1 sekund med motor av innan vi börjar köra
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  delay(1000); 
  Serial.println("Motor still for 1s. Now starting backward movement...");

  // 2) Kör bakåt tills posSw aktiveras första gången => posCount = 0
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  Serial.println("Motor going backward until first posSw activation...");

  // Vänta tills posSw blir LOW
  while (digitalRead(posSw) == HIGH) {
    if (checkExitAuto()) return;  // Möjlighet att avbryta
  }
  // Nu är posSw = LOW => vi har hittat "noll-läget"
  posCount = 0;  // Ställ in att detta är "0"
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  Serial.println("posSw activated first time => posCount=0, motor off.");

  // (IGNORERA första aktiveringen vid framåt-körning)
  // => alltså vänta tills posSw går HIGH igen innan vi börjar räkna nya pulser.
  Serial.println("Waiting for posSw to go HIGH again (to ignore that first activation)...");
  while (digitalRead(posSw) == LOW) {
    if (checkExitAuto()) return;
  }
  Serial.println("posSw is HIGH -> ready to start forward cycle.");

  // 3) Pendla mellan 0 och targetPosition
  while (autoModeActive) {
    // ============= UPP-CYKEL (0 -> targetPosition) =============
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    Serial.println("Pendling: motor forward (upp).");
    
    while (posCount < targetPosition) {
      if (checkExitAuto()) return;
      if (digitalRead(posSw) == LOW) {
        posCount++;
        delay(300); 
        Serial.print("posCount (upp) = ");
        Serial.println(posCount);
      }
    }

    // Stanna uppe
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    Serial.print("Reached targetPosition (");
    Serial.print(targetPosition);
    Serial.println("), motor off.");
    delay(500);

    // ============= NER-CYKEL (targetPosition -> 0) =============
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    Serial.println("Pendling: motor backward (ner).");
    
    while (posCount > 0) {
      if (checkExitAuto()) return;
      if (digitalRead(posSw) == LOW) {
        posCount--;
        delay(300);
        Serial.print("posCount (ner) = ");
        Serial.println(posCount);
      }
    }

    // Tillbaka på 0
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    Serial.println("Back at posCount=0, motor off.");
    delay(500);
  }

  Serial.println("=== AVSLUTAR AUTOMATISK DRIFT ===");
}

// ----------------------------------------------------------
//   checkExitAuto()
//   - Stoppar motorn direkt när EXAKT en knapp trycks.
//   - Om den knappen hålls in i >= 1 sek => avsluta auto-läge
//   - Om knappen släpps innan 1 sek => fortsätt auto-läge
//   - Vid riktig avbrytning => 1s buzzer + 1s motorstill
// ----------------------------------------------------------
bool checkExitAuto() {
  static bool   exitAttempt = false;      // Om vi påbörjat en "avsluts-timer"
  static unsigned long singlePressTime = 0;

  bool sw1 = (digitalRead(rotSw1) == LOW);
  bool sw2 = (digitalRead(rotSw2) == LOW);
  bool singlePressed = (sw1 ^ sw2); // Exakt en knapp är nedtryckt (XOR)

  // Om exakt en knapp är nedtryckt
  if (singlePressed) {
    if (!exitAttempt) {
      // Första gången vi ser en "singel-knapp"
      exitAttempt = true;
      singlePressTime = millis();
      // Motor STOPP direkt!
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      Serial.println("Single button => motor stops. Hold >=1s to confirm exit.");
    }
    else {
      // exitAttempt är redan true => kolla om 1s har gått
      if (millis() - singlePressTime >= 1000) {
        // Kolla om knappen fortfarande är nedtryckt
        sw1 = (digitalRead(rotSw1) == LOW);
        sw2 = (digitalRead(rotSw2) == LOW);
        if ((sw1 ^ sw2)) {
          // Användaren håller kvar knappen => AVBRYT auto-läget!
          Serial.println("Auto-läge avbryts (knapp hölls in i 1s).");

          // 1s buzzer-ljud
          tone(buzzerPin, 1000);
          delay(1000);
          noTone(buzzerPin);

          // 1s motorstill, ignorerar allt
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, LOW);
          delay(1000);

          autoModeActive = false;
          exitAttempt = false;
          singlePressTime = 0;
          return true; // Returnera att vi AVBRÖT
        }
        else {
          // Knappen släpptes precis innan 1 sek
          exitAttempt = false;
          singlePressTime = 0;
          Serial.println("Knapp släpptes innan 1s fullbordades => fortsätt auto-läge.");
        }
      }
    }
  }
  else {
    // Antingen ingen knapp eller båda => nollställ exitAttempt
    if (exitAttempt) {
      Serial.println("Exit-knapp släppt => återgå till pendling.");
    }
    exitAttempt = false;
    singlePressTime = 0;
  }

  return false;
}

// ----------------------------------------------------------
//   setTargetPosition()
//   - Låter användaren justera targetPosition (1–5).
//   - Vid varje ändring: buzzer ljuder "targetPosition" gånger
//   - Skriver även ut i serial hur många "targetPosition" som valts
// ----------------------------------------------------------
void setTargetPosition() {
  while (true) {
    // Kolla om rotSw1 trycks => öka
    if (digitalRead(rotSw1) == LOW) {
      if (targetPosition < 5) {
        targetPosition++;
      }
      beepMultiple(targetPosition);
      Serial.print("Target position increased to: ");
      Serial.println(targetPosition);
      // Vänta 500ms innan ny ändring
      delay(500);
    }

    // Kolla om rotSw2 trycks => minska
    if (digitalRead(rotSw2) == LOW) {
      if (targetPosition > 1) {
        targetPosition--;
      }
      beepMultiple(targetPosition);
      Serial.print("Target position decreased to: ");
      Serial.println(targetPosition);
      // Vänta 500ms innan ny ändring
      delay(500);
    }

    // Avsluta setup om båda knappar trycks
    if ((digitalRead(rotSw1) == LOW) && (digitalRead(rotSw2) == LOW)) {
      delay(1000);
      Serial.println("Avslutar inställning av targetPosition.");
      break;
    }
  }
}

// ----------------------------------------------------------
//   beepMultiple(count)
//   - Spelar upp "count" st pulser.
//   - Varje puls: 500ms toner + 500ms tystnad.
// ----------------------------------------------------------
void beepMultiple(int count) {
  for (int i = 0; i < count; i++) {
    tone(buzzerPin, 1000);   // 1000 Hz
    delay(500);             // 0,5s ljud
    noTone(buzzerPin);
    delay(500);             // 0,5s tystnad
  }
}
