  

---

## 🔧 **System Overview**
This software is designed to control a **motorized transducer mount for fishing**, allowing smooth movement of a sonar transducer. It offers:
- **Manual and Automatic Motor Control**
- **Target Position Adjustments**
- **Motor Speed Adjustment**
- **Buzzer Feedback & Debugging**

---

## 📌 **Pin Configuration**
| **Component**        | **Pin** | **Description**                   |
|----------------------|--------|-----------------------------------|
| **Motor Driver EN**  | 9      | Speed control (PWM)              |
| **Motor Driver IN1** | 8      | Direction control                 |
| **Motor Driver IN2** | 7      | Direction control                 |
| **Position Switch**  | 5      | Detects motor position            |
| **Rotation Button 1** | 6      | User input                        |
| **Rotation Button 2** | 11     | User input                        |
| **Buzzer**           | 10     | Audio feedback                    |

---

## 🚀 **Main Functions**
### 🔹 `void runManualMode(bool sw1, bool sw2)`
**Description:**  
Controls the motor manually based on button inputs.  

**Input:**
- `sw1 (bool)`: State of rotation switch 1 (`true = pressed`)
- `sw2 (bool)`: State of rotation switch 2 (`true = pressed`)

**Output:**
- Moves motor **FORWARD**, **BACKWARD**, or **STOPS** based on button states.
- Prints debug information with **limited frequency**.

---

### 🔹 `void runAutomaticMode()`
**Description:**  
Controls motor movement in an **automatic loop**, moving back and forth based on a **target position**.

**Input:**  
- Uses `autoModeActive` flag to determine if auto mode is enabled.
  
**Process:**
- Moves **BACKWARD** until reaching a limit switch.
- Alternates between **UP & DOWN movements** based on `targetPosition`.
- Can be **aborted by holding any button for 1s**.

---

### 🔹 `void checkBothLongestPress(bool sw1, bool sw2)`
**Description:**  
Handles **long press actions** for both buttons:
- **1s Hold** → Activates Auto Mode  
- **3s Hold** → Opens **Set Target Position** menu  
- **5s Hold** → Opens **Set Motor Speed** menu  

**Input:**
- `sw1 (bool)`: Button state  
- `sw2 (bool)`: Button state  

**Output:**
- Activates specific **modes** or **settings** based on hold duration.
- Prints **hold time in 0.5s increments** for debugging.

---

### 🔹 `void setTargetPosition()`
**Description:**  
Allows the user to set a **target position (2-7)** using button presses.  

**Process:**
- **Increase position:** Press `rotSw1`
- **Decrease position:** Press `rotSw2`
- **Exit Menu:** Hold both buttons for **1 second**

**Output:**
- Updates the `targetPosition` variable.
- Provides **beep feedback** & prints debug messages.

---

### 🔹 `void setMotorSpeed()`
**Description:**  
Allows the user to **adjust motor speed** (25 - 250) and **saves it to EEPROM**.

**Process:**
- **Increase speed:** Press `rotSw1`
- **Decrease speed:** Press `rotSw2`
- **Exit Menu:** Hold both buttons for **1 second**
- Adds **300ms delay before exit to prevent false single-click detection**

**Output:**
- Updates `currentSpeed` and stores it in EEPROM.
- Provides **beep feedback** & prints debug messages.

---

### 🔹 `void updateDebounceTime()`
**Description:**  
Adjusts **button debounce delay** dynamically based on **motor speed**.

**Process:**
- Uses **a static debounce time of 450ms** (previously dynamic).
- **Limits unnecessary debug messages** by only printing when debounce time changes.

---

### 🔹 `void handleAutoButtons()`
**Description:**  
Monitors button inputs **during auto mode** to allow **manual exit**.

**Process:**
- **Holding a button for 1s** exits **Auto Mode**.
- Stops motors before returning to **Manual Mode**.
- Adds a **1-second delay before exiting** to prevent unintended re-entry.

**Output:**
- **Aborts auto mode on long press** and resets flags.

---

### 🔹 `void beepMultiple(int n)`
**Description:**  
Provides **audible feedback** using the **buzzer**.

**Input:**
- `n (int)`: Number of beeps

**Process:**
- **1 Beep** → Confirmation
- **2 Beeps** → Exit menu
- **3+ Beeps** → Mode activation feedback

---

## 🔄 **System Flow Diagram**
```plaintext
+------------------+
| System Startup  |
+------------------+
        |
        v
+-----------------------+
| Load Motor Speed from |
| EEPROM & Initialize   |
+-----------------------+
        |
        v
+---------------------------+
| Manual or Auto Mode Check |
+---------------------------+
   |                     |
   v                     v
[Manual Mode]        [Auto Mode]
   |                     |
   v                     v
[Button Press]       [Auto Sequence]
   |                     |
   v                     v
[Target Position]    [Stop on Abort]
   |
   v
[Exit to Manual Mode]
