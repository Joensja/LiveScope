# LiveScope motor control
# Dual Motor Control with menues to change settings

This code demonstrates a dual-motor control system where:

- **Motor A** is controlled via two digital pins (`motorPin1`, `motorPin2`) in a simple “HIGH/LOW” manner.
- **Motor B** is driven by an [L298N library](https://github.com/alex-mous/L298N) using an enable pin (PWM for speed control) plus two digital inputs (forward/backward).

The system also features:
- A **position sensor** (`posSw`) used for an automatic “pendulum” mode.
- **Two push buttons** (`rotSw1`, `rotSw2`) for both **manual movement** and **entering special menus** (via “largest time wins” press durations).
- A **buzzer** (`buzzerPin`) for acoustic feedback in various operations.

Below is an overview of how each module works.

---

## Pin Configuration

| **Pin**        | **Function**                                |
|----------------|---------------------------------------------|
| `motorPin1=2`  | Motor A output (simple digital output 1)    |
| `motorPin2=4`  | Motor A output (simple digital output 2)    |
| `EN=9`         | PWM enable pin for Motor B (L298N)          |
| `IN1=8`        | Direction pin 1 for Motor B (L298N)         |
| `IN2=7`        | Direction pin 2 for Motor B (L298N)         |
| `posSw=5`      | Position sensor (active LOW switch)         |
| `rotSw1=6`     | Button 1 (active LOW) for forward/manual, etc. |
| `rotSw2=11`    | Button 2 (active LOW) for backward/manual, etc. |
| `buzzerPin=10` | Buzzer for generating beeps                  |

The pin directions are as follows:
- **Motor A**: `motorPin1` and `motorPin2` are `OUTPUT`.  
- **Motor B**: uses the L298N library with `EN`, `IN1`, and `IN2`.  
- **Buttons**: `rotSw1` and `rotSw2` are `INPUT_PULLUP`.  
- **Position sensor**: `posSw` is also `INPUT_PULLUP`.  
- **Buzzer**: `buzzerPin` is `OUTPUT`.

---

## Functionality

### 1. Manual Mode

When **auto mode** is not active, the code checks `rotSw1` and `rotSw2`:
- **Motor A**:
  - `rotSw1` pressed => `motorPin1=HIGH, motorPin2=LOW` (forward).
  - `rotSw2` pressed => `motorPin1=LOW, motorPin2=HIGH` (reverse).
  - Both or none => stop (LOW, LOW).

- **Motor B** (L298N):
  - Same logic: `forward(currentSpeed)` if `rotSw1` is pressed alone, `backward(currentSpeed)` if `rotSw2` is pressed alone, otherwise `stop()`.
  - The `currentSpeed` value for Motor B is set by the user in a menu (see “setMotorSpeed” below).

### 2. Automatic Mode

When **auto mode** is active:
1. The code performs **three quick beeps** (`beepMultiple(3)`) then waits 1 second with motors stopped.
2. It moves **Motor A** and **Motor B** backward until `posSw` goes LOW (meaning `posCount=0`).  
3. It enters a **pendulum** routine, repeatedly going forward until `posCount` reaches `targetPosition`, then going backward until `posCount` returns to zero, pausing 1 second between direction changes.

If the user holds exactly one button for **2 seconds**, the code **aborts auto mode** (one-second beep, then stops the motors for one second).

### 3. “Largest Time Wins” on Both Buttons

When **both** buttons `rotSw1` and `rotSw2` are held, a “largest time wins” approach is used upon **release**:
- **>=10 seconds:** Open `setTargetPosition` menu (with **5** 1-second beeps).
- **>=7 seconds:** Open `setMotorSpeed` menu (with **3** 1-second beeps).
- **>=3 seconds:** Activate auto mode (with **1** 1-second beep).
- Otherwise, do nothing if <3 seconds.

If the system was already in auto mode, it will pause the motors during the wait/menuselection and resume after the menu.

### 4. setMotorSpeed()

- **Triggered** by a 7-second dual-button press.
- The motor is stopped during this menu.
- The user can increase/decrease `currentSpeed` in **25**-unit increments, with **wrap-around** from 250 to 25 or from 25 to 250.
- Each time a speed step is set, the code **beeps** `X` times, each beep being 500ms on and 500ms off, where `X = currentSpeed / 25`.
- After the user finishes (presses both buttons for 1s), the menu exits, resuming any previous state (manual or auto).

### 5. setTargetPosition()

- **Triggered** by a 10-second dual-button press.
- Also stops the motors during adjustments.
- The user can set `targetPosition` in the range **1..7**, with wrap-around if desired (1 <-> 7).
- Each time the position is changed, the code calls `beepMultiple500(...)`, beeping the new target position number times (500ms beep each).
- The menu exits if the user holds both buttons again for ~1 second.

### 6. Aborting Menus in Auto Mode

If auto mode is active, but a menu is triggered (e.g., holding both buttons for 7 or 10 seconds), the code sets `autoModeActive = false` during the menu. When the menu finishes, it restores `autoModeActive` to the previous value so the auto mode can continue.

---

## Buzzer Feedback

1. **Short pulses** (250ms ON, 250ms OFF) for triple beep calls (`beepMultiple(n)`).
2. **Single beep** (1s) for auto mode activation at 3 seconds.
3. **Multiple beeps** with 1-second on, 1-second off for bigger announcements:
   - 3 beeps (7s) => `setMotorSpeed`
   - 5 beeps (10s) => `setTargetPosition`
4. **Menus** for speed or position changes beep in 500ms intervals: if the user sets `currentSpeed = 100` => beep 4 times (4 = 100/25), each 500ms on + 500ms off.

---

## Debug Prints

- **Manual Mode**: Logs `[Manual] Motor B => FORWARD, speed=...` or `...BACKWARD, speed=...`, or `...STOP`.
- **Auto Mode**: Logs `[Auto] Motor B => FORWARD...` or `BACKWARD`, plus the `speed` value. Also logs pendulum progress: `posCount(up)=` or `posCount(down)=`.
- **Menus**:
  - **`setMotorSpeed()`** logs `// Debug: motorB speed => XXX` every time the speed changes.  
  - **`setTargetPosition()`** logs `// Debug: targetPosition => XXX` every time the position changes.

---

## Summary

- **Pin 2 & 4**: Simple outputs for **Motor A**.
- **Pin 9, 8, 7**: L298N library for **Motor B** (speed + direction).
- **Pins 6 & 11**: Two push buttons, read as `LOW` when pressed (using `INPUT_PULLUP`).
- **Pin 5**: Position sensor for the pendulum logic (`LOW` triggers an increment/decrement of `posCount`).
- **Pin 10**: Buzzer for acoustic feedback.

The code ensures that **Motor B’s speed** can be set independently and stored (`currentSpeed`), while **Motor A** is always driven at full digital levels. Users can hold **both** buttons for different durations (3, 7, or 10 seconds) to trigger **Auto Mode**, **Speed Menu**, or **Position Menu**. If **Auto Mode** is active, it can be aborted by holding a single button for 2 seconds.

Enjoy and adapt further to your project’s hardware specifics!
 
