# 🧭 Arduino Live Transducer pole Advanced code

This Arduino project is a motor control system that supports manual and automatic modes, Bluetooth control, heading correction and dual motor/sensor configurations.

---

## ⚙️ Features

- Manual control via buttons/pedal (left/right)
- Sweep mode with configurable angle
- Sweep mode activated depending on there the motor is at the moment. 
- Setable manual and sweep speed independently
- Compass mode using magnetometer + PID controller
- Bluetooth (HC-05) communication with live tuning
- EEPROM storage for speed, PID values, and settings
- Supports both L298N and L298P motor drivers
- Works with micro switch or Hall sensor(Work in progress)
- Built-in calibration for heading direction in compass mode
- Menu system via buttons or bluetooth(Terminal or Android App)
- Buzzer feedback for settings and modes

---

## 📚 Libraries Used

```cpp
#include <Arduino.h>
#include <L298N.h>
#include <Adafruit_HMC5883_U.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
```

---

## 🔌 I/O Configuration

| Component             | Pin(s)             | Notes                                              |
|----------------------|--------------------|----------------------------------------------------|
| Motor Driver (L298P) | EN = 3, IN1 = 12, IN2 = 9 | Shield-style L298P                            |
| Motor Driver (L298N) | EN = 10, IN1 = 9, IN2 = 8 | External L298N module                          |
| POS Switch           | 7 or 4             | Used for sweep position detection                  |
| Hall Sensor          | A1                 | Used if `sensorMode = 2`                           |
| Rotation Buttons     | rotSw1 = 6/3, rotSw2 = 11/2 | Used for left/right and mode switching        |
| Pedal Button         | 5                  | Alternate input for long-press functions           |
| Buzzer               | 10 or 7            | Audible feedback                                   |
| Bluetooth HC-05      | RX = 8, TX = 13      | SoftwareSerial                                     |
| Magnetometer         | I2C (SDA, SCL)      | HMC5883L                                           |

> Pins vary depending on selected `driverMode`.

---

## 🎛️ Operating Modes

### 🔹 **Manual Mode**
- Press **rotSw1** → Move **Forward**
- Press **rotSw2** → Move **Backward**
- Release buttons → **Motor Stops**
- Speed can be adjusted via **button controls**

### 🔄 **Sweep Mode**
- The motor **moves back & forth** within a **defined angle**
- The **position switch (POS/Hall)** tracks movement steps
- **Sweep angle** can be set via menu or commands
- Activated by holding both buttons for **300ms**

### 🧭 **Compass Mode** (Work in progress)
- Uses the **HMC5883L magnetometer** to track heading
- PID control adjusts motor to reach **set heading**
- Heading setpoint and PID values adjustable via Serial/Bluetooth
- Activated by holding both buttons for **3 seconds**

### 🎚️ **Speed Adjustment**
- **Hold both buttons for 5 seconds** → Enter Speed Adjustment Menu
- **Tap both buttons quickly** → Switch between Sweep & Manual speed
- **Press rotSw1** → **Increase speed**
- **Press rotSw2** → **Decrease speed**
- **Hold both buttons for 3 seconds** → **Exit the menu**

### 🧪 **Calibration** (Work in progress)
- Activated by holding buttons for **8 seconds** or sending `Q` command
- Motor runs test to determine if heading increases CW or CCW
- Stores direction factor in EEPROM for compass correction

---

## 💾 EEPROM Address Map

| Address | Description            |
|---------|------------------------|
| 0       | Manual speed           |
| 1       | Sweep speed            |
| 4–12    | PID values (Kp, Ki, Kd)|
| 12      | Turn threshold         |
| 20      | Motor direction factor |

---

## 📡 Bluetooth Commands (To be updated)

| Command | Description                              |
|---------|------------------------------------------|
| `L` / `R` | Turn left or right manually            |
| `+` / `-` | Adjust manual speed up/down            |
| `U` / `H` | Adjust sweep speed up/down             |
| `P3.5`    | Set Kp value                           |
| `I0.2`    | Set Ki value                           |
| `D0.6`    | Set Kd value                           |
| `T4.0`    | Set turn threshold                     |
| `A` / `M` | Activate/Deactivate sweep mode         |
| `C`       | Stop compass mode                      |
| `X`       | Toggle magnetometer ON/OFF             |
| `Q`       | Calibrate motor direction              |

---

## 💡 Usage Notes

- Sensor type (`sensorMode`) can be set to:
  - `1` = POS switch (default)
  - `2` = Hall effect sensor
- Button hold durations trigger different modes:
  - **300ms** = Sweep mode
  - **3s** = Compass mode
  - **5s** = Speed adjustment menu
  - **8s** = Calibration
- Settings persist via EEPROM
- All serial and Bluetooth commands work interchangeably

---

## 🔋 Hardware Requirements

- Arduino Uno/Nano or compatible
- L298N or L298P motor driver
- HMC5883L magnetometer (I2C) (Optional)
- HC-05 Bluetooth module (optional)
- POS switch or Hall sensor (One of the two)
- Pedal where you can activate both buttons at the same time or an optinal button(pedalbutton)
- Buzzer (audio feedback)

---

## 🖼️ Optional Enhancements

- Add OLED/LCD for status display
- Add rotary encoder for setting sweep angle
- Add failsafe (e.g. max run time)

---

## 📜 License

MIT License — feel free to use, modify, and improve!

---

---

## 🧭 Menu System Overview

The system includes interactive button-based menus, allowing for configuration without a computer or app.

### 🔘 Menu Navigation

- **Enter Menus** by holding both rotation buttons or pedal:
  - **300ms** → Start Sweep Mode
  - **3s** → Compass Mode
  - **5s** → Speed Adjustment Menu
  - **8s** → Calibration Mode

### 📊 Speed Adjustment Menu

-Speed is depending on the motor gears. PWM-signal 50-250 is what is allowed to set.(will probably change)
- Switch between **manual** and **sweep** speed by holding both buttons/pedal for 1 second
- **rotSw1** → Increase speed by 25 (max 250)
- **rotSw2** → Decrease speed by 25 (min 50)
- EEPROM saves the new values automatically
- Exit the menu by holding both buttons or pedalbutton for 3 seconds

### 🔁 Sweep Angle Menu

- Enter by sending a command or using button press pattern
- **rotSw1** → Increase sweep angle (max 8 steps)
- **rotSw2** → Decrease sweep angle (min 2 steps)
- Exit by holding both buttons or pressing pedalbutton for 3 seconds

### 📌 Safety and Debounce

- All button inputs are debounced (≈50ms)
- Long-press timers are reset when buttons are released
- Timeout logic protects against lock-ups during sweep

---

## 📋 Menu Navigation Table

| Hold Duration | Input                   | Function Triggered             |
|---------------|------------------------|--------------------------------|
| 300 ms        | Both Buttons / Pedal   | Start Sweep Mode               |
| 3 seconds     | Both Buttons / Pedal   | Activate Compass Mode          |
| 5 seconds     | Both Buttons / Pedal   | Enter Speed Adjustment Menu    |
| 8 seconds     | Both Buttons / Pedal   | Calibrate Motor Direction      |
| 1 second      | Both Buttons or Pedal (in Speed Menu) | Switch between Manual/Sweep Speed |
| 3 seconds     | Both Buttons / Pedal (in Menu) | Exit Current Menu          |

> 🛠 All menus give audible feedback (beeps) and save settings to EEPROM.



## Arduino Motor Controller – Simple vs Advanced Version

### Simple Version: DC Motor Control with L298N, Sweep & Bluetooth

This version is designed to be reliable, extendable, and easy to troubleshoot.  
**Main features:**
- **Manual mode:** Control motor direction (left/right) using two buttons or a pedal.
- **Sweep mode:** Motor moves automatically between two end-stops (using limit switch or hall sensor).
- **Bluetooth control:** Adjust speed (+/-) and sweep timeout via Bluetooth app/terminal.
- **Menu system:** Long/short button or pedal presses activate sweep mode or speed adjustment menu.
- **EEPROM:** Remembers last set speed (and optionally sweep timeout) after power cycle.
- **Feedback:** Buzzer signals for actions and alarms (e.g., speed at min/max, timeout alarm).
- **Timeout protection:** If the motor fails to reach the next limit switch within a set time, sweep mode stops and an alarm sounds.

**Intended for:**  
Standard DC motors controlled by an L298N bridge, without the need for advanced position or heading control.

---

### Advanced Version: DC/Stepper/Compass + PID

The advanced version builds on the simple foundation and adds several features for precision and autonomy:

**Main features:**
- **Stepper and DC motor support:** Choose motor type (DC with L298N or stepper).
- **Compass mode:** Uses a magnetometer to keep the device pointing to a specific compass heading.
- **PID control:** Uses a PID algorithm for smooth and accurate heading adjustment.
- **Sweep with angle:** Sweep between configurable angles, not just physical end-stops.
- **EEPROM for multiple parameters:** Stores not just speed, but also PID values, angles, and settings.
- **Dual speed settings:** Separate speed for manual and sweep modes, switchable via menu or Bluetooth.
- **Extended menu system:** Access more parameters (PID tuning, sweep angle, heading, etc.) via Bluetooth or button menus.
- **Calibration:** Includes routines for motor direction calibration and magnetometer zeroing.
- **Debounce, double-press, and safety checks:** More robust handling of button presses and error states.

**Intended for:**  
Projects requiring auto-alignment, heading keeping, or integration with sensors like a compass. Also supports stepper motors for precise angular movement.

---

### Key Differences

| Feature             | Simple Version                               | Advanced Version                                   |
|---------------------|----------------------------------------------|----------------------------------------------------|
| Motor types         | DC motor (L298N) only                        | DC + Stepper (L298N or Yfrobot/Stepper)            |
| Sweep mode          | Basic (between two physical end stops)       | Configurable angle (by step count or heading)      |
| Compass/magnetometer| No                                           | Yes – Compass heading control with PID             |
| PID algorithm       | No                                           | Yes                                                |
| Speed setting       | One speed for all                            | Separate sweep/manual speeds, menu switchable      |
| Bluetooth config    | Speed, sweep timeout                         | Speed, angle, PID, heading, calibration, etc.      |
| EEPROM storage      | Speed (and optional timeout)                 | Speed(s), PID params, heading, sweep angle, etc.   |
| Button logic        | Short/long press for menu/sweep              | Includes double-press, mode switching, angle menu  |
| Calibration         | Manual (hardware)                            | Motor direction/magnetometer calibration via menu  |
| Safety features     | Timeout alarm for sweep                      | More robust (dead zone, compass error checks, etc) |
| Code complexity     | Easy to extend and troubleshoot              | Modular, expandable, more advanced logic           |

---

### Summary

- Use the **simple version** for reliable sweep/manual control of a DC motor with basic limit switches and Bluetooth speed adjustment.
- Use the **advanced version** if you want precision, compass/heading control, PID, support for stepper motors, and more flexible configuration.

Both versions are modular and well-commented, making them suitable for further customization.

