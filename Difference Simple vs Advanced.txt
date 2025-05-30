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

