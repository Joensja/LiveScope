# 🚀 Arduino Motor Control System for Live transducer pole

An **Arduino-based motor control system** supporting **L298N & L298P motor drivers**, featuring **manual & sweep modes**, **speed adjustments**, and **Bluetooth control**.

---

## 📌 Features

✅ **Supports L298N & L298P Motor Drivers**  
✅ **Manual & Sweep Modes for motor control**  
✅ **Adjustable speed with EEPROM storage**  
✅ **Bluetooth control via HC-05 module**  
✅ **Button-controlled settings adjustments**  
✅ **Buzzer alerts for user feedback**  

---

## 🔧 Hardware Requirements

- **Arduino Board** (Uno, Mega, etc.)
- **L298N / L298P Motor Driver**
- **DC Motors**
- **HC-05 Bluetooth Module** (Optional)
- **Push Buttons**
- **Buzzer** for alerts

---

## 🛠️ Wiring Setup

| Component              | Arduino Pin (L298P) | Arduino Pin (L298N) |
|------------------------|--------------------|--------------------|
| **Motor Driver EN**    | `3`                | `10`              |
| **Motor Driver IN1**   | `12`               | `9`               |
| **Motor Driver IN2**   | `-`               | `8`               |
| **Position Switch**    | `7`                | `4`               |
| **Rotation Switch 1**  | `6`                | `3`               |
| **Rotation Switch 2**  | `11`               | `2`               |
| **Buzzer**            | `10`               | `7`               |

---

## 🎛️ Operating Modes

### 🔹 **Manual Mode**
- Press **rotSw1** → Move **Forward**
- Press **rotSw2** → Move **Backward**
- Release buttons → **Motor Stops**
- Speed can be adjusted via **button controls**

### 🔄 **Sweep Mode**
- The motor **moves back & forth** within a **defined angle**.
- The **position switch (posSw)** is used for movement tracking.
- Sweep **angle can be set** 

### 🎚️ **Speed Adjustment**
- **Hold both buttons for 5 seconds** → Enter Speed Adjustment Mode
- **Tap both buttons quickly** → Switch between **Sweep & Manual speed**
- **Press rotSw1** → **Increase speed**
- **Press rotSw2** → **Decrease speed**
- **Hold both buttons for 3 seconds** → **Exit the menu**

---

## 📝 Button Press Timing Actions

| Button Hold Time | Action |
|-----------------|--------|
| **300ms** | Activate Sweep Mode |
| **3s** | Open Sweep Angle Adjustment |
| **5s** | Open Speed Adjustment Menu |
| **8s** | Continuous 5-second beep warning |

---

