# 🚀 Motor Control with Bluetooth & Button Input (Arduino)

## 📖 Overview
This project controls a **DC motor** via an **L298N motor driver** using:
- **Bluetooth (HC-05)**
- **Manual Button Controls**
- **EEPROM for speed storage**
- **Automatic Mode & Manual Mode**

The user can **adjust speeds, switch modes, and control the motor** via:
1. **Physical buttons**
2. **Bluetooth commands via HC-05**

## 🔧 Hardware Components
| Component       | Description |
|----------------|------------|
| Arduino Board  | Main controller |
| L298N Motor Driver | Controls motor speed and direction |
| HC-05 Bluetooth Module | Allows remote control via smartphone or PC |
| Push Buttons  | Used for manual control and menu selection |
| Buzzer | Provides audible feedback |
| EEPROM | Stores speed settings for Auto & Manual mode |

---

## 📌 **Features**
### 🎛️ **Manual Mode**
- Hold one button to move **forward** or **backward**.
- Adjust speed in increments of **25**.
- Maximum speed: **250**, Minimum speed: **25**.
- Long beep **(400ms)** when changing speed.

### 🔄 **Automatic Mode**
- Motor sweeps between **positions**.
- Speed is adjustable in **auto mode** separately from manual mode.
- Two **quick beeps** when adjusting speed.

### 🎮 **Button Input & Holding Actions**
| Hold Time | Action |
|-----------|--------|
| 1s  | **Switch to Auto Mode** (1 beep) |
| 3s  | **Open Target Position Menu** (2 beeps) |
| 5s  | **Open Speed Adjustment Menu** (3 beeps) |
| 8s  | **Trigger 5s Continuous Beep, No Action** |

### 📡 **Bluetooth Commands**
| Command | Action |
|---------|--------|
| `L` | Rotate Left |
| `R` | Rotate Right |
| `S` | Stop Motor |
| `+` / `-` | Increase / Decrease **Manual Speed** |
| `U` / `D` | Increase / Decrease **Auto Speed** |
| `A` | Activate Auto Mode |
| `M` | Deactivate Auto Mode |

### 🔊 **Buzzer Feedback**
| Beep Type | Meaning |
|-----------|---------|
| 🔊 **1 Long Beep** | Manual speed change |
| 🔊 **2 Quick Beeps** | Auto speed change |
| 🔊 **3 Short Beeps** | Switching speed mode |
| 🔊 **2 Quick + 2 Long Beeps** | Speed limit (Min/Max) reached |
| 🔊 **5s Continuous Beep** | 8s button hold detected (No action) |

---

## 🎮 **Button Press Flowchart**
```plaintext
                     +---------------------------+
                     |  Press Both Buttons       |
                     +---------------------------+
                                |
            +---------------------------------+
            | Hold for:                      |
            |                                 |
   +--------+--------+------------+----------+--------+
   | 1s     | 3s     | 5s         | 8s       | Release|
   | (1B)   | (2B)   | (3B)       | (5s Beep)|        |
   | Switch | Target | Adjust     | No Action| Reset  |
   | Mode   | Pos    | Speed Mode |          |        |
   +--------+--------+------------+----------+--------+
