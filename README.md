# 🤖 Autonomous Color-Tracking Line Follower

<div align="center">

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green.svg?style=for-the-badge)

**An intelligent robot that navigates colored paths using real-time color sensing and adaptive decision-making**

[Features](#-features) • [Hardware](#-hardware) • [How It Works](#-how-it-works) • [Setup](#-setup) • [Team](#-team)

</div>

---

## 🎯 The Challenge

Build a robot capable of:
- Following a **black line** from the starting point
- Detecting a **color split** (red or green branching paths)
- **Autonomously choosing** and following one colored path to the finish

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🎨 **Multi-Color Detection** | Distinguishes between white, black, red, and green surfaces in real-time |
| 🔄 **Adaptive Path Correction** | Probes left/right to self-correct when drifting off the line |
| 🧠 **Smart Split Detection** | Stable color classification before committing to a path |
| ⚡ **Auto-Calibration** | Automatically calibrates to the white floor surface at startup |
| 📊 **Debug Output** | Real-time serial logging for performance tuning |

## 🔧 Hardware

### Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Microcontroller** | Arduino (Uno/Nano) | Brain of the operation |
| **Color Sensor** | TCS3200 | RGB color detection |
| **Motor Driver** | L298N | Dual H-Bridge motor control |
| **Motors** | 2× DC Motors | Differential drive locomotion |
| **Distance Sensor** | HC-SR04 | Obstacle detection (expansion ready) |

### Pin Configuration

```
┌─────────────────────────────────────────────────────────┐
│                    PIN ASSIGNMENTS                      │
├─────────────────────────────────────────────────────────┤
│  TCS3200 Color Sensor          L298N Motor Driver       │
│  ├── S0  → Pin 4               ├── ENA → A0             │
│  ├── S1  → Pin 5               ├── IN1 → A1             │
│  ├── S2  → Pin 6               ├── IN2 → A2             │
│  ├── S3  → Pin 7               ├── ENB → A3             │
│  └── OUT → Pin 12              ├── IN3 → Pin 13         │
│                                └── IN4 → A4             │
│  HC-SR04 Ultrasonic                                     │
│  ├── TRIG → Pin 8                                       │
│  └── ECHO → Pin 9                                       │
└─────────────────────────────────────────────────────────┘
```

## 🧠 How It Works

### State Machine

```
┌─────────┐     Detect Red      ┌────────────┐
│  START  │ ─────────────────── │ FOLLOW_RED │
│ (Black) │                     └────────────┘
└────┬────┘
     │
     │ Detect Green   ┌──────────────┐
     └───────────────▶│ FOLLOW_GREEN │
                      └──────────────┘
```

### Line Following Algorithm

Our **single-sensor probing technique** enables accurate line following without the need for a sensor array:

1. **Assess** - Read color score at current heading
2. **Probe Left** - Rotate left and measure line confidence
3. **Probe Right** - Rotate right and measure line confidence  
4. **Decide** - Turn toward the direction with higher score
5. **Advance** - Drive forward along corrected heading

### Color Classification

The TCS3200 outputs frequency proportional to detected light intensity. We sample RGB + Clear channels, normalize the readings, and classify:

```cpp
// Normalized RGB gives color-independent of ambient lighting
// Red dominant?   → RED path
// Green dominant? → GREEN path  
// Dark surface?   → BLACK line
// Otherwise       → WHITE floor
```

## 🚀 Setup

### 1. Hardware Assembly

1. Mount the TCS3200 sensor facing downward, ~1-2cm from the ground
2. Connect motors to L298N outputs (A/B channels)
3. Wire all components according to pin configuration above
4. Power the L298N with 7-12V battery supply

### 2. Software Upload

```bash
# Clone this repository
git clone https://github.com/yourusername/UtraHacks-Bot.git

# Open in Arduino IDE or PlatformIO
# Upload to your Arduino board
```

### 3. Calibration

1. Place robot on **white surface**
2. Power on and wait ~1.2 seconds
3. The robot auto-calibrates its white baseline
4. Place on track and watch it go! 🏁

## ⚙️ Tuning Parameters

Fine-tune behavior by adjusting these constants:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `FWD_PWM` | 220 | Forward motor speed (0-255) |
| `ROT_PWM` | 220 | Rotation motor speed (0-255) |
| `SCAN_PULSE_MS` | 70 | How long to probe left/right |
| `STEER_BASE_MS` | 140 | Base turn duration |
| `STABLE_MS` | 280 | Time to confirm color before split decision |

## 📈 Performance Metrics

- **Response Time**: ~140ms decision cycle
- **Color Samples**: 3 samples per channel (median filter)
- **Accuracy**: Handles tight curves and 90° turns
- **Split Detection**: Stable classification before path commitment

## 🛤️ Future Improvements

- [ ] Implement ultrasonic obstacle avoidance
- [ ] Add PID control for smoother line following
- [ ] Integrate IMU for drift compensation
- [ ] Wireless telemetry via Bluetooth/WiFi

## 👥 Team

Built with ❤️ for **UTra Hacks**

---

<div align="center">

**⭐ Star this repo if you found it helpful! ⭐**

</div>
