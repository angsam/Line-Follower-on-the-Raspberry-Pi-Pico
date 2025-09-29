# Line Follower (Raspberry Pi Pico W, MicroPython)

This repository contains a library that allows you to run a simple line follower on the Raspberry Pi Pico. It consists of:
- `line_follower.py` — the library
- `main.py` — a minimal runner

The robot uses an **L298N** motor driver, **three analog reflectance sensors** (ADC on GP26/27/28), and an **optional HC‑SR04** ultrasonic sensor for basic obstacle handling.

## Features
- PID line control (P/D by default, `Ki` usually 0 for this project)
- Non‑linear steering (`TURN_GAMMA`, `TURN_GAIN`, `TURN_MAX`) and straight‑line turbo
- Automatic sensor calibration (move the robot over track & background for 6 s)
- Robust “lost line” behavior: gentle arcs → stronger arcs → spin search
- Optional obstacle slowdown/stop using HC‑SR04
- Clean, dependency‑free MicroPython code

## Pinout (defaults)
- **Motors (L298N):** ENA=GP21, IN1=GP20, IN2=GP19, ENB=GP16, IN3=GP18, IN4=GP17  
- **Line sensors (analog):** L=GP26 (ADC0), C=GP27 (ADC1), R=GP28 (ADC2)  
- **Ultrasonic (optional):** TRIG=GP14, ECHO=GP15

You can change pins in `main.py` where `motors_init`, `sensors_init`, and `ultrasound_init` are called.

## Quick start
1. Copy `line_follower.py` and `main.py` to the Pico W (e.g. with **Thonny** or `mpremote cp`).
2. Power the robot and open the serial console.
3. You’ll see **“Calibration (6 s)…”** — slowly move the sensors across black line and background.
4. The robot will start following the line using the default parameters.

More details can be found in Teachers’ guidelines for the project "Smart car: how to reduce the number of car accidents?" published at https://www.iot.fizyka.pw.edu.pl (Results tab).

## Parameters (excerpt)
All parameters are returned by `params_default()` in `line_follower.py`:
```python
{
    'BASE_SPEED': 48000,   # Base PWM (0..65535)
    'Kp': 0.70, 'Ki': 0.00, 'Kd': 0.22, 'I_MAX': 2.0,  # PID
    'TURN_GAIN': 2.4, 'TURN_MAX': 2.6, 'TURN_GAMMA': 0.85,  # steering
    'TURN_SLOWDOWN': 0.7, 'TURN_BETA': 1.7,  # slow down in curves
    'STRAIGHT_EPS': 0.10, 'STRAIGHT_BOOST': 1.05,  # speed-up on straights
    'REV_THRESHOLD': 1.1,  # allow inner wheel reverse above this correction
    'ERROR_SIGN': 1.0,  # flip sign if sensors are mirrored
    'DETECT_THR': 0.18, 'SEARCH_SUM_THR': 0.12,  # line detect & loss
    'LOST_MS1': 300, 'LOST_MS2': 1200,  # arc phases before spin
    'SPIN_PWM': 52000, 'SPIN_MS_EACH': 1500,     # spin search
    'DIST_STOP_CM': 12, 'DIST_SLOW_CM': 30,      # obstacle thresholds
    'LOOP_DT_MS': 6
}
```

## Tuning tips
1. Start with **`Kp`** (increase until you see small oscillations), then add **`Kd`** to damp them. Keep **`Ki = 0`** unless your track demands it.
2. Adjust **`TURN_GAIN`** (steering amplitude) and **`TURN_MAX`** (clip) to get confident cornering without over‑steer.
3. If straights feel too slow, use **`STRAIGHT_BOOST`**; if corners are too fast, increase **`TURN_SLOWDOWN`** or **`TURN_BETA`**.
4. If your sensors are mounted mirrored, set **`ERROR_SIGN = -1.0`** or use `flip_lr=True` in `sensors_init`.
5. For noisy sensors, raise **`DETECT_THR`** slightly; if the robot loses the line, check **`SEARCH_SUM_THR`** and the “lost line” time thresholds.

## Obstacle handling (optional)
`ultrasound_init()` sets up HC‑SR04. The loop periodically measures distance and scales speed:
- `DIST_STOP_CM` — below this the robot **stops**,
- `DIST_SLOW_CM` — below this the robot **slows down** linearly to zero.

Set `ultrasonic_sensor = None` in `main.py` to disable this feature.

## How to run your own pins
Edit these calls in `main.py`:
```python
motor = lf.motors_init(ena_pin=21, in1_pin=20, in2_pin=19, enb_pin=16, in3_pin=18, in4_pin=17, drive_dir=-1)
line_sensors = lf.sensors_init(left=26, center=27, right=28, alpha=0.7, flip_lr=False)
ultrasonic_sensor = lf.ultrasound_init(trig_pin=14, echo_pin=15)  # or None
```

## 📚 Project Context

This library was developed as part of the IoT4Schools project (Erasmus+, 2023-1-PL01-KA220-SCH-000154043) — an international educational initiative bringing Internet of Things and cloud computing to schools across Europe using Raspberry Pi Pico W and MicroPython.

Disclaimer:
*The European Commission's support to produce this publication does not constitute an endorsement of the contents, which reflect the views only of the authors, and the Commission cannot be held responsible for any use which may be made of the information contained therein.*

## 📝 License

License: CC BY-NC 4.0
Attribution-NonCommercial 4.0 International
You are free to share and adapt the material for non-commercial purposes, with proper attribution.
