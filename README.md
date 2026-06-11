# Robot Laberinto — Ultrasonido

Maze-solving robot for the **Amateur** competition category.  
Algorithm: **Left-Hand Rule** (wall follower, left wall priority).

**Build status:** ✅ Compiles clean — 27.6% Flash / 14.5% RAM (Arduino Uno)

---

## Hardware

| Component | Model | Role |
|---|---|---|
| Microcontroller | Arduino Uno (ATmega328P) | Brain |
| Motor driver | L298N | Controls both DC motors |
| Motors | 2× Motorreductor DC (yellow, plastic gears) | Drive wheels |
| Scanner servo | SG90 | Rotates the ultrasonic sensor |
| Distance sensor | HC-SR04 | Detects walls (left / front / right) |
| Floor sensor | TCRT5000 or FC-51 | Detects white exit cell |
| Battery | 2× 18650 in series (~7.4 V) | Powers everything |

---

## Wiring

### Power Circuit

```
[ 18650 ] ──[ 18650 ] ──── [SWITCH] ──── VS (12V in) ─── L298N
                                            │
                                          (+) battery positive → L298N VS
                                          (–) battery negative → L298N GND

L298N 5V out ────────────────────────────── Arduino VIN  (or 5V pin if jumper removed)
L298N GND   ────────────────────────────── Arduino GND
```

> **Important (competition rule):** No buck converters, step-up or step-down regulators allowed in Amateur.  
> The L298N has a built-in 7805 linear regulator (5V out) — this is legal.  
> Total voltage: 2 × 18650 = ~7.4 V nominal, 8.4 V fully charged. Well under the 13 V limit.

---

### L298N → Arduino Uno

| L298N Pin | Arduino Pin | Description |
|---|---|---|
| IN1 | D2 | Motor A (left) direction 1 |
| IN2 | D3 | Motor A (left) direction 2 |
| IN3 | D4 | Motor B (right) direction 1 |
| IN4 | D5 | Motor B (right) direction 2 |
| ENA | D9 (PWM) | Motor A speed — hardware PWM |
| ENB | D10 (PWM) | Motor B speed — hardware PWM |
| GND | GND | Common ground |
| 5V out | VIN | Powers Arduino |

> **Note:** Remove the ENA and ENB jumpers from the L298N board — otherwise speed control via PWM has no effect.

---

### SG90 Servo → Arduino Uno

| Servo Wire | Arduino Pin | Description |
|---|---|---|
| Signal (orange/yellow) | D6 (PWM) | PWM control via `Servo.h` |
| VCC (red) | 5V | Power (from Arduino 5V or L298N 5V out) |
| GND (brown/black) | GND | Common ground |

**Physical mounting:** Mount the servo on the front of the chassis, horn pointing forward. The HC-SR04 sits on top of the servo horn.

```
Servo angles (defined in Config.h):
  160° → sensor points LEFT
   90° → sensor points FRONT (center)
   20° → sensor points RIGHT

If readings feel mirrored → swap SERVO_LEFT_DEG and SERVO_RIGHT_DEG in Config.h
```

---

### HC-SR04 → Arduino Uno

| HC-SR04 Pin | Arduino Pin | Description |
|---|---|---|
| VCC | 5V | Power |
| TRIG | D7 | Trigger pulse (output from Arduino) |
| ECHO | D8 | Echo pulse (input to Arduino) |
| GND | GND | Common ground |

> **Note:** HC-SR04 ECHO pin outputs 5V — safe for Arduino Uno's 5V I/O.

---

### IR Floor Sensor (TCRT5000 / FC-51) → Arduino Uno

| IR Module Pin | Arduino Pin | Description |
|---|---|---|
| VCC | 5V | Power |
| GND | GND | Common ground |
| D0 (digital out) | D11 | Exit detection — LOW = white floor |

**Mounting:** Sensor faces **downward**, as close to the floor as practical (5–15 mm). Adjust the onboard potentiometer to reliably distinguish black floor (HIGH) from white exit cell (LOW).

---

### Status LED

The built-in LED on pin 13 is used for status signaling — no extra wiring needed.

| Pattern | Meaning |
|---|---|
| 3 slow blinks at boot | Initialized — robot is about to start |
| Rapid blink (5 Hz) | **Exit found** — run complete |
| SOS pattern (3 fast, 3 slow, 3 fast) | Stuck timeout — stopped to avoid penalty |

---

### Full Wiring Summary (Arduino Uno)

```
D2  ── L298N IN1  (Motor A dir 1)
D3  ── L298N IN2  (Motor A dir 2)
D4  ── L298N IN3  (Motor B dir 1)
D5  ── L298N IN4  (Motor B dir 2)
D6  ── SG90  Signal
D7  ── HC-SR04 TRIG
D8  ── HC-SR04 ECHO
D9  ── L298N ENA  (PWM)
D10 ── L298N ENB  (PWM)
D11 ── IR sensor D0
D13 ── Built-in LED (no wiring needed)

5V  ── HC-SR04 VCC, SG90 VCC, IR sensor VCC
GND ── All component GNDs (common ground)
VIN ── L298N 5V output (to power Arduino from battery)
```

---

## Code Architecture

```
robot-laberinto-ultrasonido/
├── include/
│   └── Config.h              ← ALL calibration constants live here
├── lib/
│   ├── MotorDriver/
│   │   ├── MotorDriver.h     ← L298N dual motor interface
│   │   └── MotorDriver.cpp
│   ├── UltrasonicScanner/
│   │   ├── UltrasonicScanner.h  ← SG90 + HC-SR04 — 3-position sweep
│   │   └── UltrasonicScanner.cpp
│   └── MazeSolver/
│       ├── MazeSolver.h      ← Left-Hand Rule state machine
│       └── MazeSolver.cpp
├── src/
│   └── main.cpp              ← setup() / loop() — orchestrates everything
└── platformio.ini
```

### Algorithm — Left-Hand Rule

At each decision point (one cell), the solver:

```
1. Check IR sensor → white floor? → STOP, signal EXIT FOUND
2. Scan LEFT, FRONT, RIGHT with ultrasonic
3. Priority:
   - No wall on LEFT  → turn left  → advance one cell
   - No wall on FRONT → advance straight
   - No wall on RIGHT → turn right → advance one cell
   - All blocked      → U-turn (two 90° right turns) → advance
4. Check stuck timeout (12 s) → if triggered → STOP, SOS blink
```

**Guarantee:** Left-Hand Rule always finds the exit in simply-connected mazes (no isolated wall islands). Competition mazes typically satisfy this.

---

## Build & Flash

```bash
# Install PlatformIO CLI if needed
pip install platformio

# Build
pio run

# Flash to Arduino Uno (connect via USB first)
pio run --target upload

# Open Serial Monitor for debug output (9600 baud)
pio device monitor --baud 9600
```

---

## Calibration Guide

All constants are in [`include/Config.h`](include/Config.h). **Never hardcode these in logic files.**

### Step 1 — TURN_TIME_MS (most critical)

This controls the 90° pivot turn accuracy. A wrong value means the robot won't align with corridors.

```
1. Upload the code.
2. Place the robot on the competition surface (black matte wood).
3. Open Serial Monitor.
4. Mark the starting orientation with tape.
5. Observe the turn — if it overshoots 90°, reduce TURN_TIME_MS by 20 ms.
   If it undershoots, increase by 20 ms.
6. Typical range: 450–650 ms depending on battery charge.

IMPORTANT: Re-calibrate whenever you change batteries or floor surface.
```

### Step 2 — WALL_DISTANCE_THRESHOLD_CM

The HC-SR04 is mounted on the servo at varying heights. Walls are 15 cm tall.

```
1. Place robot 10 cm from a wall → Serial should print distance ~10 cm.
2. Place robot 20 cm from a wall → Serial should print ~20 cm.
3. Set WALL_DISTANCE_THRESHOLD_CM to a value between these two
   (default: 15 cm — adjust if sensor is mounted high and reads shorter).
```

### Step 3 — IR Floor Sensor

```
1. Place the IR sensor over the BLACK floor → onboard LED should be OFF.
2. Place over WHITE cell → onboard LED should be ON.
3. Turn the potentiometer until the transition is clean and reliable.
4. The code uses digitalRead (LOW = white) — no code changes needed
   once the potentiometer is correctly adjusted.
```

### Step 4 — MOTOR_SPEED / TURN_SPEED

```
Default: MOTOR_SPEED=130, TURN_SPEED=110 (out of 255).
- If the robot moves too slowly → increase in steps of 10.
- If it overshoots cells or drifts too much on turns → decrease.
- Always re-calibrate TURN_TIME_MS after changing TURN_SPEED.
```

### Step 5 — SERVO_LEFT_DEG / SERVO_RIGHT_DEG

```
Default: LEFT=160°, RIGHT=20°, CENTER=90°.
If distance readings feel mirrored (left reads right wall):
  → Swap SERVO_LEFT_DEG and SERVO_RIGHT_DEG values in Config.h.
```

---

## Competition Checklist

- [ ] Switch is visible and accessible (homologation requirement)
- [ ] Robot fits within a single 25×25 cm cell with margin to turn
- [ ] No external cables connected during the run
- [ ] IR floor sensor pot adjusted for black/white on competition floor
- [ ] `TURN_TIME_MS` calibrated on the actual competition surface
- [ ] Battery fully charged (8.4 V = 2× 18650 full charge)
- [ ] USB cable unplugged before competition run (no external comms allowed)
- [ ] Robot stops and blinks on exit detection — verified before competition

---

## Competition Rules Reference

| Rule | Value | Implementation |
|---|---|---|
| Category | Amateur | Arduino Uno + L298N + HC-SR04 (all permitted) |
| Battery limit | ≤ 13 V | 2× 18650 = 7.4 V ✅ |
| No buck/boost converters | Prohibited in Amateur | L298N uses 7805 linear regulator (legal) ✅ |
| Switch required | Mandatory | Must be visible and accessible |
| Immobility timeout | 15 s = penalty | `STUCK_TIMEOUT_MS = 12000` (3 s safety margin) |
| Exit detection | White floor cell | IR sensor on D11 |
| No external comms during run | Mandatory | Disconnect USB before run |
| Cell size | 25 cm × 25 cm | `CELL_WIDTH_CM = 25` |
| Wall height | 15 cm | Affects sensor mounting height |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot turns less/more than 90° | `TURN_TIME_MS` off | Calibrate on competition floor |
| Robot ignores left wall, goes straight | Servo angles mirrored | Swap `SERVO_LEFT_DEG`/`SERVO_RIGHT_DEG` |
| Robot doesn't stop at exit | IR sensor not detecting white | Adjust potentiometer on sensor module |
| Motors don't move | ENA/ENB jumpers still on L298N | Remove the jumpers |
| Robot stops immediately (SOS blink) | Stuck timeout at start | Check sensor wiring — HC-SR04 GND connected? |
| Serial shows 400 cm everywhere | HC-SR04 not wired correctly | Check TRIG/ECHO pins and common GND |
| Robot veers to one side | Motors have different friction | Fine-tune `MOTOR_SPEED` per motor in `MotorDriver.cpp` |
