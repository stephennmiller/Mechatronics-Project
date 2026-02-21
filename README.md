# Mechatronics Project

Arduino Mega script for a maze-solving robot with line and wall navigation.

## Description

This project implements a maze-solving robot using an Arduino Mega 2560. The robot navigates a line maze (black tape on a light surface) using 4 IR sensors, then transitions through an overlap zone into a walled maze using 3 ultrasonic sensors. It uses PID control for both modes, a non-blocking state machine, and dynamic wall switching to solve simply-connected mazes.

## Features

- **Dual navigation modes** — line following (IR) and wall following (ultrasonic) with automatic transition
- **PID control** — separate, isolated controllers for each mode with two-layer anti-windup
- **Non-blocking state machine** — no `delay()` calls during navigation; sensors read continuously even during turns
- **Dynamic wall switching** — debounced left/right wall selection based on sensor readings
- **Junction handling** — T-junction and four-way detection with consistent left-turn heuristic
- **Calibration-friendly** — all tunable constants in a single config section at the top of the file
- **Togglable debug output** — `#define DEBUG` compiles serial output in or out
- **Per-motor trim offsets** — compensate for physical motor differences without changing PID gains
- **L298N short-brake** — active braking for faster stops

## Hardware

### Bill of Materials

| Qty | Component | Purpose | Notes |
|-----|-----------|---------|-------|
| 1 | Arduino Mega 2560 | Main controller | Provides 54 digital I/O pins, 16 analog inputs, 15 PWM outputs |
| 4 | DC gear motors | Drive wheels | One per wheel; skid-steer (tank drive) configuration |
| 4 | Wheels | Locomotion | Sized to match motor shafts |
| 2 | L298N dual H-bridge motor drivers | Motor control | Each drives 2 motors (left pair + right pair); supports PWM speed and direction |
| 4 | IR reflectance sensors (analog) | Line detection | Mounted in a row across the front; detect black tape on light surface |
| 3 | HC-SR04 ultrasonic sensors | Wall detection | Front, left, and right facing; 2-200 cm range |
| 6 | LEDs (any color) | Status indicators | Power, error, line mode, wall mode, right wall, left wall |
| 6 | 220-330 ohm resistors | LED current limiting | One per LED |
| 1 | 8V battery pack (e.g., 6xAA or 2S LiPo) | Power supply | Must supply enough current for 4 motors + logic |
| 2 | Resistors for voltage divider | Battery monitoring | Scale 8V down to 0-5V for analog pin A4; e.g., 30k + 10k |
| 1 | Robot chassis / frame | Structure | Must fit 4 motors, sensors, and the Mega |
| - | Jumper wires, breadboard, standoffs | Wiring | As needed for connections |

### Wiring Summary

**Motor Drivers (L298N):**

| L298N | Channel | Motor | PWM Pin | Direction Pins |
|-------|---------|-------|---------|----------------|
| #1 | A | Left Front | 2 | 22, 23 |
| #1 | B | Left Rear | 3 | 24, 25 |
| #2 | A | Right Front | 4 | 26, 27 |
| #2 | B | Right Rear | 5 | 28, 29 |

- Connect each L298N's `VCC` to the battery pack positive terminal
- Connect each L298N's `GND` to common ground (shared with Arduino GND)
- Connect each L298N's `5V` output to nothing (or use one to power the Arduino via the 5V pin if not using USB)
- The ENA/ENB jumpers must be **removed** — the PWM pins replace them

**IR Sensors:**

| Sensor | Position | Pin |
|--------|----------|-----|
| IR1 | Leftmost | A0 |
| IR2 | Center-left | A1 |
| IR3 | Center-right | A2 |
| IR4 | Rightmost | A3 |

- Mount in a straight line across the front of the chassis, evenly spaced to span the tape width
- Power from Arduino 5V; signal wire to the analog pin

**Ultrasonic Sensors (HC-SR04):**

| Sensor | Direction | Trig Pin | Echo Pin |
|--------|-----------|----------|----------|
| Front | Forward | 30 | 31 |
| Left | Left side | 32 | 33 |
| Right | Right side | 34 | 35 |

- Power from Arduino 5V; GND to common ground
- Mount at consistent height (~5-10 cm from ground) for reliable wall detection

**LEDs:**

| LED | Function | Pin |
|-----|----------|-----|
| Power | Robot is running | 36 |
| Error | Fault detected (e.g., low battery) | 37 |
| Line | Line-following mode active | 38 |
| Wall | Wall-following mode active | 39 |
| Right | Following right wall | 40 |
| Left | Following left wall | 41 |

- Each LED needs a 220-330 ohm resistor in series to GND

**Battery Monitor:**

- Voltage divider from battery positive to pin A4
- Formula: `R2 / (R1 + R2)` must scale the max battery voltage to under 5V
- Example: R1 = 30k, R2 = 10k → 8V * 10/(30+10) = 2V at pin (safe)
- Update `VOLTAGE_SCALE` in Section 3 of `MazeRobot.ino` to match your resistors

## Dependencies

- [NewPing library](https://bitbucket.org/teckel12/arduino-new-ping/downloads/) (install via Arduino Library Manager)

## Setup

1. Wire the components as per the pin definitions in Section 2 of `MazeRobot.ino`.
2. Install the NewPing library in the Arduino IDE.
3. Upload `MazeRobot.ino` to your Arduino Mega.

## Calibration

Before running the maze, tune these values in **Section 3** of `MazeRobot.ino`:

| Constant | What to do |
|---|---|
| `IR_HIGH_ON_LINE` | Read raw serial values over tape vs floor. Set to `1` if higher = on tape. |
| `IR_LINE_THRESHOLD` | Set halfway between on-tape and off-tape raw readings. |
| `VOLTAGE_SCALE` | Measure your voltage divider resistors and update the formula. |
| `TRIM_*` | If the robot drifts, adjust per-motor offsets (+/- 30). |
| `TURN_90_DURATION` | Run a turn test and adjust until the robot turns exactly 90 degrees. |
| `KP/KI/KD_LINE/WALL` | Start with Kp only, increase until oscillation, back off 20%, then add Kd and Ki. |

## Usage

- Place the robot at the start of a line maze.
- It follows the line until transitioning to a wall maze (detected when IR confidence drops and walls appear).
- The robot navigates walls, handles junctions, and stops with a victory blink at the exit (open space on all three sensors).
- Open the Serial Monitor at **115200 baud** to see debug output (when `DEBUG` is defined).

## Code Structure

The sketch is organized into numbered sections:

1. **Debug Toggle** — `#define DEBUG` and macros
2. **Pin Assignments** — hardware wiring
3. **Calibration Constants** — all tunable values
4. **Type Definitions** — `PIDController`, `Motor`, `Timer` structs
5. **State Machine** — `RobotState` enum
6. **Global Instances** — motors, sensors, PID controllers
7. **Drive Helpers** — `driveTank()`, `driveStop()`, `startTurn()`
8. **Sensor Functions** — IR and ultrasonic reading
9. **Line Following** — PID line tracking with line-loss handling
10. **Wall Following** — PID wall tracking with junction detection
11. **State Transitions** — overlap zone detection, exit detection
12. **Error/Battery Handling** — voltage monitoring, error state
13. **Setup & Main Loop** — initialization and state machine dispatch
