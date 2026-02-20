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

- Arduino Mega 2560
- 4 DC motors (via 2 L298N drivers, individual control)
- 4 IR sensors (line detection, evenly spaced in a row)
- 3 Ultrasonic sensors (front, left, right wall detection)
- 8V battery pack with voltage divider on A4
- 6 LEDs (power, error, line mode, wall mode, right wall, left wall)

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
