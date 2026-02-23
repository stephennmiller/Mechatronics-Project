<!-- markdownlint-disable MD024 -->
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.4.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.4.0) - 2026-02-23 ([#37](https://github.com/stephennmiller/Mechatronics-Project/pull/37))

### Added

- `calibrateIRSensors()` function in Section 8 — 4-second auto-calibration routine at startup that samples on-line and off-line surfaces, computes per-sensor thresholds, and auto-detects polarity via majority vote
- `irLineThresh[]`, `irNoLineThresh[]`, `irHighOnLine`, `irCalibrated` globals in Section 6 for runtime calibration state
- `NUM_IR_SENSORS`, `IR_CAL_DURATION_MS`, `IR_CAL_SETTLE_MS`, `IR_CAL_SAMPLE_INTERVAL`, `IR_CAL_MIN_RANGE` calibration constants in Section 3
- LED feedback during calibration: fast blink (phase 1), solid ON (phase 2), error blink on failure
- Debug output of per-sensor min/max/threshold/noLine values after calibration
- Calibration usage instructions in README

### Changed

- `readIRSensors()` now uses per-sensor `irLineThresh[i]` and runtime `irHighOnLine` instead of compile-time `IR_LINE_THRESHOLD` and `IR_HIGH_ON_LINE`
- `linePosition()` uses runtime `irHighOnLine` instead of compile-time `IR_HIGH_ON_LINE`
- `lineCount()` and `linePosition()` loop bounds use `NUM_IR_SENSORS` instead of hardcoded `4`
- `isWallMazeTransition()` uses per-sensor `irNoLineThresh[i]` and runtime `irHighOnLine`
- `setup()` replaces the 3-second countdown with `calibrateIRSensors()` call
- Renamed `IR_HIGH_ON_LINE`, `IR_LINE_THRESHOLD`, `IR_NO_LINE_THRESH` to `IR_DEFAULT_HIGH_ON_LINE`, `IR_DEFAULT_LINE_THRESH`, `IR_DEFAULT_NO_LINE_THRESH` (hardcoded fallback values)

## [v1.3.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.3.0) - 2026-02-23 ([#36](https://github.com/stephennmiller/Mechatronics-Project/pull/36))

### Added

- `JunctionStrategy` enum with four maze-solving strategies: `STRATEGY_LEFT_WALL`, `STRATEGY_RIGHT_WALL`, `STRATEGY_ALTERNATING`, `STRATEGY_RANDOM`
- `getJunctionTurn()` function in Section 7 that returns a `TurnDir` based on the active strategy
- `junctionStrategy` and `junctionCount` globals in Section 6 for strategy selection and junction tracking
- `PIN_RANDOM_SEED` constant in Section 2 for the floating analog pin used by `randomSeed()`
- Debug logging of strategy name at startup and turn direction at each junction

### Changed

- Junction handling in `followWall()` now calls `getJunctionTurn()` instead of hardcoded `TURN_LEFT`

## [v1.2.1](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.2.1) - 2026-02-22 ([#35](https://github.com/stephennmiller/Mechatronics-Project/pull/35))

### Added

- `USIndex` enum (`US_FRONT`, `US_LEFT`, `US_RIGHT`) replacing raw `[0]`/`[1]`/`[2]` indices on `dist[]` and `distFiltered[]`
- `BACKUP_SPEED` constant replacing `BASE_SPEED / 2` in `startBackupAndTurn()`
- `LINE_MIN_SPEED` constant (`-50`) allowing negative wheel speeds in `followLine()` for sharper PID corrections
- `NUM_MOTORS` constant replacing magic `4` in motor loop iterations
- `stateTimer` FSM invariant comment documenting single-timed-state design

### Changed

- `readUltrasonicSensors()` now uses front-priority scheduling: front sensor reads every call, left/right alternate — halves obstacle detection latency (~90ms to ~60ms)
- `followLine()` PID constrain floor changed from `0` to `LINE_MIN_SPEED` (`-50`)
- `DEBUG_PRINTF` buffer increased from 80 to 120 characters
- `VOLTAGE_SCALE` macro now has explicit parentheses around division: `(((R1+R2)/R2) * (5.0/1023.0))`

## [v1.2.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.2.0) - 2026-02-22 ([#34](https://github.com/stephennmiller/Mechatronics-Project/pull/34))

### Added

- Stuck detection watchdog (`checkStuck()`) with three independent signals:
  - Ultrasonic stagnation — no `distFiltered[]` change >2 cm for 3 seconds
  - PID-output saturation — control output pinned near ±255 for 3 seconds
  - Turn timeout — `STATE_TURNING` exceeds 2 seconds
- Auto-recovery via `startBackupAndTurn()` with alternating left/right escape direction
- Escalation to `ERROR_STATE` after 2 failed retries within a 5-second cooldown window
- `NUM_US_SENSORS` constant replacing magic number 3 in sensor arrays
- `lastPidOutput` global for PID saturation tracking
- `docs/ARCHITECTURE.md` with Mermaid state machine diagram, loop execution flow, stuck detection algorithm description, and file layout table
- Constants: `STUCK_TIMEOUT_MS`, `STUCK_DIST_THRESHOLD`, `STUCK_MAX_RETRIES`, `STUCK_COOLDOWN_MS`, `STUCK_PID_THRESHOLD`, `TURN_TIMEOUT_MS`

### Fixed

- Use `%d` with `(int)` casts in `DEBUG_PRINTF` instead of `%.0f` — AVR `snprintf` does not support float format specifiers
- Skip long-range sensors (>`WALL_FAR_THRESH`) in stuck movement check to prevent HC-SR04 noise from causing false snapshot refreshes

## [v1.1.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.1.0) - 2026-02-21 ([#33](https://github.com/stephennmiller/Mechatronics-Project/pull/33))

### Added

- Dead-end detection in `followWall()` — walls on all three sides trigger a single backup + 180-degree U-turn instead of two 90-degree recovery cycles
- `TURN_180_DURATION` constant for tuning U-turn timing
- `pendingTurnDuration` global to carry turn duration through the backup-then-turn state sequence
- Optional `duration` parameter to `startTurn()` and `startBackupAndTurn()` with defaults preserving existing behavior

## [v1.0.1](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.0.1) - 2026-02-21 ([#2](https://github.com/stephennmiller/Mechatronics-Project/pull/2))

### Fixed

- Replace `DEBUG_PRINTF` calls using `%f` with chained `Serial.print()` — AVR `snprintf` does not support `%f`, so debug output was printing garbage
- Use `distFiltered[0]` instead of raw `dist[0]` for front obstacle check, preventing false triggers from noisy 0 cm spikes
- Initialize `dist[]` to `MAX_DISTANCE` to prevent phantom 0 cm walls during the first 3 loop iterations
- Clamp trim in `motorSet()` to preserve motor direction — adjusted speed stays within `[1, 255]` or `[-255, -1]`

### Changed

- Make `irPins[]` and `sonars[]` `static const` to avoid rebuilding constant arrays on the stack every `loop()` iteration

## [v1.0.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.0.0) - 2026-02-21 ([#1](https://github.com/stephennmiller/Mechatronics-Project/pull/1))

### Added

- Dual-mode navigation: line following (IR PID) transitions automatically to wall following (ultrasonic PID)
- Non-blocking 7-state FSM with millis()-based timers
- Isolated PID controllers for line and wall modes with two-layer anti-windup
- Motor abstraction with per-motor trim offsets, signed speed convention, coast and active brake modes
- Dynamic wall selection with debounced switching
- Left-wall-following strategy at T-junctions and four-way intersections
- Exit detection when all three ultrasonic sensors read open space
- Staggered ultrasonic reads (one per loop) to prevent echo crosstalk
- EMA-filtered distances for stable navigation
- 3-second startup countdown with LED blinks
- Low-battery detection with automatic error state and motor shutdown
- Single config section at the top of `MazeRobot.ino` for all tunable constants
