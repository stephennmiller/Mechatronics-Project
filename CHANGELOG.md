<!-- markdownlint-disable MD024 -->
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.1.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.1.0) - 2026-02-21

### Added

- Dead-end detection in `followWall()` — walls on all three sides trigger a single backup + 180-degree U-turn instead of two 90-degree recovery cycles
- `TURN_180_DURATION` constant for tuning U-turn timing
- `pendingTurnDuration` global to carry turn duration through the backup-then-turn state sequence
- Optional `duration` parameter to `startTurn()` and `startBackupAndTurn()` with defaults preserving existing behavior

## [v1.0.1](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.0.1) - 2026-02-21

### Fixed

- Replace `DEBUG_PRINTF` calls using `%f` with chained `Serial.print()` — AVR `snprintf` does not support `%f`, so debug output was printing garbage
- Use `distFiltered[0]` instead of raw `dist[0]` for front obstacle check, preventing false triggers from noisy 0 cm spikes
- Initialize `dist[]` to `MAX_DISTANCE` to prevent phantom 0 cm walls during the first 3 loop iterations
- Clamp trim in `motorSet()` to preserve motor direction — adjusted speed stays within `[1, 255]` or `[-255, -1]`

### Changed

- Make `irPins[]` and `sonars[]` `static const` to avoid rebuilding constant arrays on the stack every `loop()` iteration

## [v1.0.0](https://github.com/stephennmiller/Mechatronics-Project/releases/tag/v1.0.0) - 2026-02-21

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
