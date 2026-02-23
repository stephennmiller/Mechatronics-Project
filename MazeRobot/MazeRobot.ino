// ========================================================
//  MazeRobot.ino - Maze-Solving Robot for Arduino Mega
//  Hardware: 4 DC motors (2x L298N), 4 IR sensors,
//            3 Ultrasonic sensors, 6 LEDs
//  UCF Mechatronics Project
//
//  OVERVIEW:
//  This robot navigates two types of mazes in sequence:
//    1. LINE MAZE - follows black tape on a light surface
//       using 4 IR reflectance sensors and PID control.
//    2. WALL MAZE - follows walls using 3 ultrasonic
//       distance sensors and PID control.
//
//  The robot automatically detects when the line maze
//  transitions into the wall maze (an overlap zone where
//  both tape and walls are present) and switches modes.
//
//  ARCHITECTURE:
//  - Non-blocking state machine: the main loop() never
//    blocks. All timed actions (turns, pauses) use
//    millis()-based timers so sensors keep reading.
//  - Isolated PID controllers: line and wall modes each
//    have their own PID state (integral, previous error)
//    to prevent cross-contamination on mode switches.
//  - Motor abstraction: each motor is a struct with its
//    own trim offset for physical calibration.
//
//  STATE MACHINE DIAGRAM:
//    LINE_FOLLOWING ──> MODE_SWITCH ──> WALL_FOLLOWING
//          │                               │
//          │ (stuck)                  ┌─────┼─────┐
//          └──────> BACKING_UP <─────┘     v     v
//                        │          (stuck) │  EXIT_FOUND
//                        v                  │
//                    TURNING ───────────────┘
//
//    Any state can transition to ERROR_STATE on low battery
//    or repeated stuck detection.
//
//  DEPENDENCIES:
//  - NewPing library (install via Arduino IDE Library Manager)
// ========================================================

#include <NewPing.h>

// Forward declarations to prevent Arduino auto-prototype errors.
// The Arduino preprocessor generates function prototypes at the top
// of the file, before struct/enum definitions. Explicit prototypes
// with forward-declared types suppress the broken auto-prototypes.
struct PIDController;
struct Motor;
struct Timer;
enum RobotState : uint8_t;
enum TurnDir : uint8_t;
enum JunctionStrategy : uint8_t;

void pidReset(PIDController* pid);
float pidCompute(PIDController* pid, float error);
void motorInit(const Motor* m);
void motorSet(const Motor* m, int speed);
void motorBrake(const Motor* m);
void timerStart(Timer* t, unsigned long durationMs);
bool timerExpired(Timer* t);
bool timerRunning(Timer* t);
TurnDir getJunctionTurn();
void startTurn(TurnDir dir, RobotState afterState, unsigned long duration);
void startBackupAndTurn(TurnDir dir, RobotState afterState, unsigned long turnDuration);
bool checkStuck();

// =====================================================
// SECTION 1: DEBUG TOGGLE
//
// When DEBUG is defined, serial output is compiled in.
// When commented out, all DEBUG_* macros expand to nothing,
// saving ~2KB flash and eliminating serial blocking delays.
//
// Use 115200 baud in the Serial Monitor to view output.
// Debug messages are throttled to every 200ms to avoid
// flooding the serial buffer and slowing down the loop.
// =====================================================
#define DEBUG

#ifdef DEBUG
  // DEBUG_PRINT/PRINTLN: direct wrappers around Serial
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)

  // DEBUG_PRINTF: formatted output using snprintf into a stack buffer.
  // Limited to 120 chars per message. Uses the GCC ## extension to
  // handle zero variadic arguments gracefully.
  #define DEBUG_PRINTF(fmt, ...) do { \
    char _dbuf[120]; \
    snprintf(_dbuf, sizeof(_dbuf), fmt, ##__VA_ARGS__); \
    Serial.print(_dbuf); \
  } while(0)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

// =====================================================
// SECTION 2: PIN ASSIGNMENTS
//
// Change these ONLY when you physically rewire the robot.
// All PWM-capable pins (ENA/ENB) must be on Arduino Mega
// PWM pins: 2-13 and 44-46. Direction pins (IN1-IN8) can
// be any digital pin.
//
// WIRING SUMMARY:
//   L298N #1 controls the LEFT side (front + rear motors)
//   L298N #2 controls the RIGHT side (front + rear motors)
//   Each L298N has two H-bridge channels (A and B):
//     Channel A: ENA (speed via PWM), IN1+IN2 (direction)
//     Channel B: ENB (speed via PWM), IN3+IN4 (direction)
// =====================================================

// L298N #1 - Left Motors
#define PIN_ENA1 2    // Left Front Speed (PWM) - L298N #1 Channel A
#define PIN_IN1  22   // Left Front Direction 1
#define PIN_IN2  23   // Left Front Direction 2
#define PIN_ENB1 3    // Left Rear Speed (PWM)  - L298N #1 Channel B
#define PIN_IN3  24   // Left Rear Direction 1
#define PIN_IN4  25   // Left Rear Direction 2

// L298N #2 - Right Motors
#define PIN_ENA2 4    // Right Front Speed (PWM) - L298N #2 Channel A
#define PIN_IN5  26   // Right Front Direction 1
#define PIN_IN6  27   // Right Front Direction 2
#define PIN_ENB2 5    // Right Rear Speed (PWM)  - L298N #2 Channel B
#define PIN_IN7  28   // Right Rear Direction 1
#define PIN_IN8  29   // Right Rear Direction 2

// IR Sensors
// Arranged in a line across the front of the robot, evenly spaced.
// Viewed from behind the robot: IR1 is leftmost, IR4 is rightmost.
// These are analog inputs returning 0-1023.
#define PIN_IR1  A0   // Leftmost sensor
#define PIN_IR2  A1   // Center-left sensor
#define PIN_IR3  A2   // Center-right sensor
#define PIN_IR4  A3   // Rightmost sensor

// Ultrasonic Sensors (HC-SR04 compatible, used with NewPing library)
// Each sensor has a TRIG (output) and ECHO (input) pin.
#define PIN_TRIG_FRONT 30
#define PIN_ECHO_FRONT 31
#define PIN_TRIG_LEFT  32
#define PIN_ECHO_LEFT  33
#define PIN_TRIG_RIGHT 34
#define PIN_ECHO_RIGHT 35

// Status LEDs
// These provide visual feedback about the robot's current state.
#define PIN_LED_POWER  36   // ON when robot is powered and running
#define PIN_LED_ERROR  37   // ON when in error state (e.g., low battery)
#define PIN_LED_LINE   38   // ON during line-following mode
#define PIN_LED_WALL   39   // ON during wall-following mode
#define PIN_LED_RIGHT  40   // ON when following the right wall
#define PIN_LED_LEFT   41   // ON when following the left wall

// Battery voltage sense (via voltage divider to keep within 0-5V)
#define PIN_BATTERY    A4

// Floating (unconnected) analog pin used as noise source for randomSeed()
#define PIN_RANDOM_SEED A5

// =====================================================
// SECTION 3: CALIBRATION CONSTANTS
// >>> TUNE THESE ON THE ACTUAL ROBOT <<<
// =====================================================

// -- Motor Calibration --
#define NUM_MOTORS            4   // Left-front, left-rear, right-front, right-rear
#define BASE_SPEED          150   // Default forward PWM (0-255)
#define SLOW_SPEED          100   // Tight-space PWM
#define TURN_SPEED           75   // PWM during spin turns
#define BACKUP_SPEED         75   // PWM during reverse before turns
#define LINE_MIN_SPEED      -50   // Min wheel speed in line PID (negative = reverse for sharp turns)

// Per-motor speed trim offsets (-30 to +30).
// If the robot drifts right, increase left trims or decrease right trims.
#define TRIM_LEFT_FRONT       0
#define TRIM_LEFT_REAR        0
#define TRIM_RIGHT_FRONT      0
#define TRIM_RIGHT_REAR       0

// -- Turn Timing (ms) --
// These depend on motor speed, wheel diameter, and track width.
// Calibrate by running a 90-degree turn and adjusting until accurate.
#define TURN_90_DURATION    300   // ms to spin for ~90 degrees at TURN_SPEED
#define BACKUP_DURATION     200   // ms to reverse before turning
#define TURN_180_DURATION   (2 * TURN_90_DURATION)  // Derived from 90° duration
#define MODE_SWITCH_PAUSE   500   // ms pause when switching line->wall

// -- IR Sensor Calibration --
// Hardcoded defaults (fallback if auto-calibration fails)
#define IR_DEFAULT_HIGH_ON_LINE   1
#define IR_DEFAULT_LINE_THRESH  500
#define IR_DEFAULT_NO_LINE_THRESH 200

// Auto-calibration parameters
#define NUM_IR_SENSORS           4
#define IR_CAL_DURATION_MS    4000   // Total calibration window
#define IR_CAL_SETTLE_MS      1000   // Phase 1: on-line baseline collection
#define IR_CAL_SAMPLE_INTERVAL  10   // ms between reads during cal
#define IR_CAL_MIN_RANGE       100   // Min (max-min) per sensor for valid cal

// -- Wall/Ultrasonic Calibration --
#define MAX_DISTANCE        200   // Max ultrasonic range (cm)
// Named indices for ultrasonic sensor arrays (dist[], distFiltered[])
enum USIndex : uint8_t { US_FRONT = 0, US_LEFT = 1, US_RIGHT = 2, NUM_US_SENSORS = 3 };
#define WALL_SETPOINT      25.0   // Target wall-follow distance (cm)
#define WALL_CLOSE_THRESH    30   // "Wall is nearby" (cm)
#define WALL_FAR_THRESH      50   // "Wall is far away" (cm)
#define FRONT_OBSTACLE_DIST  15   // Front wall trigger distance (cm)
#define EXIT_THRESHOLD      100   // Open space = maze exit (cm)
#define DEBOUNCE_COUNT        3   // Wall-switch debounce cycles
#define WALL_TRANSITION_DEBOUNCE 5 // Consecutive readings before line->wall switch

// -- PID Tuning --
// Start with Kp only (Ki=0, Kd=0). Increase Kp until oscillation,
// then back off ~20%. Add Kd to dampen overshoot. Add Ki last, small values.
#define KP_LINE           25.0
#define KI_LINE            0.1
#define KD_LINE           10.0
#define INTEGRAL_MAX_LINE 100.0   // Anti-windup clamp

#define KP_WALL           20.0
#define KI_WALL            0.05
#define KD_WALL           15.0
#define INTEGRAL_MAX_WALL  80.0

// -- Battery --
// Voltage divider: V_battery * R2/(R1+R2) = V_pin
// Adjust DIVIDER_R1 and DIVIDER_R2 to match YOUR resistor values.
#define DIVIDER_R1          30000.0  // High-side resistor (ohms)
#define DIVIDER_R2          10000.0  // Low-side resistor (ohms)
#define VOLTAGE_SCALE       (((DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2) * (5.0 / 1023.0))
#define VOLTAGE_THRESHOLD   6.5

// -- Ultrasonic Filtering --
#define US_ALPHA            0.3   // EMA smoothing factor (0-1, higher = less smoothing)

// -- Stuck Detection / Watchdog --
// Monitors ultrasonic readings during active navigation. If no sensor
// changes by more than STUCK_DIST_THRESHOLD for STUCK_TIMEOUT_MS, the
// robot is assumed stuck and a recovery maneuver is attempted.
#define STUCK_TIMEOUT_MS       3000   // No progress for this long = stuck
#define STUCK_DIST_THRESHOLD    2.0   // cm change in any sensor = "moving"
                                      // (well above ~0.5cm EMA noise floor)
#define STUCK_MAX_RETRIES         2   // Recovery attempts before error state
#define STUCK_COOLDOWN_MS      5000   // Repeated stuck within this window
                                      // increments retry counter; outside
                                      // this window resets counter to 1
#define STUCK_PID_THRESHOLD  10.0     // PID output within this of ±255 = saturated
#define TURN_TIMEOUT_MS      2000     // Turn exceeding this duration = stuck

// -- Junction Strategy --
// See Section 6 (junctionStrategy) for maze-solving algorithm selection.

// =====================================================
// SECTION 4: TYPE DEFINITIONS
//
// Three core structs power the robot:
//   PIDController - feedback control for line/wall following
//   Motor         - hardware abstraction for a single DC motor
//   Timer         - non-blocking delay replacement
// =====================================================

// --- PID Controller ---
//
// A self-contained PID (Proportional-Integral-Derivative) controller.
// Each navigation mode (line following, wall following) gets its own
// instance so they don't share state. This prevents bugs like:
//   - Integral carryover from one mode poisoning the other
//   - Derivative spikes when switching modes (prevError mismatch)
//
// TWO-LAYER ANTI-WINDUP:
//   Layer 1 (direct clamping): The integral accumulator is clamped
//     to +/- integralMax every cycle. This sets a hard ceiling on
//     how much the integral term can contribute.
//   Layer 2 (conditional integration): When the PID output is
//     saturated (hit outMin or outMax), the integral is only updated
//     if the error would REDUCE the saturation. This prevents the
//     integral from growing deeper into saturation when the robot
//     is stuck (e.g., against a wall).
//
// USAGE:
//   PIDController pid = { Kp, Ki, Kd, outMin, outMax, integralMax,
//                         0, 0, 0, false };
//   pidReset(&pid);           // Call before first use or on mode switch
//   float out = pidCompute(&pid, error);  // Call each loop iteration
//
struct PIDController {
  // Configuration (set once at initialization, tune as needed)
  float Kp, Ki, Kd;      // Proportional, Integral, Derivative gains
  float outMin, outMax;   // Output clamp range (e.g., -255 to 255)
  float integralMax;      // Max absolute value for integral accumulator

  // State (managed internally by pidReset/pidCompute)
  float integral;         // Running integral sum (accumulated error * dt)
  float prevError;        // Error from previous iteration (for derivative)
  unsigned long lastTime; // Timestamp of last computation (ms)
  bool initialized;       // False until first pidCompute() call
};

// pidReset: Clear all PID state. Call this when:
//   - Switching between navigation modes
//   - Switching which wall to follow (resets integral to prevent overshoot)
//   - The robot has been stopped and is resuming
void pidReset(PIDController* pid) {
  pid->integral = 0.0;
  pid->prevError = 0.0;
  pid->lastTime = millis();
  pid->initialized = false;
}

// pidCompute: Calculate PID output given the current error.
//
// Parameters:
//   pid   - pointer to the PID controller instance
//   error - current error (setpoint - measurement, or measurement - setpoint
//           depending on your convention; be consistent)
//
// Returns:
//   Control output, clamped to [outMin, outMax].
//
// On the FIRST call after reset, only the P term is used. This avoids
// a derivative spike from (error - 0) / dt when prevError hasn't been set.
//
// If less than 1ms has elapsed since the last call, only the P term is
// returned to avoid division-by-nearly-zero in the derivative calculation.
float pidCompute(PIDController* pid, float error) {
  unsigned long now = millis();

  // First call after reset: use P-only to avoid derivative spike
  if (!pid->initialized) {
    pid->lastTime = now;
    pid->prevError = error;
    pid->initialized = true;
    return constrain(pid->Kp * error, pid->outMin, pid->outMax);
  }

  // Calculate time delta in seconds
  float dt = (now - pid->lastTime) / 1000.0;
  if (dt <= 0.001) {
    // Less than 1ms elapsed - too short for a meaningful derivative.
    // This can happen if loop() runs faster than 1ms on the Mega.
    return constrain(pid->Kp * error, pid->outMin, pid->outMax);
  }

  // P term: proportional to current error
  float pTerm = pid->Kp * error;

  // I term: integral of error over time (anti-windup layer 1: direct clamp)
  float newIntegral = pid->integral + error * dt;
  newIntegral = constrain(newIntegral, -pid->integralMax, pid->integralMax);
  float iTerm = pid->Ki * newIntegral;

  // D term: rate of change of error
  float dTerm = pid->Kd * (error - pid->prevError) / dt;

  // Sum all terms
  float output = pTerm + iTerm + dTerm;

  // Anti-windup layer 2: conditional integration.
  // If output is saturated, only allow the integral to change if it
  // would move AWAY from saturation (i.e., the error has reversed).
  if (output > pid->outMax) {
    output = pid->outMax;
    if (error < 0) pid->integral = newIntegral;  // Error reversed: allow shrink
    // else: error is still pushing into saturation, don't grow integral
  } else if (output < pid->outMin) {
    output = pid->outMin;
    if (error > 0) pid->integral = newIntegral;  // Error reversed: allow shrink
  } else {
    pid->integral = newIntegral;  // Normal operation: commit integral
  }

  pid->prevError = error;
  pid->lastTime = now;
  return output;
}

// --- Motor ---
//
// Represents a single DC motor connected to one channel of an L298N
// H-bridge driver. Each motor has:
//   - enablePin: PWM output controlling speed (0-255)
//   - in1Pin/in2Pin: direction control (see truth table below)
//   - trim: per-motor speed offset for physical calibration
//
// L298N DIRECTION TRUTH TABLE:
//   IN1=HIGH, IN2=LOW  -> Forward
//   IN1=LOW,  IN2=HIGH -> Backward
//   IN1=LOW,  IN2=LOW  -> Coast (motor free-spins)
//   IN1=HIGH, IN2=HIGH -> Short brake (motor resists rotation)
//
struct Motor {
  uint8_t enablePin;  // PWM pin for speed control
  uint8_t in1Pin;     // Direction pin 1
  uint8_t in2Pin;     // Direction pin 2
  int8_t trim;        // Speed offset (-30 to +30) to compensate for
                      // physical differences between motors. If robot
                      // drifts right, increase left motor trims.
};

// motorInit: Configure all three motor pins as outputs.
void motorInit(const Motor* m) {
  pinMode(m->enablePin, OUTPUT);
  pinMode(m->in1Pin, OUTPUT);
  pinMode(m->in2Pin, OUTPUT);
}

// motorSet: Set motor speed with a signed convention.
//   Positive speed (1 to 255)  = forward
//   Negative speed (-255 to -1) = backward
//   Zero                        = coast (free-spin stop)
//
// The trim offset is applied only to non-zero speeds (and preserves the
// original sign), so speed=0 always coasts regardless of trim.
void motorSet(const Motor* m, int speed) {
  int adjustedSpeed;
  if (speed > 0) {
    adjustedSpeed = constrain(speed + m->trim, 1, 255);
  } else if (speed < 0) {
    adjustedSpeed = constrain(speed - m->trim, -255, -1);
  } else {
    adjustedSpeed = 0;
  }

  if (adjustedSpeed > 0) {
    digitalWrite(m->in1Pin, HIGH);
    digitalWrite(m->in2Pin, LOW);
    analogWrite(m->enablePin, adjustedSpeed);
  } else if (adjustedSpeed < 0) {
    digitalWrite(m->in1Pin, LOW);
    digitalWrite(m->in2Pin, HIGH);
    analogWrite(m->enablePin, -adjustedSpeed);  // analogWrite takes positive value
  } else {
    // Coast: both direction pins LOW, no PWM
    digitalWrite(m->in1Pin, LOW);
    digitalWrite(m->in2Pin, LOW);
    analogWrite(m->enablePin, 0);
  }
}

// motorBrake: Active short-brake using the L298N.
// Both direction pins HIGH causes the H-bridge to short the motor
// terminals, creating electromagnetic braking. This stops the motor
// much faster than coast (motorSet with 0), which just lets it spin down.
void motorBrake(const Motor* m) {
  digitalWrite(m->in1Pin, HIGH);
  digitalWrite(m->in2Pin, HIGH);
  analogWrite(m->enablePin, 255);
}

// --- Non-Blocking Timer ---
//
// Replaces delay() calls with millis()-based timing. This is critical
// because delay() blocks the entire loop - no sensor reads, no battery
// checks, no emergency stops during the delay.
//
// USAGE:
//   Timer t = { 0, 0, false };
//   timerStart(&t, 300);              // Start a 300ms timer
//   // ... in loop() ...
//   if (timerExpired(&t)) { ... }     // Check if time has elapsed
//
// The timer automatically marks itself as not running when it expires.
// timerExpired() returns true if the timer has expired OR was never started.
struct Timer {
  unsigned long duration;   // How long the timer runs (ms)
  unsigned long startTime;  // When the timer was started (millis())
  bool running;             // True while counting down
};

// timerStart: Begin a countdown of durationMs milliseconds.
void timerStart(Timer* t, unsigned long durationMs) {
  t->duration = durationMs;
  t->startTime = millis();
  t->running = true;
}

// timerExpired: Check if the timer's duration has elapsed.
// Returns true if expired or if the timer was never started.
// Automatically sets running=false on expiration.
bool timerExpired(Timer* t) {
  if (!t->running) return true;  // Not running = "done"
  if (millis() - t->startTime >= t->duration) {
    t->running = false;
    return true;
  }
  return false;
}

// timerRunning: Check if the timer is currently counting down.
bool timerRunning(Timer* t) {
  return t->running;
}

// =====================================================
// SECTION 5: STATE MACHINE
//
// The robot's behavior is governed by a finite state machine (FSM).
// Each state represents a distinct operating mode. Transitions
// between states happen based on sensor readings and timer events.
//
// Key design principle: EVERY state returns quickly from loop().
// No state ever blocks. Long-duration actions (turns, pauses) use
// the Timer struct and check for completion each loop iteration.
// This ensures sensors are always being read and the battery is
// always being checked, even mid-maneuver.
//
// STATE DESCRIPTIONS:
//   LINE_FOLLOWING - PID tracks the black tape using IR sensors.
//                    Transitions to MODE_SWITCH when tape disappears
//                    and walls are detected (overlap zone).
//
//   MODE_SWITCH    - Brief non-blocking pause (MODE_SWITCH_PAUSE ms)
//                    while transitioning from line to wall mode.
//                    Resets wall PID, updates LEDs. Transitions to
//                    WALL_FOLLOWING when timer expires.
//
//   WALL_FOLLOWING - PID maintains distance from the nearest wall.
//                    Handles junction detection, wall switching, and
//                    speed adjustment. Transitions to BACKING_UP or
//                    TURNING on obstacles/junctions, or EXIT_FOUND
//                    when all sensors read open space.
//
//   BACKING_UP     - Robot reverses for BACKUP_DURATION ms before
//                    a turn. Needed because the robot may be too
//                    close to a front wall to turn in place.
//                    Transitions to TURNING when timer expires.
//
//   TURNING        - Robot spins in place for TURN_90_DURATION ms.
//                    Transitions to returnState (usually WALL_FOLLOWING)
//                    when timer expires.
//
//   ERROR_STATE    - Motors stopped, error LED on. Currently requires
//                    a hardware reset to recover. Could be extended
//                    with a timed retry (e.g., re-check battery after
//                    5 seconds).
//
//   EXIT_FOUND     - Maze solved. Motors stopped, victory LED blink.
//                    Terminal state.
// =====================================================

enum RobotState : uint8_t {
  STATE_LINE_FOLLOWING,   // Following black tape with IR sensors
  STATE_WALL_FOLLOWING,   // Following walls with ultrasonic sensors
  STATE_TURNING,          // Non-blocking spin turn in progress
  STATE_BACKING_UP,       // Non-blocking reverse before a turn
  STATE_MODE_SWITCH,      // Pause between line -> wall transition
  STATE_ERROR,            // Fault detected, motors stopped
  STATE_EXIT_FOUND        // Maze solved, robot stopped
};

// Direction for spin turns (used by startTurn/startBackupAndTurn)
enum TurnDir : uint8_t { TURN_LEFT, TURN_RIGHT };

// Junction-handling strategy for wall maze navigation.
// Controls which direction the robot turns at T-junctions and four-way
// intersections. See Section 6 for the active strategy selection.
enum JunctionStrategy : uint8_t {
  STRATEGY_LEFT_WALL,    // Always turn left (left-wall following)
  STRATEGY_RIGHT_WALL,   // Always turn right (right-wall following)
  STRATEGY_ALTERNATING,  // Alternate left/right at each junction
  STRATEGY_RANDOM        // Random turn direction (seeded from floating pin)
};

// =====================================================
// SECTION 6: GLOBAL INSTANCES
//
// All runtime objects are declared here. Each motor, sensor,
// and PID controller is initialized with values from the
// calibration constants in Section 3.
// =====================================================

// --- Motors ---
// Four motors, individually controlled. Left side shares speed
// commands, right side shares speed commands (skid-steer).
// The trim offsets come from the TRIM_* calibration constants.
Motor motorLF = { PIN_ENA1, PIN_IN1, PIN_IN2, TRIM_LEFT_FRONT };   // Left Front
Motor motorLR = { PIN_ENB1, PIN_IN3, PIN_IN4, TRIM_LEFT_REAR };    // Left Rear
Motor motorRF = { PIN_ENA2, PIN_IN5, PIN_IN6, TRIM_RIGHT_FRONT };  // Right Front
Motor motorRR = { PIN_ENB2, PIN_IN7, PIN_IN8, TRIM_RIGHT_REAR };   // Right Rear
Motor* allMotors[] = { &motorLF, &motorLR, &motorRF, &motorRR };   // Array for batch ops

// --- Ultrasonic Sensors ---
// NewPing handles the trigger/echo pulse timing internally.
// ping_cm() returns distance in cm, or 0 if no echo received.
NewPing sonarFront(PIN_TRIG_FRONT, PIN_ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(PIN_TRIG_LEFT, PIN_ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT, MAX_DISTANCE);

// --- PID Controllers ---
// Separate instances for line and wall modes. This prevents the
// integral and derivative state from one mode leaking into the other.
// Format: { Kp, Ki, Kd, outMin, outMax, integralMax,
//           integral, prevError, lastTime, initialized }
PIDController pidLine = {
  KP_LINE, KI_LINE, KD_LINE,
  -255.0, 255.0, INTEGRAL_MAX_LINE,
  0, 0, 0, false
};
PIDController pidWall = {
  KP_WALL, KI_WALL, KD_WALL,
  -255.0, 255.0, INTEGRAL_MAX_WALL,
  0, 0, 0, false
};

// --- State Machine ---
RobotState currentState = STATE_LINE_FOLLOWING;  // Start in line mode
Timer stateTimer = { 0, 0, false };              // Shared timer for timed states
                                                  // INVARIANT: only one timed state
                                                  // (BACKING_UP, TURNING, MODE_SWITCH)
                                                  // is active at a time by FSM design

// --- Turn Management ---
// When a turn or backup is initiated, these track what turn to perform
// and which state to return to after the maneuver completes.
TurnDir pendingTurn;        // Direction of the next/current turn
RobotState returnState;     // State to resume after turn completes
unsigned long pendingTurnDuration = TURN_90_DURATION;  // Duration for the next turn

// --- Junction Strategy ---
// Controls turn direction at T-junctions and four-way intersections.
// Change junctionStrategy to experiment with different solving algorithms.
// See JunctionStrategy enum in Section 5 for options.
JunctionStrategy junctionStrategy = STRATEGY_LEFT_WALL;
uint8_t junctionCount = 0;  // Junction counter for STRATEGY_ALTERNATING

// --- IR Calibration State ---
// Populated by calibrateIRSensors() at startup. If auto-calibration fails,
// all arrays are loaded with IR_DEFAULT_* values and irCalibrated stays false.
int  irLineThresh[NUM_IR_SENSORS];      // Per-sensor line detection threshold
int  irNoLineThresh[NUM_IR_SENSORS];    // Per-sensor "no line" threshold
bool irHighOnLine = IR_DEFAULT_HIGH_ON_LINE;  // Polarity (runtime)
bool irCalibrated = false;              // Whether auto-cal succeeded

// --- Sensor Data ---
// Updated every loop iteration by readIRSensors() and readUltrasonicSensors().
int irRaw[NUM_IR_SENSORS];               // Raw analog readings (0-1023) from IR sensors
bool irOnLine[NUM_IR_SENSORS];           // Processed: true if sensor i detects the line
float dist[NUM_US_SENSORS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE }; // Ultrasonic distances indexed by USIndex (cm)
float distFiltered[NUM_US_SENSORS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE }; // EMA-smoothed distances
float lastPidOutput = 0;            // Last PID output (line or wall), for stuck detection

// --- Wall Following State ---
bool followRightWall = true; // Which wall the PID is tracking (true=right, false=left)
int wallSwitchCounter = 0;   // Debounce counter for wall switching
int wallTransitionCounter = 0; // Debounce counter for line->wall transition

// =====================================================
// SECTION 7: DRIVE HELPERS
//
// High-level drive functions that coordinate all four motors.
// The robot uses skid-steer (tank drive): left and right sides
// are controlled independently. Turning is achieved by running
// the sides at different speeds or opposite directions.
// =====================================================

// driveTank: Set left and right side speeds independently.
//   leftSpeed/rightSpeed: -255 (full reverse) to +255 (full forward)
//   Examples:
//     driveTank(150, 150)   -> drive straight forward
//     driveTank(-75, 75)    -> spin left in place
//     driveTank(100, 50)    -> gentle right curve
//     driveTank(-100, -100) -> drive straight backward
void driveTank(int leftSpeed, int rightSpeed) {
  motorSet(&motorLF, leftSpeed);
  motorSet(&motorLR, leftSpeed);
  motorSet(&motorRF, rightSpeed);
  motorSet(&motorRR, rightSpeed);
}

// driveStop: Coast all motors (free-spin to a stop).
// Use this for normal stops where gradual deceleration is fine.
void driveStop() {
  for (int i = 0; i < NUM_MOTORS; i++) motorSet(allMotors[i], 0);
}

// driveBrake: Active electromagnetic braking on all motors.
// Use this when you need the robot to stop quickly (e.g., before
// hitting a wall). The L298N shorts the motor terminals, creating
// back-EMF resistance. Much faster stop than driveStop().
void driveBrake() {
  for (int i = 0; i < NUM_MOTORS; i++) motorBrake(allMotors[i]);
}

// getJunctionTurn: Decide which direction to turn at a junction based on
// the active junctionStrategy. Called from followWall() when a T-junction
// or four-way intersection is detected.
TurnDir getJunctionTurn() {
  junctionCount++;
  TurnDir dir;
  switch (junctionStrategy) {
    case STRATEGY_RIGHT_WALL:
      dir = TURN_RIGHT;
      break;
    case STRATEGY_ALTERNATING:
      dir = (junctionCount % 2 == 1) ? TURN_LEFT : TURN_RIGHT;
      break;
    case STRATEGY_RANDOM:
      dir = random(2) ? TURN_LEFT : TURN_RIGHT;
      break;
    case STRATEGY_LEFT_WALL:
    default:
      dir = TURN_LEFT;
      break;
  }
  DEBUG_PRINTF("Junction #%d -> %s\n", (int)junctionCount,
               dir == TURN_LEFT ? "LEFT" : "RIGHT");
  return dir;
}

// startTurn: Begin a non-blocking spin turn.
//
// The robot spins in place by running left and right sides in
// opposite directions at TURN_SPEED. A timer is started for
// the given duration (defaults to TURN_90_DURATION). Callers can
// pass a different duration for non-90° turns (e.g. 180° U-turns).
// The main loop checks timerExpired() each iteration and
// transitions to afterState when the turn completes.
//
// Parameters:
//   dir        - TURN_LEFT or TURN_RIGHT
//   afterState - state to transition to when the turn finishes
//                (usually STATE_WALL_FOLLOWING)
//   duration   - turn time in ms (default: TURN_90_DURATION)
void startTurn(TurnDir dir, RobotState afterState, unsigned long duration = TURN_90_DURATION) {
  pendingTurn = dir;
  returnState = afterState;

  // Spin: one side forward, other side backward
  if (dir == TURN_LEFT) {
    driveTank(-TURN_SPEED, TURN_SPEED);   // Left side back, right side forward
  } else {
    driveTank(TURN_SPEED, -TURN_SPEED);   // Left side forward, right side back
  }
  timerStart(&stateTimer, duration);
  currentState = STATE_TURNING;

  DEBUG_PRINTF("Turn %s\n", dir == TURN_LEFT ? "LEFT" : "RIGHT");
}

// startBackupAndTurn: Reverse briefly, then turn.
//
// When the robot detects a front obstacle too close to turn in place,
// it first backs up for BACKUP_DURATION ms, then initiates a spin turn.
// The state machine handles this as a two-step sequence:
//   STATE_BACKING_UP -> (timer expires) -> startTurn() -> STATE_TURNING
//
// Parameters:
//   dir        - direction to turn after backing up
//   afterState - state to resume after the full maneuver
void startBackupAndTurn(TurnDir dir, RobotState afterState, unsigned long turnDuration = TURN_90_DURATION) {
  pendingTurn = dir;
  returnState = afterState;
  pendingTurnDuration = turnDuration;
  driveTank(-BACKUP_SPEED, -BACKUP_SPEED);  // Reverse at backup speed
  timerStart(&stateTimer, BACKUP_DURATION);
  currentState = STATE_BACKING_UP;
}

// =====================================================
// SECTION 8: SENSOR FUNCTIONS
//
// These functions read raw sensor data and convert it into
// usable values for the navigation algorithms.
//
// Called every loop() iteration, even during turns and pauses,
// because the state machine is non-blocking.
// =====================================================

// readIRSensors: Read all IR sensors and determine which ones see the line.
//
// Populates two arrays:
//   irRaw[0..3]    - raw analog values (0-1023)
//   irOnLine[0..3] - boolean: true if that sensor detects the line
//
// Uses per-sensor thresholds from irLineThresh[] and runtime polarity
// from irHighOnLine, both set by calibrateIRSensors() at startup.
void readIRSensors() {
  static const int irPins[] = { PIN_IR1, PIN_IR2, PIN_IR3, PIN_IR4 };
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    irRaw[i] = analogRead(irPins[i]);
    if (irHighOnLine) {
      irOnLine[i] = (irRaw[i] >= irLineThresh[i]);
    } else {
      irOnLine[i] = (irRaw[i] <= irLineThresh[i]);
    }
  }
}

// readUltrasonicSensors: Read ultrasonic sensors with front-priority scheduling.
//
// Reads TWO sensors per call (~60ms): the front sensor every time, plus
// one side sensor alternating left/right. This halves front-obstacle
// detection latency compared to a simple round-robin.
//
// Populates dist[] and distFiltered[] indexed by USIndex.
//
// NewPing's ping_cm() returns 0 when no echo is received (object
// is beyond MAX_DISTANCE or sensor error). We map 0 -> MAX_DISTANCE
// so the navigation code treats "no echo" as "far away" rather
// than "zero distance" (which would look like a wall touching the sensor).
void readUltrasonicSensors() {
  // Read two sensors per call: FRONT every time (halves obstacle-detection
  // latency from ~90ms to ~60ms), plus one side sensor alternating L/R.
  // TRADEOFF: ~60ms blocking per loop() vs ~30ms with single-sensor round-robin.
  // Revert to single-read if loop timing budget is too tight.
  static uint8_t sideIndex = US_LEFT;  // alternates between US_LEFT and US_RIGHT
  static NewPing* const sonars[] = { &sonarFront, &sonarLeft, &sonarRight };

  // Always read front sensor first
  float raw = sonars[US_FRONT]->ping_cm();
  dist[US_FRONT] = (raw == 0) ? MAX_DISTANCE : raw;
  distFiltered[US_FRONT] = US_ALPHA * dist[US_FRONT]
                          + (1.0 - US_ALPHA) * distFiltered[US_FRONT];

  // Then read alternating side sensor
  raw = sonars[sideIndex]->ping_cm();
  dist[sideIndex] = (raw == 0) ? MAX_DISTANCE : raw;
  distFiltered[sideIndex] = US_ALPHA * dist[sideIndex]
                           + (1.0 - US_ALPHA) * distFiltered[sideIndex];

  sideIndex = (sideIndex == US_LEFT) ? US_RIGHT : US_LEFT;
}

// lineCount: Return how many of the 4 IR sensors currently detect the line.
// Useful for detecting intersections (3-4 sensors) or line loss (0 sensors).
int lineCount() {
  int count = 0;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (irOnLine[i]) count++;
  }
  return count;
}

// linePosition: Calculate the weighted centroid position of the line.
//
// Returns a float from 0.0 (line is hard left) to 3.0 (line is hard right).
// The center position is 1.5 (line is between sensors 1 and 2).
// Returns -1.0 if NO sensors detect the line (line lost).
//
// The weighting uses the raw analog intensity: a sensor that is squarely
// over the line contributes more weight than one at the edge. This gives
// sub-sensor-spacing resolution for smooth PID tracking.
//
// Example with irHighOnLine=true:
//   irRaw = [50, 800, 900, 50]  -> sensors 1,2 see the line
//   position = (0*0 + 800*1 + 900*2 + 0*3) / (0+800+900+0) = 2600/1700 = 1.53
//   Error = 1.53 - 1.5 = +0.03 (very slightly right of center)
float linePosition() {
  float weightedSum = 0;
  float totalWeight = 0;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    // Weight = raw intensity for on-line sensors, 0 for off-line sensors.
    // If irHighOnLine=false, we invert the raw value so higher = more line.
    float weight = irOnLine[i] ? (float)(irHighOnLine ? irRaw[i] : 1023 - irRaw[i]) : 0.0;
    weightedSum += weight * i;   // Multiply by sensor index (0,1,2,3)
    totalWeight += weight;
  }
  if (totalWeight < 1.0) return -1.0;  // No line detected
  return weightedSum / totalWeight;
}

// calibrateIRSensors: Auto-calibrate IR thresholds at startup.
//
// Two-phase blocking routine (runs once in setup()):
//   Phase 1 (0 to IR_CAL_SETTLE_MS): Robot sits ON the line.
//     Samples all sensors at IR_CAL_SAMPLE_INTERVAL, tracks min/max,
//     accumulates initial averages for polarity detection.
//     LINE LED blinks fast to indicate "hold still on line."
//   Phase 2 (IR_CAL_SETTLE_MS to IR_CAL_DURATION_MS): User sweeps
//     robot OFF the line. Continues tracking min/max.
//     LINE LED solid ON to indicate "sweep off line now."
//
// After sampling, validates that each sensor's range >= IR_CAL_MIN_RANGE.
// If any sensor fails, falls back to IR_DEFAULT_* for ALL sensors and
// blinks the ERROR LED 3 times as a visual warning.
//
// Polarity detection: majority vote — if the initial average (on-line)
// is above the midpoint for 3+ sensors, irHighOnLine = true.
//
// Threshold computation:
//   irLineThresh[i]  = min + 50% of range (midpoint)
//   irNoLineThresh[i] = min + 25% of range (high-on-line)
//                     or max - 25% of range (low-on-line)
void calibrateIRSensors() {
  static const int irPins[] = { PIN_IR1, PIN_IR2, PIN_IR3, PIN_IR4 };

  int irMin[NUM_IR_SENSORS];
  int irMax[NUM_IR_SENSORS];
  long irInitialSum[NUM_IR_SENSORS];
  int initialCount = 0;

  // Initialize min/max/sum
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    int val = analogRead(irPins[i]);
    irMin[i] = val;
    irMax[i] = val;
    irInitialSum[i] = 0;
  }

  DEBUG_PRINTLN("=== IR CALIBRATION ===");
  DEBUG_PRINTLN("Phase 1: Hold robot ON the line...");

  unsigned long calStart = millis();
  unsigned long lastSample = calStart;
  unsigned long lastBlink = calStart;
  bool phase1 = true;
  bool ledState = false;

  // Main calibration sampling loop
  while (millis() - calStart < IR_CAL_DURATION_MS) {
    unsigned long now = millis();
    unsigned long elapsed = now - calStart;

    // Phase transition
    if (phase1 && elapsed >= IR_CAL_SETTLE_MS) {
      phase1 = false;
      digitalWrite(PIN_LED_LINE, HIGH);  // Solid ON for phase 2
      DEBUG_PRINTLN("Phase 2: Sweep robot OFF the line...");
    }

    // Phase 1 LED: fast blink (100ms toggle)
    if (phase1 && (now - lastBlink >= 100)) {
      lastBlink = now;
      ledState = !ledState;
      digitalWrite(PIN_LED_LINE, ledState ? HIGH : LOW);
    }

    // Sample at IR_CAL_SAMPLE_INTERVAL
    if (now - lastSample >= IR_CAL_SAMPLE_INTERVAL) {
      lastSample = now;
      for (int i = 0; i < NUM_IR_SENSORS; i++) {
        int val = analogRead(irPins[i]);
        if (val < irMin[i]) irMin[i] = val;
        if (val > irMax[i]) irMax[i] = val;
        if (phase1) irInitialSum[i] += val;
      }
      if (phase1) initialCount++;
    }
  }

  digitalWrite(PIN_LED_LINE, LOW);

  // Validate: every sensor must have sufficient range
  bool valid = true;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    int range = irMax[i] - irMin[i];
    if (range < IR_CAL_MIN_RANGE) {
      valid = false;
      break;
    }
  }

  if (!valid) {
    // Fallback to hardcoded defaults
    DEBUG_PRINTLN("CAL FAILED: insufficient range. Using defaults.");
    irHighOnLine = IR_DEFAULT_HIGH_ON_LINE;
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
      irLineThresh[i]  = IR_DEFAULT_LINE_THRESH;
      irNoLineThresh[i] = IR_DEFAULT_NO_LINE_THRESH;
    }
    irCalibrated = false;

    // Blink ERROR LED 3 times as visual warning
    for (int i = 0; i < 3; i++) {
      digitalWrite(PIN_LED_ERROR, HIGH);
      delay(150);
      digitalWrite(PIN_LED_ERROR, LOW);
      delay(150);
    }
    return;
  }

  // Detect polarity via majority vote:
  // If initial average (on-line) > midpoint for 3+ sensors, high-on-line
  int highVotes = 0;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    int midpoint = (irMin[i] + irMax[i]) / 2;
    int initialAvg = (initialCount > 0) ? (int)(irInitialSum[i] / initialCount) : midpoint;
    if (initialAvg > midpoint) highVotes++;
  }
  irHighOnLine = (highVotes >= 3);

  // Compute per-sensor thresholds
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    int range = irMax[i] - irMin[i];
    irLineThresh[i] = irMin[i] + range / 2;          // 50% = midpoint
    if (irHighOnLine) {
      irNoLineThresh[i] = irMin[i] + range / 4;      // 25% from bottom
    } else {
      irNoLineThresh[i] = irMax[i] - range / 4;      // 25% from top
    }
  }
  irCalibrated = true;

  // Debug output (integers only — no %f on AVR)
  DEBUG_PRINTLN("CAL OK:");
  DEBUG_PRINTF("  Polarity: %s\n", irHighOnLine ? "HIGH_ON_LINE" : "LOW_ON_LINE");
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    DEBUG_PRINTF("  IR%d: min=%d max=%d thresh=%d noLine=%d\n",
                 i, irMin[i], irMax[i], irLineThresh[i], irNoLineThresh[i]);
  }

  // Indicate success: LINE LED solid ON briefly
  digitalWrite(PIN_LED_LINE, HIGH);
}

// =====================================================
// SECTION 9: LINE FOLLOWING
//
// Uses PID control to keep the robot centered on the black tape.
//
// The line position (0.0 to 3.0) from linePosition() is compared
// to the center setpoint (1.5). The PID output is used to create
// a speed differential between left and right motors:
//   - Positive error (line is to the right) -> reduce left speed,
//     increase right speed -> robot steers right toward the line
//   - Negative error (line is to the left) -> opposite
//
// LINE LOSS HANDLING:
//   When no sensor detects the line (linePosition returns -1.0),
//   the robot slows to a creep rather than making a sharp correction.
//   This prevents wild swerving when the tape temporarily disappears
//   (e.g., a gap or dirt on the tape). The PID is NOT updated during
//   line loss, preserving the last good error state for when the
//   line reappears.
// =====================================================

void followLine() {
  float pos = linePosition();

  if (pos < 0) {
    // LINE LOST: no sensor sees the tape.
    // Strategy: creep forward slowly. Don't update PID (preserve last
    // good state). The robot will either find the tape again or
    // the wall transition detector will trigger if we've entered
    // the walled section.
    DEBUG_PRINTLN("Line lost!");
    lastPidOutput = 0;  // No PID active; avoid false saturation detection
    driveTank(SLOW_SPEED / 2, SLOW_SPEED / 2);
    return;
  }

  // Calculate error from center position (1.5 = perfectly centered)
  // Positive error = line is to the right of center
  // Negative error = line is to the left of center
  float error = pos - 1.5;
  float output = pidCompute(&pidLine, error);
  lastPidOutput = output;

  // Apply PID output as a differential: add to left, subtract from right.
  // In differential/skid-steer drive, the robot turns toward the SLOWER side.
  // Positive output (line is right of center) -> we need to steer RIGHT:
  //   error > 0 -> Kp * error > 0 -> output > 0
  //   -> leftSpeed = BASE + output (faster)
  //   -> rightSpeed = BASE - output (slower)
  //   -> slower right side -> robot turns RIGHT toward the line
  int leftSpeed = constrain(BASE_SPEED + (int)output, LINE_MIN_SPEED, 255);
  int rightSpeed = constrain(BASE_SPEED - (int)output, LINE_MIN_SPEED, 255);
  driveTank(leftSpeed, rightSpeed);

  // Throttled debug output (every 200ms to avoid serial flooding)
  #ifdef DEBUG
    static unsigned long lastLinePrint = 0;
    if (millis() - lastLinePrint > 200) {
      Serial.print("LINE pos="); Serial.print(pos, 1);
      Serial.print(" err=");     Serial.print(error, 1);
      Serial.print(" out=");     Serial.print(output, 0);
      Serial.print(" L=");      Serial.print(leftSpeed);
      Serial.print(" R=");      Serial.println(rightSpeed);
      lastLinePrint = millis();
    }
  #endif
}

// =====================================================
// SECTION 10: WALL FOLLOWING
//
// Uses PID control to maintain a constant distance (WALL_SETPOINT)
// from either the left or right wall. The robot dynamically selects
// which wall to follow based on sensor readings.
//
// ALGORITHM FLOW (each call):
//   1. Check for junctions (T-junction, four-way) -> turn if found
//   2. Check for front obstacle -> back up and turn if found
//   3. Evaluate whether to switch which wall we're following
//   4. Adjust speed for tight corridors
//   5. Run PID on wall distance -> apply speed differential
//
// JUNCTION HANDLING:
//   Turn direction at junctions is determined by junctionStrategy
//   (see getJunctionTurn() in Section 7). Default is left-wall following,
//   which guarantees solving any simply-connected maze (no loops/islands).
//
// WALL SWITCHING:
//   The robot tracks whichever wall is closest, with a debounce counter
//   to prevent oscillation in corridors where both walls are similar
//   distances. The PID is reset on switch to prevent integral carryover.
//
// PID SIGN CONVENTION (right wall following):
//   error = measured_distance - WALL_SETPOINT
//   Positive error -> robot is too far from right wall -> steer right
//   -> decrease left speed, increase right speed
// =====================================================

void followWall() {

  // --- Step 1: Junction detection ---
  // A junction is detected by checking which directions are "open"
  // (no wall within WALL_FAR_THRESH cm).
  bool openFront = (distFiltered[US_FRONT] > WALL_FAR_THRESH);
  bool wallLeft  = (distFiltered[US_LEFT]  < WALL_CLOSE_THRESH);
  bool wallRight = (distFiltered[US_RIGHT] < WALL_CLOSE_THRESH);

  // T-junction: open ahead, but walls on both sides form the T
  bool tJunction = (openFront && wallLeft && wallRight);
  // Four-way intersection: open in all directions
  bool fourWay   = (openFront && !wallLeft && !wallRight);

  if (tJunction || fourWay) {
    TurnDir junctionDir = getJunctionTurn();
    startTurn(junctionDir, STATE_WALL_FOLLOWING);
    return;
  }

  // --- Dead-end detection ---
  // All three sides blocked = dead end. Perform a 180-degree U-turn
  // instead of the two-cycle 90+90 recovery.
  bool deadEnd = (distFiltered[US_FRONT] < FRONT_OBSTACLE_DIST) &&
                 (distFiltered[US_LEFT]  < WALL_CLOSE_THRESH) &&
                 (distFiltered[US_RIGHT] < WALL_CLOSE_THRESH);

  if (deadEnd) {
    DEBUG_PRINTLN("Dead end detected");
    TurnDir dir = followRightWall ? TURN_LEFT : TURN_RIGHT;
    startBackupAndTurn(dir, STATE_WALL_FOLLOWING, TURN_180_DURATION);
    return;
  }

  // --- Step 2: Front obstacle avoidance ---
  // If a wall is directly ahead and too close, back up first
  // (to create clearance) then turn away from the followed wall.
  if (distFiltered[US_FRONT] < FRONT_OBSTACLE_DIST) {
    // Turn away from the wall we're following:
    //   Following right wall -> turn left (away from right)
    //   Following left wall  -> turn right (away from left)
    TurnDir dir = followRightWall ? TURN_LEFT : TURN_RIGHT;
    startBackupAndTurn(dir, STATE_WALL_FOLLOWING);
    return;
  }

  // --- Step 3: Dynamic wall selection with debounce ---
  // The robot should follow whichever wall is closest. But we don't
  // want to switch every cycle due to sensor noise, so we require
  // DEBOUNCE_COUNT consecutive readings before committing to a switch.
  //
  // Switch conditions:
  //   To right wall: right wall is close AND left wall is far
  //   To left wall:  left wall is close AND right wall is far
  bool shouldSwitchToRight = (distFiltered[US_RIGHT] < WALL_CLOSE_THRESH && distFiltered[US_LEFT]  > WALL_FAR_THRESH);
  bool shouldSwitchToLeft  = (distFiltered[US_LEFT]  < WALL_CLOSE_THRESH && distFiltered[US_RIGHT] > WALL_FAR_THRESH);

  if (shouldSwitchToRight && !followRightWall) {
    wallSwitchCounter++;
    if (wallSwitchCounter >= DEBOUNCE_COUNT) {
      followRightWall = true;
      wallSwitchCounter = 0;
      pidReset(&pidWall);  // Reset PID to prevent integral carryover
      digitalWrite(PIN_LED_RIGHT, HIGH);
      digitalWrite(PIN_LED_LEFT, LOW);
      DEBUG_PRINTLN("Switched to RIGHT wall");
    }
  } else if (shouldSwitchToLeft && followRightWall) {
    wallSwitchCounter++;
    if (wallSwitchCounter >= DEBOUNCE_COUNT) {
      followRightWall = false;
      wallSwitchCounter = 0;
      pidReset(&pidWall);
      digitalWrite(PIN_LED_RIGHT, LOW);
      digitalWrite(PIN_LED_LEFT, HIGH);
      DEBUG_PRINTLN("Switched to LEFT wall");
    }
  } else {
    wallSwitchCounter = 0;  // Reset counter if condition isn't sustained
  }

  // --- Step 4: Speed adjustment for tight corridors ---
  // When walls are close on BOTH sides, reduce speed for safety.
  // The robot is in a narrow passage and needs finer control.
  int speed = (distFiltered[US_LEFT] < WALL_CLOSE_THRESH && distFiltered[US_RIGHT] < WALL_CLOSE_THRESH)
              ? SLOW_SPEED : BASE_SPEED;

  // --- Step 5: PID wall distance control ---
  // Measure distance to the wall we're following
  float wallDist = followRightWall ? distFiltered[US_RIGHT] : distFiltered[US_LEFT];
  float error = wallDist - WALL_SETPOINT;
  float output = pidCompute(&pidWall, error);
  lastPidOutput = output;

  // Apply correction. In skid-steer, the robot turns toward the SLOWER side.
  // The sign is mirrored depending on which wall we're following so that
  // positive PID output always steers toward the followed wall.
  int leftSpeed, rightSpeed;
  if (followRightWall) {
    // Right wall: positive error (too far from wall) -> steer RIGHT toward wall
    // Faster left + slower right -> robot turns right
    leftSpeed  = constrain(speed + (int)output, 0, 255);
    rightSpeed = constrain(speed - (int)output, 0, 255);
  } else {
    // Left wall: positive error (too far from wall) -> steer LEFT toward wall
    // Slower left + faster right -> robot turns left
    leftSpeed  = constrain(speed - (int)output, 0, 255);
    rightSpeed = constrain(speed + (int)output, 0, 255);
  }
  driveTank(leftSpeed, rightSpeed);

  // Throttled debug output (every 200ms)
  #ifdef DEBUG
    static unsigned long lastWallPrint = 0;
    if (millis() - lastWallPrint > 200) {
      Serial.print("WALL ");    Serial.print(followRightWall ? "R" : "L");
      Serial.print(" d=");      Serial.print(wallDist, 0);
      Serial.print(" err=");    Serial.print(error, 1);
      Serial.print(" out=");    Serial.print(output, 0);
      Serial.print(" L=");      Serial.print(leftSpeed);
      Serial.print(" R=");      Serial.println(rightSpeed);
      lastWallPrint = millis();
    }
  #endif
}

// =====================================================
// SECTION 11: STATE TRANSITIONS
//
// Functions that detect when the robot should switch between
// navigation modes (line -> wall) and when the maze is solved.
//
// OVERLAP ZONE HANDLING:
//   The physical maze has a transition zone where both black tape
//   and walls are present. The robot keeps line-following until
//   ALL IR sensors report low confidence (no tape detected) AND
//   at least one ultrasonic sensor detects a wall nearby. This
//   dual condition prevents false transitions from:
//   - Temporary line loss (no walls = still in line maze)
//   - Walls appearing alongside the tape (still have tape = stay in line mode)
// =====================================================

// isWallMazeTransition: Check if we should switch from line to wall mode.
//
// Returns true when BOTH conditions are met:
//   1. Low line confidence: ALL IR sensors read below their per-sensor
//      irNoLineThresh[] (meaning the tape has ended or the robot has left it)
//   2. Walls present: at least one side ultrasonic sensor detects a wall
//      within WALL_FAR_THRESH cm
//
// This is checked every loop iteration while in STATE_LINE_FOLLOWING.
bool isWallMazeTransition() {
  // Check if ALL sensors have low confidence (tape gone)
  bool lowLineConfidence = true;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (irHighOnLine) {
      // High = on line: any sensor above threshold means tape is still visible
      if (irRaw[i] >= irNoLineThresh[i]) { lowLineConfidence = false; break; }
    } else {
      // Low = on line: any sensor below threshold means tape is still visible
      if (irRaw[i] <= irNoLineThresh[i]) { lowLineConfidence = false; break; }
    }
  }

  // Check if walls are detected on at least one side
  bool wallsPresent = (distFiltered[US_LEFT] < WALL_FAR_THRESH || distFiltered[US_RIGHT] < WALL_FAR_THRESH);

  return lowLineConfidence && wallsPresent;
}

// switchToWallMode: Transition from line following to wall following.
//
// This doesn't immediately start wall following. Instead, it enters
// STATE_MODE_SWITCH (a brief non-blocking pause) to allow the robot
// to settle before beginning PID wall tracking. This prevents
// transient sensor readings from causing erratic initial behavior.
void switchToWallMode() {
  DEBUG_PRINTLN("=== SWITCHING TO WALL MODE ===");
  currentState = STATE_MODE_SWITCH;
  driveStop();
  timerStart(&stateTimer, MODE_SWITCH_PAUSE);

  // Update LEDs to reflect new mode
  digitalWrite(PIN_LED_LINE, LOW);
  digitalWrite(PIN_LED_WALL, HIGH);
  digitalWrite(PIN_LED_RIGHT, HIGH);  // Start with right wall by default
  digitalWrite(PIN_LED_LEFT, LOW);

  // Fresh PID state for wall following (no carryover from line mode)
  pidReset(&pidWall);
  followRightWall = true;
  wallSwitchCounter = 0;
}

// isExitFound: Check if the robot has reached the maze exit.
//
// The exit is defined as open space in ALL three directions
// (front, left, right) beyond EXIT_THRESHOLD cm. This means the
// robot has left the walled portion of the maze entirely.
//
// Returns true only when all three sensors agree - a single open
// side (like passing a corridor opening) won't trigger this.
bool isExitFound() {
  return (distFiltered[US_FRONT] > EXIT_THRESHOLD &&
          distFiltered[US_LEFT]  > EXIT_THRESHOLD &&
          distFiltered[US_RIGHT] > EXIT_THRESHOLD);
}

// victoryBlink: Visual celebration when the maze is solved.
// Blinks the wall LED 5 times. INTENTIONAL: Uses blocking delay() here
// because the robot is permanently stopped at this point - no need for
// non-blocking timing.
void victoryBlink() {
  DEBUG_PRINTLN("=== EXIT FOUND ===");
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_WALL, HIGH);
    delay(200);
    digitalWrite(PIN_LED_WALL, LOW);
    delay(200);
  }
}

// =====================================================
// SECTION 12: ERROR / BATTERY / STUCK HANDLING
//
// Safety systems to protect the robot and its environment.
//
// BATTERY MONITORING:
//   The battery voltage is read through a voltage divider on A4.
//   If it drops below VOLTAGE_THRESHOLD (6.5V for an 8V pack),
//   the robot enters ERROR_STATE to prevent erratic behavior
//   from low-voltage motor driving or brownout resets.
//
//   IMPORTANT: The VOLTAGE_SCALE constant MUST be calibrated for
//   your specific voltage divider resistors. See Section 3.
//
// ERROR STATE:
//   Currently a terminal state (requires hardware reset). The error
//   LED turns on and the error message is printed via serial.
//   Future improvement: add a timed recovery check (e.g., re-read
//   battery after 5 seconds and resume if voltage has recovered).
//
// STUCK DETECTION:
//   Monitors ultrasonic sensor readings during active navigation.
//   If no sensor changes by > STUCK_DIST_THRESHOLD for STUCK_TIMEOUT_MS,
//   the robot is assumed stuck. Recovery uses startBackupAndTurn()
//   with alternating turn directions. After STUCK_MAX_RETRIES
//   within STUCK_COOLDOWN_MS, escalates to ERROR_STATE.
// =====================================================

// enterErrorState: Safely stop the robot and indicate an error.
// All motors are stopped, the error LED is lit, and all other
// mode LEDs are turned off. The error message is printed to serial.
void enterErrorState(const char* message) {
  currentState = STATE_ERROR;
  driveStop();
  digitalWrite(PIN_LED_ERROR, HIGH);
  digitalWrite(PIN_LED_LINE, LOW);
  digitalWrite(PIN_LED_WALL, LOW);
  digitalWrite(PIN_LED_RIGHT, LOW);
  digitalWrite(PIN_LED_LEFT, LOW);
  DEBUG_PRINT("ERROR: ");
  DEBUG_PRINTLN(message);
}

// checkBattery: Read battery voltage and enter error state if too low.
//
// Called every loop() iteration. Returns false (and enters error state)
// if voltage is below threshold. The main loop uses this as an early
// exit: if (!checkBattery()) return;
//
// VOLTAGE DIVIDER MATH:
//   The Arduino ADC reads 0-5V mapped to 0-1023.
//   A voltage divider with R1 (high side) and R2 (low side) scales:
//     V_pin = V_battery * R2 / (R1 + R2)
//   To recover V_battery from the ADC reading:
//     V_battery = raw * (5.0 / 1023.0) * (R1 + R2) / R2
//   This combined factor is VOLTAGE_SCALE (see Section 3).
bool checkBattery() {
  static unsigned long accumulator = 0;
  static uint16_t sampleCount = 0;
  static unsigned long lastEvalTime = 0;

  accumulator += analogRead(PIN_BATTERY);
  sampleCount++;

  // Evaluate every 500ms
  unsigned long now = millis();
  if (now - lastEvalTime < 500) return true;
  lastEvalTime = now;

  float avgRaw = (float)accumulator / sampleCount;
  accumulator = 0;
  sampleCount = 0;

  float voltage = avgRaw * VOLTAGE_SCALE;
  if (voltage < VOLTAGE_THRESHOLD) {
    enterErrorState("Battery voltage below threshold");
    return false;
  }
  return true;
}

// checkStuck: Detect if the robot is stuck and attempt recovery.
//
// Called every loop() iteration after sensor reads. Returns false
// if stuck recovery has escalated to error state. Returns true
// otherwise (including during active recovery maneuvers).
//
// Three independent stuck signals:
//   1. Ultrasonic stagnation — no distFiltered[] sensor changes by
//      > STUCK_DIST_THRESHOLD for STUCK_TIMEOUT_MS during
//      LINE_FOLLOWING or WALL_FOLLOWING.
//   2. PID-output saturation — lastPidOutput stays within
//      STUCK_PID_THRESHOLD of its ±255 limit for STUCK_TIMEOUT_MS.
//      Reset when ultrasonic movement is detected.
//   3. Turn timeout — STATE_TURNING exceeds TURN_TIMEOUT_MS,
//      indicating the robot cannot physically complete the rotation.
//
// Any signal triggers recovery via startBackupAndTurn() with
// alternating direction. After STUCK_MAX_RETRIES within
// STUCK_COOLDOWN_MS, enters error state.
bool checkStuck() {
  static float snapDist[NUM_US_SENSORS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };
  static unsigned long snapTime = 0;
  static unsigned long pidSatTime = 0;
  static unsigned long lastRecoveryTime = 0;
  static uint8_t stuckRetryCount = 0;
  static bool wasChecking = false;

  unsigned long now = millis();
  bool stuckDetected = false;
  RobotState resumeState = currentState;

  // --- Signal 3: Turn timeout ---
  // Active during STATE_TURNING. If the turn has been running longer
  // than TURN_TIMEOUT_MS, the robot is physically unable to rotate.
  if (currentState == STATE_TURNING) {
    if (stateTimer.running &&
        (now - stateTimer.startTime > TURN_TIMEOUT_MS)) {
      stuckDetected = true;
      resumeState = returnState;  // Resume the state the turn was returning to
    } else {
      wasChecking = false;
      return true;
    }
  }

  // --- Signals 1 & 2: Ultrasonic stagnation + PID saturation ---
  // Active during LINE_FOLLOWING and WALL_FOLLOWING only.
  if (!stuckDetected) {
    bool shouldCheck = (currentState == STATE_LINE_FOLLOWING ||
                        currentState == STATE_WALL_FOLLOWING);

    if (!shouldCheck) {
      wasChecking = false;
      return true;
    }

    // Just entered a monitored state from a non-monitored state —
    // capture fresh snapshot so prior maneuver time doesn't accumulate.
    if (!wasChecking) {
      for (uint8_t i = 0; i < NUM_US_SENSORS; i++) snapDist[i] = distFiltered[i];
      snapTime = now;
      pidSatTime = 0;
      wasChecking = true;
      return true;
    }

    // Check if any ultrasonic sensor has changed meaningfully.
    // Skip sensors where both snapshot and current reading are beyond
    // WALL_FAR_THRESH — HC-SR04 noise at long range (3-4 cm) exceeds
    // the threshold and would cause false snapshot refreshes.
    bool hasMoved = false;
    for (uint8_t i = 0; i < NUM_US_SENSORS; i++) {
      if (distFiltered[i] > WALL_FAR_THRESH && snapDist[i] > WALL_FAR_THRESH)
        continue;
      float diff = distFiltered[i] - snapDist[i];
      if (diff > STUCK_DIST_THRESHOLD || diff < -STUCK_DIST_THRESHOLD) {
        hasMoved = true;
        break;
      }
    }

    if (hasMoved) {
      for (uint8_t i = 0; i < NUM_US_SENSORS; i++) snapDist[i] = distFiltered[i];
      snapTime = now;
      pidSatTime = 0;  // Movement detected — reset PID saturation timer
      return true;
    }

    // Signal 2: PID-output saturation
    float pidOut = lastPidOutput;
    if (pidOut > (255.0 - STUCK_PID_THRESHOLD) ||
        pidOut < (-255.0 + STUCK_PID_THRESHOLD)) {
      if (pidSatTime == 0) pidSatTime = now;
      if (now - pidSatTime >= STUCK_TIMEOUT_MS) stuckDetected = true;
    } else {
      pidSatTime = 0;
    }

    // Signal 1: Ultrasonic stagnation
    if (!stuckDetected && (now - snapTime >= STUCK_TIMEOUT_MS)) {
      stuckDetected = true;
    }

    if (!stuckDetected) return true;
  }

  // === STUCK DETECTED ===
  if (lastRecoveryTime > 0 && (now - lastRecoveryTime < STUCK_COOLDOWN_MS)) {
    stuckRetryCount++;
  } else {
    stuckRetryCount = 1;
  }

  DEBUG_PRINTF("STUCK #%d  F=%d L=%d R=%d\n",
               stuckRetryCount,
               (int)distFiltered[US_FRONT], (int)distFiltered[US_LEFT],
               (int)distFiltered[US_RIGHT]);

  if (stuckRetryCount > STUCK_MAX_RETRIES) {
    enterErrorState("Stuck: max retries exceeded");
    return false;
  }

  // Recovery: backup-and-turn in alternating direction
  TurnDir escapeDir = (stuckRetryCount % 2 == 1) ? TURN_LEFT : TURN_RIGHT;
  startBackupAndTurn(escapeDir, resumeState);

  lastRecoveryTime = now;
  wasChecking = false;
  return true;
}

// =====================================================
// SECTION 13: SETUP & MAIN LOOP
//
// setup() runs once on power-up or reset. It configures all
// hardware pins and initializes the robot in LINE_FOLLOWING mode.
//
// loop() runs continuously and implements the state machine.
// CRITICAL DESIGN: loop() must always return quickly (< 50ms).
// No function called from loop() should ever block. This ensures:
//   - Sensors are read every iteration (~10-50ms cycle time)
//   - Battery is checked every iteration
//   - The robot can react to emergencies during any maneuver
//
// EXECUTION FLOW PER LOOP ITERATION:
//   1. Check battery -> enter error state if low
//   2. Read all sensors (IR + ultrasonic)
//   3. Check stuck -> attempt recovery or enter error state
//   4. Execute current state's logic
//   5. State logic may change currentState for next iteration
// =====================================================

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);  // 115200 baud: fast enough to minimize
                           // serial blocking time on the Mega
    DEBUG_PRINTLN("MazeRobot starting...");
  #endif

  // Seed random number generator from floating analog pin (noise source).
  // Needed by STRATEGY_RANDOM but cheap and harmless to always include.
  randomSeed(analogRead(PIN_RANDOM_SEED));

  // Initialize all motor pins (enable + 2 direction each)
  for (int i = 0; i < NUM_MOTORS; i++) motorInit(allMotors[i]);

  // Initialize all 6 LED pins and turn them all off
  const int ledPins[] = { PIN_LED_POWER, PIN_LED_ERROR, PIN_LED_LINE,
                          PIN_LED_WALL, PIN_LED_RIGHT, PIN_LED_LEFT };
  for (int pin : ledPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Turn on power LED (indicates robot is running) and
  // line mode LED (initial navigation mode)
  digitalWrite(PIN_LED_POWER, HIGH);
  digitalWrite(PIN_LED_LINE, HIGH);

  // Ensure all motors start stopped
  driveStop();

  // Auto-calibrate IR sensors (4-second window replaces the old countdown).
  // Phase 1: hold robot on line. Phase 2: sweep off line.
  // Falls back to hardcoded defaults if calibration fails.
  calibrateIRSensors();

  // Ensure LEDs reflect initial state after calibration
  digitalWrite(PIN_LED_POWER, HIGH);
  digitalWrite(PIN_LED_LINE, HIGH);

  // Log active junction strategy
  #ifdef DEBUG
  {
    const char* stratName;
    switch (junctionStrategy) {
      case STRATEGY_LEFT_WALL:   stratName = "LEFT_WALL";   break;
      case STRATEGY_RIGHT_WALL:  stratName = "RIGHT_WALL";  break;
      case STRATEGY_ALTERNATING: stratName = "ALTERNATING";  break;
      case STRATEGY_RANDOM:      stratName = "RANDOM";       break;
      default:                   stratName = "UNKNOWN";      break;
    }
    DEBUG_PRINTF("Junction strategy: %s\n", stratName);
  }
  #endif

  DEBUG_PRINTLN("Setup complete. Starting line following.");
}

void loop() {
  // --- Skip all work in terminal states ---
  if (currentState == STATE_ERROR || currentState == STATE_EXIT_FOUND) return;

  // --- Safety: check battery voltage every iteration ---
  // If battery is low, checkBattery() enters ERROR_STATE and
  // returns false. We skip all navigation logic.
  if (!checkBattery()) return;

  // --- Read ALL sensors every iteration ---
  // This happens even during turns and pauses because the state
  // machine is non-blocking. Continuous sensor updates mean we
  // can detect emergencies (e.g., unexpected obstacle) mid-maneuver.
  readIRSensors();
  readUltrasonicSensors();

  // --- Safety: detect stuck condition ---
  // If max retries exceeded, checkStuck() enters ERROR_STATE and
  // returns false. During active recovery, it returns true.
  if (!checkStuck()) return;

  // --- State machine dispatch ---
  // Each case handles one state's behavior. States transition by
  // setting currentState to a new value, which takes effect on
  // the next loop() iteration.
  switch (currentState) {

    case STATE_LINE_FOLLOWING:
      // Run PID line tracking. Also check if we've entered the
      // wall maze transition zone (tape gone + walls detected).
      followLine();
      if (isWallMazeTransition()) {
        wallTransitionCounter++;
        if (wallTransitionCounter >= WALL_TRANSITION_DEBOUNCE) {
          wallTransitionCounter = 0;
          switchToWallMode();  // -> STATE_MODE_SWITCH
        }
      } else {
        wallTransitionCounter = 0;
      }
      break;

    case STATE_MODE_SWITCH:
      // Brief pause while transitioning from line to wall mode.
      // Motors are stopped. Wait for timer, then start wall following.
      if (timerExpired(&stateTimer)) {
        currentState = STATE_WALL_FOLLOWING;
        DEBUG_PRINTLN("Wall following active.");
      }
      break;

    case STATE_WALL_FOLLOWING:
      // Run PID wall tracking. Check for maze exit first.
      if (isExitFound()) {
        currentState = STATE_EXIT_FOUND;
        driveStop();
        victoryBlink();
      } else {
        followWall();  // May transition to BACKING_UP or TURNING
      }
      break;

    case STATE_BACKING_UP:
      // Reversing before a turn. Motors are running backward.
      // When timer expires, initiate the actual spin turn.
      if (timerExpired(&stateTimer)) {
        startTurn(pendingTurn, returnState, pendingTurnDuration);  // -> STATE_TURNING
      }
      break;

    case STATE_TURNING:
      // Spinning in place. Motors running in opposite directions.
      // When timer expires, stop motors and return to previous state.
      if (timerExpired(&stateTimer)) {
        driveStop();
        currentState = returnState;  // Usually STATE_WALL_FOLLOWING
      }
      break;

    case STATE_ERROR:
      // Motors stopped, error LED on. Requires hardware reset.
      // Future: could add timed battery re-check for recovery.
      break;

    case STATE_EXIT_FOUND:
      // Maze solved. Robot is permanently stopped.
      break;
  }
}
