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
//                                          │
//                                    ┌─────┼─────┐
//                                    v     v     v
//                              BACKING_UP  │  EXIT_FOUND
//                                    │     │
//                                    v     │
//                                TURNING ──┘
//
//    Any state can transition to ERROR_STATE on low battery.
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

void pidReset(PIDController* pid);
float pidCompute(PIDController* pid, float error);
void motorInit(const Motor* m);
void motorSet(const Motor* m, int speed);
void motorBrake(const Motor* m);
void timerStart(Timer* t, unsigned long durationMs);
bool timerExpired(Timer* t);
bool timerRunning(Timer* t);
void startTurn(TurnDir dir, RobotState afterState, unsigned long duration);
void startBackupAndTurn(TurnDir dir, RobotState afterState, unsigned long turnDuration);

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
  // Limited to 80 chars per message. Uses the GCC ## extension to
  // handle zero variadic arguments gracefully.
  #define DEBUG_PRINTF(fmt, ...) do { \
    char _dbuf[80]; \
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

// =====================================================
// SECTION 3: CALIBRATION CONSTANTS
// >>> TUNE THESE ON THE ACTUAL ROBOT <<<
// =====================================================

// -- Motor Calibration --
#define BASE_SPEED          150   // Default forward PWM (0-255)
#define SLOW_SPEED          100   // Tight-space PWM
#define TURN_SPEED           75   // PWM during spin turns

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
#define TURN_180_DURATION   600   // ms to spin for ~180 degrees (2x TURN_90_DURATION)
#define MODE_SWITCH_PAUSE   500   // ms pause when switching line->wall

// -- IR Sensor Calibration --
// IR_HIGH_ON_LINE: set to 1 if higher analog value = sensor over black tape.
// Set to 0 if higher value = sensor over light surface (reflective).
// If unsure, read raw values via Serial and check.
#define IR_HIGH_ON_LINE       1
#define IR_LINE_THRESHOLD   500   // Threshold for "sensor sees line"
#define IR_NO_LINE_THRESH   200   // Below this on ALL sensors = no line detected

// -- Wall/Ultrasonic Calibration --
#define MAX_DISTANCE        200   // Max ultrasonic range (cm)
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
#define VOLTAGE_SCALE       ((DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2 * (5.0 / 1023.0))
#define VOLTAGE_THRESHOLD   6.5

// -- Ultrasonic Filtering --
#define US_ALPHA            0.3   // EMA smoothing factor (0-1, higher = less smoothing)

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

// --- Turn Management ---
// When a turn or backup is initiated, these track what turn to perform
// and which state to return to after the maneuver completes.
TurnDir pendingTurn;        // Direction of the next/current turn
RobotState returnState;     // State to resume after turn completes
unsigned long pendingTurnDuration = TURN_90_DURATION;  // Duration for the next turn

// --- Sensor Data ---
// Updated every loop iteration by readIRSensors() and readUltrasonicSensors().
int irRaw[4];               // Raw analog readings (0-1023) from IR sensors
bool irOnLine[4];           // Processed: true if sensor i detects the line
float dist[3] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE }; // Ultrasonic distances: [0]=front, [1]=left, [2]=right (cm)
float distFiltered[3] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE }; // EMA-smoothed distances

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
  for (int i = 0; i < 4; i++) motorSet(allMotors[i], 0);
}

// driveBrake: Active electromagnetic braking on all motors.
// Use this when you need the robot to stop quickly (e.g., before
// hitting a wall). The L298N shorts the motor terminals, creating
// back-EMF resistance. Much faster stop than driveStop().
void driveBrake() {
  for (int i = 0; i < 4; i++) motorBrake(allMotors[i]);
}

// startTurn: Begin a non-blocking spin turn.
//
// The robot spins in place by running left and right sides in
// opposite directions at TURN_SPEED. A timer is started for
// TURN_90_DURATION ms. The main loop checks timerExpired() each
// iteration and transitions to afterState when the turn completes.
//
// Parameters:
//   dir        - TURN_LEFT or TURN_RIGHT
//   afterState - state to transition to when the turn finishes
//                (usually STATE_WALL_FOLLOWING)
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
  driveTank(-BASE_SPEED / 2, -BASE_SPEED / 2);  // Reverse at half speed
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

// readIRSensors: Read all 4 IR sensors and determine which ones see the line.
//
// Populates two arrays:
//   irRaw[0..3]    - raw analog values (0-1023)
//   irOnLine[0..3] - boolean: true if that sensor detects the line
//
// The IR_HIGH_ON_LINE constant controls polarity interpretation:
//   If 1: higher raw value = sensor is over the black tape
//   If 0: lower raw value = sensor is over the black tape
//
// HOW TO CALIBRATE:
//   1. Enable DEBUG, open Serial Monitor at 115200 baud
//   2. Hold each sensor over the black tape, note the raw value
//   3. Hold each sensor over the light surface, note the raw value
//   4. Set IR_LINE_THRESHOLD halfway between those two values
//   5. Set IR_HIGH_ON_LINE based on which reading was higher
void readIRSensors() {
  static const int irPins[] = { PIN_IR1, PIN_IR2, PIN_IR3, PIN_IR4 };
  for (int i = 0; i < 4; i++) {
    irRaw[i] = analogRead(irPins[i]);
    if (IR_HIGH_ON_LINE) {
      irOnLine[i] = (irRaw[i] >= IR_LINE_THRESHOLD);
    } else {
      irOnLine[i] = (irRaw[i] <= IR_LINE_THRESHOLD);
    }
  }
}

// readUltrasonicSensors: Read all 3 ultrasonic sensors.
//
// Populates dist[0..2] with distances in centimeters:
//   dist[0] = front sensor
//   dist[1] = left sensor
//   dist[2] = right sensor
//
// NewPing's ping_cm() returns 0 when no echo is received (object
// is beyond MAX_DISTANCE or sensor error). We map 0 -> MAX_DISTANCE
// so the navigation code treats "no echo" as "far away" rather
// than "zero distance" (which would look like a wall touching the sensor).
void readUltrasonicSensors() {
  // Read one sensor per call to reduce blocking time (~30ms vs ~90ms)
  // and prevent echo crosstalk between adjacent sensors.
  static uint8_t sensorIndex = 0;
  static NewPing* const sonars[] = { &sonarFront, &sonarLeft, &sonarRight };

  float raw = sonars[sensorIndex]->ping_cm();
  dist[sensorIndex] = (raw == 0) ? MAX_DISTANCE : raw;
  distFiltered[sensorIndex] = US_ALPHA * dist[sensorIndex]
                             + (1.0 - US_ALPHA) * distFiltered[sensorIndex];

  sensorIndex = (sensorIndex + 1) % 3;
}

// lineCount: Return how many of the 4 IR sensors currently detect the line.
// Useful for detecting intersections (3-4 sensors) or line loss (0 sensors).
int lineCount() {
  int count = 0;
  for (int i = 0; i < 4; i++) {
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
// Example with IR_HIGH_ON_LINE=1:
//   irRaw = [50, 800, 900, 50]  -> sensors 1,2 see the line
//   position = (0*0 + 800*1 + 900*2 + 0*3) / (0+800+900+0) = 2600/1700 = 1.53
//   Error = 1.53 - 1.5 = +0.03 (very slightly right of center)
float linePosition() {
  float weightedSum = 0;
  float totalWeight = 0;
  for (int i = 0; i < 4; i++) {
    // Weight = raw intensity for on-line sensors, 0 for off-line sensors.
    // If IR_HIGH_ON_LINE=0, we invert the raw value so higher = more line.
    float weight = irOnLine[i] ? (float)(IR_HIGH_ON_LINE ? irRaw[i] : 1023 - irRaw[i]) : 0.0;
    weightedSum += weight * i;   // Multiply by sensor index (0,1,2,3)
    totalWeight += weight;
  }
  if (totalWeight < 1.0) return -1.0;  // No line detected
  return weightedSum / totalWeight;
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
    driveTank(SLOW_SPEED / 2, SLOW_SPEED / 2);
    return;
  }

  // Calculate error from center position (1.5 = perfectly centered)
  // Positive error = line is to the right of center
  // Negative error = line is to the left of center
  float error = pos - 1.5;
  float output = pidCompute(&pidLine, error);

  // Apply PID output as a differential: add to left, subtract from right.
  // In differential/skid-steer drive, the robot turns toward the SLOWER side.
  // Positive output (line is right of center) -> we need to steer RIGHT:
  //   error > 0 -> Kp * error > 0 -> output > 0
  //   -> leftSpeed = BASE + output (faster)
  //   -> rightSpeed = BASE - output (slower)
  //   -> slower right side -> robot turns RIGHT toward the line
  int leftSpeed = constrain(BASE_SPEED + (int)output, 0, 255);
  int rightSpeed = constrain(BASE_SPEED - (int)output, 0, 255);
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
//   For a simply-connected maze (no loops/islands), a consistent
//   turn direction at every junction guarantees solving the maze.
//   We use left-turn-first, which is equivalent to left-wall following.
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
  bool openFront = (distFiltered[0] > WALL_FAR_THRESH);
  bool wallLeft  = (distFiltered[1] < WALL_CLOSE_THRESH);
  bool wallRight = (distFiltered[2] < WALL_CLOSE_THRESH);

  // T-junction: open ahead, but walls on both sides form the T
  bool tJunction = (openFront && wallLeft && wallRight);
  // Four-way intersection: open in all directions
  bool fourWay   = (openFront && !wallLeft && !wallRight);

  if (tJunction || fourWay) {
    // Always turn left at junctions. For a simply-connected maze,
    // this is equivalent to left-wall following and guarantees the
    // robot will eventually find the exit (though not necessarily
    // by the shortest path).
    startTurn(TURN_LEFT, STATE_WALL_FOLLOWING);
    return;
  }

  // --- Dead-end detection ---
  // All three sides blocked = dead end. Perform a 180-degree U-turn
  // instead of the two-cycle 90+90 recovery.
  bool deadEnd = (distFiltered[0] < FRONT_OBSTACLE_DIST) &&
                 (distFiltered[1] < WALL_CLOSE_THRESH) &&
                 (distFiltered[2] < WALL_CLOSE_THRESH);

  if (deadEnd) {
    DEBUG_PRINTLN("Dead end detected");
    TurnDir dir = followRightWall ? TURN_LEFT : TURN_RIGHT;
    startBackupAndTurn(dir, STATE_WALL_FOLLOWING, TURN_180_DURATION);
    return;
  }

  // --- Step 2: Front obstacle avoidance ---
  // If a wall is directly ahead and too close, back up first
  // (to create clearance) then turn away from the followed wall.
  if (distFiltered[0] < FRONT_OBSTACLE_DIST) {
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
  bool shouldSwitchToRight = (distFiltered[2] < WALL_CLOSE_THRESH && distFiltered[1] > WALL_FAR_THRESH);
  bool shouldSwitchToLeft  = (distFiltered[1] < WALL_CLOSE_THRESH && distFiltered[2] > WALL_FAR_THRESH);

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
  int speed = (distFiltered[1] < WALL_CLOSE_THRESH && distFiltered[2] < WALL_CLOSE_THRESH)
              ? SLOW_SPEED : BASE_SPEED;

  // --- Step 5: PID wall distance control ---
  // Measure distance to the wall we're following
  float wallDist = followRightWall ? distFiltered[2] : distFiltered[1];
  float error = wallDist - WALL_SETPOINT;
  float output = pidCompute(&pidWall, error);

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
//   1. Low line confidence: ALL 4 IR sensors read below IR_NO_LINE_THRESH
//      (meaning the tape has ended or the robot has left it)
//   2. Walls present: at least one side ultrasonic sensor detects a wall
//      within WALL_FAR_THRESH cm
//
// This is checked every loop iteration while in STATE_LINE_FOLLOWING.
bool isWallMazeTransition() {
  // Check if ALL sensors have low confidence (tape gone)
  bool lowLineConfidence = true;
  for (int i = 0; i < 4; i++) {
    if (IR_HIGH_ON_LINE) {
      // High = on line: any sensor above threshold means tape is still visible
      if (irRaw[i] >= IR_NO_LINE_THRESH) { lowLineConfidence = false; break; }
    } else {
      // Low = on line: any sensor below threshold means tape is still visible
      if (irRaw[i] <= IR_NO_LINE_THRESH) { lowLineConfidence = false; break; }
    }
  }

  // Check if walls are detected on at least one side
  bool wallsPresent = (distFiltered[1] < WALL_FAR_THRESH || distFiltered[2] < WALL_FAR_THRESH);

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
  return (distFiltered[0] > EXIT_THRESHOLD &&
          distFiltered[1] > EXIT_THRESHOLD &&
          distFiltered[2] > EXIT_THRESHOLD);
}

// victoryBlink: Visual celebration when the maze is solved.
// Blinks the wall LED 5 times. Uses blocking delay() here because
// the robot is permanently stopped at this point - no need for
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
// SECTION 12: ERROR / BATTERY HANDLING
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
//   3. Execute current state's logic
//   4. State logic may change currentState for next iteration
// =====================================================

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);  // 115200 baud: fast enough to minimize
                           // serial blocking time on the Mega
    DEBUG_PRINTLN("MazeRobot starting...");
  #endif

  // Initialize all 4 motor pins (enable + 2 direction each)
  for (int i = 0; i < 4; i++) motorInit(allMotors[i]);

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

  // 3-second countdown with LED blinks for safe placement
  for (int i = 3; i > 0; i--) {
    DEBUG_PRINTF("Starting in %d...\n", i);
    digitalWrite(PIN_LED_POWER, HIGH);
    delay(500);
    digitalWrite(PIN_LED_POWER, LOW);
    delay(500);
  }
  digitalWrite(PIN_LED_POWER, HIGH);

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
