// Maze-Solving Robot Script for Arduino Mega with 2 L298Ns
// Hardware: 4 DC motors (individual control), 4 IR sensors, 3 Ultrasonic sensors, 6 LEDs
// Date: February 24, 2025

#include <NewPing.h>

// === Pin Definitions ===
// L298N #1 - Left Motors
#define PIN_ENA1 2   // Left Front Speed (PWM)
#define PIN_IN1 22   // Left Front Direction 1
#define PIN_IN2 23   // Left Front Direction 2
#define PIN_ENB1 3   // Left Rear Speed (PWM)
#define PIN_IN3 24   // Left Rear Direction 1
#define PIN_IN4 25   // Left Rear Direction 2

// L298N #2 - Right Motors
#define PIN_ENA2 4   // Right Front Speed (PWM)
#define PIN_IN5 26   // Right Front Direction 1
#define PIN_IN6 27   // Right Front Direction 2
#define PIN_ENB2 5   // Right Rear Speed (PWM)
#define PIN_IN7 28   // Right Rear Direction 1
#define PIN_IN8 29   // Right Rear Direction 2

// IR Sensors
#define PIN_IR1 A0
#define PIN_IR2 A1
#define PIN_IR3 A2
#define PIN_IR4 A3

// Ultrasonic Sensors
#define PIN_TRIG_FRONT 30
#define PIN_ECHO_FRONT 31
#define PIN_TRIG_LEFT 32
#define PIN_ECHO_LEFT 33
#define PIN_TRIG_RIGHT 34
#define PIN_ECHO_RIGHT 35

// LEDs
#define PIN_LED_POWER 36
#define PIN_LED_ERROR 37
#define PIN_LED_LINE 38
#define PIN_LED_WALL 39
#define PIN_LED_RIGHT 40  // Indicates right-wall following
#define PIN_LED_LEFT 41   // Indicates left-wall following

// Battery
#define PIN_BATTERY A4

// === Constants ===
#define MAX_DISTANCE 200
#define BASE_SPEED 150
#define SLOW_SPEED 100    // Reduced speed for tight spaces
#define TAPE_WIDTH 19
#define VOLTAGE_THRESHOLD 6.5
#define WALL_SETPOINT 25.0
#define WALL_SWITCH_THRESHOLD 50
#define DEBOUNCE_COUNT 3
#define EXIT_THRESHOLD 100  // Open space detection (cm)
#define MAX_TURNS 20       // Max turns to store in memory

// === Ultrasonic Setup ===
NewPing sonarFront(PIN_TRIG_FRONT, PIN_ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(PIN_TRIG_LEFT, PIN_ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT, MAX_DISTANCE);

// === State Machine ===
enum RobotState { LINE_FOLLOWING, WALL_FOLLOWING, ERROR_STATE, EXIT_FOUND };
RobotState currentState = LINE_FOLLOWING;

// === PID Variables ===
float Kp_line = 25.0, Ki_line = 0.1, Kd_line = 10.0;  // Line following
float Kp_wall = 20.0, Ki_wall = 0.05, Kd_wall = 15.0; // Wall following
float previousError = 0, integral = 0;

// === Globals ===
int irReadings[4];
float distances[3];
unsigned long lastUpdate = 0;
bool followRightWall = true;
int wallSwitchCounter = 0;
char turnHistory[MAX_TURNS];  // 'L' for left, 'R' for right, 'F' for forward
int turnIndex = 0;           // Current position in turn history

void setup() {
  Serial.begin(9600);
  
  int motorPins[] = {PIN_ENA1, PIN_IN1, PIN_IN2, PIN_ENB1, PIN_IN3, PIN_IN4,
                     PIN_ENA2, PIN_IN5, PIN_IN6, PIN_ENB2, PIN_IN7, PIN_IN8};
  for (int pin : motorPins) pinMode(pin, OUTPUT);

  pinMode(PIN_LED_POWER, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_LINE, OUTPUT);
  pinMode(PIN_LED_WALL, OUTPUT);
  pinMode(PIN_LED_RIGHT, OUTPUT);
  pinMode(PIN_LED_LEFT, OUTPUT);

  digitalWrite(PIN_LED_POWER, HIGH);
  digitalWrite(PIN_LED_LINE, HIGH);
  
  stopMotors();
}

void loop() {
  if (!checkBattery()) {
    enterErrorState("Low battery voltage");
    return;
  }

  readIRSensors();
  readUltrasonicSensors();

  switch (currentState) {
    case LINE_FOLLOWING:
      followLine();
      if (isWallMazeTransition()) switchToWallMode();
      break;
    case WALL_FOLLOWING:
      followWall();
      if (isExitFound()) {
        currentState = EXIT_FOUND;
        stopMotors();
        victoryBlink();
      }
      break;
    case ERROR_STATE:
    case EXIT_FOUND:
      break;  // Do nothing, motors stopped
  }
  delay(10);
}

// === Motor Control ===
void setMotor(int ena, int in1, int in2, int speed, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(ena, speed);
}

void setLeftMotors(int speed, bool forward) {
  setMotor(PIN_ENA1, PIN_IN1, PIN_IN2, speed, forward);
  setMotor(PIN_ENB1, PIN_IN3, PIN_IN4, speed, forward);
}

void setRightMotors(int speed, bool forward) {
  setMotor(PIN_ENA2, PIN_IN5, PIN_IN6, speed, forward);
  setMotor(PIN_ENB2, PIN_IN7, PIN_IN8, speed, forward);
}

void stopMotors() {
  setLeftMotors(0, true);
  setRightMotors(0, true);
}

// === Sensor Functions ===
void readIRSensors() {
  irReadings[0] = analogRead(PIN_IR1);
  irReadings[1] = analogRead(PIN_IR2);
  irReadings[2] = analogRead(PIN_IR3);
  irReadings[3] = analogRead(PIN_IR4);
  
  for (int i = 0; i < 4; i++) {
    if (irReadings[i] < 0 || irReadings[i] > 1023) enterErrorState("IR sensor failure");
  }
}

void readUltrasonicSensors() {
  distances[0] = sonarFront.ping_cm();
  distances[1] = sonarLeft.ping_cm();
  distances[2] = sonarRight.ping_cm();
  
  for (int i = 0; i < 3; i++) if (distances[i] == 0) distances[i] = MAX_DISTANCE;
}

// === Line Following ===
void followLine() {
  int position = (irReadings[0] * 0 + irReadings[1] * 1 + irReadings[2] * 2 + irReadings[3] * 3) /
                 (irReadings[0] + irReadings[1] + irReadings[2] + irReadings[3] + 1);
  float error = position - 1.5;

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp_line * error + Ki_line * integral + Kd_line * derivative;
  previousError = error;
  lastUpdate = now;

  int leftSpeed = constrain(BASE_SPEED - output, 0, 255);
  int rightSpeed = constrain(BASE_SPEED + output, 0, 255);
  setLeftMotors(leftSpeed, true);
  setRightMotors(rightSpeed, true);
}

// === Wall Following with All Enhancements ===
void followWall() {
  // Junction detection
  bool tJunction = (distances[0] > 50 && distances[1] < 30 && distances[2] < 30);
  bool fourWay = (distances[0] > 50 && distances[1] > 50 && distances[2] > 50);
  
  if (tJunction || fourWay) {
    // Turn left at T-junction or four-way (arbitrary choice)
    setLeftMotors(BASE_SPEED / 2, true);
    setRightMotors(BASE_SPEED / 2, false);  // Spin left
    delay(300);  // Adjust for 90° turn
    if (turnIndex < MAX_TURNS) turnHistory[turnIndex++] = 'L';
    return;
  }

  // Wall selection with debounce
  bool shouldSwitchToLeft = (distances[1] < 30 && distances[1] > 0 && distances[2] > WALL_SWITCH_THRESHOLD);
  bool shouldSwitchToRight = (distances[2] < 30 && distances[2] > 0);
  
  if (shouldSwitchToRight && !followRightWall) {
    wallSwitchCounter++;
    if (wallSwitchCounter >= DEBOUNCE_COUNT) {
      followRightWall = true;
      wallSwitchCounter = 0;
      integral = 0;
      digitalWrite(PIN_LED_RIGHT, HIGH);
      digitalWrite(PIN_LED_LEFT, LOW);
    }
  } else if (shouldSwitchToLeft && followRightWall) {
    wallSwitchCounter++;
    if (wallSwitchCounter >= DEBOUNCE_COUNT) {
      followRightWall = false;
      wallSwitchCounter = 0;
      integral = 0;
      digitalWrite(PIN_LED_RIGHT, LOW);
      digitalWrite(PIN_LED_LEFT, HIGH);
    }
  } else {
    wallSwitchCounter = 0;
  }

  // Speed adjustment for tight spaces
  int currentSpeed = (distances[1] < 30 && distances[2] < 30) ? SLOW_SPEED : BASE_SPEED;

  // PID control for wall distance
  float error = followRightWall ? (distances[2] - WALL_SETPOINT) : (distances[1] - WALL_SETPOINT);
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp_wall * error + Ki_wall * integral + Kd_wall * derivative;
  previousError = error;
  lastUpdate = now;

  int leftSpeed = constrain(currentSpeed - output, 0, 255);
  int rightSpeed = constrain(currentSpeed + output, 0, 255);
  setLeftMotors(leftSpeed, true);
  setRightMotors(rightSpeed, true);

  // Front obstacle avoidance with turn
  if (distances[0] < 15) {
    stopMotors();
    delay(500);
    setLeftMotors(BASE_SPEED / 2, false);
    setRightMotors(BASE_SPEED / 2, false);
    delay(200);
    // Turn 90° left (right wall) or right (left wall)
    if (followRightWall) {
      setLeftMotors(BASE_SPEED / 2, true);
      setRightMotors(BASE_SPEED / 2, false);  // Spin left
      if (turnIndex < MAX_TURNS) turnHistory[turnIndex++] = 'L';
    } else {
      setLeftMotors(BASE_SPEED / 2, false);
      setRightMotors(BASE_SPEED / 2, true);  // Spin right
      if (turnIndex < MAX_TURNS) turnHistory[turnIndex++] = 'R';
    }
    delay(300);  // Adjust for 90° turn
    // If no walls after turn, consider 180° (uncomment if needed)
    /*
    readUltrasonicSensors();
    if (distances[1] > 50 && distances[2] > 50) {
      setLeftMotors(BASE_SPEED / 2, true);
      setRightMotors(BASE_SPEED / 2, false);  // Spin left 180° total
      delay(600);
      if (turnIndex < MAX_TURNS) turnHistory[turnIndex++] = 'L';
    }
    */
  }

  // Debugging output
  Serial.print("Wall: "); Serial.print(followRightWall ? "Right" : "Left");
  Serial.print(", Dist: "); Serial.print(followRightWall ? distances[2] : distances[1]);
  Serial.print(", Output: "); Serial.println(output);
}

// === State Transitions ===
bool isWallMazeTransition() {
  bool noLine = (irReadings[0] < 200 && irReadings[1] < 200 && 
                 irReadings[2] < 200 && irReadings[3] < 200);
  bool wallsDetected = (distances[1] < 50 || distances[2] < 50);
  return noLine && wallsDetected;
}

void switchToWallMode() {
  currentState = WALL_FOLLOWING;
  digitalWrite(PIN_LED_LINE, LOW);
  digitalWrite(PIN_LED_WALL, HIGH);
  digitalWrite(PIN_LED_RIGHT, HIGH);  // Start with right wall
  digitalWrite(PIN_LED_LEFT, LOW);
  stopMotors();
  delay(500);
  integral = 0;
}

// === Exit Detection ===
bool isExitFound() {
  return (distances[0] > EXIT_THRESHOLD && distances[1] > EXIT_THRESHOLD && distances[2] > EXIT_THRESHOLD);
}

void victoryBlink() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_WALL, HIGH);
    delay(200);
    digitalWrite(PIN_LED_WALL, LOW);
    delay(200);
  }
}

// === Error Handling ===
void enterErrorState(const char* message) {
  currentState = ERROR_STATE;
  stopMotors();
  digitalWrite(PIN_LED_ERROR, HIGH);
  digitalWrite(PIN_LED_LINE, LOW);
  digitalWrite(PIN_LED_WALL, LOW);
  digitalWrite(PIN_LED_RIGHT, LOW);
  digitalWrite(PIN_LED_LEFT, LOW);
  Serial.println(message);
}

bool checkBattery() {
  int raw = analogRead(PIN_BATTERY);
  float voltage = raw * (8.0 / 1023.0); // Adjust for voltage divider
  if (voltage < VOLTAGE_THRESHOLD) {
    enterErrorState("Battery voltage below threshold");
    return false;
  }
  return true;
}