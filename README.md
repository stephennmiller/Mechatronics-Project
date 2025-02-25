# Mechatronics Project

Arduino Mega script for a maze-solving robot with line and wall navigation.

## Description
This project implements a maze-solving robot using an Arduino Mega 2560. It navigates a line maze (black tape on a light surface) using 4 IR sensors and a wall maze using 3 ultrasonic sensors. Features include PID control, dynamic wall switching, junction detection, and exit finding.

## Hardware
- Arduino Mega 2560
- 4 DC motors (via 2 L298N drivers)
- 4 IR sensors (line detection)
- 3 Ultrasonic sensors (wall detection)
- 8V battery pack
- 6 LEDs (status indicators)

## Dependencies
- [NewPing library](https://bitbucket.org/teckel12/arduino-new-ping/downloads/) (install via Arduino Library Manager)

## Setup
1. Wire the components as per the pin definitions in `MazeRobot.ino`.
2. Install the NewPing library in the Arduino IDE.
3. Upload `MazeRobot.ino` to your Arduino Mega.

## Usage
- Place the robot at the start of a line maze.
- It follows the line until transitioning to a wall maze (detected by IR and ultrasonic sensors).
- The robot navigates walls, handles junctions, and stops at an exit (open space).