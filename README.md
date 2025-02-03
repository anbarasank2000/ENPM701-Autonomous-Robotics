# ENPM701-Autonomous-Robotics
[![Grand Challenge]()
[![Video Title](https://img.youtube.com/vi/bd-VS6yTScw/0.jpg)](https://youtu.be/bd-VS6yTScw)

# Autonomous Mobile Robot - Grand Challenge

## Overview
This project involves building an autonomous mobile robot capable of self-navigation and perception. The final goal is to complete the **Grand Challenge**, where the robot must navigate a Martian-like environment, pick nine colored blocks (red, green, blue) in order, and deliver them to a designated construction zone.

## Table of Contents
- [Hardware Design](#hardware-design)
- [Software Implementation](#software-implementation)
  - [Stop Sign Detection](#stop-sign-detection)
  - [Arrow Detection](#arrow-detection)
  - [Motion Control](#motion-control)
  - [Localization](#localization)
  - [Distance Detection](#distance-detection)
- [Grand Challenge Execution](#grand-challenge-execution)
- [Lessons Learned](#lessons-learned)

---

## Hardware Design
The robot is built using a mobile platform kit and consists of the following components:
- **Chassis:** Mobile base with four DC motors
- **Encoders:** Magnetic encoders for speed monitoring
- **Processing Unit:** Raspberry Pi 3B+
- **Motor Control:** H-Bridge for motor actuation
- **Gripper:** Servo motor for pick-and-place operations
- **Sensors:**
  - Camera (for object detection and navigation)
  - Ultrasonic sensor (for obstacle avoidance and repositioning)
  - Inertial Measurement Unit (IMU) (for orientation tracking)

## Software Implementation
The software was developed in incremental steps with weekly progress updates.

### Stop Sign Detection
- Used **HSV masking** to detect green traffic lights.
- Fine-tuning HSV values was crucial for accuracy.

### Arrow Detection
- Detected arrows using **contour detection**.
- Determined the direction of the arrow for navigation decisions.

### Motion Control
- Compared robot movement using **only encoders** vs. **IMU-assisted motion**.
- Found that integrating IMU feedback resulted in **more accurate trajectory tracking**.

### Localization
- Implemented basic math operations to calculate the next position of the robot.
- Faced issues with **accumulated errors** affecting long-term trajectory.
- Improved accuracy by **repositioning the robot using ultrasonic sensors** after each run.

### Distance Detection
- Collected data points on the **centroid position of blocks** in the image frame.
- Used interpolation to estimate real-time distances for precise pick-up.

## Grand Challenge Execution
### Rules:
- **Correct block placement:** +1 point
- **Incorrect block placement:** -1 point

### Final Run Summary:
- The robot successfully followed the trajectory but encountered issues:
  - **False positive detection** when picking the second red block.
  - **Failure in stopping logic** caused an early stop before reaching the construction zone.
  - **Ultrasonic sensor limitation** led to an unhandled error.

A simple **if-condition** could have fixed the error, allowing the robot to complete the task perfectly.

## Lessons Learned
Through this project, I gained hands-on experience in:
- **Perception** (image processing and object detection)
- **Navigation** (localization, and sensor fusion)
- **Motion Control** (PID control, trajectory tracking, and feedback loops)

This project has been a fantastic learning experience, and I’m grateful for the guidance of **Dr. Michell** throughout the semester.

---

## Acknowledgments
Thanks to **Dr. Michell** for his support in this project!

**Peace!** ✌️

