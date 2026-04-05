# Self-Balancing Robot

A two-wheeled self-balancing robot. This project utilizes an Arduino-based controller to maintain vertical stability using an IMU sensor and stepper motors.

## Overview

The robot uses a **PID (Proportional-Integral-Derivative)** control loop to stay upright. It processes data from an **MPU6050** (Accelerometer + Gyroscope) and uses a **Complementary Filter** to estimate its tilt angle accurately. Additionally, an ultrasonic sensor is included for basic obstacle detection/avoidance logic.

## Hardware Components

- **Microcontroller:** Arduino compatible board.
- **IMU Sensor:** MPU6050 (for tilt sensing via I2C).
- **Motors:** 2x Stepper Motors (driven using `AccelStepper` library).
- **Distance Sensor:** HC-SR04 Ultrasonic sensor.
- **Power:** External battery source for motors.

## Software Architecture

The software is organized around a simple **Task Scheduler** to manage different operations at specific intervals.

### 1. Task Scheduler
Instead of using a complex RTOS, the project implements a lightweight scheduler that triggers tasks based on a tick counter.
- **High Priority:** Control loop (PID and motor updates) runs every tick.
- **Lower Priority:** Sensor reading and statistical calculations (Welford's algorithm) run every 40 ticks.

### 2. Angle Estimation
Combining accelerometer and gyroscope data is critical:
- **Accelerometer:** Provides a stable long-term reference for gravity but is noisy during movement.
- **Gyroscope:** Provides very accurate short-term tilt data but drifts over time.
- **Complementary Filter:** `Angle = alpha * (Angle + Gyro_Data) + (1 - alpha) * Accel_Angle`. This blends both sensors to get a clean, drift-free tilt estimate.

### 3. PID Control
The core of the balancing logic:
- **Proportional (Kp):** Corrects the error based on current tilt.
- **Integral (Ki):** Corrects accumulated steady-state errors.
- **Derivative (Kd):** Predicts future error to dampen oscillations.

### 4. Welford's Algorithm
The project implements **Welford's online algorithm** to calculate the running mean and variance of the angle and distance measurements. This allows for statistical analysis of sensor noise without storing large arrays of data.

## Configuration

Key constants can be tuned in `main.ino`:
- `Kp`, `Ki`, `Kd`: PID gains for balancing.
- `TARGET_ANGLE`: The mechanical balance point (default is -90.0).
- `alpha`: The filter coefficient (balancing responsiveness vs. noise).

## How to Run

1.  Connect the hardware according to the pin definitions in `main.ino`.
2.  Install the `AccelStepper` library in your Arduino IDE.
3.  Upload `main.ino` to your board.
4.  Keep the robot steady at its balance point during startup for calibration.
