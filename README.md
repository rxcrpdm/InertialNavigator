# Multi-Sensor Fusion for Robot Localization and Navigation

## Introduction

This project implements a multi-sensor navigation and localization system in C++ using the Qt framework and Eigen library. It combines **Inertial Measurement Unit (IMU) data and wheel encoder measurements** to estimate a mobile robot's orientation, velocity, and position in a world coordinate frame.

The system integrates inertial navigation with wheel odometry through an **Error-State Extended Kalman Filter (ESKF)** to improve motion estimation by combining complementary sensor measurements.

The project is organized into four main components:

### 1. Wheel Odometry (`WheelOdometry.cpp`)

Implements differential-drive wheel odometry using left and right wheel encoder measurements. It calculates the robot's linear velocity and angular velocity (yaw rate) based on wheel displacement, wheel radius, wheelbase, and elapsed time.

### 2. Inertial Navigation System (`InertialNavigator.cpp`)

Implements an Inertial Navigation System (INS) using accelerometer and gyroscope measurements. It estimates:

- **Orientation:** Integrates gyroscope measurements using quaternion-based rotation updates.
- **Attitude stabilization:** Applies a complementary filter to correct roll and pitch estimates using accelerometer measurements.
- **Velocity:** Transforms acceleration from the body frame to the world frame, compensates for gravity, and integrates the resulting acceleration.
- **Position:** Estimates position through velocity integration.

The navigation updates run periodically using Qt's `QTimer`.

### 3. Error-State Kalman Filter Sensor Fusion (`EskFusion.cpp`)

Implements an Error-State Extended Kalman Filter (ESKF) to fuse IMU measurements with wheel odometry. The filter performs two main operations:

- **Prediction (Propagation):** Uses accelerometer and gyroscope measurements to propagate the estimated position, velocity, orientation, and error-state covariance.
- **Correction (Measurement Update):** Incorporates wheel odometry measurements to correct the estimated linear velocity and yaw rate, updating the navigation state and covariance.

This approach combines the high-frequency motion information provided by the IMU with wheel-based motion measurements to improve the overall state estimation.

### 4. Main Application (`main.cpp`)

Provides the application entry point and initializes the navigation system. It creates an `InertialNavigator` instance, connects its state update signal, and outputs the estimated orientation, velocity, and position.

## Key Features

- Differential-drive wheel odometry.
- IMU-based inertial navigation.
- Quaternion-based 3D orientation estimation.
- Gyroscope integration for attitude propagation.
- Complementary filtering for roll and pitch stabilization.
- Accelerometer coordinate transformation and gravity compensation.
- Error-State Extended Kalman Filter (ESKF).
- IMU and wheel odometry sensor fusion.
- Periodic state updates using Qt signals and timers.

## Technologies and Libraries

- **C++** – Core implementation.
- **Qt Framework** – Application management, timers, signals, and slots.
- **QtMath** – Mathematical operations.
- **Eigen** – Matrix operations and Kalman filter calculations.
- **Quaternions** – 3D orientation representation and rotation calculations.

## Project Objective

The primary objective is to develop a modular sensor fusion framework for mobile robot localization. By combining inertial navigation and wheel odometry, the system aims to provide more reliable estimates of a robot's motion than either sensing approach can provide independently.

The architecture provides a foundation for further development of autonomous mobile robot navigation, motion tracking, and localization systems.
