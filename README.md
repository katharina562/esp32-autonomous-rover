# esp32-autonomous-rover
ESP32-based mobile robot featuring autonomous exploration, obstacle avoidance, 3D scanning, and wheel odometry

## Current Features & Demonstrations

The rover is built around an ESP32 microcontroller and can be controlled wirelessly through a browser-based interface.

### 1. Remote Control

The web interface allows the user to control the rover's movement, including forward and backward driving and turning.

**Demo:**
https://github.com/user-attachments/assets/8f8575f6-74b2-44f0-b447-ffe4e2f09ae9


### 2. Sensor Positioning

A VL53L0X time-of-flight distance sensor is mounted on a two-axis pan-tilt mechanism. Two servo motors allow the sensor to be oriented horizontally and vertically through the web interface.

**Demo:** [Sensor Control](videos/sensor_control.mp4)

### 3. Odometry & Live Route Visualization

The rover estimates its position and orientation using wheel encoders and an MPU-6050 gyroscope. The estimated trajectory is visualized in real time on a 2D map within the web interface.

This provides the foundation for further mapping and autonomous navigation capabilities.

**Demo:** [Live Route Visualization](videos/odometry.mp4)
