# esp32-autonomous-rover
ESP32-based mobile robot featuring autonomous exploration, obstacle avoidance, 3D scanning, and wheel odometry

## Current Features & Demonstrations

The rover is built around an ESP32 microcontroller and can be controlled wirelessly through a browser-based interface.

### 1. Remote Control

The web interface allows the user to control the rover's movement, including forward and backward driving and turning.

**Demo:**

https://github.com/user-attachments/assets/eec29200-4fc8-4817-8b46-1d74e6bbfdf3

### 2. Sensor Positioning

A VL53L0X time-of-flight distance sensor is mounted on a two-axis pan-tilt mechanism. Two servo motors allow the sensor to be oriented horizontally and vertically through the web interface.

**Demo:** 

https://github.com/user-attachments/assets/48ac1fff-b560-4883-97ed-4a4beecf0579

### 3. Odometry & Live Route Visualization

The rover estimates its position and orientation using wheel encoders and an MPU-6050 gyroscope. The estimated trajectory is visualized in real time on a 2D map within the web interface.

This provides the foundation for further mapping and autonomous navigation capabilities.

**Demo:** 

https://github.com/user-attachments/assets/345ec12e-ef98-480d-b979-ba7b330e9cce
