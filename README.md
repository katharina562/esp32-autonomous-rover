# esp32-autonomous-rover
ESP32-based mobile robot featuring autonomous exploration, obstacle avoidance, 3D scanning, and wheel odometry

## Current Features & Demonstrations

The rover is built around an ESP32 microcontroller and can be controlled wirelessly through a browser-based interface.

### 1. Remote Control

The web interface allows the user to control the rover's movement, including forward and backward driving and turning.

**Demo:**

https://github.com/user-attachments/assets/8bb33d9a-1f30-42fa-82fc-9abaeb37072a

### 2. Sensor Positioning

A VL53L0X time-of-flight distance sensor is mounted on a two-axis pan-tilt mechanism. Two servo motors allow the sensor to be oriented horizontally and vertically through the web interface.

**Demo:** 

https://github.com/user-attachments/assets/f83fb8cf-c4f7-41cf-93d6-9d1266867854

### 3. Odometry & Live Route Visualization

The rover estimates its position and orientation using wheel encoders and an MPU-6050 gyroscope. The estimated trajectory is visualized in real time on a 2D map within the web interface.

This provides the foundation for further mapping and autonomous navigation capabilities.

**Demo:** 

https://github.com/user-attachments/assets/b02f4e5a-2c89-4f80-b7a9-094c55a21b3f

### 4. Mapping

Using the distance sensor that is connected to two 180° servo motors (left to right; up and down), the rover can scan its environment and visualize it on a map in 2d and 3d. In combination with the odometry, multiple scans from different positions can map a whole area.

**Demo:**

https://github.com/user-attachments/assets/fbc69777-5223-4215-b996-3d4d0732e0c3

### 5. Autonomous exploration 

Using the distance sensor for obstacle avoidance, the rover autonomously explores its surroundings by moving short distances and performing 180° scans at five different angles. When an obstacle is detected, it performs more detailed scans to map its surroundings. If an obstacle gets too close, the rover reverses, turns, and continues exploring in a different direction.

### 6. Autonomous mapping 

The rover autonomously explores a user-defined area, using obstacle avoidance to navigate safely around obstacles. While moving, it performs 2D scans (in the future 3d) and combines them into a map of its surroundings. Based on this map, it continuously selects new scan positions to gradually explore and map the entire area.
