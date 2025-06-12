# System Architecture - mecanum_lulu

## Overview

The mecanum_lulu robot consists of multiple layers of control and hardware:

[Power Supply]
|
+--- 7.4V Battery (motors & servos)
+--- 3.7V Battery (ESP32 control board)

[Mechanical Platform]
|
+--- Triangular Acrylic Chassis
+--- 3 x DC Motors + Encoders
+--- MG996R Servo (optional sail control)

[Control System]
|
+--- ESP32 (main controller)
|
+--- PWM Motor Driver Control
+--- Encoder Feedback Reading
+--- Servo PWM Control
+--- Sensor Input (Ultrasonic, IMU)
+--- Serial/Wi-Fi Communication

[Computing (future)]
|
+--- Raspberry Pi (AI processing)
|
+--- Computer Vision (Camera)
+--- Reinforcement Learning
+--- Remote Web/App Control


## Communication

- ESP32 controls real-time motor actuation
- Raspberry Pi handles AI vision & path planning
- Serial / Wi-Fi bridge between ESP32 and Raspberry Pi

## Control Flow (Initial Phase)

1. Receive control input via Serial.
2. Translate input into motor PWM values.
3. Use encoder feedback to monitor wheel movement.
4. Adjust control parameters for better motion precision.

## To Do (Future Phases)

- Add sensor data fusion (IMU + Ultrasonic)
- Train AI model for object avoidance and navigation
- Implement remote control interface (Wi-Fi)
