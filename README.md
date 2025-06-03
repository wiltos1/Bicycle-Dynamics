# Bicycle Dynamics

A collection of Arduino sketches for monitoring and logging bicycle dynamics data. This repository currently contains two main sketches:

- **ControlPanel**: Code for the ESP32 based Touchscreen/LCD. Collects data from the Nano and logs it to an SD card as well as provides a live data interface for user
- **Arduino Nano Data Collection**: An Arduino Nano sketch that reads data from two BNO055 IMUs and Hall‐effect sensors, calculates yaw (with reset capability), and streams sensor data (orientation, acceleration, RPM) over serial.

Each sketch lives in its own folder under the repo root. The Arduino IDE expects the folder name to match the main `.ino` file inside it.

