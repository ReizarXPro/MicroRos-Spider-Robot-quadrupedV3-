 Quadruped Balance & Locomotion Controller with PID, Walking Gait, and Enhanced GUI

This advanced quadruped robot controller combines PID-based balance control with dynamic walking gaits and a comprehensive, cyberpunk-styled GUI for real-time monitoring, tuning, and manual control.

## New in v2.0 (Upgraded GUI)
- **3D Orientation Visualization**: New tab with real-time 3D cube representation of robot attitude (roll/pitch) using neon cyberpunk styling and depth-based effects
- **Tabbed Visualization Matrix**: Switch between classic roll/pitch oscillation plots and the new 3D view
- **Improved Monitoring**: Cleaner layout with enhanced status feedback and seamless integration of walking + balance controls
- **Maintained Features**: Full walking gait support (tripod, etc.), keyboard/manual movement, PID tuning, stance selection, leg-by-leg manual override, sensor calibration

<img width="1800" height="1053" alt="Control Panel" src="https://github.com/user-attachments/assets/e5f8522c-b13b-45c4-a62e-b9b13039859b" />
<img width="1800" height="1053" alt="Oscillation Plots" src="https://github.com/user-attachments/assets/aac83d0f-02f9-47be-b4b6-c1afaedda17d" />
<img width="1800" height="1053" alt="3D Visualization Tab (New)" src="https://github.com/user-attachments/assets/46d6ee62-dfba-4cb1-b855-e2750555c620" />

## Features

### Controller (quadruped_controller_pid.py)
- Dual PID controllers (separate for roll and pitch)
- Complementary filter for stable orientation estimation
- Gyroscope and accelerometer calibration
- Multiple base stances (stand, crouch, wide, narrow)
- Dynamic walking gaits with configurable speed, step height/length, and cycle time
- Safety features: stability monitoring, fall risk detection, emergency mode

### GUI (quadruped_gui.py)
- **Cyberpunk aesthetic** with neon accents, custom frames, and precise sliders
- **Real-time monitoring**: Roll/pitch plots, 3D attitude cube, leg states, system status, walking status
- **Interactive controls**:
  - PID tuning (Kp, Ki, Kd for roll & pitch)
  - Walking enable/disable + gait selection
  - Keyboard (WASD/Arrows + Space) or button-based movement
  - Stance selection
  - Per-leg manual override with balance weights
  - One-click sensor calibration
- **Scrollable control panel** for easy access on smaller screens

## Installation

### Prerequisites
```bash
# ROS 2 Humble (Ubuntu 22.04 recommended)
sudo apt update && sudo apt install ros-humble-desktop

# Python dependencies
pip3 install matplotlib numpy rclpy

# Tkinter (usually pre-installed)
sudo apt install python3-tk
File Structure
textquadruped_controller/
├── quadruped_controller_pid.py   # Main controller node
├── quadruped_gui.py              # Upgraded cyberpunk GUI
├── launch.sh                     # Optional launch helper
└── README.md                     # This file
Usage
Quick Start
Bash# Terminal 1: Launch controller
source /opt/ros/humble/setup.bash
python3 quadruped_controller_pid.py

# Terminal 2: Launch GUI
source /opt/ros/humble/setup.bash
python3 quadruped_gui.py
Keyboard Controls (in GUI window)

W / ↑: Forward
S / ↓: Backward
A / ←: Left
D / →: Right
Space: Stop walking

Optional Launch Script
Bashchmod +x launch.sh
./launch.sh
# Choose option to run controller, GUI, or both
Configuration
Servo Mapping (8 servos expected)

0: Front Right Hip
1: Front Right Knee
2: Front Left Hip
3: Front Left Knee
4: Rear Right Hip
5: Rear Right Knee
6: Rear Left Hip
7: Rear Left Knee

ROS 2 Topics

Subscribe: /esp32/mpu6050/data (Float32MultiArray)
Format: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp]

Publish: esp32/servo/angles (Int32MultiArray)
Format: 8 servo angles (0-180°)


Calibration

Place robot on flat, stable surface
Ensure complete stillness
Click ◢ CALIBRATE SENSORS ◣ in GUI
Wait ~2 seconds for completion

PID Tuning Guide
Start with:
textRoll/Pitch Kp: 2.0–3.0
Roll/Pitch Ki: 0.05–0.2
Roll/Pitch Kd: 0.3–0.8
Process:

Increase Kp until fast response (watch for oscillation)
Add Ki to eliminate steady-state error
Add Kd to dampen overshoot
Test on various surfaces and disturbances

Additional tunable parameters (via GUI):

Max correction angle
Deadzone
Walking speed, step height/length, cycle time

Troubleshooting

Oscillation: Lower Kp/Kd, increase deadzone
Slow correction: Increase Kp, recalibrate sensors
Drift: Recalibrate gyro, ensure level surface
No response: Check ROS topics with ros2 topic echo

Hardware Notes (ESP32 Side Example)
C++// Publish IMU data frequently
float data[7] = {ax, ay, az, gx, gy, gz, temp};
pub_imu.publish(Float32MultiArray with data);

// Subscribe to servo commands
void servo_cb(const Int32MultiArray& msg) {
  for(int i = 0; i < 8; i++) servo[i].write(msg.data[i]);
}
Contributing
Issues, PRs, and feature requests welcome! Especially:

Additional gait patterns
Joystick/gamepad support
Magnetometer integration for full yaw tracking

License
MIT License
Acknowledgments

ROS 2 community
Open-source robotics projects
Cyberpunk 2077 for aesthetic inspiration 😎

textThis upgraded README now reflects all the new features (espe
