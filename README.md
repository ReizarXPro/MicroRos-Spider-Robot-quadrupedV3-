# Quadruped Balance Controller with PID and GUI

This enhanced quadruped balance controller uses PID control algorithms and provides a comprehensive GUI for tuning and monitoring your quadruped robot's balance system.

## Features

### Enhanced Controller
- **PID Control**: Separate PID controllers for roll and pitch axes
- **IMU Integration**: Uses MPU6050 accelerometer and gyroscope data
- **Complementary Filter**: Combines accel and gyro data for stable angle estimation
- **Calibration**: Gyroscope and accelerometer calibration routines
- **Flexible Configuration**: Multiple base positions and adjustable parameters

### GUI Features
- **Real-time Monitoring**: Live attitude plots and status display
- **PID Tuning**: Interactive sliders for Kp, Ki, Kd parameters
- **Calibration Tools**: One-click calibration for sensors
- **Position Control**: Switch between different robot poses
- **Servo Monitoring**: Live display of all servo angles

## Installation

### Prerequisites
```bash
# Install ROS2 (Ubuntu 22.04 - Humble)
sudo apt update
sudo apt install ros-humble-desktop

# Install Python dependencies
pip3 install matplotlib numpy
sudo apt install python3-tk

# Install ROS2 Python packages
sudo apt install python3-rclpy python3-std-msgs
```

### File Structure
```
quadruped_controller/
├── quadruped_balance_controller_pid.py  # Enhanced controller with PID
├── quadruped_gui.py                     # GUI application
├── launch.sh                           # Launch script
└── README.md                           # This file
```

## Usage

### Quick Start
1. Make the launch script executable:
```bash
chmod +x launch.sh
```

2. Run the launcher:
```bash
./launch.sh
```

3. Select option 3 to launch both controller and GUI

### Manual Launch

#### Terminal 1 - Controller Node
```bash
source /opt/ros/humble/setup.bash
python3 quadruped_balance_controller_pid.py
```

#### Terminal 2 - GUI
```bash
source /opt/ros/humble/setup.bash
python3 quadruped_gui.py
```

## Configuration

### Robot Setup
The controller expects 8 servos mapped as follows:
- 0: Front Right Hip
- 1: Front Right Knee  
- 2: Front Left Hip
- 3: Front Left Knee
- 4: Rear Right Hip
- 5: Rear Right Knee
- 6: Rear Left Hip
- 7: Rear Left Knee

### ROS2 Topics
- **Input**: `/esp32/mpu6050/data` (Float32MultiArray)
  - Format: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature]
- **Output**: `esp32/servo/angles` (Int32MultiArray)
  - Format: [servo0_angle, servo1_angle, ..., servo7_angle]

### IMU Data Format
The MPU6050 data should be published as Float32MultiArray with 7 elements:
```
data[0] = accel_x (m/s²)
data[1] = accel_y (m/s²)  
data[2] = accel_z (m/s²)
data[3] = gyro_x (rad/s)
data[4] = gyro_y (rad/s)
data[5] = gyro_z (rad/s)
data[6] = temperature (°C)
```

## Calibration Process

### Gyroscope Calibration
1. Place robot on level, stable surface
2. Ensure robot is completely still
3. Click "Calibrate Gyroscope" in GUI
4. Wait for calibration to complete (~2 seconds)

### Accelerometer Calibration  
1. Place robot on known level surface
2. Click "Calibrate Accelerometer" in GUI
3. Wait for calibration to complete (~2 seconds)

## PID Tuning Guide

### Understanding PID Parameters

**Proportional (Kp)**:
- Controls immediate response to error
- Higher values = faster response, may cause oscillation
- Start with: 1.0-3.0

**Integral (Ki)**:
- Eliminates steady-state error
- Higher values = eliminates offset, may cause instability
- Start with: 0.05-0.2

**Derivative (Kd)**:
- Predicts future error, reduces overshoot
- Higher values = less overshoot, may amplify noise
- Start with: 0.1-1.0

### Tuning Process
1. Start with conservative values (Kp=1.0, Ki=0.05, Kd=0.1)
2. Increase Kp until system responds quickly but doesn't oscillate
3. Add Ki to eliminate steady-state error
4. Add Kd to reduce overshoot and improve stability
5. Test on different surfaces and angles

### Recommended Starting Values
```
Roll PID:  Kp=2.0, Ki=0.1, Kd=0.5
Pitch PID: Kp=2.0, Ki=0.1, Kd=0.5
```

## Troubleshooting

### Common Issues

**Robot oscillates continuously**:
- Reduce Kp values
- Increase deadzone
- Check for mechanical play in joints

**Slow response to tilting**:
- Increase Kp values
- Reduce deadzone
- Verify IMU calibration

**Robot doesn't return to level**:
- Increase Ki values
- Check for sensor drift
- Recalibrate sensors

**Noisy behavior**:
- Reduce Kd values
- Lower complementary filter alpha
- Check electrical connections

### Log Analysis
Monitor the ROS2 logs for:
```bash
ros2 topic echo /esp32/mpu6050/data
ros2 topic echo esp32/servo/angles
```

## Advanced Configuration

### Complementary Filter
Adjust the alpha parameter (0.98 default):
- Higher values (0.99): More gyroscope influence, less noise
- Lower values (0.95): More accelerometer influence, more responsive

### Control Loop Rate
Default: 50Hz (20ms)
- Increase for faster response
- Decrease if computational load is high

### Safety Limits
- Maximum correction: ±45° (adjustable)
- Servo limits: 0-180°
- Deadzone: 2° (adjustable via GUI)

## Hardware Integration

### ESP32 Side
Ensure your ESP32 publishes IMU data at consistent rate:
```cpp
// Example ESP32 code structure
void publishIMUData() {
  float data[7] = {ax, ay, az, gx, gy, gz, temp};
  // Publish to /esp32/mpu6050/data
}

void servoCallback(const std_msgs::Int32MultiArray& msg) {
  // Apply received servo angles
  for(int i = 0; i < 8; i++) {
    servo[i].write(msg.data[i]);
  }
}
```

## Contributing

Feel free to submit issues and enhancement requests!

### Development Setup
```bash
# Clone repository
git clone <your-repo>
cd quadruped_controller

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS2 community for excellent robotics framework
- MPU6050 library contributors
- Matplotlib for visualization capabilities
