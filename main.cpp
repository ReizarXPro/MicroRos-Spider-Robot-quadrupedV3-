#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

//  MPU6050 library
#include "MPU6050.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>

// Servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

// Servo configuration
#define NUM_SERVOS 8
const uint8_t SERVO_CHANNELS[NUM_SERVOS] = {0, 1, 2, 3, 4, 5, 6, 7}; // Adjust these to your actual pin connections
int servo_angles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90, 90, 90};     // Default starting positions

// Define second I2C bus pins for MPU6050
#define MPU_SDA_PIN 32
#define MPU_SCL_PIN 33

// HC-SR04 ultrasonic sensor pins
#define HCSR04_TRIG_PIN 25
#define HCSR04_ECHO_PIN 26

// Create a second TwoWire instance for MPU6050
TwoWire I2Ctwo = TwoWire(1);

// MPU6050 sensor using the second I2C bus
MPU6050 mpu(0x68, &I2Ctwo);
bool mpu_initialized = false;
// Data array for MPU readings [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp]
#define MPU_DATA_SIZE 7
float mpu_data[MPU_DATA_SIZE] = {0};

// HC-SR04 variables
float hcsr04_distance = 0.0;
bool hcsr04_initialized = false;

// Micro-ROS objects
rcl_subscription_t servo_array_subscriber;
rcl_publisher_t status_publisher;
rcl_publisher_t mpu_publisher;     // Publisher for MPU6050 data
rcl_publisher_t hcsr04_publisher;  // New publisher for HC-SR04 data
std_msgs__msg__Int32MultiArray servo_array_msg;
std_msgs__msg__Float32MultiArray mpu_array_msg;  // Message for MPU6050 data
std_msgs__msg__Float32 hcsr04_msg;               // Message for HC-SR04 data
std_msgs__msg__String status_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t mpu_timer;     // Timer for MPU readings
rcl_timer_t hcsr04_timer;  // Timer for HC-SR04 readings

// Network configuration
char ssid[] = "YOUR WIFI SSD";  
char password[] = "YOUR PASSWORD"; 
IPAddress agent_ip(192, 168, 1, 10); //Your Pc IP
uint16_t agent_port = 8888;

// Message buffers
char status_buffer[100];
int32_t servo_data_buffer[NUM_SERVOS];
float mpu_data_buffer[MPU_DATA_SIZE];

// Control timing
unsigned long last_wifi_check = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000; // Check WiFi every 5 seconds instead of every loop

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  Serial.println("Error detected, entering error loop");
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Calculate pulse width from angle
int pulseWidth(int angle) {
  int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  int analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// Set servo to specific angle - optimized for minimal latency
void setServoAngle(uint8_t servo_index, int angle) {
  if (servo_index >= NUM_SERVOS) return;
  
  // Constrain angle to valid range
  angle = constrain(angle, 0, 180);
  
  // Skip if no change
  if (servo_angles[servo_index] == angle) return;
  
  servo_angles[servo_index] = angle;
  
  // Set the servo position
  pwm.setPWM(SERVO_CHANNELS[servo_index], 0, pulseWidth(angle));
}

// Initialize the MPU6050 sensor
bool initMPU6050() {
  // Initialize the second I2C bus
  I2Ctwo.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  
  Serial.println("Initializing MPU6050 on secondary I2C bus...");
  
  // Initialize the MPU6050
  mpu.initialize();
  
  // Verify connection
  bool connected = mpu.testConnection();
  if (connected) {
    Serial.println("MPU6050 connection successful");
    
    // Configure the MPU6050 for desired operation
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // Set gyro range to ±250 deg/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // Set accel range to ±2g
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);               // Set digital low pass filter for smoother readings
    
    return true;
  } else {
    Serial.println("MPU6050 connection failed");
    return false;
  }
}

// Initialize the HC-SR04 sensor
bool initHCSR04() {
  Serial.println("Initializing HC-SR04 ultrasonic sensor...");
  
  // Set pin modes
  pinMode(HCSR04_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
  
  // Initialize trigger pin to LOW
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  
  Serial.println("HC-SR04 sensor initialized");
  return true;
}

// Read HC-SR04 distance
float readHCSR04Distance() {
  // Clear the trigger pin
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send a 10us pulse to trigger pin
  digitalWrite(HCSR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  
  // Read the echo pin and calculate distance
  unsigned long duration = pulseIn(HCSR04_ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  // Calculate distance in centimeters
  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = (duration * 0.0343) / 2 (divide by 2 for round trip)
  float distance = (duration * 0.0343) / 2.0;
  
  // Return -1 if timeout occurred (no echo received)
  if (duration == 0) {
    return -1.0;
  }
  
  // Limit distance to reasonable range (2cm to 400cm for HC-SR04)
  
  if (distance < 2.0 || distance > 400.0) {
    return -1.0;
  }
  
  return distance;
}

// Read MPU6050 data
void readMPU6050Data() {
  if (!mpu_initialized) return;
  
  int16_t ax, ay, az, gx, gy, gz, temp;
  
  // Read raw values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = mpu.getTemperature();
  
  // Convert to physical units
  // Acceleration in g (±2g range with 16-bit ADC gives ~0.000061 g/unit)
  mpu_data[0] = ax / 16384.0;  // X acceleration
  mpu_data[1] = ay / 16384.0;  // Y acceleration
  mpu_data[2] = az / 16384.0;  // Z acceleration
  
  // Gyro in degrees/second (±250 deg/s range with 16-bit ADC gives ~0.0076 deg/s/unit)
  mpu_data[3] = gx / 131.0;    // X gyro
  mpu_data[4] = gy / 131.0;    // Y gyro
  mpu_data[5] = gz / 131.0;    // Z gyro
  
  // Temperature in °C
  mpu_data[6] = temp / 340.0 + 36.53;  // Temperature formula from datasheet
}

// Single callback for all servo angles - optimized for speed
void servo_array_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  if (msg->data.size < NUM_SERVOS) {
    return; // Skip invalid messages to reduce latency (error reporting moved to periodic status)
  }
  
  // Update all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, msg->data.data[i]);
  }
}

// Timer callback for periodic status updates
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish periodic status update
    sprintf(status_buffer, "[%d,%d,%d,%d,%d,%d,%d,%d]", 
      servo_angles[0], servo_angles[1], servo_angles[2], servo_angles[3],
      servo_angles[4], servo_angles[5], servo_angles[6], servo_angles[7]);
    status_msg.data.data = status_buffer;
    status_msg.data.size = strlen(status_buffer);
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
  }
}

// Timer callback for MPU6050 readings
void mpu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && mpu_initialized) {
    // Read new MPU data
    readMPU6050Data();
    
    // Copy data to the message buffer
    for (int i = 0; i < MPU_DATA_SIZE; i++) {
      mpu_array_msg.data.data[i] = mpu_data[i];
    }
    
    // Publish MPU data
    RCSOFTCHECK(rcl_publish(&mpu_publisher, &mpu_array_msg, NULL));
  }
}

// Timer callback for HC-SR04 readings
void hcsr04_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && hcsr04_initialized) {
    // Read distance from HC-SR04
    hcsr04_distance = readHCSR04Distance();
    
    // Update message
    hcsr04_msg.data = hcsr04_distance;
    
    // Publish distance data
    RCSOFTCHECK(rcl_publish(&hcsr04_publisher, &hcsr04_msg, NULL));
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  
  // Set initial servo positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(SERVO_CHANNELS[i], 0, pulseWidth(servo_angles[i]));
  }
  
  // Initialize LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize MPU6050 on secondary I2C bus
  mpu_initialized = initMPU6050();
  
  // Initialize HC-SR04 sensor
  hcsr04_initialized = initHCSR04();
  
  Serial.println("Connecting to WiFi...");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  // Set WiFi to higher priority
  WiFi.setSleep(false); // Disable WiFi power-saving mode
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink while connecting
  }
  
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED after connection
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Configure transport
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  
  // Initialize allocator
  allocator = rcl_get_default_allocator();
  
  // Initialize support, node, etc.
  Serial.println("Initializing ROS 2 node...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_controller", "", &support));
  
  // Initialize message objects
  servo_array_msg.data.capacity = NUM_SERVOS;
  servo_array_msg.data.size = NUM_SERVOS;
  servo_array_msg.data.data = servo_data_buffer;
  
  mpu_array_msg.data.capacity = MPU_DATA_SIZE;
  mpu_array_msg.data.size = MPU_DATA_SIZE;
  mpu_array_msg.data.data = mpu_data_buffer;
  
  status_msg.data.capacity = sizeof(status_buffer);
  status_msg.data.data = status_buffer;
  
  // Create subscriber for servo angle array
  Serial.println("Creating subscribers...");
  RCCHECK(rclc_subscription_init_default(
    &servo_array_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "esp32/servo/angles"));
    
  // Create publisher for status messages
  Serial.println("Creating status publisher...");
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32/servo/status"));
  
  // Create publisher for MPU6050 data
  Serial.println("Creating MPU6050 publisher...");
  RCCHECK(rclc_publisher_init_default(
    &mpu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "esp32/mpu6050/data"));
  
  // Create publisher for HC-SR04 data
  Serial.println("Creating HC-SR04 publisher...");
  RCCHECK(rclc_publisher_init_default(
    &hcsr04_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "esp32/hcsr04/distance"));
  
  // Create timer for periodic status updates - increase frequency but still don't crowd the network
  Serial.println("Creating servo status timer...");
  const unsigned int timer_timeout = 500;  // 2Hz instead of 0.5Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Create timer for MPU6050 readings at a higher frequency
  Serial.println("Creating MPU6050 timer...");
  const unsigned int mpu_timer_timeout = 100;  // 10Hz
  RCCHECK(rclc_timer_init_default(
    &mpu_timer,
    &support,
    RCL_MS_TO_NS(mpu_timer_timeout),
    mpu_timer_callback));
  
  // Create timer for HC-SR04 readings
  Serial.println("Creating HC-SR04 timer...");
  const unsigned int hcsr04_timer_timeout = 200;  // 5Hz (not too frequent to avoid interference)
  RCCHECK(rclc_timer_init_default(
    &hcsr04_timer,
    &support,
    RCL_MS_TO_NS(hcsr04_timer_timeout),
    hcsr04_timer_callback));
  
  // Initialize executor with more callbacks
  Serial.println("Creating executor...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator)); // Now 4 callbacks
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_array_subscriber, &servo_array_msg, &servo_array_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &mpu_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &hcsr04_timer));
  
  // Publish initial status
  sprintf(status_buffer, "ESP32 %d-Servo Controller with MPU6050 and HC-SR04 started", NUM_SERVOS);
  status_msg.data.size = strlen(status_buffer);
  RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
  
  Serial.println("Micro-ROS setup completed");
  Serial.println("Pin assignments:");
  Serial.println("  MPU6050: SDA=32, SCL=33");
  Serial.println("  HC-SR04: TRIG=25, ECHO=26");
  Serial.println("  PWM Servo Driver: SDA=21, SCL=22 (default I2C)");
}

void loop() {
  // Less frequent WiFi checks to reduce interruptions to servo control
  unsigned long current_time = millis();
  if (current_time - last_wifi_check > WIFI_CHECK_INTERVAL) {
    last_wifi_check = current_time;
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
      WiFi.begin(ssid, password);
      
      // Quick indicator flash but don't block for long
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  
  // Spin executor with shorter timeout for more responsive control
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); // 10ms instead of 100ms
}
