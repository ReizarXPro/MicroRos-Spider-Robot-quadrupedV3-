#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
import numpy as np
import math
import time
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

class ControlMode(Enum):
    FULL_AUTO = "full_auto"
    MIXED_CONTROL = "mixed_control" 
    MANUAL_PRIORITY = "manual_priority"
    EMERGENCY_AUTO = "emergency_auto"

class WalkingGait(Enum):
    TRIPOD = "tripod"  # Most stable - 3 legs down, 1 up
    TROT = "trot"      # Diagonal pairs
    WAVE = "wave"      # Sequential leg lifting
    CREEP = "creep"    # Very slow, stable gait

@dataclass
class ServoControl:
    manual_enabled: bool = False
    manual_angle: float = 90.0
    balance_weight: float = 1.0
    blend_mode: str = "additive"
    last_manual_time: float = 0.0
    is_lifted: bool = False  # Track if leg is lifted for walking

class AdvancedPIDController:
    """Enhanced PID Controller with adaptive parameters and anti-windup."""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=45.0, 
                 integral_limit=None, derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit or output_limit * 2
        self.derivative_filter = derivative_filter
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.last_time = time.time()
        
        # Adaptive parameters
        self.adaptive_enabled = True
        self.error_history = []
        self.max_history = 20
        
    def update(self, setpoint, current_value):
        """Enhanced PID update with adaptive behavior."""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.02
            
        error = setpoint - current_value
        
        # Store error history for adaptive control
        self.error_history.append(abs(error))
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self.integral
        
        # Derivative term with filtering
        if dt > 0:
            raw_derivative = (error - self.prev_error) / dt
            derivative = self.derivative_filter * raw_derivative + (1 - self.derivative_filter) * self.prev_derivative
            self.prev_derivative = derivative
        else:
            derivative = 0
            
        d_term = self.kd * derivative
        
        # Adaptive gain adjustment
        if self.adaptive_enabled and len(self.error_history) >= 5:
            avg_error = sum(self.error_history[-5:]) / 5
            if avg_error > 10:  # High error - increase response
                adaptive_gain = 1.2
            elif avg_error < 2:  # Low error - smooth response
                adaptive_gain = 0.8
            else:
                adaptive_gain = 1.0
                
            p_term *= adaptive_gain
            
        output = p_term + i_term + d_term
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        self.prev_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.error_history = []
        self.last_time = time.time()

class QuadrupedBalanceController(Node):
    """
    Advanced Quadruped Balance Controller with sophisticated walking gaits and balance control
    """
    
    def __init__(self):
        super().__init__('enhanced_quadruped_balance_controller')
        
        # ROS2 Publishers and Subscribers
        self.imu_subscription = self.create_subscription(
            Float32MultiArray,
            '/esp32/mpu6050/data',
            self.imu_callback,
            10
        )
        
        self.servo_angles_pub = self.create_publisher(
            Int32MultiArray,
            'esp32/servo/angles',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'quadruped/status',
            10
        )
        
        # Robot configuration - CORRECTED based on your description
        self.num_servos = 8
        self.servo_names = [
            "Front Left Knee", "Front Left Hip",      # FL: servos 0,1
            "Front Right Knee", "Front Right Hip",    # FR: servos 2,3  
            "Rear Left Knee", "Rear Left Hip",        # RL: servos 4,5 (actually front in real robot)
            "Rear Right Knee", "Rear Right Hip"       # RR: servos 6,7
        ]
        
        # CORRECTED: RL and FL are front side of robot
        # Hip servos: 90° = center, <90° = inward, >90° = outward
        # Knee servos: FL,FR (0,2): 180° = up, 0° = down
        #              RL,RR (4,6): 0° = up, 180° = down
        self.leg_servos = {
            'FL': [0, 1],  # Front Left (knee, hip)
            'RL': [4, 5],  # Rear Left - actually front side
            'FR': [2, 3],  # Front Right  
            'RR': [6, 7]   # Rear Right
        }
        
        # Servo control objects
        self.servo_controls = [ServoControl() for _ in range(self.num_servos)]
        
        # Base positions for different stances - CORRECTED for servo orientations
        # The stand position is the real stand position in real robot so never change it 
        self.base_positions = {
            'stand': [180, 90, 0, 90, 0, 90, 180, 90],
            'crouch': [180, 120, 0, 120, 0, 120, 180, 120],
            'wide': [160, 80, 20, 80, 20, 80, 160, 80],
            'narrow': [180, 100, 20, 100, 20, 100, 180, 100]
        }
        self.current_base = 'stand'
        self.current_angles = self.base_positions[self.current_base].copy()
        
        # IMU data
        self.accel_x = self.accel_y = self.accel_z = 0.0
        self.gyro_x = self.gyro_y = self.gyro_z = 0.0
        self.temperature = 0.0
        
        # Calibration offsets
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 0.0]
        
        # Calculated angles
        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_rate = 0.0
        
        # Enhanced PID Controllers
        # Never change the pid value unless needed
        
        self.roll_pid = AdvancedPIDController(kp=0.5755, ki=0.0144, kd=0.0180, output_limit=35.0)
        self.pitch_pid = AdvancedPIDController(kp=0.5755, ki=0.0072, kd=0.0360, output_limit=35.0)
        
        # Control modes and parameters
        self.control_mode = ControlMode.MIXED_CONTROL
        self.balance_enabled = True
        self.emergency_threshold = 20.0  # degrees
        self.stability_threshold = 12.0  # degrees
        
        # Balance parameters
        self.deadzone = 1.5
        self.max_correction = 40
        self.compensation_gain = 1.8
        self.stability_margin = 5.0
        
        # Complementary filter
        self.alpha = 0.96
        self.dt = 0.02
        
        # Dynamic stability tracking
        self.stability_score = 100.0
        self.fall_risk_level = 0
        
        # Manual control timeout
        self.manual_timeout = 2.0
        
        # ENHANCED WALKING SYSTEM
        self.walking_enabled = False
        self.walking_direction = [0.0, 0.0]  # [forward/back, left/right]
        self.walking_speed = 1.0  # multiplier
        self.current_gait = WalkingGait.TRIPOD
        self.gait_phase = 0.0
        self.step_height = 25.0
        self.step_length = 15.0  # how far hips move during step
        self.leg_cycle_time = 2.0  # seconds per full cycle
        self.swing_duration = 0.25  # Default swing duration (will be set by gait)
        self.gait_cycle_phases = {}  # phase offset for each leg
        
        # Walking state tracking
        self.support_legs = set(['FL', 'RL', 'FR'])  # legs currently supporting
        self.swing_leg = 'RR'  # leg currently in swing phase
        
        # Initialize gait phases
        self.setup_gait_phases()
        
        # Control loop timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        # Initialize
        self.publish_servo_angles()
        self.get_logger().info('Enhanced Quadruped Balance Controller initialized')
        
    def setup_gait_phases(self):
        """Setup phase offsets for different gaits with improved timing."""
        if self.current_gait == WalkingGait.TRIPOD:
            # A more stable, overlapping tripod gait.
            # RR & FL work together, then RL & FR work together.
            self.gait_cycle_phases = {
                'RR': 0.0,
                'FL': 0.1,  # Lifts slightly after RR
                'RL': 0.5,
                'FR': 0.6   # Lifts slightly after RL
            }
            self.swing_duration = 0.4  # Allow longer swing time (40% of cycle)
        elif self.current_gait == WalkingGait.TROT:
            # Trot: diagonal pairs move together perfectly
            self.gait_cycle_phases = {
                'FL': 0.0, 'RR': 0.0,    # Diagonal pair 1
                'RL': 0.5, 'FR': 0.5     # Diagonal pair 2
            }
            self.swing_duration = 0.45 # Swing nearly half the time
        elif self.current_gait == WalkingGait.WAVE:
            # Wave: sequential around the body
            self.gait_cycle_phases = {
                'FL': 0.0,
                'RL': 0.25,
                'RR': 0.5,
                'FR': 0.75
            }
            self.swing_duration = 0.3
        elif self.current_gait == WalkingGait.CREEP:
            # Creep: very slow, always 3 legs down
            self.gait_cycle_phases = {
                'FL': 0.0,
                'FR': 0.25,
                'RR': 0.5,
                'RL': 0.75
            }
            self.swing_duration = 0.2            
    def imu_callback(self, msg):
        """Process IMU data with enhanced filtering."""
        if len(msg.data) >= 7:
            self.accel_x = msg.data[0] - self.accel_offset[0]
            self.accel_y = msg.data[1] - self.accel_offset[1]
            self.accel_z = msg.data[2] - self.accel_offset[2]
            self.gyro_x = msg.data[3] - self.gyro_offset[0]
            self.gyro_y = msg.data[4] - self.gyro_offset[1]
            self.gyro_z = msg.data[5] - self.gyro_offset[2]
            self.temperature = msg.data[6]
            
            self.calculate_orientation()
            self.update_stability_assessment()
            
    def calculate_orientation(self):
        """Enhanced orientation calculation with noise filtering."""
        # Calculate accelerometer angles
        accel_roll = math.atan2(self.accel_y, self.accel_z) * 180.0 / math.pi
        accel_pitch = math.atan2(-self.accel_x, 
                                math.sqrt(self.accel_y**2 + self.accel_z**2)) * 180.0 / math.pi
        
        # Gyroscope rates
        gyro_roll_rate = self.gyro_x * 180.0 / math.pi
        gyro_pitch_rate = self.gyro_y * 180.0 / math.pi
        self.yaw_rate = self.gyro_z * 180.0 / math.pi
        
        # Adaptive complementary filter
        accel_magnitude = math.sqrt(self.accel_x**2 + self.accel_y**2 + self.accel_z**2)
        accel_confidence = max(0.3, min(1.0, 1.0 - abs(accel_magnitude - 9.81) / 5.0))
        
        adaptive_alpha = self.alpha * accel_confidence
        
        # Apply complementary filter
        self.roll_angle = adaptive_alpha * (self.roll_angle + gyro_roll_rate * self.dt) + \
                         (1 - adaptive_alpha) * accel_roll
        self.pitch_angle = adaptive_alpha * (self.pitch_angle + gyro_pitch_rate * self.dt) + \
                          (1 - adaptive_alpha) * accel_pitch
        
        # Limit angles
        self.roll_angle = np.clip(self.roll_angle, -90, 90)
        self.pitch_angle = np.clip(self.pitch_angle, -90, 90)
        
    def update_stability_assessment(self):
        """Assess robot stability and fall risk with walking consideration."""
        # Calculate stability metrics
        tilt_magnitude = math.sqrt(self.roll_angle**2 + self.pitch_angle**2)
        angular_velocity = math.sqrt(self.gyro_x**2 + self.gyro_y**2 + self.gyro_z**2) * 180.0 / math.pi
        
        # Adjust stability thresholds when walking (more tolerant)
        emergency_thresh = self.emergency_threshold * (1.5 if self.walking_enabled else 1.0)
        stability_thresh = self.stability_threshold * (1.3 if self.walking_enabled else 1.0)
        
        # Update stability score (0-100)
        target_stability = max(0, 100 - tilt_magnitude * 1.5 - angular_velocity * 0.5)
        self.stability_score = 0.85 * self.stability_score + 0.15 * target_stability
        
        # Determine fall risk level
        if tilt_magnitude > emergency_thresh:
            self.fall_risk_level = 3  # Emergency
        elif tilt_magnitude > stability_thresh:
            self.fall_risk_level = 2  # Warning
        elif tilt_magnitude > stability_thresh * 0.6:
            self.fall_risk_level = 1  # Caution
        else:
            self.fall_risk_level = 0  # Safe
            
        # Auto-switch control mode based on stability
        if self.fall_risk_level >= 3:
            self.control_mode = ControlMode.EMERGENCY_AUTO
            # Emergency stop walking
            if self.walking_enabled:
                self.walking_enabled = False
                self.get_logger().warn("Emergency stop - walking disabled due to instability")
        elif self.control_mode == ControlMode.EMERGENCY_AUTO and self.fall_risk_level <= 1:
            self.control_mode = ControlMode.MIXED_CONTROL
            
    def get_walking_state(self) -> Dict[str, any]:
        """Get current walking state for each leg."""
        walking_state = {}
        
        if not self.walking_enabled:
            # When walking disabled, all legs are in support
            for leg_name in self.leg_servos.keys():
                walking_state[leg_name] = {
                    'phase': 0.0,
                    'is_swing': False,
                    'is_support': True
                }
            return walking_state
        
        for leg_name in self.leg_servos.keys():
            # Calculate current gait phase for this leg
            leg_phase_offset = self.gait_cycle_phases.get(leg_name, 0.0)
            current_leg_phase = (self.gait_phase + leg_phase_offset) % 1.0
            
            # Determine if leg is in swing (lifted) or support phase using variable swing duration
            is_swing = current_leg_phase < self.swing_duration
                
            walking_state[leg_name] = {
                'phase': current_leg_phase,
                'is_swing': is_swing,
                'is_support': not is_swing,
                'leg_phase_offset': leg_phase_offset,
                'global_phase': self.gait_phase,
                'swing_duration': self.swing_duration
            }
            
        return walking_state
        
    def calculate_walking_adjustments(self) -> Dict[str, Tuple[float, float]]:
        """Calculate hip and knee adjustments for walking gait."""
        adjustments = {}
        
        if not self.walking_enabled:
            return {leg: (0.0, 0.0) for leg in self.leg_servos.keys()}
            
        walking_state = self.get_walking_state()
        
        for leg_name, leg_state in walking_state.items():
            knee_idx, hip_idx = self.leg_servos[leg_name]
            phase = leg_state['phase']
            
            if leg_state['is_swing']:
                # Swing phase - lift leg and move forward
                if self.current_gait in [WalkingGait.TRIPOD, WalkingGait.CREEP]:
                    swing_progress = phase / 0.4  # swing is 40% of cycle
                else:
                    swing_progress = phase / 0.5  # swing is 50% of cycle
                    
                swing_progress = min(swing_progress, 1.0)
                
                # Knee adjustment - lift leg with CORRECT servo orientations
                # FL knee (servo 0): base=180° (down) -> DECREASE to lift (180° → 170°, 160°...)
                # FR knee (servo 2): base=0° (down) -> INCREASE to lift (0° → 10°, 20°...)
                # RL knee (servo 4): base=0° (down) -> INCREASE to lift (0° → 10°, 20°...)  
                # RR knee (servo 6): base=180° (down) -> DECREASE to lift (180° → 170°, 160°...)
                base_knee_angle = self.base_positions[self.current_base][knee_idx]
                
                # Calculate lift amount using sine wave for smooth motion
                lift_amount = self.step_height * math.sin(swing_progress * math.pi)
                
                if knee_idx in [0, 6]:  # FL(0), RR(6) knees: base=180°, lift by DECREASING
                    knee_adjustment = -lift_amount  # Negative = decrease angle = lift up
                else:  # FR(2), RL(4) knees: base=0°, lift by INCREASING
                    knee_adjustment = lift_amount   # Positive = increase angle = lift up
                    
                # Debug output to verify calculations  
                if swing_progress > 0.1:
                    target_angle = base_knee_angle + knee_adjustment
                    self.get_logger().info(
                        f"{leg_name} SWING: knee_idx={knee_idx}, base={base_knee_angle}°, "
                        f"adjustment={knee_adjustment:.1f}°, target={target_angle:.1f}°, progress={swing_progress:.2f}",
                        throttle_duration_sec=1.0
                    )
                    
                # Hip adjustment - move leg forward/back and left/right based on direction
                forward_component = self.walking_direction[0] * self.step_length
                lateral_component = self.walking_direction[1] * self.step_length
                
                # Flip forward direction for left legs based on real robot hip orientations
                if leg_name in ['FL', 'RL']:
                    forward_component *= -1.0
                
                # Apply direction based on leg position
                if leg_name in ['FL', 'RL']:  # Left legs
                    hip_adjustment = forward_component - lateral_component
                else:  # Right legs
                    hip_adjustment = forward_component + lateral_component
                    
                # Scale by swing progress (move most at mid-swing)
                hip_adjustment *= math.sin(swing_progress * math.pi) * 0.8
                
            else:
                # Support phase - keep leg down, slight compensation for body movement
                knee_adjustment = 0.0
                
                # Calculate support phase progress
                support_phase = phase - self.swing_duration
                if support_phase < 0:
                    support_phase += 1.0
                support_duration = 1.0 - self.swing_duration
                support_progress = support_phase / support_duration
                
                # Gradual hip movement to push body forward during support
                forward_component = self.walking_direction[0] * self.step_length * 0.2
                lateral_component = self.walking_direction[1] * self.step_length * 0.2
                
                # Flip forward direction for left legs
                if leg_name in ['FL', 'RL']:
                    forward_component *= -1.0
                
                # Apply smooth support movement
                support_movement = math.sin(support_progress * math.pi) * 0.5  # Gentle push
                
                # Compute temp as in swing
                if leg_name in ['FL', 'RL']:
                    temp = forward_component - lateral_component
                else:
                    temp = forward_component + lateral_component
                
                # Opposite for support phase
                hip_adjustment = -temp * support_movement
                    
            # Mark servo as lifted for balance compensation
            self.servo_controls[knee_idx].is_lifted = leg_state['is_swing']
            
            adjustments[leg_name] = (knee_adjustment, hip_adjustment)
            
        return adjustments
        
    def calculate_balance_correction(self):
        """Calculate servo corrections using the proven balance logic from working controller."""
        corrections = [0] * self.num_servos
        
        if not self.balance_enabled:
            return corrections
            
        # Apply deadzone
        roll_input = self.roll_angle if abs(self.roll_angle) > self.deadzone else 0
        pitch_input = self.pitch_angle if abs(self.pitch_angle) > self.deadzone else 0
        
        # Get PID outputs (setpoint is 0 for level)
        roll_correction = self.roll_pid.update(0, roll_input)
        pitch_correction = self.pitch_pid.update(0, pitch_input)
        
        # Get walking state to identify which legs are lifted
        walking_state = self.get_walking_state() if self.walking_enabled else {}
        support_legs = [leg for leg, state in walking_state.items() if state.get('is_support', True)]
        
        # Enhance correction for remaining support legs if some legs are lifted
        if len(support_legs) < 4:
            correction_boost = 4.0 / max(len(support_legs), 2)
            roll_correction *= correction_boost
            pitch_correction *= correction_boost
        
        # Apply roll correction to hip servos (using proven mapping from working controller)
        # Skip correction for lifted legs during walking
        if not (self.walking_enabled and walking_state.get('FR', {}).get('is_swing', False)):
            corrections[3] += roll_correction   # Front Right Hip (servo 3)
        if not (self.walking_enabled and walking_state.get('RR', {}).get('is_swing', False)):
            corrections[7] += roll_correction   # Rear Right Hip (servo 7)
        if not (self.walking_enabled and walking_state.get('FL', {}).get('is_swing', False)):
            corrections[1] -= roll_correction   # Front Left Hip (servo 1)
        if not (self.walking_enabled and walking_state.get('RL', {}).get('is_swing', False)):
            corrections[5] -= roll_correction   # Rear Left Hip (servo 5)
        
        # Apply pitch correction to hip servos
        if not (self.walking_enabled and walking_state.get('FR', {}).get('is_swing', False)):
            corrections[3] += pitch_correction  # Front Right Hip (servo 3)
        if not (self.walking_enabled and walking_state.get('FL', {}).get('is_swing', False)):
            corrections[1] += pitch_correction  # Front Left Hip (servo 1)
        if not (self.walking_enabled and walking_state.get('RR', {}).get('is_swing', False)):
            corrections[7] -= pitch_correction  # Rear Right Hip (servo 7)
        if not (self.walking_enabled and walking_state.get('RL', {}).get('is_swing', False)):
            corrections[5] -= pitch_correction  # Rear Left Hip (servo 5)
        
        # Apply smaller corrections to knees for stability (only to support legs)
        knee_roll_correction = roll_correction * 0.3
        if not (self.walking_enabled and walking_state.get('FR', {}).get('is_swing', False)):
            corrections[2] += knee_roll_correction   # Front Right Knee (servo 2)
        if not (self.walking_enabled and walking_state.get('RR', {}).get('is_swing', False)):
            corrections[6] += knee_roll_correction   # Rear Right Knee (servo 6)
        if not (self.walking_enabled and walking_state.get('FL', {}).get('is_swing', False)):
            corrections[0] -= knee_roll_correction   # Front Left Knee (servo 0)
        if not (self.walking_enabled and walking_state.get('RL', {}).get('is_swing', False)):
            corrections[4] -= knee_roll_correction   # Rear Left Knee (servo 4)
        
        # Apply limits
        for i in range(self.num_servos):
            corrections[i] = np.clip(corrections[i], -self.max_correction, self.max_correction)
            
        return corrections
        
    def blend_servo_commands(self, servo_idx: int, balance_correction: float, walking_adjustment: float = 0.0) -> float:
        """Intelligently blend manual, balance, and walking commands for a servo."""
        ctrl = self.servo_controls[servo_idx]
        base_angle = self.base_positions[self.current_base][servo_idx]
        
        # Check manual timeout
        current_time = time.time()
        manual_expired = (current_time - ctrl.last_manual_time) > self.manual_timeout
        
        if manual_expired and ctrl.manual_enabled:
            ctrl.manual_enabled = False
            
        # Emergency mode overrides everything
        if self.control_mode == ControlMode.EMERGENCY_AUTO:
            return base_angle + balance_correction
            
        # Manual control active - check balance weight
        if ctrl.manual_enabled:
            if ctrl.balance_weight == 0.0:
                # Pure manual control - no walking or balance
                return ctrl.manual_angle
            else:
                # Manual with some balance/walking blend
                if self.walking_enabled and abs(walking_adjustment) > 0.1:
                    # Manual + walking + balance
                    if ctrl.blend_mode == "additive":
                        return ctrl.manual_angle + walking_adjustment + balance_correction * ctrl.balance_weight
                    elif ctrl.blend_mode == "weighted":
                        manual_component = ctrl.manual_angle * (1.0 - ctrl.balance_weight)
                        auto_component = (base_angle + walking_adjustment + balance_correction) * ctrl.balance_weight
                        return manual_component + auto_component
                else:
                    # Manual + balance only
                    if ctrl.blend_mode == "additive":
                        return ctrl.manual_angle + balance_correction * ctrl.balance_weight
                    elif ctrl.blend_mode == "weighted":
                        manual_component = ctrl.manual_angle * (1.0 - ctrl.balance_weight)
                        balance_component = (base_angle + balance_correction) * ctrl.balance_weight
                        return manual_component + balance_component
        
        # Automatic control (no manual override)
        if self.walking_enabled:
            # Walking + balance - THIS IS THE KEY PATH FOR GAIT MOVEMENT
            final_angle = base_angle + walking_adjustment + balance_correction
            
            # Debug significant walking adjustments
            if abs(walking_adjustment) > 1.0:
                self.get_logger().info(
                    f"Servo {servo_idx}: base={base_angle}° + walk={walking_adjustment:.1f}° + balance={balance_correction:.1f}° = {final_angle:.1f}°",
                    throttle_duration_sec=2.0
                )
            
            return final_angle
        else:
            # Balance only (static)
            return base_angle + balance_correction
        
    def control_loop(self):
        """Enhanced control loop with walking and balance integration."""
        # Update gait phase
        if self.walking_enabled:
            phase_increment = (self.dt / self.leg_cycle_time) * self.walking_speed
            self.gait_phase += phase_increment
            if self.gait_phase >= 1.0:
                self.gait_phase -= 1.0
        
        # Calculate corrections and adjustments using proven balance logic
        balance_corrections = self.calculate_balance_correction()
        walking_adjustments = self.calculate_walking_adjustments()
        
        # CRITICAL: Update servo lifting states BEFORE blending commands
        if self.walking_enabled:
            walking_state = self.get_walking_state()
            for leg_name, leg_data in walking_state.items():
                knee_idx, hip_idx = self.leg_servos[leg_name]
                # Mark servos as lifted if in swing phase
                self.servo_controls[knee_idx].is_lifted = leg_data['is_swing']
                self.servo_controls[hip_idx].is_lifted = leg_data['is_swing']
        else:
            # Reset all lifting states when not walking
            for ctrl in self.servo_controls:
                ctrl.is_lifted = False
        
        # Apply to each servo
        for i in range(self.num_servos):
            # Find which leg this servo belongs to and get walking adjustment
            leg_name = None
            walking_adj = 0.0
            
            for leg, indices in self.leg_servos.items():
                if i in indices:
                    leg_name = leg
                    knee_idx, hip_idx = indices
                    knee_adj, hip_adj = walking_adjustments.get(leg_name, (0.0, 0.0))
                    
                    if i == knee_idx:
                        walking_adj = knee_adj
                        # Debug: Log significant knee adjustments
                        if abs(knee_adj) > 1.0 and self.walking_enabled:
                            self.get_logger().info(f"Applying knee adjustment: {leg_name} servo{i} = {knee_adj:.1f}°", throttle_duration_sec=2.0)
                    elif i == hip_idx:
                        walking_adj = hip_adj
                    break
                    
            # Blend all commands (manual + balance + walking)
            blended_angle = self.blend_servo_commands(i, balance_corrections[i], walking_adj)
            self.current_angles[i] = int(np.clip(blended_angle, 0, 180))
            
        self.publish_servo_angles()
        
    def publish_servo_angles(self):
        """Publish servo angles to ESP32."""
        msg = Int32MultiArray()
        msg.data = self.current_angles
        self.servo_angles_pub.publish(msg)
        
    def publish_status(self):
        """Publish detailed status information."""
        walking_state = self.get_walking_state() if self.walking_enabled else {}
        
        status_data = {
            'roll': round(self.roll_angle, 2),
            'pitch': round(self.pitch_angle, 2),
            'yaw_rate': round(self.yaw_rate, 2),
            'stability_score': round(self.stability_score, 1),
            'fall_risk': self.fall_risk_level,
            'control_mode': self.control_mode.value,
            'balance_enabled': self.balance_enabled,
            'walking_enabled': self.walking_enabled,
            'walking_direction': self.walking_direction,
            'gait_type': self.current_gait.value,
            'temperature': round(self.temperature, 1),
            'walking_state': walking_state
        }
        
        msg = String()
        msg.data = str(status_data)
        self.status_pub.publish(msg)
        
    # --- Public Interface Methods ---
    
    def set_walking_direction(self, forward: float, lateral: float):
        """Set walking direction. forward: +1=forward, -1=backward. lateral: +1=right, -1=left."""
        self.walking_direction = [np.clip(forward, -1.0, 1.0), np.clip(lateral, -1.0, 1.0)]
        self.get_logger().info(f"Walking direction set to: forward={forward:.2f}, lateral={lateral:.2f}")
        
    def set_walking_parameters(self, speed: float = 1.0, step_height: float = 25.0, 
                              step_length: float = 15.0, cycle_time: float = 2.0):
        """Set walking parameters."""
        self.walking_speed = np.clip(speed, 0.1, 3.0)
        self.step_height = np.clip(step_height, 5.0, 90.0)
        self.step_length = np.clip(step_length, 5.0, 90.0)
        self.leg_cycle_time = np.clip(cycle_time, 0.5, 5.0)
        self.get_logger().info(f"Walking parameters updated: speed={speed}, height={step_height}°, length={step_length}°, cycle={cycle_time}s")
        
    def set_gait_type(self, gait: str):
        """Change gait type."""
        try:
            self.current_gait = WalkingGait(gait)
            self.setup_gait_phases()
            self.get_logger().info(f"Gait changed to: {gait}")
        except ValueError:
            self.get_logger().warn(f"Invalid gait type: {gait}")
            
    def enable_walking(self, enable: bool):
        """Enable/disable walking."""
        if enable and self.fall_risk_level >= 2:
            self.get_logger().warn("Cannot enable walking - robot is unstable")
            return False
            
        self.walking_enabled = enable
        
        if enable:
            self.gait_phase = 0.0  # Reset gait cycle
            self.get_logger().info("Walking enabled")
        else:
            # Reset walking adjustments
            self.walking_direction = [0.0, 0.0]
            self.get_logger().info("Walking disabled")
            
        return True
        
    def move_forward(self, speed: float = 1.0):
        """Move robot forward."""
        if self.enable_walking(True):
            self.set_walking_direction(speed, 0.0)
            
    def move_backward(self, speed: float = 1.0):
        """Move robot backward."""
        if self.enable_walking(True):
            self.set_walking_direction(-speed, 0.0)
            
    def move_left(self, speed: float = 1.0):
        """Move robot left."""
        if self.enable_walking(True):
            self.set_walking_direction(0.0, -speed)
            
    def move_right(self, speed: float = 1.0):
        """Move robot right."""
        if self.enable_walking(True):
            self.set_walking_direction(0.0, speed)
            
    def stop_walking(self):
        """Stop walking and return to balance mode."""
        self.enable_walking(False)
        
    def set_pid_parameters(self, axis, kp, ki, kd):
        """Set PID parameters for a given axis ('roll' or 'pitch')."""
        if axis == 'roll':
            self.roll_pid.kp = kp
            self.roll_pid.ki = ki
            self.roll_pid.kd = kd
            self.get_logger().info(f"Roll PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
        elif axis == 'pitch':
            self.pitch_pid.kp = kp
            self.pitch_pid.ki = ki
            self.pitch_pid.kd = kd
            self.get_logger().info(f"Pitch PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
        else:
            self.get_logger().warn(f"Invalid PID axis: {axis}")

    def set_compensation_parameters(self, gain, max_compensation):
        """Set balance compensation parameters."""
        self.compensation_gain = gain
        self.max_correction = max_compensation
        self.get_logger().info(f"Compensation updated: Gain={gain}, Max Correction={max_compensation}°")
        
    def set_balance_enabled(self, enabled: bool):
        """Enable or disable the balance system."""
        self.balance_enabled = enabled
        if not enabled:
            # Reset PID controllers when disabling to prevent integral windup
            self.roll_pid.reset()
            self.pitch_pid.reset()
        self.get_logger().info(f"Balance system {'enabled' if enabled else 'disabled'}")
        
    def set_manual_servo_control(self, servo_idx: int, angle: float, 
                               balance_weight: float = 0.0, 
                               blend_mode: str = "additive"):
        """Set manual control for a specific servo with advanced options."""
        if 0 <= servo_idx < self.num_servos:
            ctrl = self.servo_controls[servo_idx]
            ctrl.manual_enabled = True
            ctrl.manual_angle = np.clip(angle, 0, 180)
            ctrl.balance_weight = np.clip(balance_weight, 0.0, 1.0)
            ctrl.blend_mode = blend_mode
            ctrl.last_manual_time = time.time()
            
            self.get_logger().info(
                f'Manual control set for servo {servo_idx} ({self.servo_names[servo_idx]}): '
                f'angle={angle}, weight={balance_weight}, mode={blend_mode}'
            )
            
    def disable_manual_servo_control(self, servo_idx: int):
        """Disable manual control for a specific servo."""
        if 0 <= servo_idx < self.num_servos:
            self.servo_controls[servo_idx].manual_enabled = False
            self.get_logger().info(f'Manual control disabled for servo {servo_idx}')
            
    def set_leg_manual_control(self, leg_name: str, knee_angle: float, hip_angle: float,
                             balance_weight: float = 0.0):
        """Set manual control for an entire leg."""
        if leg_name in self.leg_servos:
            knee_idx, hip_idx = self.leg_servos[leg_name]
            self.set_manual_servo_control(knee_idx, knee_angle, balance_weight)
            self.set_manual_servo_control(hip_idx, hip_angle, balance_weight)
            
    def set_control_mode(self, mode: str):
        """Set overall control mode."""
        try:
            self.control_mode = ControlMode(mode)
            self.get_logger().info(f'Control mode set to: {mode}')
        except ValueError:
            self.get_logger().warn(f'Invalid control mode: {mode}')
            
    def set_base_stance(self, stance: str):
        """Change base stance."""
        if stance in self.base_positions:
            self.current_base = stance
            # When changing stance, disable all manual servo control to reflect the new base pose
            for i in range(self.num_servos):
                self.disable_manual_servo_control(i)
            self.get_logger().info(f'Base stance changed to: {stance}')
        else:
            self.get_logger().warn(f'Unknown stance: {stance}')
            
    def calibrate_sensors(self, samples: int = 200):
        """Enhanced sensor calibration."""
        self.get_logger().info(f'Starting sensor calibration with {samples} samples...')
        
        # Temporarily disable walking during calibration
        was_walking = self.walking_enabled
        if was_walking:
            self.enable_walking(False)
        
        gyro_sum = [0.0, 0.0, 0.0]
        accel_sum = [0.0, 0.0, 0.0]
        
        # Collect calibration data
        for _ in range(samples):
            gyro_sum[0] += self.gyro_x + self.gyro_offset[0]
            gyro_sum[1] += self.gyro_y + self.gyro_offset[1] 
            gyro_sum[2] += self.gyro_z + self.gyro_offset[2]
            accel_sum[0] += self.accel_x + self.accel_offset[0]
            accel_sum[1] += self.accel_y + self.accel_offset[1]
            accel_sum[2] += self.accel_z + self.accel_offset[2]
            time.sleep(0.005)
            
        self.gyro_offset = [s / samples for s in gyro_sum]
        self.accel_offset[0] = accel_sum[0] / samples
        self.accel_offset[1] = accel_sum[1] / samples
        # Don't offset Z-axis completely as it includes gravity
        
        # Reset PID controllers and orientation after calibration
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        
        # Restore walking state
        if was_walking:
            self.enable_walking(True)
        
        self.get_logger().info('Sensor calibration complete')
            
    def debug_specific_legs(self, leg_names=['FL', 'FR']):
        """Debug specific legs during walking to see why they're not lifting."""
        if not self.walking_enabled:
            self.get_logger().info("Walking is disabled - enable walking first")
            return
            
        walking_state = self.get_walking_state()
        walking_adjustments = self.calculate_walking_adjustments()
        
        self.get_logger().info(f"=== DEBUGGING LEGS: {leg_names} ===")
        self.get_logger().info(f"Global phase: {self.gait_phase:.3f}, Gait: {self.current_gait.value}")
        self.get_logger().info(f"Step height: {self.step_height}°, Cycle time: {self.leg_cycle_time}s")
        
        for leg_name in leg_names:
            if leg_name not in self.leg_servos:
                continue
                
            leg_state = walking_state[leg_name]
            knee_idx, hip_idx = self.leg_servos[leg_name]
            knee_adj, hip_adj = walking_adjustments.get(leg_name, (0.0, 0.0))
            base_knee = self.base_positions[self.current_base][knee_idx]
            current_knee = self.current_angles[knee_idx]
            
            self.get_logger().info(
                f"{leg_name}: Phase={leg_state['phase']:.3f}, IsSwing={leg_state['is_swing']}, "
                f"KneeIdx={knee_idx}, BaseAngle={base_knee}°, CurrentAngle={current_knee}°, "
                f"KneeAdjustment={knee_adj:.1f}°, IsLifted={self.servo_controls[knee_idx].is_lifted}"
            )

    def get_full_status(self) -> Dict:
        """Get comprehensive status information."""
        walking_state = self.get_walking_state() if self.walking_enabled else {}
        
        return {
            'orientation': {
                'roll': self.roll_angle,
                'pitch': self.pitch_angle,
                'yaw_rate': self.yaw_rate
            },
            'stability': {
                'score': self.stability_score,
                'fall_risk': self.fall_risk_level,
                'emergency_active': self.control_mode == ControlMode.EMERGENCY_AUTO
            },
            'control': {
                'mode': self.control_mode.value,
                'balance_enabled': self.balance_enabled,
                'walking_enabled': self.walking_enabled,
                'base_stance': self.current_base
            },
            'walking': {
                'direction': self.walking_direction,
                'speed': self.walking_speed,
                'gait': self.current_gait.value,
                'phase': self.gait_phase,
                'step_height': self.step_height,
                'step_length': self.step_length,
                'cycle_time': self.leg_cycle_time,
                'leg_states': walking_state
            },
            'servos': {
                'angles': self.current_angles.copy(),
                'manual_status': [ctrl.manual_enabled for ctrl in self.servo_controls],
                'balance_weights': [ctrl.balance_weight for ctrl in self.servo_controls],
                'lifted_status': [ctrl.is_lifted for ctrl in self.servo_controls]
            },
            'sensors': {
                'temperature': self.temperature,
                'accel': [self.accel_x, self.accel_y, self.accel_z],
                'gyro': [self.gyro_x, self.gyro_y, self.gyro_z]
            }
        }

    def test_leg_movement(self, leg_name: str = None):
        """Test leg lifting for debugging. If no leg specified, tests all legs sequentially."""
        if leg_name:
            legs_to_test = [leg_name]
        else:
            legs_to_test = ['FL', 'RL', 'FR', 'RR']
        
        original_walking = self.walking_enabled
        self.walking_enabled = False  # Disable walking during test
        
        self.get_logger().info("Starting leg movement test...")
        
        for leg in legs_to_test:
            if leg not in self.leg_servos:
                self.get_logger().warn(f"Invalid leg name: {leg}")
                continue
                
            knee_idx, hip_idx = self.leg_servos[leg]
            base_knee = self.base_positions[self.current_base][knee_idx]
            base_hip = self.base_positions[self.current_base][hip_idx]
            
            self.get_logger().info(f"Testing {leg} leg (knee servo {knee_idx}, hip servo {hip_idx})")
            
            # Lift leg based on CORRECT servo orientations
            if knee_idx in [0, 6]:  # FL(0), RR(6): base=180° (down), lift by DECREASING
                lifted_knee = max(0, base_knee - 40)  # Decrease angle to lift
            else:  # FR(2), RL(4): base=0° (down), lift by INCREASING
                lifted_knee = min(180, base_knee + 40)  # Increase angle to lift
                
            self.current_angles[knee_idx] = lifted_knee
            self.publish_servo_angles()
            self.get_logger().info(f"{leg} lifted: knee={lifted_knee}°")
            
            time.sleep(1.0)  # Hold lifted position
            
            # Lower leg back down
            self.current_angles[knee_idx] = base_knee
            self.publish_servo_angles()
            self.get_logger().info(f"{leg} lowered: knee={base_knee}°")
            
            time.sleep(1.0)  # Pause between legs
        
        self.walking_enabled = original_walking
        self.get_logger().info("Leg movement test completed")

    def verify_servo_orientations(self):
        """Verify and log the CORRECT servo orientations for each leg."""
        self.get_logger().info("=== CORRECT SERVO ORIENTATION VERIFICATION ===")
        
        for leg_name, (knee_idx, hip_idx) in self.leg_servos.items():
            base_knee = self.base_positions[self.current_base][knee_idx]
            base_hip = self.base_positions[self.current_base][hip_idx]
            
            # Determine lift direction based on base position
            if knee_idx in [0, 6]:  # FL, RR
                lift_direction = f"DECREASE from {base_knee}° (down) to lift up (e.g., 180° → 150°)"
            else:  # FR, RL
                lift_direction = f"INCREASE from {base_knee}° (down) to lift up (e.g., 0° → 30°)"
                
            self.get_logger().info(
                f"{leg_name}: Knee servo {knee_idx} - {lift_direction}, "
                f"Hip servo {hip_idx} at {base_hip}°"
            )
        
        self.get_logger().info("=== END VERIFICATION ===")

    def test_fl_fr_walking(self):
        """Specific test for FL and FR legs during walking."""
        self.get_logger().info("=== Testing FL and FR walking movement ===")
        
        # Enable walking in tripod gait
        self.enable_walking(True)
        self.set_gait_type('tripod')
        self.set_walking_direction(0.0, 0.0)  # No movement, just gait
        
        # Monitor for 10 seconds
        start_time = time.time()
        while time.time() - start_time < 10.0:
            self.debug_specific_legs(['FL', 'FR'])
            time.sleep(1.0)
            
        self.enable_walking(False)
        self.get_logger().info("=== FL/FR walking test completed ===")

def main(args=None):
    rclpy.init(args=args)
    
    # For standalone testing
    controller = QuadrupedBalanceController()
    
    try:
        # Example usage for testing:
        # controller.test_leg_movement()  # Test all legs
        # controller.test_leg_movement('FR')  # Test specific leg
        # controller.debug_walking_state()  # Debug walking
        # controller.enable_walking(True)
        # controller.move_forward(0.8)
        
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down enhanced balance controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
