#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk, font
import time
import math
import numpy as np
from collections import deque
import threading

class GyroscopeVisualizationGUI(Node):
    """
    MPU6050 Gyroscope and Accelerometer Visualization with Dark Theme
    """
    def __init__(self):
        super().__init__('mpu6050_visualization')
        
        # Subscribe to MPU6050 data
        self.mpu_subscription = self.create_subscription(
            Float32MultiArray,
            'esp32/mpu6050/data',
            self.mpu_data_callback,
            10
        )
        
        # MPU data array [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp]
        self.mpu_data = [0.0] * 7
        self.prev_update_time = time.time()
        
        # For orientation tracking
        self.orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] in degrees
        self.last_gyro = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]  # Calibration offset
        
        # For graphing
        self.max_data_points = 100
        self.accel_data_x = deque(maxlen=self.max_data_points)
        self.accel_data_y = deque(maxlen=self.max_data_points)
        self.accel_data_z = deque(maxlen=self.max_data_points)
        self.gyro_data_x = deque(maxlen=self.max_data_points)
        self.gyro_data_y = deque(maxlen=self.max_data_points)
        self.gyro_data_z = deque(maxlen=self.max_data_points)
        self.temp_data = deque(maxlen=self.max_data_points)
        self.time_points = deque(maxlen=self.max_data_points)
        
        # Fill with initial zeros
        for i in range(self.max_data_points):
            self.accel_data_x.append(0)
            self.accel_data_y.append(0)
            self.accel_data_z.append(0)
            self.gyro_data_x.append(0)
            self.gyro_data_y.append(0)
            self.gyro_data_z.append(0)
            self.temp_data.append(0)
            self.time_points.append(i)
        
        # GUI elements
        self.create_dark_gui()
        
        # Calibration flag
        self.is_calibrating = False
        self.calibration_samples = []
        self.max_calibration_samples = 100
        
        # Log startup
        self.get_logger().info('MPU6050 Visualization GUI initialized')
        self.log_status("System initialized. Waiting for MPU6050 data...")
        
    def mpu_data_callback(self, msg):
        """Process incoming MPU6050 data"""
        if len(msg.data) != 7:
            self.get_logger().warning(f'Unexpected data size: {len(msg.data)}')
            return
        
        # Store the data
        self.mpu_data = list(msg.data)
        
        # Get time for orientation calculation
        current_time = time.time()
        dt = current_time - self.prev_update_time
        self.prev_update_time = current_time
        
        # Process for calibration if active
        if self.is_calibrating and len(self.calibration_samples) < self.max_calibration_samples:
            self.calibration_samples.append([self.mpu_data[3], self.mpu_data[4], self.mpu_data[5]])
            
            # If we collected enough samples, calculate the offset
            if len(self.calibration_samples) >= self.max_calibration_samples:
                self.complete_calibration()
        
        # Update orientation using gyro data with offset correction
        gyro_x = self.mpu_data[3] - self.gyro_offset[0]
        gyro_y = self.mpu_data[4] - self.gyro_offset[1]
        gyro_z = self.mpu_data[5] - self.gyro_offset[2]
        
        # Simple integration for orientation (roll, pitch, yaw)
        self.orientation[0] += gyro_x * dt
        self.orientation[1] += gyro_y * dt
        self.orientation[2] += gyro_z * dt
        
        # Complementary filter with accelerometer for roll and pitch
        # Calculate roll and pitch from accelerometer
        accel_x = self.mpu_data[0]
        accel_y = self.mpu_data[1]
        accel_z = self.mpu_data[2]
        
        # Only use accelerometer data if it's not experiencing high G forces
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if 0.8 < accel_magnitude < 1.2:  # Close to 1g (stationary)
            # Calculate roll and pitch from accelerometer (in degrees)
            accel_roll = math.atan2(accel_y, accel_z) * 180.0 / math.pi
            accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180.0 / math.pi
            
            # Get filter weight from settings (if UI is initialized)
            filter_weight = 0.98
            if hasattr(self, 'filter_weight_var'):
                filter_weight = self.filter_weight_var.get()
            
            # Complementary filter (using configurable weight)
            self.orientation[0] = self.orientation[0] * filter_weight + accel_roll * (1.0 - filter_weight)
            self.orientation[1] = self.orientation[1] * filter_weight + accel_pitch * (1.0 - filter_weight)
        
        # Normalize yaw to 0-360
        self.orientation[2] = self.orientation[2] % 360
        
        # Save for graphs
        current_time = time.time()
        self.time_points.append(current_time)
        self.accel_data_x.append(accel_x)
        self.accel_data_y.append(accel_y)
        self.accel_data_z.append(accel_z)
        self.gyro_data_x.append(gyro_x)
        self.gyro_data_y.append(gyro_y)
        self.gyro_data_z.append(gyro_z)
        self.temp_data.append(self.mpu_data[6])
        
        # Update connection status
        self.connection_status.config(text="ESP32: CONNECTED", fg='#00FF00')
        
        # Update display values
        self.update_display()
        
        # Redraw 3D visualization
        self.draw_3d_visualization()
        
    def complete_calibration(self):
        """Complete the gyroscope calibration process"""
        if not self.calibration_samples:
            return
            
        # Calculate average offset
        samples = np.array(self.calibration_samples)
        avg_x = np.mean(samples[:, 0])
        avg_y = np.mean(samples[:, 1])
        avg_z = np.mean(samples[:, 2])
        
        # Set as offset
        self.gyro_offset = [avg_x, avg_y, avg_z]
        
        # Reset orientation
        self.orientation = [0.0, 0.0, 0.0]
        
        # Update UI
        self.is_calibrating = False
        self.calibration_btn.config(text="Calibrate Gyro")
        self.log_status(f"Calibration complete. Offset: X={avg_x:.2f}, Y={avg_y:.2f}, Z={avg_z:.2f}")
    
    def start_calibration(self):
        """Start or cancel gyroscope calibration"""
        if self.is_calibrating:
            # Cancel calibration
            self.is_calibrating = False
            self.calibration_samples = []
            self.calibration_btn.config(text="Calibrate Gyro")
            self.log_status("Calibration cancelled")
        else:
            # Start calibration
            self.is_calibrating = True
            self.calibration_samples = []
            self.calibration_btn.config(text="Cancel Calibration")
            self.log_status("Calibration started - keep device still")
    
    def reset_orientation(self):
        """Reset the orientation tracking"""
        self.orientation = [0.0, 0.0, 0.0]
        self.log_status("Orientation reset to zero")
    
    def create_dark_gui(self):
        """Create the professional dark-themed visualization panel"""
        self.root = tk.Tk()
        self.root.title("MPU6050 GYROSCOPE VISUALIZATION v1.0")
        self.root.geometry("1200x800")
        self.root.configure(bg='#121212')
        
        # Custom dark theme
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure colors
        style.configure('.', background='#121212', foreground='white')
        style.configure('TFrame', background='#121212')
        style.configure('TLabel', background='#121212', foreground='white')
        style.configure('TNotebook', background='#121212', borderwidth=0)
        style.configure('TNotebook.Tab', background='#252526', foreground='white', padding=[10,5])
        style.map('TNotebook.Tab', background=[('selected', '#007ACC')])
        style.configure('TScale', background='#252526')
        style.configure('TCheckbutton', background='#121212', foreground='white')
        style.configure('TButton', background='#252526', foreground='white', borderwidth=1)
        style.map('TButton', background=[('active', '#007ACC')])
        style.configure('TLabelFrame', background='#252526', foreground='white', borderwidth=2)
        
        # Custom fonts
        title_font = font.Font(family='Helvetica', size=16, weight='bold')
        header_font = font.Font(family='Helvetica', size=12, weight='bold')
        normal_font = font.Font(family='Helvetica', size=10)
        value_font = font.Font(family='Consolas', size=11, weight='bold')
        
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Header
        header_frame = ttk.Frame(main_frame)
        header_frame.pack(fill=tk.X, pady=(0,10))
        
        # Logo and title
        self.logo = tk.Label(header_frame, text="[MPU6050 SENSOR]", font=title_font, 
                             bg='#121212', fg='#007ACC')
        self.logo.pack(side=tk.LEFT)
        
        # Status indicators
        status_frame = ttk.Frame(header_frame)
        status_frame.pack(side=tk.RIGHT)
        
        self.ros_status_var = tk.StringVar(value="ROS: ONLINE")
        self.ros_status = tk.Label(status_frame, textvariable=self.ros_status_var, font=normal_font,
                                  bg='#121212', fg='#00FF00')
        self.ros_status.pack(side=tk.LEFT, padx=10)
        
        self.connection_status = tk.Label(status_frame, text="ESP32: WAITING", font=normal_font,
                                         bg='#121212', fg='#FFAA00')
        self.connection_status.pack(side=tk.LEFT)
        
        # Main notebook (tabs)
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # Visualization tab
        viz_tab = ttk.Frame(notebook)
        notebook.add(viz_tab, text="3D VISUALIZATION")
        
        # Left panel - 3D visualization
        left_panel = ttk.Frame(viz_tab)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 3D visualization canvas
        viz_frame = ttk.LabelFrame(left_panel, text="3D Orientation", padding=10)
        viz_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.canvas_3d = tk.Canvas(viz_frame, width=600, height=400, bg='#1E1E1E', 
                                  highlightthickness=0)
        self.canvas_3d.pack(fill=tk.BOTH, expand=True)
        
        # Button panel
        button_frame = ttk.Frame(left_panel)
        button_frame.pack(fill=tk.X, pady=5)
        
        self.calibration_btn = ttk.Button(button_frame, text="Calibrate Gyro", 
                                         command=self.start_calibration)
        self.calibration_btn.pack(side=tk.LEFT, padx=5)
        
        self.reset_btn = ttk.Button(button_frame, text="Reset Orientation", 
                                   command=self.reset_orientation)
        self.reset_btn.pack(side=tk.LEFT, padx=5)
        
        # Right panel - Data display
        right_panel = ttk.Frame(viz_tab, width=300)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=10, pady=10, expand=False)
        right_panel.pack_propagate(False)  # Prevent resizing to fit contents
        
        # Orientation display
        orientation_frame = ttk.LabelFrame(right_panel, text="Orientation", padding=10)
        orientation_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Roll display
        roll_frame = ttk.Frame(orientation_frame)
        roll_frame.pack(fill=tk.X, pady=5)
        ttk.Label(roll_frame, text="Roll:", width=8).pack(side=tk.LEFT)
        self.roll_var = tk.StringVar(value="0.00°")
        ttk.Label(roll_frame, textvariable=self.roll_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        self.roll_bar = ttk.Progressbar(roll_frame, orient=tk.HORIZONTAL, length=150, mode='determinate')
        self.roll_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Pitch display
        pitch_frame = ttk.Frame(orientation_frame)
        pitch_frame.pack(fill=tk.X, pady=5)
        ttk.Label(pitch_frame, text="Pitch:", width=8).pack(side=tk.LEFT)
        self.pitch_var = tk.StringVar(value="0.00°")
        ttk.Label(pitch_frame, textvariable=self.pitch_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        self.pitch_bar = ttk.Progressbar(pitch_frame, orient=tk.HORIZONTAL, length=150, mode='determinate')
        self.pitch_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Yaw display
        yaw_frame = ttk.Frame(orientation_frame)
        yaw_frame.pack(fill=tk.X, pady=5)
        ttk.Label(yaw_frame, text="Yaw:", width=8).pack(side=tk.LEFT)
        self.yaw_var = tk.StringVar(value="0.00°")
        ttk.Label(yaw_frame, textvariable=self.yaw_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        self.yaw_bar = ttk.Progressbar(yaw_frame, orient=tk.HORIZONTAL, length=150, mode='determinate')
        self.yaw_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Sensor readings
        readings_frame = ttk.LabelFrame(right_panel, text="Sensor Readings", padding=10)
        readings_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Accelerometer readings
        accel_frame = ttk.Frame(readings_frame)
        accel_frame.pack(fill=tk.X, pady=(5, 10))
        ttk.Label(accel_frame, text="Accelerometer:", font=header_font).pack(anchor=tk.W)
        
        accel_x_frame = ttk.Frame(accel_frame)
        accel_x_frame.pack(fill=tk.X, pady=2)
        ttk.Label(accel_x_frame, text="X:", width=3).pack(side=tk.LEFT)
        self.accel_x_var = tk.StringVar(value="0.00 g")
        ttk.Label(accel_x_frame, textvariable=self.accel_x_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        accel_y_frame = ttk.Frame(accel_frame)
        accel_y_frame.pack(fill=tk.X, pady=2)
        ttk.Label(accel_y_frame, text="Y:", width=3).pack(side=tk.LEFT)
        self.accel_y_var = tk.StringVar(value="0.00 g")
        ttk.Label(accel_y_frame, textvariable=self.accel_y_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        accel_z_frame = ttk.Frame(accel_frame)
        accel_z_frame.pack(fill=tk.X, pady=2)
        ttk.Label(accel_z_frame, text="Z:", width=3).pack(side=tk.LEFT)
        self.accel_z_var = tk.StringVar(value="0.00 g")
        ttk.Label(accel_z_frame, textvariable=self.accel_z_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        # Gyroscope readings
        gyro_frame = ttk.Frame(readings_frame)
        gyro_frame.pack(fill=tk.X, pady=(5, 10))
        ttk.Label(gyro_frame, text="Gyroscope:", font=header_font).pack(anchor=tk.W)
        
        gyro_x_frame = ttk.Frame(gyro_frame)
        gyro_x_frame.pack(fill=tk.X, pady=2)
        ttk.Label(gyro_x_frame, text="X:", width=3).pack(side=tk.LEFT)
        self.gyro_x_var = tk.StringVar(value="0.00 °/s")
        ttk.Label(gyro_x_frame, textvariable=self.gyro_x_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        gyro_y_frame = ttk.Frame(gyro_frame)
        gyro_y_frame.pack(fill=tk.X, pady=2)
        ttk.Label(gyro_y_frame, text="Y:", width=3).pack(side=tk.LEFT)
        self.gyro_y_var = tk.StringVar(value="0.00 °/s")
        ttk.Label(gyro_y_frame, textvariable=self.gyro_y_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        gyro_z_frame = ttk.Frame(gyro_frame)
        gyro_z_frame.pack(fill=tk.X, pady=2)
        ttk.Label(gyro_z_frame, text="Z:", width=3).pack(side=tk.LEFT)
        self.gyro_z_var = tk.StringVar(value="0.00 °/s")
        ttk.Label(gyro_z_frame, textvariable=self.gyro_z_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        # Temperature reading
        temp_frame = ttk.Frame(readings_frame)
        temp_frame.pack(fill=tk.X, pady=(5, 0))
        ttk.Label(temp_frame, text="Temperature:", font=header_font).pack(anchor=tk.W)
        
        temp_value_frame = ttk.Frame(temp_frame)
        temp_value_frame.pack(fill=tk.X, pady=2)
        self.temp_var = tk.StringVar(value="0.00 °C")
        ttk.Label(temp_value_frame, textvariable=self.temp_var, font=value_font, 
                 width=10).pack(side=tk.LEFT)
        
        # Status log
        log_frame = ttk.LabelFrame(right_panel, text="Status Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = tk.Text(log_frame, height=6, width=30, bg='#1E1E1E', fg='white',
                              font=normal_font, relief=tk.FLAT, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.config(state=tk.DISABLED)
        
        # Data tab
        data_tab = ttk.Frame(notebook)
        notebook.add(data_tab, text="DATA GRAPHS")
        
        # Accelerometer graph frame
        accel_graph_frame = ttk.LabelFrame(data_tab, text="Accelerometer Data")
        accel_graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.accel_canvas = tk.Canvas(accel_graph_frame, bg='#1E1E1E', highlightthickness=0)
        self.accel_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Gyroscope graph frame
        gyro_graph_frame = ttk.LabelFrame(data_tab, text="Gyroscope Data")
        gyro_graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.gyro_canvas = tk.Canvas(gyro_graph_frame, bg='#1E1E1E', highlightthickness=0)
        self.gyro_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Temperature graph frame
        temp_graph_frame = ttk.LabelFrame(data_tab, text="Temperature Data")
        temp_graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.temp_canvas = tk.Canvas(temp_graph_frame, bg='#1E1E1E', highlightthickness=0)
        self.temp_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Settings tab
        settings_tab = ttk.Frame(notebook)
        notebook.add(settings_tab, text="SETTINGS")
        
        # Settings container
        settings_container = ttk.Frame(settings_tab)
        settings_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Graph settings
        graph_settings_frame = ttk.LabelFrame(settings_container, text="Graph Settings", padding=10)
        graph_settings_frame.pack(fill=tk.X, pady=(0, 15))
        
        # Data points slider
        data_points_frame = ttk.Frame(graph_settings_frame)
        data_points_frame.pack(fill=tk.X, pady=5)
        ttk.Label(data_points_frame, text="Data Points:").pack(side=tk.LEFT, padx=5)
        
        self.data_points_var = tk.IntVar(value=self.max_data_points)
        data_points_scale = ttk.Scale(data_points_frame, from_=10, to=500, 
                                     variable=self.data_points_var, orient=tk.HORIZONTAL,
                                     command=self.update_data_points)
        data_points_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        data_points_label = ttk.Label(data_points_frame, textvariable=self.data_points_var, width=4)
        data_points_label.pack(side=tk.LEFT, padx=5)
        
        # Visualization settings
        viz_settings_frame = ttk.LabelFrame(settings_container, text="Visualization Settings", padding=10)
        viz_settings_frame.pack(fill=tk.X, pady=(0, 15))
        
        # Complementary filter slider
        filter_frame = ttk.Frame(viz_settings_frame)
        filter_frame.pack(fill=tk.X, pady=5)
        ttk.Label(filter_frame, text="Gyro Filter Weight:").pack(side=tk.LEFT, padx=5)
        
        self.filter_weight_var = tk.DoubleVar(value=0.98)
        filter_scale = ttk.Scale(filter_frame, from_=0.8, to=1.0, 
                               variable=self.filter_weight_var, orient=tk.HORIZONTAL)
        filter_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.filter_weight_label = ttk.Label(filter_frame, text="0.98")
        self.filter_weight_label.pack(side=tk.LEFT, padx=5)
        
        # Update filter weight label when slider changes
        def update_filter_label(*args):
            self.filter_weight_label.config(text=f"{self.filter_weight_var.get():.2f}")
        
        self.filter_weight_var.trace("w", update_filter_label)
        
        # About section
        about_frame = ttk.LabelFrame(settings_container, text="About", padding=10)
        about_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 15))
        
        about_text = """MPU6050 Gyroscope Visualization
Version 1.0

This application visualizes data from an MPU6050 IMU sensor 
connected via an ESP32 microcontroller using ROS 2 communication.

Features:
• 3D orientation visualization
• Real-time sensor data display
• Sensor calibration
• Data graphing

© 2025 ReizarX Robotics"""
        
        about_label = ttk.Label(about_frame, text=about_text, justify=tk.LEFT, wraplength=500)
        about_label.pack(padx=10, pady=10)
        
        # Setup the update loop
        self.root.after(100, self.periodic_update)
        
    def log_status(self, message):
        """Add a message to the status log"""
        self.log_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        
    def periodic_update(self):
        """Periodic update function for UI elements"""
        # Check ROS connection
        if rclpy.ok():
            self.ros_status_var.set("ROS: ONLINE")
            self.ros_status.config(fg='#00FF00')
        else:
            self.ros_status_var.set("ROS: OFFLINE")
            self.ros_status.config(fg='#FF0000')
            
        # Update connection status timeout (turn yellow if no data for 2 seconds)
        if time.time() - self.prev_update_time > 2.0:
            self.connection_status.config(text="ESP32: NO DATA", fg='#FFAA00')
        
        # Update sensor graphs
        self.draw_graphs()
        
        # Schedule next update
        self.root.after(100, self.periodic_update)
    
    def update_display(self):
        """Update all display values with current data"""
        # Update orientation values
        roll, pitch, yaw = self.orientation
        self.roll_var.set(f"{roll:.2f}°")
        self.pitch_var.set(f"{pitch:.2f}°")
        self.yaw_var.set(f"{yaw:.2f}°")
        
        # Update progress bars (normalized to 0-100)
        self.roll_bar['value'] = (roll + 180) % 360 / 3.6  # 0-100 for -180 to 180
        self.pitch_bar['value'] = (pitch + 90) % 180 / 1.8  # 0-100 for -90 to 90
        self.yaw_bar['value'] = yaw / 3.6  # 0-100 for 0-360
        
        # Update accelerometer values
        self.accel_x_var.set(f"{self.mpu_data[0]:.2f} g")
        self.accel_y_var.set(f"{self.mpu_data[1]:.2f} g")
        self.accel_z_var.set(f"{self.mpu_data[2]:.2f} g")
        
        # Update gyroscope values (with offset correction)
        gyro_x = self.mpu_data[3] - self.gyro_offset[0]
        gyro_y = self.mpu_data[4] - self.gyro_offset[1]
        gyro_z = self.mpu_data[5] - self.gyro_offset[2]
        self.gyro_x_var.set(f"{gyro_x:.2f} °/s")
        self.gyro_y_var.set(f"{gyro_y:.2f} °/s")
        self.gyro_z_var.set(f"{gyro_z:.2f} °/s")
        
        # Update temperature
        self.temp_var.set(f"{self.mpu_data[6]:.2f} °C")
        
    def _apply_rotation(self, x, y, z, roll_rad, pitch_rad, yaw_rad):
        """Apply 3D rotation to a point"""
        # Apply yaw (rotation around Z axis)
        x_yaw = x * math.cos(yaw_rad) - y * math.sin(yaw_rad)
        y_yaw = x * math.sin(yaw_rad) + y * math.cos(yaw_rad)
        
        # Apply pitch (rotation around Y axis)
        x_pitch = x_yaw * math.cos(pitch_rad) + z * math.sin(pitch_rad)
        z_pitch = -x_yaw * math.sin(pitch_rad) + z * math.cos(pitch_rad)
        
        # Apply roll (rotation around X axis)
        y_roll = y_yaw * math.cos(roll_rad) - z_pitch * math.sin(roll_rad)
        z_roll = y_yaw * math.sin(roll_rad) + z_pitch * math.cos(roll_rad)
        
        return (x_pitch, y_roll, z_roll)
        
    def draw_3d_visualization(self):
        """Draw the 3D cube representation of orientation"""
        self.canvas_3d.delete("all")
        
        # Canvas dimensions
        width = self.canvas_3d.winfo_width()
        height = self.canvas_3d.winfo_height()
        
        # If canvas is not yet properly sized, use defaults
        if width < 10 or height < 10:
            width, height = 600, 400
            
        center_x, center_y = width / 2, height / 2
        size = min(width, height) * 0.4  # Size of the cube
        
        # Convert degrees to radians
        roll_rad = math.radians(self.orientation[0])
        pitch_rad = math.radians(self.orientation[1])
        yaw_rad = math.radians(self.orientation[2])
        
        # Define cube vertices in 3D space (centered at origin)
        cube = [
            [-1, -1, -1],  # 0: back bottom left
            [1, -1, -1],   # 1: back bottom right
            [1, 1, -1],    # 2: back top right
            [-1, 1, -1],   # 3: back top left
            [-1, -1, 1],   # 4: front bottom left
            [1, -1, 1],    # 5: front bottom right
            [1, 1, 1],     # 6: front top right
            [-1, 1, 1]     # 7: front top left
        ]
        
        # Apply rotations (yaw -> pitch -> roll) - ZYX rotation order
        rotated_cube = []
        for x, y, z in cube:
            x_rot, y_rot, z_rot = self._apply_rotation(x, y, z, roll_rad, pitch_rad, yaw_rad)
            
            # Scale and translate to canvas coordinates
            screen_x = center_x + x_rot * size
            screen_y = center_y - y_rot * size  # Negative because Y is down in screen coordinates
            
            rotated_cube.append((screen_x, screen_y, z_rot))
        
        # Draw cube edges with depth information (simple z-sorting)
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Back face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Front face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges
        ]
        
        # Sort edges by average z-coordinate (simple depth sorting)
        z_sorted_edges = []
        for i, j in edges:
            avg_z = (rotated_cube[i][2] + rotated_cube[j][2]) / 2
            z_sorted_edges.append((i, j, avg_z))
        
        z_sorted_edges.sort(key=lambda e: e[2])
        
        # Draw edges from back to front
        for i, j, _ in z_sorted_edges:
            x1, y1, _ = rotated_cube[i]
            x2, y2, _ = rotated_cube[j]
            
            # Adjust color based on depth
            z_avg = (rotated_cube[i][2] + rotated_cube[j][2]) / 2
            # Normalize z from [-1.73, 1.73] to [0, 1]
            z_norm = (z_avg + 1.73) / 3.46
            
            # Use blue for edges behind the center, transitioning to cyan to front
            r = int(0)
            g = int(140 + 115 * z_norm)
            b = int(255)
            color = f'#{r:02x}{g:02x}{b:02x}'
            
            # Line width based on depth (thicker for closer edges)
            line_width = 1 + int(2 * z_norm)
            
            self.canvas_3d.create_line(x1, y1, x2, y2, fill=color, width=line_width)
        
        # Draw coordinate axes (X: red, Y: green, Z: blue)
        axis_length = size * 0.8
        
        # X-axis (Red)
        x_axis = [0, 0, 0, 1, 0, 0]  # [x1, y1, z1, x2, y2, z2]
        x_end = self._apply_rotation(x_axis[3], x_axis[4], x_axis[5], roll_rad, pitch_rad, yaw_rad)
        self.canvas_3d.create_line(
            center_x, center_y, 
            center_x + x_end[0] * axis_length, 
            center_y - x_end[1] * axis_length,
            fill="#FF3030", width=3, arrow=tk.LAST
        )
        self.canvas_3d.create_text(
            center_x + x_end[0] * axis_length + 15, 
            center_y - x_end[1] * axis_length,
            text="X", fill="#FF3030", font=("Arial", 12, "bold")
        )
        
        # Y-axis (Green)
        y_axis = [0, 0, 0, 0, 1, 0]
        y_end = self._apply_rotation(y_axis[3], y_axis[4], y_axis[5], roll_rad, pitch_rad, yaw_rad)
        self.canvas_3d.create_line(
            center_x, center_y, 
            center_x + y_end[0] * axis_length, 
            center_y - y_end[1] * axis_length,
            fill="#30FF30", width=3, arrow=tk.LAST
        )
        self.canvas_3d.create_text(
            center_x + y_end[0] * axis_length + 15, 
            center_y - y_end[1] * axis_length,
            text="Y", fill="#30FF30", font=("Arial", 12, "bold")
        )
        
        # Z-axis (Blue)
        z_axis = [0, 0, 0, 0, 0, 1]
        z_end = self._apply_rotation(z_axis[3], z_axis[4], z_axis[5], roll_rad, pitch_rad, yaw_rad)
        self.canvas_3d.create_line(
            center_x, center_y, 
            center_x + z_end[0] * axis_length, 
            center_y - z_end[1] * axis_length,
            fill="#3030FF", width=3, arrow=tk.LAST
        )
        self.canvas_3d.create_text(
            center_x + z_end[0] * axis_length + 15, 
            center_y - z_end[1] * axis_length,
            text="Z", fill="#3030FF", font=("Arial", 12, "bold")
        )
        
        # Add orientation values as text
        text_y = height - 60
        self.canvas_3d.create_text(
            10, text_y, 
            text=f"Roll: {self.orientation[0]:.2f}°", 
            fill="#FFFFFF", anchor=tk.W, font=("Arial", 10)
        )
        self.canvas_3d.create_text(
            10, text_y + 20, 
            text=f"Pitch: {self.orientation[1]:.2f}°", 
            fill="#FFFFFF", anchor=tk.W, font=("Arial", 10)
        )
        self.canvas_3d.create_text(
            10, text_y + 40, 
            text=f"Yaw: {self.orientation[2]:.2f}°", 
            fill="#FFFFFF", anchor=tk.W, font=("Arial", 10)
        )

    def update_data_points(self, value):
        """Update the number of data points for graphs"""
        new_max = int(float(value))  # Convert scale value to integer
        if new_max != self.max_data_points:
            self.max_data_points = new_max
            # Update deques with new maxlen
            self.accel_data_x = deque(self.accel_data_x, maxlen=new_max)
            self.accel_data_y = deque(self.accel_data_y, maxlen=new_max)
            self.accel_data_z = deque(self.accel_data_z, maxlen=new_max)
            self.gyro_data_x = deque(self.gyro_data_x, maxlen=new_max)
            self.gyro_data_y = deque(self.gyro_data_y, maxlen=new_max)
            self.gyro_data_z = deque(self.gyro_data_z, maxlen=new_max)
            self.temp_data = deque(self.temp_data, maxlen=new_max)
            self.time_points = deque(self.time_points, maxlen=new_max)
            self.log_status(f"Updated graph data points to {new_max}")

    def draw_graphs(self):
        """Draw the accelerometer, gyroscope, and temperature graphs"""
        # Clear canvases
        self.accel_canvas.delete("all")
        self.gyro_canvas.delete("all")
        self.temp_canvas.delete("all")

        # Canvas dimensions
        accel_width = self.accel_canvas.winfo_width()
        accel_height = self.accel_canvas.winfo_height()
        gyro_width = self.gyro_canvas.winfo_width()
        gyro_height = self.gyro_canvas.winfo_height()
        temp_width = self.temp_canvas.winfo_width()
        temp_height = self.temp_canvas.winfo_height()

        # Use defaults if canvas not properly sized
        if accel_width < 10 or accel_height < 10:
            accel_width, accel_height = 600, 200
        if gyro_width < 10 or gyro_height < 10:
            gyro_width, gyro_height = 600, 200
        if temp_width < 10 or temp_height < 10:
            temp_width, temp_height = 600, 200

        # Margins
        margin_x, margin_y = 50, 30

        # Accelerometer graph
        if len(self.accel_data_x) > 1:
            # Find min and max for scaling
            accel_max = max(max(self.accel_data_x), max(self.accel_data_y), max(self.accel_data_z), 1.0)
            accel_min = min(min(self.accel_data_x), min(self.accel_data_y), min(self.accel_data_z), -1.0)
            accel_range = accel_max - accel_min if accel_max != accel_min else 1.0

            # Draw axes
            self.accel_canvas.create_line(margin_x, accel_height - margin_y, accel_width - margin_x, accel_height - margin_y, fill="#FFFFFF")  # X-axis
            self.accel_canvas.create_line(margin_x, margin_y, margin_x, accel_height - margin_y, fill="#FFFFFF")  # Y-axis

            # Draw labels
            self.accel_canvas.create_text(margin_x - 30, margin_y, text=f"{accel_max:.1f} g", fill="#FFFFFF", font=("Arial", 8))
            self.accel_canvas.create_text(margin_x - 30, accel_height - margin_y, text=f"{accel_min:.1f} g", fill="#FFFFFF", font=("Arial", 8))
            self.accel_canvas.create_text(accel_width - margin_x, accel_height - margin_y + 20, text="Time", fill="#FFFFFF", font=("Arial", 8))

            # Plot data
            x_scale = (accel_width - 2 * margin_x) / (len(self.accel_data_x) - 1)
            y_scale = (accel_height - 2 * margin_y) / accel_range

            # X-axis
            points_x = []
            for i, val in enumerate(self.accel_data_x):
                x = margin_x + i * x_scale
                y = accel_height - margin_y - (val - accel_min) * y_scale
                points_x.append((x, y))
            for i in range(len(points_x) - 1):
                self.accel_canvas.create_line(points_x[i], points_x[i+1], fill="#FF3030", width=2)

            # Y-axis
            points_y = []
            for i, val in enumerate(self.accel_data_y):
                x = margin_x + i * x_scale
                y = accel_height - margin_y - (val - accel_min) * y_scale
                points_y.append((x, y))
            for i in range(len(points_y) - 1):
                self.accel_canvas.create_line(points_y[i], points_y[i+1], fill="#30FF30", width=2)

            # Z-axis
            points_z = []
            for i, val in enumerate(self.accel_data_z):
                x = margin_x + i * x_scale
                y = accel_height - margin_y - (val - accel_min) * y_scale
                points_z.append((x, y))
            for i in range(len(points_z) - 1):
                self.accel_canvas.create_line(points_z[i], points_z[i+1], fill="#3030FF", width=2)

            # Legend
            self.accel_canvas.create_line(accel_width - 100, 20, accel_width - 80, 20, fill="#FF3030", width=2)
            self.accel_canvas.create_text(accel_width - 70, 20, text="X", fill="#FFFFFF", anchor=tk.W, font=("Arial", 8))
            self.accel_canvas.create_line(accel_width - 100, 40, accel_width - 80, 40, fill="#30FF30", width=2)
            self.accel_canvas.create_text(accel_width - 70, 40, text="Y", fill="#FFFFFF", anchor=tk.W, font=("Arial", 8))
            self.accel_canvas.create_line(accel_width - 100, 60, accel_width - 80, 60, fill="#3030FF", width=2)
            self.accel_canvas.create_text(accel_width - 70, 60, text="Z", fill="#FFFFFF", anchor=tk.W, font=("Arial", 8))

        # Gyroscope graph
        if len(self.gyro_data_x) > 1:
            # Find min and max for scaling
            gyro_max = max(max(self.gyro_data_x), max(self.gyro_data_y), max(self.gyro_data_z), 1.0)
            gyro_min = min(min(self.gyro_data_x), min(self.gyro_data_y), min(self.gyro_data_z), -1.0)
            gyro_range = gyro_max - gyro_min if gyro_max != gyro_min else 1.0

            # Draw axes
            self.gyro_canvas.create_line(margin_x, gyro_height - margin_y, gyro_width - margin_x, gyro_height - margin_y, fill="#FFFFFF")  # X-axis
            self.gyro_canvas.create_line(margin_x, margin_y, margin_x, gyro_height - margin_y, fill="#FFFFFF")  # Y-axis

            # Draw labels
            self.gyro_canvas.create_text(margin_x - 30, margin_y, text=f"{gyro_max:.1f} °/s", fill="#FFFFFF", font=("Arial", 8))
            self.gyro_canvas.create_text(margin_x - 30, gyro_height - margin_y, text=f"{gyro_min:.1f} °/s", fill="#FFFFFF", font=("Arial", 8))
            self.gyro_canvas.create_text(gyro_width - margin_x, gyro_height - margin_y + 20, text="Time", fill="#FFFFFF", font=("Arial", 8))

            # Plot data
            x_scale = (gyro_width - 2 * margin_x) / (len(self.gyro_data_x) - 1)
            y_scale = (gyro_height - 2 * margin_y) / gyro_range

            # X-axis
            points_x = []
            for i, val in enumerate(self.gyro_data_x):
                x = margin_x + i * x_scale
                y = gyro_height - margin_y - (val - gyro_min) * y_scale
                points_x.append((x, y))
            for i in range(len(points_x) - 1):
                self.gyro_canvas.create_line(points_x[i], points_x[i+1], fill="#FF3030", width=2)

            # Y-axis
            points_y = []
            for i, val in enumerate(self.gyro_data_y):
                x = margin_x + i * x_scale
                y = gyro_height - margin_y - (val - gyro_min) * y_scale
                points_y.append((x, y))
            for i in range(len(points_y) - 1):
                self.gyro_canvas.create_line(points_y[i], points_y[i+1], fill="#30FF30", width=2)

            # Z-axis
            points_z = []
            for i, val in enumerate(self.gyro_data_z):
                x = margin_x + i * x_scale
                y = gyro_height - margin_y - (val - gyro_min) * y_scale
                points_z.append((x, y))
            for i in range(len(points_z) - 1):
                self.gyro_canvas.create_line(points_z[i], points_z[i+1], fill="#3030FF", width=2)

            # Legend
            self.gyro_canvas.create_line(gyro_width - 100, 20, gyro_width - 80, 20, fill="#FF3030", width=2)
            self.gyro_canvas.create_text(gyro_width - 70, 20, text="X", fill="#FFFFFF", anchor=tk.W, font=("Arial", 8))
            self.gyro_canvas.create_line(gyro_width - 100, 40, gyro_width - 80, 40, fill="#30FF30", width=2)
            self.gyro_canvas.create_text(gyro_width - 70, 40, text="Y", fill="#FFFFFF", anchor=tk.W, font=("Arial", 8))
            self.gyro_canvas.create_line(gyro_width - 100, 60, gyro_width - 80, 60, fill="#3030FF", width=2)
            self.gyro_canvas.create_text(gyro_width - 70, 60, text="Z", fill="#FFFFFF", anchor=tk.W, font=("Arial", 8))

        # Temperature graph
        if len(self.temp_data) > 1:
            # Find min and max for scaling
            temp_max = max(self.temp_data, default=25.0)
            temp_min = min(self.temp_data, default=15.0)
            temp_range = temp_max - temp_min if temp_max != temp_min else 1.0

            # Draw axes
            self.temp_canvas.create_line(margin_x, temp_height - margin_y, temp_width - margin_x, temp_height - margin_y, fill="#FFFFFF")  # X-axis
            self.temp_canvas.create_line(margin_x, margin_y, margin_x, temp_height - margin_y, fill="#FFFFFF")  # Y-axis

            # Draw labels
            self.temp_canvas.create_text(margin_x - 30, margin_y, text=f"{temp_max:.1f} °C", fill="#FFFFFF", font=("Arial", 8))
            self.temp_canvas.create_text(margin_x - 30, temp_height - margin_y, text=f"{temp_min:.1f} °C", fill="#FFFFFF", font=("Arial", 8))
            self.temp_canvas.create_text(temp_width - margin_x, temp_height - margin_y + 20, text="Time", fill="#FFFFFF", font=("Arial", 8))

            # Plot data
            x_scale = (temp_width - 2 * margin_x) / (len(self.temp_data) - 1)
            y_scale = (temp_height - 2 * margin_y) / temp_range

            points = []
            for i, val in enumerate(self.temp_data):
                x = margin_x + i * x_scale
                y = temp_height - margin_y - (val - temp_min) * y_scale
                points.append((x, y))
            for i in range(len(points) - 1):
                self.temp_canvas.create_line(points[i], points[i+1], fill="#FFFF00", width=2)

    def destroy(self):
        """Clean up when shutting down"""
        self.root.quit()
        super().destroy()

def ros_spin(node):
    """Run ROS spin in a separate thread"""
    rclpy.spin(node)

def main():
    rclpy.init()
    node = GyroscopeVisualizationGUI()
    
    # Run ROS spin in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()
    
    try:
        # Start Tkinter main loop
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
