#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rclpy
from quadruped_controller_pid import QuadrupedBalanceController, ControlMode, WalkingGait

class CyberpunkStyle:
    """Cyberpunk color scheme and styling constants."""
    
    BG_DARK = '#0a0a0a'
    BG_PANEL = '#1a1a1a'
    BG_WIDGET = '#2a2a2a'
    BG_HOVER = '#3a3a3a'
    
    ACCENT_CYAN = '#00ffff'
    ACCENT_BLUE = '#0080ff'
    ACCENT_PURPLE = '#8000ff'
    ACCENT_GREEN = '#00ff80'
    ACCENT_RED = '#ff4040'
    ACCENT_ORANGE = '#ff8000'
    ACCENT_YELLOW = '#ffff00'
    ACCENT_PINK = '#ff00ff'
    
    TEXT_PRIMARY = '#ffffff'
    TEXT_SECONDARY = '#b0b0b0'
    TEXT_DISABLED = '#606060'
    
    BORDER_ACTIVE = '#00ffff'
    BORDER_INACTIVE = '#404040'
    
    FONT_MAIN = ('Consolas', 10)
    FONT_HEADER = ('Consolas', 11, 'bold')
    FONT_TITLE = ('Consolas', 12, 'bold')
    FONT_MONO = ('Courier New', 9)

class CyberpunkFrame(tk.Frame):
    """Custom frame with cyberpunk styling."""
    
    def __init__(self, parent, title="", accent_color=CyberpunkStyle.ACCENT_CYAN, **kwargs):
        super().__init__(parent, bg=CyberpunkStyle.BG_PANEL, relief='flat', bd=2, **kwargs)
        
        if title:
            title_frame = tk.Frame(self, bg=CyberpunkStyle.BG_PANEL, height=30)
            title_frame.pack(fill='x', padx=2, pady=(2, 5))
            title_frame.pack_propagate(False)
            
            title_label = tk.Label(title_frame, text=f"▶ {title.upper()}", 
                                 bg=CyberpunkStyle.BG_PANEL, fg=accent_color,
                                 font=CyberpunkStyle.FONT_HEADER, anchor='w')
            title_label.pack(side='left', padx=5, pady=5)
            
            separator = tk.Frame(title_frame, bg=accent_color, height=1)
            separator.pack(fill='x', padx=5, pady=(15, 5))

class CyberpunkScale(tk.Frame):
    """Custom scale widget with cyberpunk styling and precise entry."""
    
    def __init__(self, parent, label, from_=0, to=100, resolution=0.01, 
                 initial_value=0, callback=None, accent_color=CyberpunkStyle.ACCENT_CYAN):
        super().__init__(parent, bg=CyberpunkStyle.BG_PANEL)
        
        self.callback = callback
        self.resolution = resolution
        self.accent_color = accent_color
        
        label_frame = tk.Frame(self, bg=CyberpunkStyle.BG_PANEL)
        label_frame.pack(fill='x', pady=(0, 2))
        
        self.label_widget = tk.Label(label_frame, text=label, bg=CyberpunkStyle.BG_PANEL, 
                                     fg=CyberpunkStyle.TEXT_PRIMARY, font=CyberpunkStyle.FONT_MAIN,
                                     anchor='w')
        self.label_widget.pack(side='left')
        
        self.value_label = tk.Label(label_frame, text=f"{initial_value:.4f}", 
                                   bg=CyberpunkStyle.BG_PANEL, fg=accent_color,
                                   font=CyberpunkStyle.FONT_MONO, anchor='e')
        self.value_label.pack(side='right')
        
        scale_frame = tk.Frame(self, bg=CyberpunkStyle.BG_WIDGET, relief='sunken', bd=1)
        scale_frame.pack(fill='x', pady=(0, 5))
        
        self.var = tk.DoubleVar(value=initial_value)
        self.scale = tk.Scale(scale_frame, from_=from_, to=to, resolution=resolution,
                             orient='horizontal', variable=self.var, showvalue=0,
                             bg=CyberpunkStyle.BG_WIDGET, fg=CyberpunkStyle.TEXT_PRIMARY,
                             activebackground=accent_color, highlightthickness=0,
                             troughcolor=CyberpunkStyle.BG_DARK, sliderlength=20,
                             font=CyberpunkStyle.FONT_MAIN, command=self._on_change)
        self.scale.pack(fill='x', padx=2, pady=2)
        
        entry_frame = tk.Frame(self, bg=CyberpunkStyle.BG_PANEL)
        entry_frame.pack(fill='x')
        
        tk.Label(entry_frame, text="Set:", bg=CyberpunkStyle.BG_PANEL,
                fg=CyberpunkStyle.TEXT_SECONDARY, font=CyberpunkStyle.FONT_MAIN).pack(side='left')
        
        self.entry_var = tk.StringVar(value=f"{initial_value:.4f}")
        self.entry = tk.Entry(entry_frame, textvariable=self.entry_var, width=10,
                             bg=CyberpunkStyle.BG_WIDGET, fg=CyberpunkStyle.TEXT_PRIMARY,
                             insertbackground=accent_color, relief='flat', bd=1,
                             font=CyberpunkStyle.FONT_MONO)
        self.entry.pack(side='right')
        self.entry.bind('<Return>', self._on_entry_change)
        
    def _on_change(self, value_str):
        value = float(value_str)
        self.value_label.config(text=f"{value:.4f}")
        self.entry_var.set(f"{value:.4f}")
        if self.callback:
            self.callback(value)
    
    def _on_entry_change(self, event=None):
        try:
            value = float(self.entry_var.get())
            if self.scale.cget('from_') <= value <= self.scale.cget('to'):
                self.var.set(value)
            else:
                self.entry_var.set(f"{self.var.get():.4f}")
        except ValueError:
            self.entry_var.set(f"{self.var.get():.4f}")
    
    def get(self):
        return self.var.get()
    
    def set(self, value):
        self.var.set(value)
        self.entry_var.set(f"{value:.4f}")
        self.value_label.config(text=f"{value:.4f}")

class QuadrupedGUI:
    """Enhanced GUI for Quadruped Controller with walking capabilities."""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("◢ QUADRUPED NEURAL LOCOMOTION MATRIX ◣")
        self.root.geometry("1800x1000")
        self.root.configure(bg=CyberpunkStyle.BG_DARK)
        
        self.init_ros()
        
        # Data for plotting
        self.time_data = list(range(-100, 0))
        self.roll_data = [0] * 100
        self.pitch_data = [0] * 100
        self.max_plot_points = 100
        
        self.is_running = True
        self.update_rate = 25  # Hz
        
        # Movement control state
        self.movement_active = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        plt.style.use('dark_background')
        
        self.setup_styles()
        self.create_widgets()
        
        self.update_thread = threading.Thread(target=self.update_data_loop, daemon=True)
        self.update_thread.start()
        
        # Keyboard bindings
        self.setup_keyboard_controls()
        
    def setup_styles(self):
        """Configures custom ttk styles."""
        style = ttk.Style(self.root)
        style.theme_use('clam')
        
        style.configure("Cyber.Vertical.TScrollbar", gripcount=0, background=CyberpunkStyle.BG_WIDGET,
                        troughcolor=CyberpunkStyle.BG_PANEL, bordercolor=CyberpunkStyle.BORDER_INACTIVE,
                        arrowcolor=CyberpunkStyle.TEXT_SECONDARY, relief='flat')
        style.map("Cyber.Vertical.TScrollbar", background=[('active', CyberpunkStyle.ACCENT_CYAN)])

        style.configure('TNotebook', background=CyberpunkStyle.BG_PANEL, borderwidth=0)
        style.configure('TNotebook.Tab', background=CyberpunkStyle.BG_WIDGET, foreground=CyberpunkStyle.TEXT_SECONDARY,
                        font=CyberpunkStyle.FONT_MAIN, borderwidth=0, padding=[10, 5])
        style.map('TNotebook.Tab', background=[('selected', CyberpunkStyle.ACCENT_YELLOW)],
                  foreground=[('selected', CyberpunkStyle.BG_DARK)])
        
        style.configure('TCombobox',
            fieldbackground=CyberpunkStyle.BG_WIDGET, background=CyberpunkStyle.BG_WIDGET,
            foreground=CyberpunkStyle.TEXT_PRIMARY, arrowcolor=CyberpunkStyle.ACCENT_CYAN,
            selectbackground=CyberpunkStyle.BG_WIDGET, selectforeground=CyberpunkStyle.ACCENT_CYAN,
            bordercolor=CyberpunkStyle.BORDER_INACTIVE, lightcolor=CyberpunkStyle.BG_WIDGET,
            darkcolor=CyberpunkStyle.BG_WIDGET, insertcolor=CyberpunkStyle.TEXT_PRIMARY
        )
        style.map('TCombobox', fieldbackground=[('readonly', CyberpunkStyle.BG_WIDGET)])

    def setup_keyboard_controls(self):
        """Setup keyboard controls for walking."""
        self.root.focus_set()
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        
    def on_key_press(self, event):
        """Handle key press for movement control."""
        key = event.keysym.lower()
        
        if key in ['w', 'up'] and not self.movement_active['forward']:
            self.movement_active['forward'] = True
            self.controller.move_forward(0.8)
        elif key in ['s', 'down'] and not self.movement_active['backward']:
            self.movement_active['backward'] = True
            self.controller.move_backward(0.8)
        elif key in ['a', 'left'] and not self.movement_active['left']:
            self.movement_active['left'] = True
            self.controller.move_left(0.8)
        elif key in ['d', 'right'] and not self.movement_active['right']:
            self.movement_active['right'] = True
            self.controller.move_right(0.8)
        elif key == 'space':
            self.controller.stop_walking()
            self.movement_active = {'forward': False, 'backward': False, 'left': False, 'right': False}
            
    def on_key_release(self, event):
        """Handle key release for movement control."""
        key = event.keysym.lower()
        
        if key in ['w', 'up']:
            self.movement_active['forward'] = False
        elif key in ['s', 'down']:
            self.movement_active['backward'] = False
        elif key in ['a', 'left']:
            self.movement_active['left'] = False
        elif key in ['d', 'right']:
            self.movement_active['right'] = False
            
        # Stop if no movement keys are pressed
        if not any(self.movement_active.values()):
            self.controller.stop_walking()

    def init_ros(self):
        try:
            rclpy.init()
            self.controller = QuadrupedBalanceController()
            self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.controller,), daemon=True)
            self.ros_thread.start()
        except Exception as e:
            messagebox.showerror("ROS Error", f"Failed to initialize ROS: {e}")
            self.root.destroy()
            
    def create_widgets(self):
        header_frame = tk.Frame(self.root, bg=CyberpunkStyle.BG_DARK)
        header_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        title_label = tk.Label(header_frame, text="QUADRUPED NEURAL LOCOMOTION MATRIX", 
                              bg=CyberpunkStyle.BG_DARK, fg=CyberpunkStyle.ACCENT_CYAN,
                              font=('Consolas', 16, 'bold'))
        title_label.pack(side='top', pady=10)
        
        # Controls hint
        hint_label = tk.Label(header_frame, text="WASD/Arrow Keys: Move | Space: Stop | Click for Manual Control", 
                             bg=CyberpunkStyle.BG_DARK, fg=CyberpunkStyle.TEXT_SECONDARY,
                             font=CyberpunkStyle.FONT_MAIN)
        hint_label.pack(side='top', pady=2)
        
        main_frame = tk.Frame(self.root, bg=CyberpunkStyle.BG_DARK)
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.create_control_panel(main_frame)
        self.create_monitor_panel(main_frame)
        
    def create_control_panel(self, parent):
        container = tk.Frame(parent, bg=CyberpunkStyle.BG_DARK, width=500)
        container.pack(side='left', fill='y', padx=(0, 5), expand=False)
        container.pack_propagate(False)

        canvas = tk.Canvas(container, bg=CyberpunkStyle.BG_DARK, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview, style="Cyber.Vertical.TScrollbar")
        scrollable_frame = CyberpunkFrame(canvas, "CONTROL MATRIX", CyberpunkStyle.ACCENT_CYAN)

        scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw", width=470)
        canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        
        # --- LOCOMOTION CONTROL ---
        locomotion_frame = CyberpunkFrame(scrollable_frame, "LOCOMOTION CONTROL", CyberpunkStyle.ACCENT_PINK)
        locomotion_frame.pack(fill='x', pady=5, padx=5)
        
        # Walking enable/disable
        self.walking_enabled_var = tk.BooleanVar(value=False)
        walking_check = tk.Checkbutton(locomotion_frame, text="◢ ENABLE WALKING ◣", 
                                      variable=self.walking_enabled_var, command=self.toggle_walking,
                                      bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_PRIMARY,
                                      selectcolor=CyberpunkStyle.BG_WIDGET, font=CyberpunkStyle.FONT_MAIN)
        walking_check.pack(pady=5, fill='x')
        
        # Gait selection
        gait_frame = tk.Frame(locomotion_frame, bg=CyberpunkStyle.BG_PANEL)
        gait_frame.pack(fill='x', pady=5)
        tk.Label(gait_frame, text="Gait:", bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_PRIMARY,
                 font=CyberpunkStyle.FONT_MAIN).pack(side='left', padx=5)
        self.gait_var = tk.StringVar(value="tripod")
        gait_options = [gait.value for gait in WalkingGait]
        gait_menu = ttk.Combobox(gait_frame, textvariable=self.gait_var, values=gait_options, 
                                state='readonly', style='TCombobox', width=15)
        gait_menu.pack(side='left', fill='x', expand=True, padx=5)
        gait_menu.bind('<<ComboboxSelected>>', lambda e: self.controller.set_gait_type(self.gait_var.get()))
        
        # Movement buttons grid
        move_grid = tk.Frame(locomotion_frame, bg=CyberpunkStyle.BG_PANEL)
        move_grid.pack(pady=10)
        
        # Create movement buttons
        move_buttons = [
            ("", 0, 0), ("◢ FORWARD ◣", 0, 1), ("", 0, 2),
            ("◢ LEFT ◣", 1, 0), ("◢ STOP ◣", 1, 1), ("◢ RIGHT ◣", 1, 2),
            ("", 2, 0), ("◢ BACK ◣", 2, 1), ("", 2, 2)
        ]
        
        move_commands = {
            "◢ FORWARD ◣": lambda: self.controller.move_forward(0.8),
            "◢ BACK ◣": lambda: self.controller.move_backward(0.8),
            "◢ LEFT ◣": lambda: self.controller.move_left(0.8),
            "◢ RIGHT ◣": lambda: self.controller.move_right(0.8),
            "◢ STOP ◣": lambda: self.controller.stop_walking()
        }
        
        move_colors = {
            "◢ FORWARD ◣": CyberpunkStyle.ACCENT_GREEN,
            "◢ BACK ◣": CyberpunkStyle.ACCENT_BLUE,
            "◢ LEFT ◣": CyberpunkStyle.ACCENT_ORANGE,
            "◢ RIGHT ◣": CyberpunkStyle.ACCENT_PURPLE,
            "◢ STOP ◣": CyberpunkStyle.ACCENT_RED
        }
        
        for text, row, col in move_buttons:
            if text:
                btn = tk.Button(move_grid, text=text, command=move_commands[text],
                               bg=CyberpunkStyle.BG_WIDGET, fg=move_colors[text],
                               relief='flat', font=CyberpunkStyle.FONT_MAIN, width=10, height=2)
                btn.grid(row=row, column=col, padx=2, pady=2)
        
        # Walking parameters
        self.walking_speed_scale = CyberpunkScale(locomotion_frame, "Walking Speed:", from_=0.1, to=2.0, 
                                                 resolution=0.1, initial_value=1.0, 
                                                 callback=self.update_walking_params,
                                                 accent_color=CyberpunkStyle.ACCENT_PINK)
        self.walking_speed_scale.pack(fill='x', padx=5)
        
        self.step_height_scale = CyberpunkScale(locomotion_frame, "Step Height (°):", from_=10.0, to=90.0,
                                               resolution=1.0, initial_value=25.0,
                                               callback=self.update_walking_params,
                                               accent_color=CyberpunkStyle.ACCENT_PINK)
        self.step_height_scale.pack(fill='x', padx=5)
        
        self.step_length_scale = CyberpunkScale(locomotion_frame, "Step Length (°):", from_=5.0, to=90.0,
                                               resolution=1.0, initial_value=15.0,
                                               callback=self.update_walking_params,
                                               accent_color=CyberpunkStyle.ACCENT_PINK)
        self.step_length_scale.pack(fill='x', padx=5)
        
        self.cycle_time_scale = CyberpunkScale(locomotion_frame, "Cycle Time (s):", from_=0.5, to=4.0,
                                              resolution=0.1, initial_value=2.0,
                                              callback=self.update_walking_params,
                                              accent_color=CyberpunkStyle.ACCENT_PINK)
        self.cycle_time_scale.pack(fill='x', padx=5)
        
        # --- SYSTEM CONTROL ---
        sys_frame = CyberpunkFrame(scrollable_frame, "SYSTEM CONTROL", CyberpunkStyle.ACCENT_GREEN)
        sys_frame.pack(fill='x', pady=5, padx=5)
        
        self.balance_enabled_var = tk.BooleanVar(value=True)
        balance_check = tk.Checkbutton(sys_frame, text="◢ ENABLE BALANCE ◣", variable=self.balance_enabled_var,
                                     command=self.toggle_balance, bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_PRIMARY,
                                     selectcolor=CyberpunkStyle.BG_WIDGET, font=CyberpunkStyle.FONT_MAIN)
        balance_check.pack(pady=5, fill='x')

        mode_frame = tk.Frame(sys_frame, bg=CyberpunkStyle.BG_PANEL)
        mode_frame.pack(fill='x', pady=5)
        tk.Label(mode_frame, text="Mode:", bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_PRIMARY,
                 font=CyberpunkStyle.FONT_MAIN).pack(side='left', padx=5)
        self.control_mode_var = tk.StringVar(value=self.controller.control_mode.value)
        mode_options = [mode.value for mode in ControlMode if mode != ControlMode.EMERGENCY_AUTO]
        mode_menu = ttk.Combobox(mode_frame, textvariable=self.control_mode_var, values=mode_options, state='readonly', style='TCombobox')
        mode_menu.pack(side='left', fill='x', expand=True)
        mode_menu.bind('<<ComboboxSelected>>', lambda e: self.controller.set_control_mode(self.control_mode_var.get()))

        cal_btn = tk.Button(sys_frame, text="◢ CALIBRATE SENSORS ◣", command=self.calibrate_sensors,
                           bg=CyberpunkStyle.BG_WIDGET, fg=CyberpunkStyle.ACCENT_PURPLE, relief='flat', font=CyberpunkStyle.FONT_MAIN)
        cal_btn.pack(pady=5, fill='x', padx=5)
        
        # --- STANCE CONTROL ---
        stance_frame = CyberpunkFrame(scrollable_frame, "STANCE CONTROL", CyberpunkStyle.ACCENT_YELLOW)
        stance_frame.pack(fill='x', pady=5, padx=5)
        
        self.stance_var = tk.StringVar(value="stand")
        stances = [("STAND", "stand"), ("CROUCH", "crouch"), ("WIDE", "wide"), ("NARROW", "narrow")]
        stance_btn_frame = tk.Frame(stance_frame, bg=CyberpunkStyle.BG_PANEL)
        stance_btn_frame.pack(fill='x', pady=5)
        for text, value in stances:
            btn = tk.Radiobutton(stance_btn_frame, text=f"◢ {text} ◣", variable=self.stance_var,
                               value=value, command=self.change_stance, indicatoron=0,
                               bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_PRIMARY,
                               selectcolor=CyberpunkStyle.ACCENT_YELLOW, font=CyberpunkStyle.FONT_MAIN)
            btn.pack(side='left', padx=2, fill='x', expand=True)

        # --- MANUAL OVERRIDE ---
        manual_frame = CyberpunkFrame(scrollable_frame, "MANUAL OVERRIDE", CyberpunkStyle.ACCENT_ORANGE)
        manual_frame.pack(fill='x', pady=5, padx=5)
        notebook = ttk.Notebook(manual_frame, style='TNotebook')
        notebook.pack(fill='x', expand=True, pady=5)
        self.create_leg_control_tab(notebook)
        
        # --- TUNING PARAMETERS ---
        tuning_frame = CyberpunkFrame(scrollable_frame, "TUNING PARAMETERS", CyberpunkStyle.ACCENT_CYAN)
        tuning_frame.pack(fill='x', pady=5, padx=5)
        
        # Roll PID
        self.roll_kp_scale = CyberpunkScale(tuning_frame, "Roll Kp:", from_=0.0, to=10.0, resolution=0.0001, 
                                          initial_value=self.controller.roll_pid.kp, callback=self.update_roll_pid)
        self.roll_kp_scale.pack(fill='x', pady=2)
        self.roll_ki_scale = CyberpunkScale(tuning_frame, "Roll Ki:", from_=0.0, to=2.0, resolution=0.0001, 
                                          initial_value=self.controller.roll_pid.ki, callback=self.update_roll_pid)
        self.roll_ki_scale.pack(fill='x', pady=2)
        self.roll_kd_scale = CyberpunkScale(tuning_frame, "Roll Kd:", from_=0.0, to=5.0, resolution=0.0001, 
                                          initial_value=self.controller.roll_pid.kd, callback=self.update_roll_pid)
        self.roll_kd_scale.pack(fill='x', pady=2)
        
        # Pitch PID
        self.pitch_kp_scale = CyberpunkScale(tuning_frame, "Pitch Kp:", from_=0.0, to=10.0, resolution=0.0001, 
                                           initial_value=self.controller.pitch_pid.kp, callback=self.update_pitch_pid,
                                           accent_color=CyberpunkStyle.ACCENT_ORANGE)
        self.pitch_kp_scale.pack(fill='x', pady=2)
        self.pitch_ki_scale = CyberpunkScale(tuning_frame, "Pitch Ki:", from_=0.0, to=2.0, resolution=0.0001, 
                                           initial_value=self.controller.pitch_pid.ki, callback=self.update_pitch_pid,
                                           accent_color=CyberpunkStyle.ACCENT_ORANGE)
        self.pitch_ki_scale.pack(fill='x', pady=2)
        self.pitch_kd_scale = CyberpunkScale(tuning_frame, "Pitch Kd:", from_=0.0, to=5.0, resolution=0.0001, 
                                           initial_value=self.controller.pitch_pid.kd, callback=self.update_pitch_pid,
                                           accent_color=CyberpunkStyle.ACCENT_ORANGE)
        self.pitch_kd_scale.pack(fill='x', pady=2)

        # Other Params
        self.max_correction_scale = CyberpunkScale(tuning_frame, "Max Correction (°):", from_=0, to=90, resolution=1, 
                                                 initial_value=self.controller.max_correction, 
                                                 callback=lambda v: setattr(self.controller, 'max_correction', v),
                                                 accent_color=CyberpunkStyle.ACCENT_YELLOW)
        self.max_correction_scale.pack(fill='x', pady=2)
        self.deadzone_scale = CyberpunkScale(tuning_frame, "Deadzone (°):", from_=0.0, to=10.0, resolution=0.01, 
                                           initial_value=self.controller.deadzone, 
                                           callback=lambda v: setattr(self.controller, 'deadzone', v),
                                           accent_color=CyberpunkStyle.ACCENT_YELLOW)
        self.deadzone_scale.pack(fill='x', pady=2)

    def create_leg_control_tab(self, notebook):
        leg_tab_frame = tk.Frame(notebook, bg=CyberpunkStyle.BG_PANEL)
        notebook.add(leg_tab_frame, text='▶ LEGS')
        
        self.leg_controls = {}
        # Corrected leg layout - RL and FL are front
        leg_layout = [('RL', 'FL'), ('RR', 'FR')]  # Front row, Back row
        leg_colors = {'FL': CyberpunkStyle.ACCENT_GREEN, 'RL': CyberpunkStyle.ACCENT_CYAN,
                      'FR': CyberpunkStyle.ACCENT_ORANGE, 'RR': CyberpunkStyle.ACCENT_PURPLE}
        
        for row_idx, leg_row in enumerate(leg_layout):
            for col_idx, leg_name in enumerate(leg_row):
                leg_frame = CyberpunkFrame(leg_tab_frame, f"{leg_name} Leg", leg_colors[leg_name])
                leg_frame.grid(row=row_idx, column=col_idx, padx=5, pady=5, sticky='nsew')
                
                knee_idx, hip_idx = self.controller.leg_servos[leg_name]
                
                hip_angle_scale = CyberpunkScale(leg_frame, "Hip Angle (°):", from_=0, to=180, resolution=1, 
                                               initial_value=self.controller.current_angles[hip_idx],
                                               accent_color=leg_colors[leg_name])
                hip_angle_scale.pack(fill='x', padx=5)

                knee_angle_scale = CyberpunkScale(leg_frame, "Knee Angle (°):", from_=0, to=180, resolution=1, 
                                                initial_value=self.controller.current_angles[knee_idx],
                                                accent_color=leg_colors[leg_name])
                knee_angle_scale.pack(fill='x', padx=5)

                balance_weight_scale = CyberpunkScale(leg_frame, "Balance Weight:", from_=0.0, to=1.0, resolution=0.01, 
                                                      initial_value=1.0, accent_color=CyberpunkStyle.ACCENT_BLUE)
                balance_weight_scale.pack(fill='x', padx=5)
                
                set_btn = tk.Button(leg_frame, text=f"◢ SET {leg_name} ◣", 
                                    command=lambda n=leg_name: self.set_leg_manual(n),
                                    bg=CyberpunkStyle.BG_WIDGET, fg=leg_colors[leg_name], relief='flat')
                set_btn.pack(pady=5, fill='x', padx=5)

                self.leg_controls[leg_name] = {
                    'hip_scale': hip_angle_scale, 'knee_scale': knee_angle_scale, 'weight_scale': balance_weight_scale
                }
        
        leg_tab_frame.grid_rowconfigure(0, weight=1)
        leg_tab_frame.grid_rowconfigure(1, weight=1) 
        leg_tab_frame.grid_columnconfigure(0, weight=1)
        leg_tab_frame.grid_columnconfigure(1, weight=1)

    def create_monitor_panel(self, parent):
        monitor_frame = CyberpunkFrame(parent, "NEURAL FEEDBACK MATRIX", CyberpunkStyle.ACCENT_GREEN)
        monitor_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        # Status panels
        status_container = tk.Frame(monitor_frame, bg=CyberpunkStyle.BG_PANEL)
        status_container.pack(fill='x', pady=5)
        
        # System status
        sys_status_frame = CyberpunkFrame(status_container, "SYSTEM STATUS", CyberpunkStyle.ACCENT_CYAN)
        sys_status_frame.pack(fill='x', pady=5, padx=5, side='left', expand=True)
        
        status_grid = tk.Frame(sys_status_frame, bg=CyberpunkStyle.BG_PANEL)
        status_grid.pack(fill='x', pady=5)
        
        self.status_labels = {}
        status_items = [("ROLL", "roll", "°"), ("PITCH", "pitch", "°"), ("STABILITY", "stability", "%"),
                        ("FALL RISK", "fall_risk", ""), ("TEMP", "temperature", "°C"), ("MODE", "control_mode", "")]
        
        for i, (label, key, unit) in enumerate(status_items):
            row, col = divmod(i, 2)
            tk.Label(status_grid, text=f"{label}:", bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_SECONDARY,
                     font=CyberpunkStyle.FONT_MAIN, anchor='w').grid(row=row*2, column=col, sticky='w', padx=5, pady=1)
            val_widget = tk.Label(status_grid, text="--", bg=CyberpunkStyle.BG_PANEL, font=CyberpunkStyle.FONT_MONO, anchor='w')
            val_widget.grid(row=row*2+1, column=col, sticky='w', padx=5, pady=1)
            self.status_labels[key] = (val_widget, unit)
        
        # Walking status
        walk_status_frame = CyberpunkFrame(status_container, "WALKING STATUS", CyberpunkStyle.ACCENT_PINK)
        walk_status_frame.pack(fill='both', pady=5, padx=5, side='right', expand=True)

        walk_grid = tk.Frame(walk_status_frame, bg=CyberpunkStyle.BG_PANEL)
        walk_grid.pack(fill='x', pady=5)
        
        self.walking_status_labels = {}
        walking_items = [("STATUS", "enabled"), ("GAIT", "gait"), ("SPEED", "speed"), 
                        ("DIRECTION", "direction"), ("PHASE", "phase"), ("CYCLE", "cycle")]
        
        for i, (label, key) in enumerate(walking_items):
            row, col = divmod(i, 2)
            tk.Label(walk_grid, text=f"{label}:", bg=CyberpunkStyle.BG_PANEL, fg=CyberpunkStyle.TEXT_SECONDARY,
                     font=CyberpunkStyle.FONT_MAIN, anchor='w').grid(row=row*2, column=col, sticky='w', padx=5, pady=1)
            val_widget = tk.Label(walk_grid, text="--", bg=CyberpunkStyle.BG_PANEL, font=CyberpunkStyle.FONT_MONO, anchor='w')
            val_widget.grid(row=row*2+1, column=col, sticky='w', padx=5, pady=1)
            self.walking_status_labels[key] = val_widget
        
        # Leg states
        leg_states_frame = CyberpunkFrame(monitor_frame, "LEG STATES", CyberpunkStyle.ACCENT_YELLOW)
        leg_states_frame.pack(fill='x', pady=5, padx=5)

        self.leg_state_labels = {}
        leg_state_grid = tk.Frame(leg_states_frame, bg=CyberpunkStyle.BG_PANEL)
        leg_state_grid.pack(fill='x', pady=5)
        
        # Display legs in actual robot layout
        leg_positions = [('RL', 0, 0), ('FL', 0, 2), ('RR', 1, 0), ('FR', 1, 2)]
        leg_colors = {'FL': CyberpunkStyle.ACCENT_GREEN, 'RL': CyberpunkStyle.ACCENT_CYAN,
                      'FR': CyberpunkStyle.ACCENT_ORANGE, 'RR': CyberpunkStyle.ACCENT_PURPLE}
        
        for leg, row, col in leg_positions:
            leg_frame = tk.Frame(leg_state_grid, bg=CyberpunkStyle.BG_PANEL, relief='ridge', bd=1)
            leg_frame.grid(row=row, column=col, padx=10, pady=5, sticky='ew')
            
            tk.Label(leg_frame, text=f"{leg}:", bg=CyberpunkStyle.BG_PANEL, fg=leg_colors[leg],
                     font=CyberpunkStyle.FONT_HEADER).pack()
            
            state_label = tk.Label(leg_frame, text="SUPPORT", bg=CyberpunkStyle.BG_PANEL, 
                                  fg=CyberpunkStyle.ACCENT_GREEN, font=CyberpunkStyle.FONT_MONO)
            state_label.pack()
            
            phase_label = tk.Label(leg_frame, text="Phase: 0.0", bg=CyberpunkStyle.BG_PANEL,
                                  fg=CyberpunkStyle.TEXT_SECONDARY, font=CyberpunkStyle.FONT_MONO)
            phase_label.pack()
            
            self.leg_state_labels[leg] = {'state': state_label, 'phase': phase_label}
        
        # Center separator
        tk.Label(leg_state_grid, text="◢ ROBOT ◣", bg=CyberpunkStyle.BG_PANEL, 
                fg=CyberpunkStyle.ACCENT_CYAN, font=CyberpunkStyle.FONT_HEADER).grid(row=0, column=1, rowspan=2)
        
        leg_state_grid.grid_columnconfigure(0, weight=1)
        leg_state_grid.grid_columnconfigure(1, weight=0)
        leg_state_grid.grid_columnconfigure(2, weight=1)
        
        # Plotting area
        plot_frame = CyberpunkFrame(monitor_frame, "NEURAL OSCILLATION ANALYSIS", CyberpunkStyle.ACCENT_PURPLE)
        plot_frame.pack(fill='both', expand=True, pady=5, padx=5)
        
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8), facecolor=CyberpunkStyle.BG_PANEL)
        self.fig.tight_layout(pad=3.0)
        
        self.roll_line, = self.ax1.plot(self.time_data, self.roll_data, color=CyberpunkStyle.ACCENT_BLUE, linewidth=2, label='Roll')
        self.pitch_line, = self.ax2.plot(self.time_data, self.pitch_data, color=CyberpunkStyle.ACCENT_ORANGE, linewidth=2, label='Pitch')

        for ax, color, title in [(self.ax1, CyberpunkStyle.ACCENT_BLUE, "ROLL AXIS"),
                                 (self.ax2, CyberpunkStyle.ACCENT_ORANGE, "PITCH AXIS")]:
            ax.set_facecolor(CyberpunkStyle.BG_WIDGET)
            ax.set_title(title, color=color, fontsize=12)
            ax.set_ylabel("Degrees", color=CyberpunkStyle.TEXT_PRIMARY)
            ax.grid(True, alpha=0.2, color=CyberpunkStyle.ACCENT_CYAN)
            ax.set_ylim(-45, 45)
            ax.tick_params(colors=CyberpunkStyle.TEXT_SECONDARY)
            ax.legend(loc='upper right')
            for spine in ax.spines.values(): 
                spine.set_color(CyberpunkStyle.BORDER_INACTIVE)
        
        self.ax2.set_xlabel("Time Steps", color=CyberpunkStyle.TEXT_PRIMARY)
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=5, pady=5)
        self.canvas.draw()
    
    # --- Action Methods ---
    def toggle_walking(self):
        """Toggle walking mode."""
        enabled = self.walking_enabled_var.get()
        success = self.controller.enable_walking(enabled)
        if not success:
            self.walking_enabled_var.set(False)
            messagebox.showwarning("Walking Disabled", "Cannot enable walking - robot is unstable!")
    
    def update_walking_params(self, _=None):
        """Update walking parameters from GUI."""
        self.controller.set_walking_parameters(
            speed=self.walking_speed_scale.get(),
            step_height=self.step_height_scale.get(),
            step_length=self.step_length_scale.get(),
            cycle_time=self.cycle_time_scale.get()
        )
        
    def update_roll_pid(self, _=None):
        self.controller.set_pid_parameters('roll', self.roll_kp_scale.get(), self.roll_ki_scale.get(), self.roll_kd_scale.get())
        
    def update_pitch_pid(self, _=None):
        self.controller.set_pid_parameters('pitch', self.pitch_kp_scale.get(), self.pitch_ki_scale.get(), self.pitch_kd_scale.get())
        
    def toggle_balance(self):
        self.controller.set_balance_enabled(self.balance_enabled_var.get())
    
    def change_stance(self):
        stance = self.stance_var.get()
        self.controller.set_base_stance(stance)
        # Update GUI leg controls to reflect new stance
        new_angles = self.controller.base_positions[stance]
        for leg, indices in self.controller.leg_servos.items():
            knee_idx, hip_idx = indices
            self.leg_controls[leg]['knee_scale'].set(new_angles[knee_idx])
            self.leg_controls[leg]['hip_scale'].set(new_angles[hip_idx])

    def set_leg_manual(self, leg_name):
        controls = self.leg_controls[leg_name]
        self.controller.set_leg_manual_control(
            leg_name=leg_name, hip_angle=controls['hip_scale'].get(),
            knee_angle=controls['knee_scale'].get(), balance_weight=controls['weight_scale'].get()
        )
            
    def calibrate_sensors(self):
        if not hasattr(self, 'controller'): return
        was_enabled = self.controller.balance_enabled
        was_walking = self.controller.walking_enabled
        
        if was_enabled: self.controller.set_balance_enabled(False)
        if was_walking: self.controller.enable_walking(False)
        
        messagebox.showinfo("Calibration", "Place robot on a level surface. Calibration will take ~2 seconds.")
        
        def calibrate_thread():
            self.controller.calibrate_sensors(200)
            if was_enabled: self.root.after(0, lambda: self.controller.set_balance_enabled(True))
            if was_walking: self.root.after(0, lambda: self.controller.enable_walking(True))
            self.root.after(0, lambda: messagebox.showinfo("Calibration", "Sensor calibration complete!"))
            
        threading.Thread(target=calibrate_thread, daemon=True).start()
            
    # --- Update Loop & GUI Elements ---
    def update_data_loop(self):
        while self.is_running:
            try:
                if hasattr(self, 'controller') and self.controller:
                    status = self.controller.get_full_status()
                    self.root.after(0, self.update_gui_elements, status)
                    
                    self.roll_data.pop(0)
                    self.pitch_data.pop(0)
                    self.roll_data.append(status['orientation']['roll'])
                    self.pitch_data.append(status['orientation']['pitch'])
                    
                    self.root.after(0, self.update_plot)
                time.sleep(1.0 / self.update_rate)
            except Exception as e:
                print(f"Update loop error: {e}")
                time.sleep(0.1)
                
    def update_gui_elements(self, status):
        try:
            # System status updates
            self.status_labels['roll'][0].config(text=f"{status['orientation']['roll']:.1f}{self.status_labels['roll'][1]}")
            self.status_labels['pitch'][0].config(text=f"{status['orientation']['pitch']:.1f}{self.status_labels['pitch'][1]}")
            self.status_labels['temperature'][0].config(text=f"{status['sensors']['temperature']:.1f}{self.status_labels['temperature'][1]}")
            self.status_labels['stability'][0].config(text=f"{status['stability']['score']:.1f}{self.status_labels['stability'][1]}")
            self.status_labels['control_mode'][0].config(text=f"{status['control']['mode'].replace('_', ' ').upper()}")

            # Fall risk with colors
            risk_levels = {0: "SAFE", 1: "CAUTION", 2: "WARNING", 3: "EMERGENCY"}
            risk_colors = {0: CyberpunkStyle.ACCENT_GREEN, 1: CyberpunkStyle.ACCENT_YELLOW, 2: CyberpunkStyle.ACCENT_ORANGE, 3: CyberpunkStyle.ACCENT_RED}
            risk_level = status['stability']['fall_risk']
            self.status_labels['fall_risk'][0].config(text=risk_levels.get(risk_level, "N/A"), fg=risk_colors.get(risk_level))

            # Walking status updates
            walking_data = status['walking']
            self.walking_status_labels['enabled'].config(
                text="ACTIVE" if status['control']['walking_enabled'] else "INACTIVE",
                fg=CyberpunkStyle.ACCENT_GREEN if status['control']['walking_enabled'] else CyberpunkStyle.ACCENT_RED
            )
            self.walking_status_labels['gait'].config(text=walking_data['gait'].upper())
            self.walking_status_labels['speed'].config(text=f"{walking_data['speed']:.1f}x")
            
            direction_text = f"F:{walking_data['direction'][0]:.1f} L:{walking_data['direction'][1]:.1f}"
            self.walking_status_labels['direction'].config(text=direction_text)
            self.walking_status_labels['phase'].config(text=f"{walking_data['phase']:.2f}")
            self.walking_status_labels['cycle'].config(text=f"{walking_data['cycle_time']:.1f}s")

            # Leg state updates
            for leg_name, leg_data in walking_data['leg_states'].items():
                if leg_name in self.leg_state_labels:
                    state_text = "SWING" if leg_data['is_swing'] else "SUPPORT"
                    state_color = CyberpunkStyle.ACCENT_ORANGE if leg_data['is_swing'] else CyberpunkStyle.ACCENT_GREEN
                    
                    self.leg_state_labels[leg_name]['state'].config(text=state_text, fg=state_color)
                    self.leg_state_labels[leg_name]['phase'].config(text=f"Phase: {leg_data['phase']:.2f}")

            # Update GUI controls to match controller state
            self.balance_enabled_var.set(status['control']['balance_enabled'])
            self.walking_enabled_var.set(status['control']['walking_enabled'])
            self.control_mode_var.set(status['control']['mode'])
            self.gait_var.set(walking_data['gait'])

        except Exception as e:
            print(f"GUI update error: {e}")
            
    def update_plot(self):
        try:
            self.roll_line.set_ydata(self.roll_data)
            self.pitch_line.set_ydata(self.pitch_data)
            self.canvas.draw_idle()
        except Exception as e:
            print(f"Plot update error: {e}")
            
    def on_closing(self):
        self.is_running = False
        time.sleep(0.1)
        if hasattr(self, 'controller'):
            self.controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()
        
    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

if __name__ == '__main__':
    try:
        app = QuadrupedGUI()
        app.run()
    except Exception as e:
        print(f"Application failed to start: {e}")
