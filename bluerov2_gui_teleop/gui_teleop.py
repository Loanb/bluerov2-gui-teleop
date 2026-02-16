#!/usr/bin/env python3
"""
Keyboard teleoperation node for BlueROV2.

Reads keyboard input and publishes commands to the ROV.
Also displays telemetry feedback in a GUI.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Wrench, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import math
import sys
import signal
import threading
import tkinter as tk
from tkinter import font
from time import time
from queue import Queue
from pynput import keyboard
try:
    from cv_bridge import CvBridge
    import cv2
    from PIL import Image as PILImage, ImageTk
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False


class KeyboardTeleop(Node):
    """Keyboard teleoperation node for BlueROV2."""

    _instance = None

    def __init__(self):
        super().__init__('keyboard_teleop')
        KeyboardTeleop._instance = self
        
        # Declare parameters
        self.declare_parameter('namespace', 'bluerov2')
        self.declare_parameter('depth_sign', -1)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('buoyancy_compensation', -15.0)  # Force to counteract positive buoyancy
        
        namespace = self.get_parameter('namespace').value
        self.depth_sign = self.get_parameter('depth_sign').value
        self.show_gui = self.get_parameter('show_gui').value
        self.buoyancy_comp = self.get_parameter('buoyancy_compensation').value
        
        # Topic names
        self.cmd_topic = f'/{namespace}/wrench'
        self.odom_topic = f'/{namespace}/odom'
        
        # Publishers and subscribers
        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Wrench, self.cmd_topic, qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos
        )
        
        # Camera subscriber
        self.camera_topic = f'/{namespace}/image'
        if CAMERA_AVAILABLE:
            self.cv_bridge = CvBridge()
            self.latest_image = None
            self.camera_sub = self.create_subscription(
                Image, self.camera_topic, self._camera_callback, qos
            )
        
        # State variables
        self.armed = False
        self.gain = 1.0
        self.max_gain = 2.0
        self.min_gain = 0.2
        self.gain_step = 0.1
        
        # Command state
        self.fx_cmd = 0.0
        self.fy_cmd = 0.0
        self.fz_cmd = 0.0
        self.tz_cmd = 0.0
        
        # Button press state (for GUI buttons)
        self.button_pressed = {
            'Z': False, 'S': False, 'Q': False, 'D': False,
            'A': False, 'E': False, 'R': False, 'F': False
        }
        
        # Feedback state
        self.speed = 0.0
        self.depth = 0.0
        self.last_display_time = time()
        self.display_interval = 0.1
        
        # Keyboard
        self.key_queue = Queue()
        self.keyboard_thread = None
        self.keyboard_running = False
        
        # GUI window
        self.root = None
        self.gui_thread = None
        self.gui_labels = {}
        self.gui_running = False
        self.camera_label = None
        self.camera_image_id = None
        
        # Timer
        self.create_timer(0.05, self._control_loop)
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Start keyboard reader thread
        self._start_keyboard_thread()
        
        # Start GUI if enabled
        if self.show_gui:
            self._start_gui()
        
        self.get_logger().info(f'Keyboard teleop started. Namespace: {namespace}')
        self.get_logger().info('Commands: Z/S=fwd/bwd, Q/D=left/right, A/E=yaw, R/F=up/down')
        self.get_logger().info('P=arm/disarm, +/-=gain, SPACE=stop')

    def _start_keyboard_thread(self):
        """Start background thread to read keyboard using pynput."""
        self.keyboard_running = True
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()
        self.get_logger().info('Keyboard listener started (pynput)')

    def _on_key_press(self, key):
        """Handle key press events from pynput."""
        try:
            if not self.keyboard_running:
                return False
            
            # Handle character keys
            if hasattr(key, 'char') and key.char:
                self.key_queue.put(key.char.upper())
            # Handle special keys
            elif key == keyboard.Key.space:
                self.key_queue.put(' ')
            elif key == keyboard.Key.ctrl_c or key == keyboard.Key.ctrl:
                # Allow Ctrl+C to work
                return False
        except Exception as e:
            self.get_logger().warn(f'Key press error: {e}')
        return True

    def _gui_run(self):
        """Run the GUI main loop."""
        try:
            self.root = tk.Tk()
            self.root.title('BlueROV2 Control Panel')
            self.root.geometry('1100x700')
            self.root.resizable(False, False)
            
            # Configure style
            bg_color = '#1e1e1e'
            fg_color = '#ffffff'
            self.root.configure(bg=bg_color)
            
            title_font = font.Font(family='Arial', size=14, weight='bold')
            label_font = font.Font(family='Courier', size=11)
            value_font = font.Font(family='Courier', size=16, weight='bold')
            
            # Main container with two columns
            main_container = tk.Frame(self.root, bg=bg_color)
            main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            # LEFT COLUMN: Camera feed
            left_frame = tk.Frame(main_container, bg=bg_color)
            left_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 10))
            
            # Camera title
            cam_title = tk.Label(left_frame, text='📹 Camera View', 
                               font=title_font, bg=bg_color, fg='#00ffff')
            cam_title.pack(pady=5)
            
            # Camera display (640x400)
            if CAMERA_AVAILABLE:
                self.camera_label = tk.Label(left_frame, bg='#000000', 
                                            width=640, height=400)
                self.camera_label.pack()
                # Start camera update timer
                self.root.after(50, self._update_camera_display)
            else:
                no_cam = tk.Label(left_frame, text='Camera not available\nInstall: cv_bridge, opencv-python, pillow',
                                font=label_font, bg='#000000', fg='#ff0000',
                                width=80, height=25)
                no_cam.pack()
            
            # RIGHT COLUMN: Telemetry and controls
            right_frame = tk.Frame(main_container, bg=bg_color)
            right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            
            # Title
            title = tk.Label(right_frame, text='BlueROV2 Control', 
                           font=title_font, bg=bg_color, fg=fg_color)
            title.pack(pady=10)
            
            # Armed status
            frame1 = tk.Frame(right_frame, bg=bg_color)
            frame1.pack(fill=tk.X, padx=20, pady=5)
            tk.Label(frame1, text='Status:', font=label_font, bg=bg_color, fg=fg_color).pack(side=tk.LEFT)
            self.gui_labels['armed'] = tk.Label(frame1, text='DISARMED', 
                                               font=value_font, bg=bg_color, fg='red')
            self.gui_labels['armed'].pack(side=tk.LEFT, padx=20)
            
            # Speed
            frame2 = tk.Frame(right_frame, bg=bg_color)
            frame2.pack(fill=tk.X, padx=20, pady=5)
            tk.Label(frame2, text='Speed:', font=label_font, bg=bg_color, fg=fg_color).pack(side=tk.LEFT)
            self.gui_labels['speed'] = tk.Label(frame2, text='0.00 m/s', 
                                               font=value_font, bg=bg_color, fg='#00ff00')
            self.gui_labels['speed'].pack(side=tk.LEFT, padx=20)
            
            # Depth
            frame3 = tk.Frame(right_frame, bg=bg_color)
            frame3.pack(fill=tk.X, padx=20, pady=5)
            tk.Label(frame3, text='Depth:', font=label_font, bg=bg_color, fg=fg_color).pack(side=tk.LEFT)
            self.gui_labels['depth'] = tk.Label(frame3, text='0.00 m', 
                                               font=value_font, bg=bg_color, fg='#00ff00')
            self.gui_labels['depth'].pack(side=tk.LEFT, padx=20)
            
            # Gain
            frame4 = tk.Frame(right_frame, bg=bg_color)
            frame4.pack(fill=tk.X, padx=20, pady=5)
            tk.Label(frame4, text='Gain:', font=label_font, bg=bg_color, fg=fg_color).pack(side=tk.LEFT)
            self.gui_labels['gain'] = tk.Label(frame4, text='1.0x', 
                                              font=value_font, bg=bg_color, fg='#ffff00')
            self.gui_labels['gain'].pack(side=tk.LEFT, padx=20)
            
            # Commands
            frame5 = tk.Frame(right_frame, bg=bg_color)
            frame5.pack(fill=tk.X, padx=20, pady=10)
            self.gui_labels['cmd'] = tk.Label(frame5, 
                                             text='Fx=   0.0 Fy=   0.0 Fz=   0.0 Tz=   0.0',
                                             font=font.Font(family='Courier', size=10),
                                             bg=bg_color, fg='#aaaaaa', justify=tk.LEFT)
            self.gui_labels['cmd'].pack(side=tk.LEFT)
            
            # Separator
            tk.Frame(right_frame, height=2, bg='#444444').pack(fill=tk.X, padx=20, pady=10)
            
            # Control Buttons Section
            btn_font = font.Font(family='Arial', size=10, weight='bold')
            btn_width = 5
            btn_height = 1
            
            controls_frame = tk.Frame(right_frame, bg=bg_color)
            controls_frame.pack(pady=10)
            
            # ARM/DISARM and STOP buttons
            top_controls = tk.Frame(controls_frame, bg=bg_color)
            top_controls.pack(pady=5)
            
            self.btn_arm = tk.Button(top_controls, text='ARM\n(P)', font=btn_font, 
                                     width=btn_width, height=btn_height, bg='#ff4444', fg='white',
                                     command=lambda: self.key_queue.put('P'))
            self.btn_arm.pack(side=tk.LEFT, padx=5)
            
            tk.Button(top_controls, text='STOP\n(SPC)', font=btn_font, 
                     width=btn_width, height=btn_height, bg='#ff8800', fg='white',
                     command=lambda: self.key_queue.put(' ')).pack(side=tk.LEFT, padx=5)
            
            # Movement buttons (3 sections)
            button_section = tk.Frame(controls_frame, bg=bg_color)
            button_section.pack(pady=10)
            
            # Horizontal movement (left column)
            horiz_frame = tk.Frame(button_section, bg=bg_color)
            horiz_frame.pack(side=tk.LEFT, padx=10)
            tk.Label(horiz_frame, text='Horizontal', font=label_font, 
                    bg=bg_color, fg='#00ffff').pack()
            
            # Forward (Z)
            btn_z = tk.Button(horiz_frame, text='↑\nZ', font=btn_font, width=btn_width, height=btn_height,
                     bg='#2222aa', fg='white')
            btn_z.bind('<ButtonPress-1>', lambda e: self._button_press('Z'))
            btn_z.bind('<ButtonRelease-1>', lambda e: self._button_release('Z'))
            btn_z.pack(pady=2)
            
            # Left/Right (Q/D)
            lr_frame = tk.Frame(horiz_frame, bg=bg_color)
            lr_frame.pack()
            btn_q = tk.Button(lr_frame, text='←\nQ', font=btn_font, width=btn_width, height=btn_height,
                     bg='#2222aa', fg='white')
            btn_q.bind('<ButtonPress-1>', lambda e: self._button_press('Q'))
            btn_q.bind('<ButtonRelease-1>', lambda e: self._button_release('Q'))
            btn_q.pack(side=tk.LEFT, padx=2)
            
            btn_d = tk.Button(lr_frame, text='→\nD', font=btn_font, width=btn_width, height=btn_height,
                     bg='#2222aa', fg='white')
            btn_d.bind('<ButtonPress-1>', lambda e: self._button_press('D'))
            btn_d.bind('<ButtonRelease-1>', lambda e: self._button_release('D'))
            btn_d.pack(side=tk.LEFT, padx=2)
            
            # Backward (S)
            btn_s = tk.Button(horiz_frame, text='↓\nS', font=btn_font, width=btn_width, height=btn_height,
                     bg='#2222aa', fg='white')
            btn_s.bind('<ButtonPress-1>', lambda e: self._button_press('S'))
            btn_s.bind('<ButtonRelease-1>', lambda e: self._button_release('S'))
            btn_s.pack(pady=2)
            
            # Vertical movement (middle column)
            vert_frame = tk.Frame(button_section, bg=bg_color)
            vert_frame.pack(side=tk.LEFT, padx=10)
            tk.Label(vert_frame, text='Vertical', font=label_font, 
                    bg=bg_color, fg='#00ff00').pack()
            
            btn_r = tk.Button(vert_frame, text='↑ UP\nR', font=btn_font, width=btn_width, height=btn_height,
                     bg='#22aa22', fg='white')
            btn_r.bind('<ButtonPress-1>', lambda e: self._button_press('R'))
            btn_r.bind('<ButtonRelease-1>', lambda e: self._button_release('R'))
            btn_r.pack(pady=5)
            
            btn_f = tk.Button(vert_frame, text='↓ DOWN\nF', font=btn_font, width=btn_width, height=btn_height,
                     bg='#22aa22', fg='white')
            btn_f.bind('<ButtonPress-1>', lambda e: self._button_press('F'))
            btn_f.bind('<ButtonRelease-1>', lambda e: self._button_release('F'))
            btn_f.pack(pady=5)
            
            # Rotation (right column)
            rot_frame = tk.Frame(button_section, bg=bg_color)
            rot_frame.pack(side=tk.LEFT, padx=10)
            tk.Label(rot_frame, text='Rotation', font=label_font, 
                    bg=bg_color, fg='#ffff00').pack()
            
            btn_a = tk.Button(rot_frame, text='↶ LEFT\nA', font=btn_font, width=btn_width, height=btn_height,
                     bg='#aaaa22', fg='white')
            btn_a.bind('<ButtonPress-1>', lambda e: self._button_press('A'))
            btn_a.bind('<ButtonRelease-1>', lambda e: self._button_release('A'))
            btn_a.pack(pady=5)
            
            btn_e = tk.Button(rot_frame, text='↷ RIGHT\nE', font=btn_font, width=btn_width, height=btn_height,
                     bg='#aaaa22', fg='white')
            btn_e.bind('<ButtonPress-1>', lambda e: self._button_press('E'))
            btn_e.bind('<ButtonRelease-1>', lambda e: self._button_release('E'))
            btn_e.pack(pady=5)
            
            # Gain controls
            gain_frame = tk.Frame(controls_frame, bg=bg_color)
            gain_frame.pack(pady=10)
            tk.Label(gain_frame, text='Gain:', font=label_font, 
                    bg=bg_color, fg='#ffffff').pack(side=tk.LEFT, padx=5)
            tk.Button(gain_frame, text='−', font=font.Font(size=20, weight='bold'), 
                     width=3, height=1, bg='#666666', fg='white',
                     command=lambda: self.key_queue.put('-')).pack(side=tk.LEFT, padx=2)
            tk.Button(gain_frame, text='+', font=font.Font(size=20, weight='bold'), 
                     width=3, height=1, bg='#666666', fg='white',
                     command=lambda: self.key_queue.put('+')).pack(side=tk.LEFT, padx=2)
            
            # Set window close protocol
            self.root.protocol("WM_DELETE_WINDOW", self._on_gui_close)
            
            # Run the GUI main loop
            self.root.mainloop()
                    
        except Exception as e:
            pass  # GUI creation failed, continue without GUI

    def _on_gui_close(self):
        """Handle GUI window close event."""
        self.gui_running = False
        if self.root:
            try:
                self.root.destroy()
            except Exception:
                pass
        self.root = None

    def _camera_callback(self, msg):
        """Handle camera image messages."""
        if not CAMERA_AVAILABLE:
            return
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize to fit display (640x400)
            cv_image = cv2.resize(cv_image, (640, 400))
            # Convert BGR to RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Store the latest image
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().warn(f'Camera callback error: {e}')

    def _update_camera_display(self):
        """Update camera display in GUI (called periodically)."""
        if not CAMERA_AVAILABLE or not self.gui_running or not self.camera_label:
            return
        
        try:
            if self.latest_image is not None:
                # Convert numpy array to PIL Image
                pil_image = PILImage.fromarray(self.latest_image)
                # Convert to PhotoImage for tkinter
                photo = ImageTk.PhotoImage(pil_image)
                # Update label
                self.camera_label.config(image=photo)
                self.camera_label.image = photo  # Keep a reference
            else:
                # No image yet - show waiting message
                if not hasattr(self, '_no_image_shown'):
                    self.camera_label.config(text='Waiting for camera feed...', 
                                           fg='#888888', font=('Arial', 14))
                    self._no_image_shown = True
        except Exception as e:
            pass
        
        # Schedule next update (20 FPS)
        if self.gui_running and self.root:
            self.root.after(50, self._update_camera_display)

    def _button_press(self, key):
        """Handle button press event from GUI."""
        self.button_pressed[key] = True
        if not self.armed:
            self.get_logger().info('ROV is DISARMED - press ARM first!')

    def _button_release(self, key):
        """Handle button release event from GUI."""
        self.button_pressed[key] = False

    def _read_key(self):
        """Read a key from the queue if available."""
        try:
            return self.key_queue.get_nowait()
        except:
            return None

    def _odom_callback(self, msg):
        """Handle odometry feedback."""
        # Extract speed (magnitude of linear velocity)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.speed = math.sqrt(vx**2 + vy**2 + vz**2)
        
        # Extract depth
        self.depth = msg.pose.pose.position.z * self.depth_sign

    def _publish_command(self):
        """Get next key from queue (non-blocking)."""
        try:
            wrench = Wrench()
            wrench.force = Vector3(x=self.fx_cmd, y=self.fy_cmd, z=self.fz_cmd)
            wrench.torque = Vector3(x=0.0, y=0.0, z=self.tz_cmd)
            self.cmd_pub.publish(wrench)
        except Exception:
            pass

    @staticmethod
    def _signal_handler(signum, frame):
        """Handle Ctrl+C signal."""
        if KeyboardTeleop._instance:
            KeyboardTeleop._instance.get_logger().info('Shutting down...')
            KeyboardTeleop._instance.shutdown()
            sys.exit(0)

    def _start_gui(self):
        """Start telemetry GUI in a separate thread."""
        self.gui_running = True
        self.gui_thread = threading.Thread(target=self._gui_run, daemon=True)
        self.gui_thread.start()

    def _display_telemetry(self):
        """Update GUI with telemetry."""
        if not self.show_gui or not self.root:
            return
            
        if time() - self.last_display_time >= self.display_interval:
            try:
                self.root.after(0, self._update_gui_labels)
            except Exception:
                pass
            self.last_display_time = time()

    def _update_gui_labels(self):
        """Update GUI labels (thread-safe)."""
        try:
            self.gui_labels['armed'].config(
                text=('DISARMED' if not self.armed else 'ARMED'),
                fg=('red' if not self.armed else 'green')
            )
            # Update ARM button color
            if hasattr(self, 'btn_arm'):
                self.btn_arm.config(bg=('#44ff44' if self.armed else '#ff4444'))
            
            self.gui_labels['speed'].config(text=f'{self.speed:.2f} m/s')
            self.gui_labels['depth'].config(text=f'{self.depth:.2f} m')
            self.gui_labels['gain'].config(text=f'{self.gain:.1f}x')
            self.gui_labels['cmd'].config(
                text=f'Fx={self.fx_cmd:6.1f} Fy={self.fy_cmd:6.1f} Fz={self.fz_cmd:6.1f} Tz={self.tz_cmd:6.1f}'
            )
        except Exception:
            pass

    def _control_loop(self):
        """Main control loop: read keyboard, update commands, publish."""
        # Read keyboard input (for ARM, STOP, gain controls)
        key = self._read_key()
        if key:
            self._handle_key(key)
        
        # Calculate commands based on pressed buttons
        max_force = 100.0 * self.gain  # Increased from 20.0 to overcome strong buoyancy
        max_torque = 20.0 * self.gain
        
        # Reset commands
        self.fx_cmd = 0.0
        self.fy_cmd = 0.0
        self.fz_cmd = 0.0
        self.tz_cmd = 0.0
        
        # Apply commands only if armed and buttons are pressed
        if self.armed:
            if self.button_pressed['Z']:  # Forward
                self.fx_cmd += max_force
            if self.button_pressed['S']:  # Backward
                self.fx_cmd -= max_force
            if self.button_pressed['Q']:  # Left
                self.fy_cmd -= max_force
            if self.button_pressed['D']:  # Right
                self.fy_cmd += max_force
            if self.button_pressed['A']:  # Yaw left
                self.tz_cmd += max_torque
            if self.button_pressed['E']:  # Yaw right
                self.tz_cmd -= max_torque
            if self.button_pressed['R']:  # Up
                self.fz_cmd += max_force
            if self.button_pressed['F']:  # Down
                self.fz_cmd -= max_force
            
            # Apply buoyancy compensation when underwater and no Z command
            if not self.button_pressed['R'] and not self.button_pressed['F']:
                if abs(self.depth) > 0.1:  # Only apply when underwater (>10cm depth)
                    self.fz_cmd = self.buoyancy_comp
        
        # Publish command
        self._publish_command()
        
        # Display telemetry
        self._display_telemetry()

    def _handle_key(self, key):
        """Handle keyboard input for ARM, STOP, gain, and movement."""
        # Arm/Disarm
        if key == 'P':
            self.armed = not self.armed
            armed_str = 'ARMED' if self.armed else 'DISARMED'
            self.get_logger().info(f'→ {armed_str}')
            return
        
        # Stop all commands
        if key == ' ':
            # Release all buttons
            for k in self.button_pressed:
                self.button_pressed[k] = False
            self.get_logger().info('→ All commands STOPPED')
            return
        
        # Gain adjustment
        if key == '+' or key == '=':
            self.gain = min(self.gain + self.gain_step, self.max_gain)
            self.get_logger().info(f'→ Gain: {self.gain:.1f}x')
            return
        
        if key == '-':
            self.gain = max(self.gain - self.gain_step, self.min_gain)
            self.get_logger().info(f'→ Gain: {self.gain:.1f}x')
            return
        
        # Movement commands from keyboard (toggle button state)
        if key in self.button_pressed:
            # Toggle button state on keyboard press
            self.button_pressed[key] = True

    def shutdown(self):
        """Cleanup on shutdown."""
        # Stop keyboard listener
        self.keyboard_running = False
        if hasattr(self, 'listener'):
            self.listener.stop()
        
        # Stop GUI
        self.gui_running = False
        if self.root:
            try:
                self.root.quit()  # Stop mainloop
                self.root.destroy()
            except Exception:
                pass
        
        # Send zero commands
        try:
            self.armed = False
            self.fx_cmd = 0.0
            self.fy_cmd = 0.0
            self.fz_cmd = 0.0
            self.tz_cmd = 0.0
            self._publish_command()
        except Exception:
            pass
        
        self.get_logger().info('Keyboard teleop shutdown')


def main():
    """Main entry point."""
    rclpy.init()
    node = KeyboardTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
