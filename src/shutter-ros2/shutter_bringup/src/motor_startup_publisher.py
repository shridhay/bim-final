#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class MotorStartupPublisher(Node):
    def __init__(self):
        super().__init__('motor_startup_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_controller/command', 10)

        # ============================================
        # MOVEMENT PATTERNS (cycles every 4 seconds)
        # ============================================
        # Pattern 1: Slow small movements
        self.pattern1_amplitude = math.pi / 8   # Smaller amplitude (~22.5°)
        self.pattern1_frequency = 0.5           # Slow frequency
        
        # Pattern 2: Faster small movements
        self.pattern2_amplitude = math.pi / 8   # Smaller amplitude (same)
        self.pattern2_frequency = 1.5           # Faster frequency
        
        # Pattern 3: Medium pace small movements
        self.pattern3_amplitude = math.pi / 8   # Smaller amplitude (same)
        self.pattern3_frequency = 1.0           # Medium frequency
        
        self.pattern_change_interval = 4.0      # Change pattern every 4 seconds
        # ============================================

        # Current values (will change over time)
        self.amplitude = self.pattern1_amplitude
        self.frequency = self.pattern1_frequency
        self.update_rate = 50.0           # Control loop rate in Hz

        # Joint positions [motor1, motor2, motor3, motor4]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

        self.angle = 0.0  # global phase angle
        self.start_time = time.time()
        self.current_pattern = 1  # Start with pattern 1
        
        # Timer for control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        # Timer for pattern changes
        self.pattern_timer = self.create_timer(self.pattern_change_interval, self.change_pattern)
        
        self.get_logger().info(f'Motor startup publisher: Cycling through 3 patterns every {self.pattern_change_interval}s')
        self.get_logger().info(f'  Pattern 1: amplitude={self.pattern1_amplitude:.3f} rad, frequency={self.pattern1_frequency} Hz (slow small)')
        self.get_logger().info(f'  Pattern 2: amplitude={self.pattern2_amplitude:.3f} rad, frequency={self.pattern2_frequency} Hz (faster small)')
        self.get_logger().info(f'  Pattern 3: amplitude={self.pattern3_amplitude:.3f} rad, frequency={self.pattern3_frequency} Hz (medium small)')
    
    def change_pattern(self):
        """Called every 4 seconds to change movement pattern"""
        self.current_pattern = (self.current_pattern % 3) + 1  # Cycle 1->2->3->1
        
        if self.current_pattern == 1:
            self.amplitude = self.pattern1_amplitude
            self.frequency = self.pattern1_frequency
            self.get_logger().info('Changed to Pattern 1: Slow small movements')
        elif self.current_pattern == 2:
            self.amplitude = self.pattern2_amplitude
            self.frequency = self.pattern2_frequency
            self.get_logger().info('Changed to Pattern 2: Faster small movements')
        else:  # pattern 3
            self.amplitude = self.pattern3_amplitude
            self.frequency = self.pattern3_frequency
            self.get_logger().info('Changed to Pattern 3: Medium pace small movements')

    def timer_callback(self):
        # Calculate angle increment based on frequency
        # frequency (Hz) * 2π (radians per cycle) / update_rate (updates per second)
        angle_increment = (self.frequency * 2 * math.pi) / self.update_rate
        self.angle += angle_increment

        # Each motor oscillates with different phase offsets using cosine
        self.joint_positions[0] = self.amplitude * math.cos(self.angle)          # Motor1
        self.joint_positions[1] = self.amplitude * math.cos(self.angle + math.pi/2)  # Motor2
        self.joint_positions[2] = self.amplitude * math.cos(self.angle + math.pi)    # Motor3
        self.joint_positions[3] = self.amplitude * math.cos(self.angle + 3*math.pi/2)  # Motor4

        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorStartupPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
