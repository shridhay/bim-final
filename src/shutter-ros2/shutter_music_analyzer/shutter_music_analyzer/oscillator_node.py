#!/usr/bin/env python3

"""
ROS2 node that subscribes to music feature topics and generates
oscillator-driven joint motion commands for all 4 joints.

This node implements Phase 3: Connecting Audio Features to Oscillator.
Each joint has its own oscillator with phase offsets for coordinated motion.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from std_msgs.msg import Float64MultiArray
import math
import time

from shutter_music_analyzer.oscillator import MusicDrivenOscillator


class OscillatorControlNode(Node):
    def __init__(self):
        super().__init__('oscillator_control')
        
        # Parameters
        self.declare_parameter('k_tempo', 0.1)  # Tempo sensitivity
        self.declare_parameter('k_energy', 0.5)  # Energy sensitivity
        self.declare_parameter('base_amplitude', 0.5)  # Base amplitude A_0
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('enable_beat_sync', True)  # Sync phase to beats
        self.declare_parameter('use_phase_offsets', True)  # Use phase offsets for coordinated motion
        
        k_tempo = self.get_parameter('k_tempo').value
        k_energy = self.get_parameter('k_energy').value
        base_amp = self.get_parameter('base_amplitude').value
        self.enable_beat_sync = self.get_parameter('enable_beat_sync').value
        use_phase_offsets = self.get_parameter('use_phase_offsets').value
        
        # Create 4 oscillators, one for each joint
        # Phase offsets create coordinated motion: each joint starts at a different point in the sine wave
        # Joint 0: starts at 0° (sin wave), Joint 1: starts at 90° (cos wave), etc.
        # This makes the joints move in a wave pattern instead of all moving together
        phase_offsets = [0.0, math.pi/2, math.pi, 3*math.pi/2] if use_phase_offsets else [0.0, 0.0, 0.0, 0.0]
        
        self.oscillators = []
        for i, phase_offset in enumerate(phase_offsets):
            osc = MusicDrivenOscillator(
                amplitude=base_amp,
                frequency=1.0,
                phase=phase_offset,  # Each joint has different phase
                k_tempo=k_tempo,
                k_energy=k_energy,
                base_amplitude=base_amp
            )
            self.oscillators.append(osc)
        
        # Current musical features (shared across all oscillators)
        # These get updated when music topics publish new values
        self.current_tempo = None
        self.current_energy = None
        self.beat_times = []
        self.last_beat_time = None
        
        # Time tracking: measure actual elapsed time for accurate motion
        # node_start_time: when this node started (for absolute time reference)
        # last_update_time: when control_loop last ran (for calculating dt)
        self.node_start_time = time.time()
        self.last_update_time = None
        
        # Subscribers: listen to music analysis topics
        # These callbacks update oscillator parameters (frequency, amplitude) when music changes
        self.tempo_sub = self.create_subscription(
            Float32,
            'music/tempo',
            self.tempo_callback,
            10
        )
        
        self.energy_sub = self.create_subscription(
            Float32,
            'music/current_energy',
            self.energy_callback,
            10
        )
        
        self.beat_times_sub = self.create_subscription(
            Float32MultiArray,
            'music/beat_times',
            self.beat_times_callback,
            10
        )
        
        # Publisher: sends joint position commands to the hardware interface
        # The hardware interface reads these and moves the physical motors
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_group_controller/command',
            10
        )
        
        # Control loop timer: ROS2 automatically calls control_loop() at regular intervals
        # This is like timer_callback in motor_startup_publisher, but uses actual elapsed time
        control_rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info(f'Oscillator control node started (4-joint control)')
        self.get_logger().info(f'  k_tempo: {k_tempo}, k_energy: {k_energy}, base_amplitude: {base_amp}')
        self.get_logger().info(f'  Phase offsets: {phase_offsets}')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')
    
    def tempo_callback(self, msg):
        """Called automatically when music/tempo topic publishes a new tempo value"""
        # This updates the oscillator frequency (how fast they oscillate)
        # Higher tempo = faster oscillation
        tempo_bpm = msg.data
        self.current_tempo = tempo_bpm
        
        # Update ALL oscillators with the new tempo
        for osc in self.oscillators:
            osc.update_from_tempo(tempo_bpm)
        
        self.get_logger().debug(f'Updated tempo: {tempo_bpm:.1f} BPM → ω = {self.oscillators[0].omega:.3f} rad/s')
    
    def energy_callback(self, msg):
        """Called automatically when music/current_energy topic publishes a new energy value"""
        # This updates the oscillator amplitude (how far they move)
        # Higher energy = larger movement range
        energy = msg.data
        self.current_energy = energy
        
        # Update ALL oscillators with the new energy
        for osc in self.oscillators:
            osc.update_from_energy(energy)
        
        self.get_logger().debug(f'Updated energy: {energy:.3f} → A = {self.oscillators[0].A:.3f}')
    
    def beat_times_callback(self, msg):
        """Called automatically when music/beat_times topic publishes upcoming beat times"""
        # Beat sync resets the oscillator phase to align with the music beat
        # This makes the robot motion "in sync" with the music rhythm
        self.beat_times = list(msg.data)
        if len(self.beat_times) > 0 and self.enable_beat_sync:
            # Find the next upcoming beat and sync all oscillators to it
            current_time = time.time() - self.node_start_time
            upcoming_beats = [bt for bt in self.beat_times if bt >= current_time]
            if len(upcoming_beats) > 0:
                next_beat = upcoming_beats[0]
                # Sync ALL oscillators to the beat (adjusts their phase)
                for osc in self.oscillators:
                    osc.sync_to_beat(next_beat, current_time)
                self.last_beat_time = next_beat
                self.get_logger().debug(f'Synced all oscillators to beat at t={next_beat:.3f}s')
    
    def control_loop(self):
        """
        Main control loop - called automatically by ROS2 timer at control_rate Hz
        
        This is the heart of the node: it calculates current joint positions from oscillators
        and publishes them to the hardware. Runs continuously while the node is active.
        """
        # Calculate how much time has passed since last loop iteration
        # dt = actual elapsed time (handles timer jitter better than fixed increments)
        current_time = time.time() - self.node_start_time
        
        if self.last_update_time is None:
            # First call: use expected time step
            dt = 1.0 / self.get_parameter('control_rate').value
        else:
            # Subsequent calls: use actual elapsed time
            dt = current_time - self.last_update_time
        
        self.last_update_time = current_time
        
        # Step each oscillator forward by dt and get current joint position
        # .step(dt) advances the oscillator's internal time and calculates: A sin(ωt + φ)
        # This is different from update_from_tempo/energy which change parameters
        joint_positions = []
        for osc in self.oscillators:
            joint_position = osc.step(dt)  # Returns current angle: A sin(ωt + φ)
            joint_positions.append(joint_position)
        
        # Package all 4 joint positions into a message
        joint_cmd = Float64MultiArray()
        joint_cmd.data = joint_positions  # [θ₀, θ₁, θ₂, θ₃] - one angle per joint
        
        # Send command to hardware interface (which moves the physical motors)
        self.joint_cmd_pub.publish(joint_cmd)
        
        # Log periodically
        if int(current_time * 10) % 50 == 0:  # Every 5 seconds
            self.get_logger().info(
                f't={current_time:.2f}s: Joints=[{joint_positions[0]:.3f}, {joint_positions[1]:.3f}, '
                f'{joint_positions[2]:.3f}, {joint_positions[3]:.3f}], '
                f'A={self.oscillators[0].A:.3f}, ω={self.oscillators[0].omega:.3f}, '
                f'tempo={self.current_tempo or "N/A"}, energy={self.current_energy or "N/A"}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OscillatorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
