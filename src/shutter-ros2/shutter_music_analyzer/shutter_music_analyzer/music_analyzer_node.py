#!/usr/bin/env python3

"""
ROS2 node for analyzing pre-recorded music files and publishing beat information.
Loads a song file and extracts beat timing for robot choreography.
Supports continuous feature updates synchronized with playback time.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import librosa
import os
import time
from std_msgs.msg import Float32MultiArray, Float32


class MusicBeatAnalyzerNode(Node):
    def __init__(self):
        super().__init__('music_beat_analyzer')
        
        # Parameters
        self.declare_parameter('music_file', '/home/user/song.wav')
        self.declare_parameter('publish_interval', 0.1)  # seconds
        self.declare_parameter('playback_start_time', 0.0)  # offset for synchronization
        
        music_file = self.get_parameter('music_file').value
        self.publish_interval = self.get_parameter('publish_interval').value
        self.playback_start_time = self.get_parameter('playback_start_time').value
        
        # Publishers
        self.beat_times_pub = self.create_publisher(Float32MultiArray, 'music/beat_times', 10)
        self.tempo_pub = self.create_publisher(Float32, 'music/tempo', 10)
        self.energy_pub = self.create_publisher(Float32MultiArray, 'music/energy', 10)
        self.onset_pub = self.create_publisher(Float32MultiArray, 'music/onsets', 10)
        self.current_energy_pub = self.create_publisher(Float32, 'music/current_energy', 10)
        
        # Store analyzed features
        self.tempo = None
        self.beat_times = None
        self.onset_times = None
        self.energy = None
        self.energy_times = None
        self.audio_duration = 0.0
        
        # Playback tracking
        self.node_start_time = None
        self.is_playing = False
        
        # Load and analyze music upfront
        if self.analyze_song(music_file):
            self.get_logger().info('Music beat analyzer node started')
            # Start continuous updates
            self.node_start_time = time.time()
            self.is_playing = True
            self.timer = self.create_timer(self.publish_interval, self.timer_callback)
        else:
            self.get_logger().error('Failed to analyze song. Node will not publish updates.')
    
    def analyze_song(self, music_file):
        """Load song and extract all features upfront"""
        try:
            # Check if file exists
            if not os.path.exists(music_file):
                self.get_logger().error(f'Music file not found: {music_file}')
                return False
            
            self.get_logger().info(f'Loading music file: {music_file}')
            
            # Load audio file
            y, sr = librosa.load(music_file, sr=None)
            self.audio_duration = len(y) / sr
            self.get_logger().info(f'Loaded audio: {len(y)} samples at {sr} Hz, duration: {self.audio_duration:.2f}s')
            
            # Detect tempo and beats
            tempo, beats = librosa.beat.beat_track(y=y, sr=sr)
            self.tempo = float(tempo)
            
            # Convert beat frames to time (seconds)
            self.beat_times = librosa.frames_to_time(beats, sr=sr)
            
            # Detect onsets
            onset_frames = librosa.onset.onset_detect(y=y, sr=sr)
            self.onset_times = librosa.frames_to_time(onset_frames, sr=sr)
            
            # Compute energy over time
            S = np.abs(librosa.stft(y))
            energy = np.sqrt(np.sum(S ** 2, axis=0))
            
            # Normalize energy
            if np.max(energy) > 0:
                energy = energy / np.max(energy)
            
            # Convert energy frames to time
            self.energy_times = librosa.frames_to_time(np.arange(len(energy)), sr=sr)
            self.energy = energy
            
            # Publish initial full feature set
            self.publish_all_features()
            
            self.get_logger().info(f'Tempo: {self.tempo:.1f} BPM')
            self.get_logger().info(f'Found {len(self.beat_times)} beats')
            self.get_logger().info(f'Found {len(self.onset_times)} onsets')
            self.get_logger().info(f'Energy samples: {len(self.energy)}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error analyzing music: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def publish_all_features(self):
        """Publish complete feature sets (called once at startup)"""
        # Publish tempo
        tempo_msg = Float32()
        tempo_msg.data = self.tempo
        self.tempo_pub.publish(tempo_msg)
        
        # Publish beat times
        beat_msg = Float32MultiArray()
        beat_msg.data = self.beat_times.tolist()
        self.beat_times_pub.publish(beat_msg)
        
        # Publish onset times
        onset_msg = Float32MultiArray()
        onset_msg.data = self.onset_times.tolist()
        self.onset_pub.publish(onset_msg)
        
        # Publish energy array
        energy_msg = Float32MultiArray()
        energy_msg.data = self.energy.tolist()
        self.energy_pub.publish(energy_msg)
    
    def timer_callback(self):
        """Publish current-time features synchronized with playback"""
        if not self.is_playing or self.tempo is None:
            return
        
        # Calculate current playback time
        current_time = time.time() - self.node_start_time + self.playback_start_time
        
        # Check if we've reached the end
        if current_time >= self.audio_duration:
            self.get_logger().info('Reached end of audio file')
            self.is_playing = False
            return
        
        # Publish tempo (constant, but republish periodically)
        tempo_msg = Float32()
        tempo_msg.data = self.tempo
        self.tempo_pub.publish(tempo_msg)
        
        # Publish current energy value (interpolated from energy array)
        current_energy = self.get_energy_at_time(current_time)
        energy_msg = Float32()
        energy_msg.data = current_energy
        self.current_energy_pub.publish(energy_msg)
        
        # Optionally publish upcoming beats/onsets in a window
        # (This could be useful for anticipatory control)
        upcoming_beats = self.get_upcoming_beats(current_time, window=2.0)
        if len(upcoming_beats) > 0:
            beat_msg = Float32MultiArray()
            beat_msg.data = upcoming_beats.tolist()
            self.beat_times_pub.publish(beat_msg)
    
    def get_energy_at_time(self, t):
        """Get energy value at a specific time (interpolated)"""
        if self.energy is None or len(self.energy) == 0:
            return 0.0
        
        # Find closest time index
        idx = np.searchsorted(self.energy_times, t)
        
        if idx >= len(self.energy):
            return self.energy[-1]
        elif idx == 0:
            return self.energy[0]
        else:
            # Linear interpolation
            t1, t2 = self.energy_times[idx-1], self.energy_times[idx]
            e1, e2 = self.energy[idx-1], self.energy[idx]
            if t2 > t1:
                alpha = (t - t1) / (t2 - t1)
                return float(e1 + alpha * (e2 - e1))
            return float(e1)
    
    def get_upcoming_beats(self, current_time, window=2.0):
        """Get beats within a time window from current time"""
        if self.beat_times is None:
            return np.array([])
        
        mask = (self.beat_times >= current_time) & (self.beat_times <= current_time + window)
        return self.beat_times[mask]


def main(args=None):
    rclpy.init(args=args)
    node = MusicBeatAnalyzerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

