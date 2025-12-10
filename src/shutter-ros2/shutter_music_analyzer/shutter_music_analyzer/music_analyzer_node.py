#!/usr/bin/env python3

"""
ROS2 node for analyzing pre-recorded music files and publishing beat information.
Loads a song file and extracts beat timing for robot choreography.
Supports continuous feature updates synchronized with playback time.
Uses scipy instead of librosa for audio analysis.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy import signal
from scipy.io import wavfile
import os
import time
import subprocess
import shutil
import imageio_ffmpeg
from std_msgs.msg import Float32MultiArray, Float32

# Optional audio playback
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


class MusicBeatAnalyzerNode(Node):
    def __init__(self):
        super().__init__('music_beat_analyzer')

        # Parameters
        self.declare_parameter('music_file', '/home/user/song.wav')
        self.declare_parameter('publish_interval', 0.1)  # seconds
        self.declare_parameter('playback_start_time', 0.0)  # offset for synchronization
        self.declare_parameter('tempo_bpm', 0.0)  # 0 = auto-detect, otherwise use this value
        self.declare_parameter('play_audio', True)  # Play audio out loud

        music_file = self.get_parameter('music_file').value
        self.publish_interval = self.get_parameter('publish_interval').value
        self.playback_start_time = self.get_parameter('playback_start_time').value
        self.tempo_bpm = self.get_parameter('tempo_bpm').value
        self.play_audio = self.get_parameter('play_audio').value

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
        self.audio_player = None
        
        # Change tracking for logging
        self.last_energy = None
        self.energy_change_threshold = 0.05
        
        # Initialize audio playback if requested
        if self.play_audio:
            if PYGAME_AVAILABLE:
                try:
                    pygame.mixer.init()
                    self.get_logger().info('Audio playback enabled (pygame)')
                except Exception as e:
                    self.get_logger().warn(f'Failed to initialize audio playback: {e}')
                    self.play_audio = False
            else:
                self.get_logger().warn('pygame not available - install with: pip install pygame')
                self.get_logger().warn('Audio playback disabled')
                self.play_audio = False

        if self.analyze_song(music_file):
            self.get_logger().info('Music beat analyzer node started')
            self.node_start_time = time.time()
            self.is_playing = True
            
            if self.play_audio and PYGAME_AVAILABLE:
                self.start_audio_playback(music_file)
            
            self.timer = self.create_timer(self.publish_interval, self.timer_callback)
        else:
            self.get_logger().error('Failed to analyze song. Node will not publish updates.')
    
    def analyze_song(self, music_file):
        """Load song and extract features, with auto-repair for bad WAV formats."""
        try:
            if not os.path.exists(music_file):
                self.get_logger().error(f'Music file not found: {music_file}')
                return False

            self.get_logger().info(f'Loading music file: {music_file}')
            sr = None
            y = None
            
            needs_conversion = False
            if not music_file.lower().endswith('.wav'):
                self.get_logger().info('File is not a WAV. Converting to WAV for analysis...')
                needs_conversion = True
            else:
                try:
                    sr, y = wavfile.read(music_file)
                except ValueError:
                    self.get_logger().warn('Scipy failed to read WAV. Attempting repair...')
                    needs_conversion = True

            if needs_conversion:
                try:
                    import imageio_ffmpeg
                    ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
                    
                    # SAFE FILENAME HANDLING:
                    # song.mp3 -> song_clean.wav
                    # song.wav -> song_clean.wav
                    base_name, _ = os.path.splitext(music_file)
                    clean_file = base_name + "_clean.wav"
                    
                    cmd = [
                        ffmpeg_exe, '-y', '-v', 'error', 
                        '-i', music_file, 
                        '-acodec', 'pcm_s16le', 
                        '-ar', '44100', 
                        '-bitexact', 
                        clean_file
                    ]
                    
                    self.get_logger().info(f'Converting/Repairing audio...')
                    subprocess.run(cmd, check=True)
                    self.get_logger().info(f'Loaded converted file: {clean_file}')
                    
                    sr, y = wavfile.read(clean_file)
                    
                except Exception as e:
                    self.get_logger().error(f'Conversion failed: {e}')
                    return False

            if len(y.shape) > 1:
                y = np.mean(y, axis=1)
            
            if y.dtype != np.float32:
                y = y.astype(np.float32) / np.max(np.abs(y))
            
            self.audio_duration = len(y) / sr
            
            frequencies, times, Zxx = signal.stft(y, sr)
            S = np.abs(Zxx)
            
            energy = np.sqrt(np.sum(S ** 2, axis=0))

            if np.max(energy) > 0:
                energy = energy / np.max(energy)
            
            energy_min = np.min(energy)
            energy_max = np.max(energy)
            if energy_max > energy_min:
                energy = (energy - energy_min) / (energy_max - energy_min) 
            else:
                energy = np.zeros_like(energy) 

            self.energy = energy
            self.energy_times = times
            self.beat_times, self.onset_times = self.detect_beats_and_onsets(y, sr)

            if self.tempo_bpm > 0:
                self.tempo = float(self.tempo_bpm)
                self.get_logger().info(f'Using provided tempo: {self.tempo:.1f} BPM')
            else:
                self.tempo = self.calculate_tempo_from_beats()
                self.get_logger().info(f'Auto-detected tempo: {self.tempo:.1f} BPM')

            self.publish_all_features()

            self.get_logger().info(f'Tempo: {self.tempo:.1f} BPM')
            
            return True

        except Exception as e:
            self.get_logger().error(f'Error analyzing music: {e}')
            import traceback
            traceback.print_exc()
            return False

    def compute_onset_strength(self, y, sr):
        """Compute onset strength signal using spectral flux"""
        # Compute STFT
        frequencies, times, Zxx = signal.stft(y, sr)
        S = np.abs(Zxx)
        
        # Compute spectral flux (difference between consecutive frames)
        spectral_flux = np.sqrt(np.sum(np.diff(S, axis=1) ** 2, axis=0))
        spectral_flux = np.concatenate([[0], spectral_flux])
        
        # Smooth the signal
        spectral_flux = signal.savgol_filter(spectral_flux, window_length=11, polyorder=2)
        
        # Rectify (keep only positive changes)
        spectral_flux = np.maximum(spectral_flux, 0)
        
        return spectral_flux, times

    def estimate_beat_period_autocorr(self, onset_strength, times, sr):
        """Estimate beat period using provided BPM or autocorrelation"""
        if self.tempo_bpm > 0:
            beat_period = 60.0 / self.tempo_bpm
            self.get_logger().info(f'Using provided tempo: {self.tempo_bpm:.1f} BPM ({beat_period:.3f}s per beat)')
            return beat_period
        
        autocorr = np.correlate(onset_strength - np.mean(onset_strength), 
                            onset_strength - np.mean(onset_strength), 
                            mode='full')
        autocorr = autocorr[len(autocorr)//2:] 
        autocorr /= np.max(autocorr) if np.max(autocorr) > 0 else 1.0
        
        hop_length = 256 // 2
        frame_rate = sr / hop_length
        lags_time = np.arange(len(autocorr)) / frame_rate
        
        mask = (lags_time >= 0.3) & (lags_time <= 1.0)
        valid_autocorr = autocorr[mask]
        valid_lags = lags_time[mask]
        
        if len(valid_autocorr) > 0:
            peak_idx = np.argmax(valid_autocorr)
            beat_period = valid_lags[peak_idx]

            peak_strength = valid_autocorr[peak_idx]

            bpm_est = 60.0 / beat_period
            self.get_logger().info(f'Raw Autocorr Candidate: {bpm_est:.1f} BPM (Strength: {peak_strength:.2f})')
            # 2. HARMONIC CHECK (The Fix for Mambo No. 5)
            # Check if there is a strong peak at HALF the period (Double the Tempo)
            # This handles cases where the algorithm picks 87 BPM instead of 174 BPM
            if bpm_est < 100:
                half_period = beat_period / 2.0
                if half_period >= 0.3:
                    idx_half = np.argmin(np.abs(valid_lags - half_period))
                    strength_half = valid_autocorr[idx_half]
                    
                    if strength_half > 0.4 * peak_strength:
                        self.get_logger().info(f'BPM Correction (Slow->Fast): Switching to {60/half_period:.1f} BPM')
                        beat_period = half_period
            elif bpm_est > 120:
                double_period = beat_period * 2.0
                if double_period <= 1.0:
                    idx_double = np.argmin(np.abs(valid_lags - double_period))
                    strength_double = valid_autocorr[idx_double]
                    threshold = 0.95 if peak_strength > 0.7 else 0.75
                    
                    if strength_double > threshold * peak_strength:
                        self.get_logger().info(f'Correction (Fast->Slow): {60/beat_period:.1f} -> {60/double_period:.1f} BPM')
                        beat_period = double_period

        else:
            onset_intervals = np.diff(times[signal.find_peaks(onset_strength, height=0.05)[0]])
            if len(onset_intervals) > 10:
                hist, bin_edges = np.histogram(onset_intervals, bins=50, range=(0.1, 1.0))
                dominant_bin = np.argmax(hist)
                beat_period = (bin_edges[dominant_bin] + bin_edges[dominant_bin + 1]) / 2
            else:
                beat_period = 0.5 
        
        beat_period = float(np.clip(beat_period, 0.3, 1.0))
        self.get_logger().info(f'Auto-detected beat period: {beat_period:.3f}s ({60/beat_period:.1f} BPM)')
        return beat_period


    def detect_beats_and_onsets(self, y, sr):
        """Detect beat and onset times using improved peak finding with autocorrelation"""
        onset_strength, times = self.compute_onset_strength(y, sr)

        onset_strength = signal.savgol_filter(onset_strength, 11, 2)
        onset_strength /= np.max(onset_strength) if np.max(onset_strength) > 0 else 1.0

        beat_period = self.estimate_beat_period_autocorr(onset_strength, times, sr)
        min_beat_interval = beat_period
        
        self.get_logger().info(f'Debug - estimated beat period: {beat_period:.3f}s ({60/beat_period:.1f} BPM)')

        hop_length = 256 // 2
        frame_rate = sr / hop_length
        distance = int(frame_rate * 0.03)
        
        onset_threshold = np.percentile(onset_strength, 80) # top (100 - number)% of peaks
        onset_indices, _ = signal.find_peaks(onset_strength, height=onset_threshold, distance=distance)
        
        frame_times = times
        onset_times = frame_times[onset_indices]
        
        self.get_logger().info(f'Debug - raw onsets detected: {len(onset_times)}, threshold: {onset_threshold:.4f}')

        beat_times = []
        if len(onset_times) > 0:
            beat_times.append(onset_times[0])
            for ot in onset_times[1:]:
                # uses (1 - number)% variation for clustering
                if ot - beat_times[-1] >= min_beat_interval * 0.85:
                    beat_times.append(ot)
        
        beat_times = np.array(beat_times)
        
        self.get_logger().info(f'Debug - beat clustering with interval {min_beat_interval:.3f}s: {len(beat_times)} beats')

        return beat_times, onset_times

    def calculate_tempo_from_beats(self):
        """Calculate tempo from detected beats"""
        if self.beat_times is None or len(self.beat_times) < 4:
            return 100.0
        
        ibis = np.diff(self.beat_times)
        
        median_ibi = np.median(ibis)
        q1 = np.percentile(ibis, 25)
        q3 = np.percentile(ibis, 75)
        iqr = q3 - q1
        
        valid_ibis = ibis[(ibis >= q1 - 1.5*iqr) & (ibis <= q3 + 1.5*iqr)]
        
        if len(valid_ibis) > 0:
            stable_ibi = np.median(valid_ibis)
            tempo = 60.0 / stable_ibi
        else:
            tempo = 100.0
        
        self.get_logger().info(f'Debug - median IBI: {median_ibi:.3f}s, stable IBI: {stable_ibi:.3f}s, valid count: {len(valid_ibis)}/{len(ibis)}')
        return float(np.clip(tempo, 60, 200))

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
            self.stop_audio_playback()
            return

        # Publish tempo (constant, but republish periodically)
        tempo_msg = Float32()
        tempo_msg.data = self.tempo
        self.tempo_pub.publish(tempo_msg)
        
        # Log initial tempo
        if self.last_energy is None:  # First call
            self.get_logger().info(f'[TEMPO] {self.tempo:.1f} BPM')

        # Publish current energy value (interpolated from energy array)
        current_energy = self.get_energy_at_time(current_time)
        energy_msg = Float32()
        energy_msg.data = current_energy
        self.current_energy_pub.publish(energy_msg)
        
        # Log energy changes
        if self.last_energy is not None:
            energy_delta = abs(current_energy - self.last_energy)
            if energy_delta >= self.energy_change_threshold:
                self.get_logger().info(f'[ENERGY] t={current_time:.2f}s: {self.last_energy:.3f} → {current_energy:.3f} (Δ{current_energy - self.last_energy:+.3f})')
        self.last_energy = current_energy

        # Optionally publish upcoming beats/onsets in a window
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
            return float(self.energy[-1])
        elif idx == 0:
            return float(self.energy[0])
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
    
    def start_audio_playback(self, music_file):
        """Start playing audio file using pygame"""
        if not PYGAME_AVAILABLE or not self.play_audio:
            return
        
        try:
            pygame.mixer.music.load(music_file)
            pygame.mixer.music.play()
            self.get_logger().info(f'Started playing audio: {music_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to play audio: {e}')
            self.play_audio = False
    
    def stop_audio_playback(self):
        """Stop audio playback"""
        if PYGAME_AVAILABLE and self.play_audio:
            try:
                pygame.mixer.music.stop()
                self.get_logger().info('Stopped audio playback')
            except Exception as e:
                self.get_logger().warn(f'Error stopping audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MusicBeatAnalyzerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_audio_playback()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()