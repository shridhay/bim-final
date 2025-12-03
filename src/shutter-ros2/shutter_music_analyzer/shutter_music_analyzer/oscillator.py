#!/usr/bin/env python3

"""
Standalone oscillator class for generating sinusoidal motion.
Formula: θ(t) = A(t) sin(ω(t)t + φ)

This is a standalone class that can be used independently of ROS2,
making it easy to test and integrate into different systems.
"""

import math
import numpy as np


class HarmonicOscillator:
    """
    Simple harmonic oscillator that generates sinusoidal motion.
    
    Formula: θ(t) = A(t) sin(ω(t)t + φ)
    
    Parameters:
        A: amplitude (can be time-varying)
        ω: frequency in rad/s (can be time-varying)
        φ: phase offset in radians
    """
    
    def __init__(self, amplitude=1.0, frequency=1.0, phase=0.0):
        """
        Initialize oscillator with default parameters.
        
        Args:
            amplitude: Initial amplitude A (default: 1.0)
            frequency: Initial frequency ω in rad/s (default: 1.0)
            phase: Phase offset φ in radians (default: 0.0)
        """
        self.A = amplitude
        self.omega = frequency
        self.phi = phase
        self.t = 0.0  # Internal time counter
        
    def set_amplitude(self, amplitude):
        """Set amplitude A"""
        self.A = amplitude
    
    def set_frequency(self, frequency):
        """Set frequency ω (in rad/s)"""
        self.omega = frequency
    
    def set_phase(self, phase):
        """Set phase offset φ (in radians)"""
        self.phi = phase
    
    def set_parameters(self, amplitude=None, frequency=None, phase=None):
        """Set multiple parameters at once"""
        if amplitude is not None:
            self.A = amplitude
        if frequency is not None:
            self.omega = frequency
        if phase is not None:
            self.phi = phase
    
    def evaluate(self, t=None):
        """
        Evaluate oscillator at time t.
        
        Args:
            t: Time in seconds. If None, uses internal time counter.
        
        Returns:
            θ(t) = A sin(ωt + φ)
        """
        if t is None:
            t = self.t
        
        return self.A * math.sin(self.omega * t + self.phi)
    
    def step(self, dt):
        """
        Advance internal time by dt and return current value.
        
        Args:
            dt: Time step in seconds
        
        Returns:
            Current oscillator value
        """
        self.t += dt
        return self.evaluate()
    
    def reset(self, t=0.0):
        """Reset internal time counter"""
        self.t = t
    
    def get_current_time(self):
        """Get current internal time"""
        return self.t


class MusicDrivenOscillator(HarmonicOscillator):
    """
    Oscillator that maps musical features to oscillator parameters.
    
    Mappings:
        ω(t) = k_T · tempo(t)  [tempo in BPM converted to rad/s]
        A(t) = A_0 + k_E · energy(t)
    
    Parameters:
        k_T: Tempo sensitivity constant (default: 0.1)
        k_E: Energy sensitivity constant (default: 0.5)
        A_0: Base amplitude (default: 0.5)
    """
    
    def __init__(self, amplitude=0.5, frequency=1.0, phase=0.0, 
                 k_tempo=0.1, k_energy=0.5, base_amplitude=0.5):
        """
        Initialize music-driven oscillator.
        
        Args:
            amplitude: Initial amplitude
            frequency: Initial frequency in rad/s
            phase: Phase offset in radians
            k_tempo: Tempo sensitivity k_T (BPM → rad/s conversion factor)
            k_energy: Energy sensitivity k_E
            base_amplitude: Base amplitude A_0
        """
        super().__init__(amplitude, frequency, phase)
        self.k_tempo = k_tempo
        self.k_energy = k_energy
        self.A_0 = base_amplitude
        
        # Current musical features
        self.current_tempo = None  # BPM
        self.current_energy = None  # Normalized [0, 1]
    
    def update_from_tempo(self, tempo_bpm):
        """
        Update frequency from tempo.
        
        Formula: ω = k_T · tempo (converted to rad/s)
        Note: tempo in BPM, need to convert to rad/s
        BPM to rad/s: (BPM / 60) * 2π = BPM * π/30
        
        Args:
            tempo_bpm: Tempo in beats per minute
        """
        self.current_tempo = tempo_bpm
        # Convert BPM to rad/s: (BPM / 60) * 2π
        omega_rad_per_s = (tempo_bpm / 60.0) * 2.0 * math.pi
        # Apply sensitivity scaling
        self.set_frequency(self.k_tempo * omega_rad_per_s)
    
    def update_from_energy(self, energy):
        """
        Update amplitude from energy.
        
        Formula: A = A_0 + k_E · energy
        
        Args:
            energy: Normalized energy value [0, 1]
        """
        self.current_energy = energy
        amplitude = self.A_0 + self.k_energy * energy
        # Clamp amplitude to reasonable range
        amplitude = max(0.0, min(amplitude, math.pi / 2))  # Max ±π/2 for safety
        self.set_amplitude(amplitude)
    
    def update_from_features(self, tempo_bpm=None, energy=None):
        """
        Update oscillator parameters from musical features.
        
        Args:
            tempo_bpm: Tempo in BPM (optional)
            energy: Normalized energy [0, 1] (optional)
        """
        if tempo_bpm is not None:
            self.update_from_tempo(tempo_bpm)
        if energy is not None:
            self.update_from_energy(energy)
    
    def sync_to_beat(self, beat_time, current_time):
        """
        Synchronize phase to align with a beat.
        
        Args:
            beat_time: Time of the beat (in seconds)
            current_time: Current time (in seconds)
        """
        # Reset phase so that at beat_time, sin(ωt + φ) = 0 (or some target)
        # For sin to be 0 at beat_time: ω*beat_time + φ = 0 → φ = -ω*beat_time
        # But we want to account for current time offset
        time_since_beat = current_time - beat_time
        # Adjust phase so oscillator aligns with beat
        self.phi = -self.omega * time_since_beat

