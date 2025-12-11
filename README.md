# ***Teaching Shutter How to Dance***

## ***Overview***

*This ROS2 package provides a complete pipeline for music-driven robotics, enabling the Shutter robot to perform coordinated, rhythm-synchronized motion. The system analyzes audio inputs in real-time to extract temporal features (tempo, beat timestamps) and spectral energy, which are then mapped to kinematic control laws driving the robot's joints.*

*The package is designed to be robust against varying audio formats and includes specific calibration mechanisms to handle hardware latency and synchronization drift.*

## ***System Architecture***

*The system consists of two primary ROS2 nodes operating in a producer-consumer relationship:*

### ***1\. Music Analyzer Node (music\_analyzer\_node.py)***

*This node acts as the sensory input layer. It processes raw audio data and broadcasts high-level musical features to the ROS network.*

* ***Audio Pipeline & Format Handling:***  
  * *Input files are validated for compatibility with scipy.io.wavfile.*  
  * ***Auto-Conversion:** If the input is a non-standard format (e.g., MP3, M4A) or contains incompatible headers (e.g., 32-bit float WAV from YouTube converters), the system invokes a subprocess using imageio-ffmpeg. This converts the audio to a standardized 16-bit PCM WAV file at 44.1kHz in a temporary buffer.*  
  * ***Signal Processing:** The audio is downmixed to mono and normalized. A Short-Time Fourier Transform (STFT) is applied to compute spectral flux, which serves as the onset strength signal.*  
* ***Bidirectional BPM Detection Algorithm:***  
  * *To prevent "Tempo Octave Errors" (misinterpreting a song as half-speed or double-speed), the node implements a custom autocorrelation logic with bidirectional verification:*  
    * ***Slow-to-Fast Check:** If the primary autocorrelation peak suggests a tempo \< 100 BPM, the algorithm checks the harmonic at 2x the frequency. If the harmonic strength exceeds a 0.4 threshold, the tempo is doubled (fixing tracks like Mambo No. 5).*  
    * ***Fast-to-Slow Check:** If the primary peak suggests \> 120 BPM, it checks the sub-harmonic at 0.5x. A dynamic threshold (0.75 to 0.95) is applied based on signal confidence to determine if the track is actually a half-time ballad (fixing tracks like Don't Speak).*  
* ***Beat Clustering:***  
  * *To prevent vocal noise or off-beats from causing tempo drift, detected onsets are filtered through a clustering algorithm. New beats are only accepted if they occur at intervals $\\ge 85\\%$ of the estimated beat period.*

### ***2\. Oscillator Control Node (oscillator\_node.py)***

*This node acts as the kinematic controller. It maps musical data to motor commands using a harmonic oscillator model.*

* ***Harmonic Motion Model:***  
  * *Each of the four robot joints is driven by an independent oscillator following the equation: $\theta(t) = A(t) \cdot \sin(\omega(t) \cdot t + \phi)$.*  
  * ***Phase Offsets ($\phi$):** To create coordinated "wave" motion rather than unison movement, each joint is assigned a static phase offset ($0, \pi/2, \pi, 3\pi/2$).*  
* ***Predictive Phase Synchronization (P-Controller):***  
  * *Original "Hard Sync" methods caused violent motion jerks. This system implements a Proportional Controller to smooth synchronization.*  
  * *The node calculates the phase error relative to the next upcoming beat.*  
  * ***Control Law:** $\omega_{new} = \omega_{base} + K_p \cdot \frac{\Delta \phi}{\Delta t}$*  
  * *This temporarily accelerates or decelerates the motor velocity ($\omega$) to align with the beat smoothly over a lookahead window (0.1s \- 0.6s).*  
* ***Logic Separation:***  
  * *The system separates `base_omega` (derived from the song's BPM) from the instantaneous `omega` (adjusted for sync). This prevents incoming tempo updates from overwriting the subtle synchronization nudges applied by the P-controller.*

---

## ***Installation & Dependencies***

*This package requires a standard ROS2 installation. Python dependencies are managed via setup.py but require manual installation of system-level tools for audio conversion.*

***Required Python Packages:***

```bash
pip install scipy numpy pygame imageio-ffmpeg
```

*Note: librosa and madmom were removed from the dependency tree to resolve conflict issues within the ROS2 environment.*

---

## **Usage**

### **1\. Standard Launch (Recommended)**

The recommended method is to use the provided launch file, which handles node orchestration and timing.

**Command:**

```bash

ros2 launch shutter_bringup shutter_with_face.launch.py \\

    use_music:=true \\

    music_file:="/absolute/path/to/song.mp3"
```

**Key Configuration Parameters:**

* `playback_offset`: (Default: \-0.1) A time offset (in seconds) applied to the robot's internal clock. This compensates for the hardware latency inherent in the pygame audio buffer and the physical speaker drivers.  
* `k_energy`: (Default: 1.0) Gain factor mapping audio volume to movement amplitude.  
* `enable_beat_sync`: (Default: true) Toggles the predictive phase synchronization logic.

### **2\. Manual Tempo Override (For Complex Rhythms)**

The built-in autocorrelation algorithm works well for tracks with strong percussion (Pop, Rock, Dance). However, tracks driven by rolling piano arpeggios (e.g., ballads like *Someone Like You*) or complex jazz rhythms can confuse the lightweight signal processing used here (since heavy libraries like librosa or madmom were not available for this implementation).

In these cases, you can bypass the auto-detection by providing the known BPM manually.

**Using Launch File:**

```bash

ros2 launch shutter_bringup shutter_with_face.launch.py \\

    use_music:=true \\

    music_file:="/path/to/song.wav" \\

    tempo_bpm:=~set_tempo~
```

**Using Node Directly:**

```bash

ros2 run shutter_music_analyzer music_analyzer --ros-args \\

    -p music_file:="/path/to/song.wav" \\

    -p tempo_bpm:=67.0
```

### **3\. Manual Node Execution**

For development or debugging individual components:

**Start the Oscillator Node:**  
```bash  
ros2 run shutter_music_analyzer oscillator_control
```


**Start the Music Analyzer Node:**  
```bash  
ros2 run shutter_music_analyzer music_analyzer --ros-args -p music_file:="/path/to/song.wav"
```

*Note: if testing the two together manually, make sure to run the oscillator node before the music analyzer node.*

___

## Verification & Monitoring

To verify the system is working, you can monitor the active topics in a separate terminal:

**1. Check Music Features:**
```bash
# See the beat detection in real-time
ros2 topic echo /music/beat_times

# See the computed tempo
ros2 topic echo /music/tempo
```

---

## Tuning Guide

If the robot's motion doesn't feel right, try adjusting these launch parameters:

| Symptom | Parameter | Adjustment |
| :--- | :--- | :--- |
| **Robot moves "ahead" of the beat** | `playback_offset` | Make it more negative (e.g., change `-0.1` to `-0.2`). |
| **Robot moves "late"** | `playback_offset` | Increase towards 0 (e.g., change `-0.1` to `0.0`). |
| **Movements are too small** | `k_energy` | Increase (e.g., `1.5` or `2.0`). |
| **Movements are too violent** | `base_amplitude` | Decrease (e.g., `0.3`). |
| **BPM detection is half or double speed** | `tempo_bpm` | Use manual override (see Usage section). |

___

## **Engineering Challenges and Solutions**

### **1\. Synchronization Latency**

Challenge: The robot appeared to move "ahead" of the music.

Root Cause: The audio subsystem (pygame \+ OS drivers) introduces a 100ms–300ms buffer delay between the "play" command and audible sound. The robot, executing kinematic commands instantly, physically led the audio.

Solution: Implemented a negative time bias (`playback_offset`). This artificially delays the robot's internal clock reference, aligning the kinematic $T=0$ with the audible $T=0$.

### **2\. Startup Race Conditions**

Challenge: The music analyzer would begin playback before the oscillator node or robot hardware was fully initialized, resulting in the robot missing the start of the choreography.

Solution: Introduced a TimerAction in the launch description to enforce a 3.0-second initialization delay for the analyzer node. Furthermore, the Oscillator node was updated to hold its internal timer at $T=0$ until the first beat message is received, ensuring perfect startup synchronization.

### **3\. Library Constraints**

Challenge: Initial designs utilizing first librosa then trying madmom proved unstable in the target environment, which ended up being due to a colcon issue where it would reference the local library instead of the .venv one. While we discovered a fix for this, we discovered it much later in the project, and had to come up with another solution to move forward.

Solution: The analysis stack was re-engineered using scipy. Custom signal processing algorithms were written to replicate the onset detection and tempo estimation features required, resulting in a lighter-weight and more portable implementation.

