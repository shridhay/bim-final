# How to Run the Music-Driven Shutter Robot

## Prerequisites

1. **Build the workspace** (if you haven't already):
   ```bash
   cd ~/CPSC459/bim-final
   colcon build --packages-select shutter_music_analyzer
   source install/setup.bash
   ```

2. **Have a music file ready** (WAV, MP3, etc. - librosa supports many formats)

## Option 1: Full System (Robot + Face + Music)

Run the complete Shutter robot with face animation and music-driven motion:

```bash
cd ~/CPSC459/bim-final
source install/setup.bash

ros2 launch shutter_bringup shutter_with_face.launch.py \
    use_music:=true \
    music_file:=/path/to/your/music.wav
```

**With custom oscillator parameters:**
```bash
ros2 launch shutter_bringup shutter_with_face.launch.py \
    use_music:=true \
    music_file:=/path/to/your/music.wav \
    k_tempo:=0.15 \
    k_energy:=0.7 \
    base_amplitude:=0.6 \
    control_rate:=50.0 \
    enable_beat_sync:=true
```

**Parameters explained:**
- `use_music:=true` - Enable music analysis (required for music-driven motion)
- `music_file:=/path/to/file.wav` - Path to your music file
- `k_tempo:=0.1` - Tempo sensitivity (higher = faster oscillation with tempo)
- `k_energy:=0.5` - Energy sensitivity (higher = larger movements with energy)
- `base_amplitude:=0.5` - Base movement range (radians)
- `control_rate:=50.0` - Control loop frequency (Hz)
- `enable_beat_sync:=true` - Sync oscillator phase to music beats

## Option 2: Music + Oscillator Only (No Robot Hardware)

If you just want to test the music analysis and oscillator without the full robot:

```bash
cd ~/CPSC459/bim-final
source install/setup.bash

ros2 launch shutter_music_analyzer music_oscillator.launch.py \
    music_file:=/path/to/your/music.wav \
    k_tempo:=0.1 \
    k_energy:=0.5
```

**Note:** 
- This will publish joint commands to `/joint_group_controller/command` but won't move physical motors unless the hardware interface is also running.
- **You don't need to run both launch files** - `shutter_with_face.launch.py` already includes the music nodes directly. These are alternatives:
  - Use `shutter_with_face.launch.py` for full robot system
  - Use `music_oscillator.launch.py` for testing music analysis only

## Option 3: Without Music (Default Oscillator)

Run the robot with oscillator but no music (uses default parameters):

```bash
ros2 launch shutter_bringup shutter_with_face.launch.py \
    use_music:=false
```

The oscillator will still run but won't respond to music features (uses default frequency/amplitude).

## Verifying It's Working

### Check Topics

In a new terminal:
```bash
source ~/CPSC459/bim-final/install/setup.bash

# See all music topics
ros2 topic list | grep music

# Monitor music features
ros2 topic echo /music/tempo
ros2 topic echo /music/current_energy
ros2 topic echo /music/beat_times

# Monitor joint commands
ros2 topic echo /joint_group_controller/command
```

### Check Node Status

```bash
ros2 node list
# Should see:
# - /music_beat_analyzer (if use_music=true)
# - /oscillator_control
# - /shutter_position_node (hardware interface)
```

### View Logs

The nodes output to screen by default. You should see:
- Music analyzer: "Music analysis started", tempo/energy updates
- Oscillator: Joint positions logged every 5 seconds

## Troubleshooting

### "Package 'shutter_music_analyzer' not found"
- Make sure you built: `colcon build --packages-select shutter_music_analyzer`
- Make sure you sourced: `source install/setup.bash`

### "Music file not found"
- Use absolute path: `/full/path/to/music.wav`
- Or relative path from current directory: `./music/song.wav`

### "No motion"
- Check if `/joint_group_controller/command` is publishing: `ros2 topic echo /joint_group_controller/command`
- Check hardware interface is running: `ros2 node list | grep shutter_position`
- If `use_music:=false`, oscillator uses defaults (may be slow/small motion)

### Music analysis not working
- Check music file format (librosa supports WAV, MP3, FLAC, etc.)
- Check music analyzer logs for errors
- Verify `/music/tempo` topic is publishing

### Dynamixel Communication Errors ("TxRxResult Incorrect status packet", "can't find dynamixel ID")
This error indicates the hardware interface can't communicate with the Dynamixel motors:

1. **Check USB device connection:**
   ```bash
   # List available USB serial devices
   ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
   
   # If no devices found, check if motors are:
   # - Powered on (check power LED)
   # - USB cable connected
   # - USB permissions set (may need to add user to dialout group)
   ```

2. **Specify correct USB port:**
   ```bash
   # If your device is /dev/ttyACM0 instead of /dev/ttyUSB0:
   ros2 launch shutter_bringup shutter_with_face.launch.py \
       use_music:=false \
       driver_device:=ttyACM0
   ```

3. **Check baud rate:**
   - Default is 4000000 (4Mbps)
   - Motors must be configured to match this baud rate
   - Use Dynamixel Wizard to verify/change baud rate

4. **Verify motor IDs:**
   - Check `shutter_hardware_interface/config/dynamixel_joints_position.yaml`
   - Motors should have IDs: 1, 2, 3, 4
   - Use Dynamixel Wizard to scan and verify motor IDs

5. **Run in simulation mode (if hardware unavailable):**
   ```bash
   ros2 launch shutter_bringup shutter_with_face.launch.py \
       simulation:=true \
       use_music:=false
   ```

## Example Music File Paths

```bash
# If your music is in your home directory
music_file:=~/Music/song.wav

# If your music is in the project directory
music_file:=~/CPSC459/bim-final/music/song.wav

# Absolute path
music_file:=/Users/skylar/Music/song.wav
```

