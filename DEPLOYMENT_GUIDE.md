# LIDAR Human Detection with Dog Bark Alarm - Deployment Guide

This guide details how to install, build, and run the LD19 LIDAR human detection project, which includes a dog bark audio alarm triggered upon detecting a human.

## 1. Hardware Requirements

*   **LD19 LIDAR**: Connected via USB. By default, the system expects it at `/dev/ttyUSB0`.
*   **Audio Output**: Speakers connected to the system to play the dog bark sound.
*   **Display**: A graphical environment (X11 or Wayland) is required to render the live LIDAR visualization.

## 2. System Requirements & Dependencies

Ensure your system is up to date and has the following installed:

### Install System Packages
The audio playback relies on `aplay` from `alsa-utils`.
```bash
sudo apt update
sudo apt install -y alsa-utils
```

### Install Rust (if not already installed)
The core LIDAR detection logic is written in Rust.
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### Permissions
Your user needs permission to read from the serial port.
```bash
sudo usermod -aG dialout $USER
# You may need to log out and log back in, or run `newgrp dialout` for this to take effect.
```

## 3. Project Structure Setup

Make sure the required Python script for the dog bark alarm is in the correct location:
*   **Dog Bark Script**: Must be located at `/home/rdrp/dog_bark_pi5.py`.
*   **LIDAR Source Code**: The Rust project should be in `/home/rdrp/Documents/lidar/lidar-ld19-humandetection/`.

## 4. Building the Project

Navigate to the project directory and build the release version using Cargo:

```bash
cd /home/rdrp/Documents/lidar/lidar-ld19-humandetection/
cargo build --release --bin main
```

## 5. Execution

To run the program, ensure your display variable is set correctly (especially if running over SSH or in certain terminal environments). 

```bash
cd /home/rdrp/Documents/lidar/lidar-ld19-humandetection/
DISPLAY=:0 cargo run --release --bin main
```

### Execution Flow & Behavior:

1.  **Calibration Phase (First 30 seconds)**:
    *   The program will output: `Calibrating for 30s — keep the scene static...`
    *   **CRITICAL:** Do not move in front of the LIDAR during this time. It is building a static map of the background environment.
2.  **Detection Phase**:
    *   Once calibration finishes, it outputs: `Calibration done. Locked X / 360 angle bins. Motion detection active.`
    *   The GUI will show static objects as black dots, motion as red dots, and humans as a blue-red gradient with a smiley face.
3.  **Alarm Trigger**:
    *   When a human is detected, the console will print `human @ [angle]°`.
    *   The `dog_bark_pi5.py` script is called in the background.
    *   **WAV Generation**: The first time the script runs, it generates a synthetic dog bark audio file (`dog_bark.wav`) in the current directory if it does not already exist. If you want a custom bark sound, just replace `dog_bark.wav` in the project directory.
    *   **Playback**: `aplay` is used to play the sound.
    *   **Cooldown**: There is a 5-second cooldown between barks to prevent overlapping audio spam.

## 6. Troubleshooting

*   **`Failed to open /dev/ttyUSB0: Device or resource busy` or `Permission denied`**:
    *   Ensure the LIDAR is plugged in (`ls /dev/ttyUSB*`).
    *   Ensure no other program (like a Docker container) is using the port.
    *   Ensure your user is in the `dialout` group.
*   **`dog_bark.wav: No such file or directory / 播放失败，退出码: 1`**:
    *   This happens if the `dog_bark_pi5.py` script fails to generate the file. Ensure you have write permissions in the directory.
*   **No GUI window opens / `minifb` panics**:
    *   Make sure `DISPLAY=:0` is correctly pointing to your active X server. Run `echo $DISPLAY` to check your current session's display variable.
