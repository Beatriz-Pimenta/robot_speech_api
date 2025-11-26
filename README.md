# robot_speech_api

This package aims to create an API for the Text-to-Speech, using ROS 2 and the tts_ros package created by mgonzs13. It allows other ROS 2 nodes to make the robot speak without dealing directly with low-level topics or action servers — just call `say(text)` or `say_and_wait(text)`.

---

## Table of Contents

- [Purpose](#purpose) 
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running](#running)
- [YAML File](#yaml-file)

---

## Purpose
- Abstracts communication with the TTS Action Server (e.g., `/say` from `tts_ros`).
- Provides two simple functions:
  - `say(text)` — sends a speech request without waiting (non-blocking).
  - `say_and_wait(text)` — sends a speech request and waits for completion (blocking).
- Includes a **demo node** with two modes:  
  - `demo` (automatic)  
  - `interactive` (user types in terminal)


## Package Structure

- The most important parts of the package are the following:

``` bash
robot_speech_api/
    launch/
        demo_speech.launch.py # Launches the TTS server and the demo node
        speech_with_server.launch.py # Optional alternative launcher
    robot_speech_api/
        __init__.py
        demo_tts.py # Demo node implementation for the TTS
        tts_api.py # TTS API implementation (Action client)
    package.xml
    setup.py
    README.md
``` 

---

## Prerequisites

- Before installing, ensure you have the following installed on your system:
  - ROS2 Humble
  - tts_ros by mgonzs13 ( https://github.com/mgonzs13/tts_ros ) or another TTS action server in your workspace
  - Python 3.8+
  - `rclpy`
  - `audio_common_msgs` -> for - `audio_common_msgs/action/TTS`

- On Ubuntu, you can install these dependencies with:

``` bash
sudo apt update
sudo apt install -y rclpy audio_common_msgs

cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
git clone https://github.com/mgonzs13/tts_ros.git
pip3 install -r tts_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build

```

---

## Installation
``` bash

cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Beatriz-Pimenta/robot_speech_api

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package (run from your ROS 2 workspace root)
colcon build --packages-select robot_speech_api

# Source the workspace
source install/setup.bash
```

---


## Running

The `robot_speech_api` package can be launched in **three main ways**, depending on your setup and desired behavior.

``` bash
# Launch the demo (automatic mode):
ros2 launch robot_speech_api demo_speech.launch.py mode:=demo
```
- Runs a predefined demonstration sequence using the TTS API.  
The robot will speak two preset sentences — one non-blocking (`say`) and one blocking (`say_and_wait`).
- This will automatically start the TTS Action Server (from `tts_ros`'s `tts_bringup`) and the Demo node, and make the robot speak a short predefined test sequence.

---------------------------------

``` bash
# For the demo in interactive mode:
ros2 launch robot_speech_api demo_speech.launch.py mode:=interactive
```
- In interactive mode, you can just type sentences directly in the terminal and the robot should speak them.
- You should see a prompt like:
``` bash
Enter a message:
```
Just type anything and press Enter. You can `type` exit to quit this mode.

---------------------------------

``` bash
# Launch speech_with_server
ros2 launch robot_speech_api speech_with_server.launch.py
```
- The speech_with_server launch file is meant for full integration testing.
- Launches both the TTS Action Server and the Speech API Node (`tts_api`).
- Ideal for integration with other ROS 2 nodes on your robot.
- Use this when you want to send speech requests programmatically instead of using the demo.

-----------------------------------

## YAML File

- Attention:
  - Both launch files (`demo_speech.launch.py` and `speech_with_server.launch.py`) use the same configuration file:
  `config/tts_params.yaml`.
  - You can copy or create another YAML for different robots and point to it in the launch file if needed.
