# Launch file to start BOTH:
  # - The TTS Action Server
  # - The Demo Speech Node
# This is useful for testing everything together in a single command.

# Supports parameter "mode":
    # - demo        : runs a predefined speech demo
    # - interactive : lets you type text in the terminal (if stdin is attached)

# Usage examples:
    # ros2 launch robot_speech_api demo_speech.launch.py mode:=demo
    # ros2 launch robot_speech_api demo_speech.launch.py mode:=interactive

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

    # Path to your existing TTS bringup launch file
    tts_launch_path = os.path.join(
        get_package_share_directory('tts_bringup'),
        'launch',
        'tts.launch.py'
    )

    # Declare a "mode" argument (demo or interactive)
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='demo',
        description='Mode for the demo node: "demo" or "interactive".'
    )

    # Include the official TTS bringup launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tts_launch_path)

    # Demo speech mode
    demo_node = Node(
        package='robot_speech_api',
        executable='demo',  # Entry point of your demo node
        name='demo_speech',
        output='screen',
        parameters=[{'mode': LaunchConfiguration('mode')}]
    )

    return LaunchDescription([
        declare_mode_arg,
        tts_bringup,
        demo_node
    ])

