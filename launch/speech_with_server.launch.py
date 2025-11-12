# Launch file to start BOTH:
  # - The TTS Action Server
  # - The Speech Node
# This is useful for testing everything together in a single command.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

    # Path to your existing TTS bringup launch file
    tts_launch_path = os.path.join(
        get_package_share_directory('tts_bringup'),
        'launch',
        'tts.launch.py'
    )

    return LaunchDescription([
        # Include the official TTS bringup launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tts_launch_path)
        ),
        
        # Start the API
        Node(
            package='robot_speech_api',        # Package with the Speech API
            executable='tts_speech',          # Entry point of the speech node
            name='tts_speech_node',                # Node name in ROS graph
            output='screen'                    # Show logs in the console
        )
    ])
