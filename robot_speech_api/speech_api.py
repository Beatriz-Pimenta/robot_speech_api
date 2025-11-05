# This is the file for the class for the API itself that will interact with the TTS action server. 
# It exposes two functions: say() and say_and_wait()

import rclpy
from rclpy.action import ActionClient     # Used to create an Action Client
from audio_common_msgs.action import TTS  # Action definition for Text-To-Speech


class SpeechAPI:
    
    def __init__(self, node):
        # The API requires a ROS 2 Node to run inside
        self._node = node

        # Create an Action Client that connects to the 'tts' Action Server
        self._client = ActionClient(node, TTS, 'say')

    def say(self, text: str):

        # Send a request to the TTS server to speak text, but do NOT wait until the robot finishes speaking.

        # Create a goal message
        goal_msg = TTS.Goal()
        goal_msg.text = text

        # Wait until the TTS server is available
        self._client.wait_for_server()

        # Send the goal asynchronously (does not block)
        self._client.send_goal_async(goal_msg)

    def say_and_wait(self, text: str) -> bool:

        # Send a request to the TTS server to speak text, and WAIT until it finishes speaking before returning.

        # Create a goal message
        goal_msg = TTS.Goal()
        goal_msg.text = text

        self._node.get_logger().info(f"Sending TTS goal: {text}")

        # Wait until the TTS server is available
        self._client.wait_for_server()

        # Send the goal and wait until the server accepts/rejects it
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            # If the goal is rejected, return False
            self._node.get_logger().error("Goal rejected by TTS server")
            return False

        # Wait until the TTS server finishes and sends back a result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)

        # Just return True if the action finished successfully
        result = result_future.result().result
        self._node.get_logger().info(f"TTS finished speaking: {result.text}")
        return True

