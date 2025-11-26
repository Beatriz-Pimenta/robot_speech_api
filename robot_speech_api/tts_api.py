# This is the file for the class for the API itself that will interact with the TTS action server. 
# It exposes two functions: say() and say_and_wait()

import rclpy
from rclpy.action import ActionClient     # Used to create an Action Client
from audio_common_msgs.action import TTS  # Action definition for Text-To-Speech


class TTS_API:
    
    def __init__(self, node):
        # The API requires a ROS 2 Node to run inside
        self._node = node

         # Read topic from parameter
        self._tts_topic = node.get_parameter_or(
            'tts_action_topic',
            node.declare_parameter('tts_action_topic', 'say')
        ).value

        # Create an Action Client that connects to the 'tts' Action Server
        self._client = ActionClient(node, TTS, self._tts_topic)

    def say(self, text: str):

        # Send a request to the TTS server to speak text, but do NOT wait until the robot finishes speaking.

        # Create a goal message
        goal_msg = TTS.Goal()
        goal_msg.text = text

        # Check if the server is ready (non-blocking)
        if not self._client.server_is_ready():
            self._node.get_logger().warn("TTS server not available. Skipping speech request.")
            return False

        # Send the goal asynchronously (does not block)
        self._client.send_goal_async(goal_msg)

        self._node.get_logger().info(f"Sent non-blocking TTS goal: '{text}'")
        return True


    def say_and_wait(self, text: str, timeout: float = 10.0) -> bool:

        # Send a request to the TTS server to speak text, and WAIT until it finishes speaking before returning.

        # Create a goal message
        goal_msg = TTS.Goal()
        goal_msg.text = text

        self._node.get_logger().info(f"Sending TTS goal: {text}")

        # Wait for server availability with timeout
        if not self._client.wait_for_server(timeout_sec=timeout):
            self._node.get_logger().error("TTS server not available (timeout).")
            return False

        # Send the goal and wait until the server accepts/rejects it
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)

        if not future.done():
            self._node.get_logger().error("Timeout while sending goal to TTS server.")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            # If the goal is rejected, return False
            self._node.get_logger().error("Goal rejected by TTS server")
            return False

        # Wait until the TTS server finishes and sends back a result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)

        if not result_future.done():
            self._node.get_logger().error("Timeout while waiting for TTS result.")
            return False

        # Just return True if the action finished successfully
        result = result_future.result().result
        self._node.get_logger().info(f"TTS finished speaking: {result.text}")
        return True

