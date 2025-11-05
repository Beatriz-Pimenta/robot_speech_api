# Example node showing how to use the SpeechAPI.
# Supports two modes:
    #  1. Interactive mode -> asks user for input text in the terminal.
    #  2. Demo mode -> runs predefined examples.


import rclpy
from rclpy.node import Node
from robot_speech_api.speech_api import SpeechAPI


class DemoNode(Node):

    def __init__(self):
        super().__init__('demo_speech')

        # Create the Speech API
        self.api = SpeechAPI(self)

        # Declare a parameter to choose mode
        # - interactive: ask user for text
        # - demo: run predefined examples
        self.declare_parameter("mode", "demo")

        # Read the parameter value
        mode = self.get_parameter("mode").get_parameter_value().string_value

        if mode == "interactive":
            self.run_interactive()
        else:
            self.run_demo()

    def run_demo(self):

        # Runs the predefined demo sequence:
        # - Speak without waiting
        # - Speak and wait

        self.api.say("Hello, I will speak but not wait.")

        success = self.api.say_and_wait("Now I will speak and wait until finished.")

        self.get_logger().info(f"say_and_wait result: {success}")

    def run_interactive(self):

        # Ask the user for input in the terminal and make the robot speak it.
        # Loops until the user types 'exit'.

        self.get_logger().info("Interactive mode: type a message (or 'exit' to quit).")

        while rclpy.ok():
            try:
                text = input("Enter a message: ")
            except EOFError:
                break

            if text.lower() in ["exit", "quit"]:
                self.get_logger().info("Exiting interactive mode.")
                break

            success = self.api.say_and_wait(text)
            self.get_logger().info(f"TTS result: {success}")


def main(args=None):
    rclpy.init(args=args)

    node = DemoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
