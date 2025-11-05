"""
robot_speech_api package
------------------------

This package provides a **high-level Python API** for controlling a Text-To-Speech
(TTS) Action Server in ROS 2. It allows other ROS 2 nodes or Python programs
to make the robot speak easily, without needing to deal with ROS topics,
actions, or low-level message types.

Structure of the package:
  ├── speech_api.py   → contains the SpeechAPI class (the main interface)
  ├── demo.py         → example node that uses the SpeechAPI
  ├── __init__.py     → this file (marks this directory as a Python package)
  └── launch/         → launch files to run the API and TTS server

This __init__.py file ensures that Python recognizes the folder
'robot_speech_api/' as a valid importable package.
"""

# -----------------------------------------------------------------------------
# IMPORTS (OPTIONAL)
# -----------------------------------------------------------------------------
# We can choose to import key classes or functions here so users can import them
# directly from the package without needing to specify the full path.
#
# For example, after adding the import below, users can write:
#   from robot_speech_api import SpeechAPI
# instead of:
#   from robot_speech_api.speech_api import SpeechAPI
#
# This is optional but considered a "nice-to-have" for convenience.
# -----------------------------------------------------------------------------

from .speech_api import SpeechAPI  # Import the main API class


# -----------------------------------------------------------------------------
# OPTIONAL PACKAGE INITIALIZATION LOGIC
# -----------------------------------------------------------------------------
# Normally, we don't need any special initialization code here.
# However, we could add package-level configuration, logging setup,
# or environment variable checks if needed.
#
# For now, we’ll just include a small message that runs when the package
# is first imported (for debugging or confirmation).
# -----------------------------------------------------------------------------

import sys

if __name__ == "__main__":
    # This block only runs if you execute this file directly, e.g.:
    #   python3 -m robot_speech_api
    print("[robot_speech_api] This is a Python package and not meant to be run directly.")
    sys.exit(0)
