"""
robot_controller ROS 2 package.

This package provides a Model-based controller node that runs ONNX inference to generate joint commands for a quadruped robot.
"""

from .rl_controller_node import main  # noqa: F401 