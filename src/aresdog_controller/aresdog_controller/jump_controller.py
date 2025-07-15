#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module contains the JumpController class for making the robot jump.
"""

import rclpy
from rclpy.node import Node

class JumpController(Node):
    """
    A ROS 2 node to handle the robot's jumping action.

    This node will orchestrate the sequence of movements required for the robot
    to perform a jump. This could involve communicating with leg controllers,
    monitoring stability, and executing a pre-defined trajectory.
    """

    def __init__(self):
        """
        Initializes the JumpController node.
        """
        super().__init__('jump_controller')
        self.get_logger().info("Jump Controller node has been started.")

        # Future implementation will go here.
        # For example, creating action servers/clients, publishers, subscribers.


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)
    jump_controller = JumpController()
    try:
        rclpy.spin(jump_controller)
    except KeyboardInterrupt:
        # Gracefully shutdown the node on keyboard interrupt
        pass
    finally:
        # Cleanly destroy the node and shutdown rclpy
        jump_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 