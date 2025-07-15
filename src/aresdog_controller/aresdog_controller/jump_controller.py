#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Node for controlling the quadruped robot to perform a jump.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import JointState
import math
import time

class JumpController(Node):
    """
    A ROS 2 node that controls a quadruped robot to perform a jump maneuver.
    
    The jump is implemented as a state machine with the following phases:
    1. CROUCH: The robot lowers its body to prepare for the jump.
    2. THRUST: The robot rapidly extends its legs to push off the ground.
    3. FLIGHT: The legs are retracted while airborne.
    4. LANDING: The legs are extended to absorb impact and cushion the landing.
    5. STAND: The robot returns to a nominal standing pose.
    
    The controller operates by interpolating the desired body height (z-axis)
    through these phases and uses an Inverse Kinematics (IK) model to calculate
    the required joint angles for the thigh and calf.
    """

    # --- Constants ---
    # Link lengths for Inverse Kinematics [m]
    L_THIGH = 0.090
    L_CALF = 0.224
    
    # Target body heights (z-coordinate of foot relative to hip) [m]
    # Note: Negative Z is down.
    Z_STAND = -0.21   # Nominal standing height
    Z_CROUCH = -0.15  # Lowest point during crouch
    Z_THRUST = -0.24  # Fully extended legs for thrust
    Z_FLIGHT = -0.18  # Legs retracted during flight
    X_NOMINAL = 0.0   # Horizontal foot position for in-place jump

    # Phase durations [seconds]
    T_CROUCH = 0.4
    T_THRUST = 0.12
    T_FLIGHT = 0.3
    T_LANDING = 0.3

    def __init__(self):
        """Initializes the node, publishers, subscribers, and state machine."""
        super().__init__('jump_controller')
        
        # Declare auto_start parameter
        self.declare_parameter('auto_start', True)
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        
        self.get_logger().info("Jump Controller starting up...")
        self.get_logger().info(f"Auto-start configured to: {self.auto_start}")

        # Publisher for joint commands
        self.action_pub = self.create_publisher(JointState, '/action', 10)

        # Subscriber to trigger the jump sequence
        self.jump_cmd_sub = self.create_subscription(
            StringMsg,
            '/jump_cmd',
            self.jump_cmd_callback,
            10)

        # Main control loop timer (200 Hz)
        self.timer = self.create_timer(1.0 / 200.0, self.control_loop)

        # State machine variables
        self.state = 'STAND'
        self.phase_start_time = self.get_clock().now()
        
        # Joint names must match the order expected by the hardware interface
        self.joint_names = [
            "FL_thigh_joint_i", "FL_thigh_joint_o", "FR_thigh_joint_i", "FR_thigh_joint_o",
            "waist_joint",
            "RL_thigh_joint_i", "RL_thigh_joint_o", "RR_thigh_joint_i", "RR_thigh_joint_o"
        ]
        
        if self.auto_start:
            # Short delay to allow subscribers to connect before starting
            time.sleep(1.0)
            self.start_jump_sequence()

    def jump_cmd_callback(self, msg):
        """Callback to start the jump sequence upon receiving a command."""
        self.get_logger().info(f"Received jump command: {msg.data}")
        if self.state == 'STAND':
            self.start_jump_sequence()

    def start_jump_sequence(self):
        """Initiates the jump state machine."""
        self.get_logger().info("STAND -> CROUCH. Starting jump sequence.")
        self.state = 'CROUCH'
        self.phase_start_time = self.get_clock().now()

    def control_loop(self):
        """
        Main loop that runs at a fixed rate to update the robot's state
        and publish joint commands.
        """
        now = self.get_clock().now()
        elapsed_time = (now - self.phase_start_time).nanoseconds * 1e-9

        # Default to standing pose
        target_z = self.Z_STAND
        
        # --- State Machine Logic ---
        if self.state == 'CROUCH':
            # Interpolate from standing height to crouch height
            progress = min(elapsed_time / self.T_CROUCH, 1.0)
            target_z = self.Z_STAND + (self.Z_CROUCH - self.Z_STAND) * progress
            if progress >= 1.0:
                self.state = 'THRUST'
                self.phase_start_time = now
                self.get_logger().info("CROUCH -> THRUST")

        elif self.state == 'THRUST':
            # Interpolate from crouch height to full extension
            progress = min(elapsed_time / self.T_THRUST, 1.0)
            target_z = self.Z_CROUCH + (self.Z_THRUST - self.Z_CROUCH) * progress
            if progress >= 1.0:
                self.state = 'FLIGHT'
                self.phase_start_time = now
                self.get_logger().info("THRUST -> FLIGHT (Takeoff!)")

        elif self.state == 'FLIGHT':
            # Interpolate from thrust height to retracted flight pose
            progress = min(elapsed_time / self.T_FLIGHT, 1.0)
            target_z = self.Z_THRUST + (self.Z_FLIGHT - self.Z_THRUST) * progress
            if progress >= 1.0:
                self.state = 'LANDING'
                self.phase_start_time = now
                self.get_logger().info("FLIGHT -> LANDING")

        elif self.state == 'LANDING':
            # Interpolate from flight pose to standing pose to absorb impact
            progress = min(elapsed_time / self.T_LANDING, 1.0)
            target_z = self.Z_FLIGHT + (self.Z_STAND - self.Z_FLIGHT) * progress
            if progress >= 1.0:
                self.state = 'STAND'
                self.phase_start_time = now
                self.get_logger().info("LANDING -> STAND. Sequence complete.")

        # --- IK Calculation and Publishing ---
        thigh_angle, calf_angle = self._inverse_kinematics(self.X_NOMINAL, target_z)

        if thigh_angle is not None and calf_angle is not None:
            self._publish_joint_command(thigh_angle, calf_angle)
        else:
            self.get_logger().warn_throttle(
                self.get_clock(), 2000,
                f"IK solution not found for target z={target_z:.3f}. Holding last position."
            )

    def _inverse_kinematics(self, x, z):
        """
        Calculates the thigh and calf joint angles for a given foot position (x, z).
        
        Args:
            x (float): The horizontal distance to the foot (in body frame).
            z (float): The vertical distance to the foot (in body frame).
            
        Returns:
            A tuple (thigh_angle, calf_angle) in radians, or (None, None) if
            the target is unreachable.
        """
        # Ensure z is negative (downwards)
        if z > 0:
            z = -z

        L1 = self.L_THIGH
        L2 = self.L_CALF
        
        dist_sq = x**2 + z**2
        
        # Check for reachability
        min_reach_sq = (L2 - L1)**2
        max_reach_sq = (L1 + L2)**2
        if not (min_reach_sq <= dist_sq <= max_reach_sq):
            self.get_logger().warn(f"Target (x={x}, z={z}) is unreachable.")
            return None, None
            
        # Calculate knee (calf) angle
        # This is the angle between the thigh and calf links.
        # Zero angle corresponds to a straight leg.
        cos_theta_calf = (dist_sq - L1**2 - L2**2) / (2 * L1 * L2)
        # Clamp value to handle potential floating point inaccuracies
        cos_theta_calf = max(-1.0, min(1.0, cos_theta_calf))
        theta_calf_abs = math.acos(cos_theta_calf)
        
        # The joint angle is typically defined as PI minus the absolute angle
        calf_angle = -(math.pi - theta_calf_abs)
        
        # Calculate thigh (hip) angle
        gamma = math.atan2(-z, -x) # Angle of the leg vector
        beta = math.atan2(L2 * math.sin(theta_calf_abs), L1 + L2 * math.cos(theta_calf_abs))
        thigh_angle = gamma - beta

        return thigh_angle, calf_angle

    def _publish_joint_command(self, thigh_angle, calf_angle):
        """
        Populates and publishes a JointState message with the calculated angles.
        
        Args:
            thigh_angle (float): The target angle for the thigh joints.
            calf_angle (float): The target angle for the calf joints.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Map the same angles to all four legs, applying inversions for left/right sides
        # The mapping is based on the URDF and hardware configuration.
        pos = [0.0] * 9
        pos[0] =  thigh_angle  # FL_thigh_i
        pos[1] = -calf_angle   # FL_thigh_o
        pos[2] = -thigh_angle  # FR_thigh_i
        pos[3] =  calf_angle   # FR_thigh_o
        pos[4] = 0.0           # waist_joint
        pos[5] =  calf_angle   # RL_thigh_i (Note: This is calf on hardware)
        pos[6] = -thigh_angle  # RL_thigh_o (Note: This is thigh on hardware)
        pos[7] = -calf_angle   # RR_thigh_i (Note: This is calf on hardware)
        pos[8] =  thigh_angle  # RR_thigh_o (Note: This is thigh on hardware)
        
        msg.position = pos
        self.action_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    jump_controller = JumpController()
    rclpy.spin(jump_controller)
    jump_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 