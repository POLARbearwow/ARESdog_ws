#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module contains the JumpController class for making the robot jump.
"""

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from enum import Enum, auto

class JumpPhase(Enum):
    STANDING = auto()
    CROUCH   = auto()
    THRUST   = auto()
    FLIGHT   = auto()
    LANDING  = auto()

class JumpController(Node):
    """原地跳跃控制器，仅通过关节角度(position)实现。

    状态机流程：STANDING -> CROUCH -> THRUST -> FLIGHT -> LANDING -> STANDING
    """

    def __init__(self):
        super().__init__('jump_controller')
        self.get_logger().info("Jump Controller node has been started.")

        # ------------------ 参数 ------------------
        self.declare_parameter('crouch_hip',  -0.5)  # Hip abduction (rad)
        self.declare_parameter('crouch_knee',  1.0)  # Knee flexion  (rad)
        self.declare_parameter('thrust_extra', 0.2)  # 额外伸展比例 (% of crouch amplitude)
        self.declare_parameter('crouch_time',  0.3)  # s
        self.declare_parameter('thrust_time',  0.05) # s (快速伸腿)
        self.declare_parameter('landing_time', 0.4)  # s (落地阻尼)
        self.declare_parameter('rate',        200.0) # Hz 控制周期

        # ------------------ ROS 接口 --------------
        self.action_pub = self.create_publisher(JointState, '/action', 10)
        self.state_pub  = self.create_publisher(String, '/jump_state', 5)
        self.create_subscription(Bool, '/jump_cmd', self.cmd_callback, 5)

        # 关节名称顺序需与 usb_bridge_node/BalanceWalk 对齐
        self.joint_names = [
            'FL_thigh_joint_i', 'FL_thigh_joint_o',
            'FR_thigh_joint_i', 'FR_thigh_joint_o',
            'waist_joint',
            'RL_thigh_joint_i', 'RL_thigh_joint_o',
            'RR_thigh_joint_i', 'RR_thigh_joint_o'
        ]

        # ------------------ 状态机 ----------------
        self.phase = JumpPhase.STANDING
        self.phase_start = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.get_parameter('rate').value, self.update)

        # 预计算目标角度
        hip_crouch   = self.get_parameter('crouch_hip').value
        knee_crouch  = self.get_parameter('crouch_knee').value
        thrust_extra = self.get_parameter('thrust_extra').value

        # 站立位 angles 全 0
        self.angles_stand = [0.0]*9

        # 蹲下位 – hips flex/abduct, knees flex
        self.angles_crouch = [0.0]*9
        # mapping indices: using same rule as BalanceWalk: hip inner/outer pairs
        # For simplicity, apply same hip & knee for all legs symmetric
        # FL
        self.angles_crouch[0] =  hip_crouch
        self.angles_crouch[1] =  knee_crouch
        # FR
        self.angles_crouch[2] = -hip_crouch
        self.angles_crouch[3] = -knee_crouch
        # waist keep 0
        # RL
        self.angles_crouch[5] =  knee_crouch
        self.angles_crouch[6] =  hip_crouch
        # RR
        self.angles_crouch[7] = -knee_crouch
        self.angles_crouch[8] = -hip_crouch

        # 伸腿最大位 = -crouch * thrust_extra + stand (approx)
        hip_thrust  = -hip_crouch * thrust_extra
        knee_thrust = -knee_crouch * thrust_extra
        self.angles_thrust = [0.0]*9
        self.angles_thrust[0] = hip_thrust
        self.angles_thrust[1] = knee_thrust
        self.angles_thrust[2] = -hip_thrust
        self.angles_thrust[3] = -knee_thrust
        self.angles_thrust[5] = knee_thrust
        self.angles_thrust[6] = hip_thrust
        self.angles_thrust[7] = -knee_thrust
        self.angles_thrust[8] = -hip_thrust

        # 空中收腿姿态 (稍折叠)
        self.angles_flight = self.angles_crouch  # reuse crouch for simplicity

        self.in_flight = False

    # --------------------------------------------
    def cmd_callback(self, msg: Bool):
        if msg.data and self.phase == JumpPhase.STANDING:
            self.switch_phase(JumpPhase.CROUCH)

    def switch_phase(self, new_phase: JumpPhase):
        self.phase = new_phase
        self.phase_start = self.get_clock().now()
        self.state_pub.publish(String(data=self.phase.name))
        self.get_logger().info(f"Jump phase -> {self.phase.name}")

    # --------------------------------------------
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.phase_start).seconds()

        if self.phase == JumpPhase.CROUCH:
            t_total = self.get_parameter('crouch_time').value
            ratio = min(dt / t_total, 1.0)
            cmd = self.interpolate(self.angles_stand, self.angles_crouch, ratio)
            self.publish_joint_state(cmd)
            if ratio >= 1.0:
                self.switch_phase(JumpPhase.THRUST)

        elif self.phase == JumpPhase.THRUST:
            t_total = self.get_parameter('thrust_time').value
            ratio = min(dt / t_total, 1.0)
            cmd = self.interpolate(self.angles_crouch, self.angles_thrust, ratio)
            self.publish_joint_state(cmd)
            if ratio >= 1.0:
                self.switch_phase(JumpPhase.FLIGHT)
                self.in_flight = True

        elif self.phase == JumpPhase.FLIGHT:
            # 简化：保持收腿姿态，等待触地检测 (此处用计时器模拟)
            self.publish_joint_state(self.angles_flight)
            if dt > 0.6:  # 假定 0.6 s 后触地，可后续改为 torque/IMU 检测
                self.switch_phase(JumpPhase.LANDING)

        elif self.phase == JumpPhase.LANDING:
            t_total = self.get_parameter('landing_time').value
            ratio = min(dt / t_total, 1.0)
            cmd = self.interpolate(self.angles_flight, self.angles_stand, ratio)
            self.publish_joint_state(cmd)
            if ratio >= 1.0:
                self.switch_phase(JumpPhase.STANDING)

        elif self.phase == JumpPhase.STANDING:
            # 发布站立保持
            self.publish_joint_state(self.angles_stand)

    # --------------------------------------------
    def publish_joint_state(self, positions):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions
        self.action_pub.publish(js)

    @staticmethod
    def interpolate(a, b, ratio):
        return [(1 - ratio) * ai + ratio * bi for ai, bi in zip(a, b)]


def main(args=None):
    rclpy.init(args=args)
    node = JumpController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 