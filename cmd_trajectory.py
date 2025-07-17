#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import numpy as np

# ------------------- 硬件常量定义 -------------------
JOINT_NAMES = [
    "FL_thigh_joint_i", "FL_thigh_joint_o",  # 0,1 (左前)
    "FR_thigh_joint_i", "FR_thigh_joint_o",  # 2,3 (右前) 
    "waist_joint",                           # 4 (不驱动)
    "RL_thigh_joint_i", "RL_thigh_joint_o",  # 5,6 (左后)
    "RR_thigh_joint_i", "RR_thigh_joint_o"   # 7,8 (右后)
]

class Leg:
    FL = 0; FR = 1; RL = 2; RR = 3  # 四条腿枚举

# ------------------- 步态参数配置 -------------------
class GaitParams:
    def __init__(self):
        # 基础参数 (单位: 米/秒)
        self.z_base = -0.18       # 站立高度
        self.z_swing = 0.03       # 抬腿高度
        self.max_stride = 0.35     # 最大步幅
        self.max_turn_offset = 0.15 # 最大转向偏移量
        
        # 对角步态相位 (FL/RR同相, FR/RL同相)
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]  
        
        # 时间参数
        self.period = 1.0         # 完整步态周期(秒)
        self.dt = 0.01            # 控制周期(10ms)

# ------------------- 逆运动学核心 -------------------
class LegKinematics:
    def __init__(self, l3=0.2, l4=0.1):
        self.l3 = l3; self.l4 = l4  # 大腿/小腿长度

    def inverse_kinematics(self, x: float, z: float) -> tuple:
        """ 计算关节角度 (注意机械结构调整符号) """
        l0 = math.hypot(x, z)
        theta_inside = math.acos((self.l4**2 + l0**2 - self.l3**2) / (2*self.l4*l0))
        theta = math.atan2(z, x)
        return theta + theta_inside, theta - theta_inside

# ------------------- 运动轨迹生成器 -------------------
class MotionGenerator:
    def __init__(self):
        self.params = GaitParams()
        self.current_phase = 0.0
        self.cmd_vel = [0.0, 0.0]  # [linear.x, angular.z]
        
    def update_command(self, vx: float, wz: float):
        """ 更新运动指令 """
        self.cmd_vel = [vx, wz]
        
    def get_foot_target(self, leg_id: int) -> tuple:
        """ 生成合成运动轨迹 (新版本) """
        # 1. 基础相位计算
        phase = (self.current_phase + self.params.phase_offsets[leg_id]) % 1.0
        
        # 2. 计算合成步幅 (linear.x 和 angular.z 共同影响)
        linear_stride = self.cmd_vel[0] * self.params.max_stride  # [-0.4, 0.4]
        angular_stride = self.cmd_vel[1] * self.params.max_turn_offset  # [-0.2, 0.2]
        
        # 3. 根据腿部分配步幅 (左右腿反向)
        if leg_id in [Leg.FR, Leg.RR]:  # 右腿
            stride = linear_stride + angular_stride
        else:  # 左腿
            stride = linear_stride - angular_stride
        half_stride = stride / 2
        # 4. 轨迹生成
        if phase < 0.5:  # 摆动相 (空中)
            z = self.params.z_base + self.params.z_swing * math.sin(phase * 2 * math.pi)
            x_progress = phase * 2  # 归一化[0,1]
            x = -half_stride + x_progress * stride  # 自动包含方向
        else:  # 支撑相 (地面)
            z = self.params.z_base
            x_progress = (phase - 0.5) * 2  # 归一化[0,1]
            x = half_stride - x_progress * stride
        
        return x, z
    
    def step(self):
        """ 更新步态相位 (速度自适应) """
        speed = abs(self.cmd_vel[0]) + abs(self.cmd_vel[1]) * 0.5
        phase_step = self.params.dt / (self.params.period )
        self.current_phase = (self.current_phase + phase_step) % 1.0

# ------------------- 主控制节点 -------------------
class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        
        # 初始化组件
        self.ik = LegKinematics()
        self.motion = MotionGenerator()
        
        # ROS接口
        self.joint_pub = self.create_publisher(JointState, '/action', 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(self.motion.params.dt, self.control_loop)
        
        self.get_logger().info("运动控制器已启动 | 支持前进/后退/转向")

    def cmd_vel_callback(self, msg: Twist):
        """ 处理速度指令 """
        self.motion.update_command(msg.linear.x, msg.angular.z)
        
    def control_loop(self):
        """ 主控制循环 """
        # 更新运动相位
        self.motion.step()
        
        # 计算各腿目标位置
        leg_targets = {
            leg: self.motion.get_foot_target(leg) 
            for leg in [Leg.FL, Leg.FR, Leg.RL, Leg.RR]
        }
        
        # 生成关节指令
        joint_cmd = [0.0] * 9
        for leg, (x, z) in leg_targets.items():
            theta1, theta4 = self.ik.inverse_kinematics(x, z)
            
            # 机械结构调整 (根据实际安装方向可能需要修改)
            if leg == Leg.FL:
                joint_cmd[0] = -theta1; joint_cmd[1] = theta4 + math.pi
            elif leg == Leg.FR:
                joint_cmd[2] = theta1; joint_cmd[3] = -theta4 - math.pi
            elif leg == Leg.RL:
                joint_cmd[5] = -theta4 - math.pi; joint_cmd[6] = theta1
            elif leg == Leg.RR:
                joint_cmd[7] = theta4 + math.pi; joint_cmd[8] = -theta1
        
        # 发布指令
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = joint_cmd
        self.joint_pub.publish(msg)
        
        # 调试信息 (限频)
        if int(self.motion.current_phase * 20) % 5 == 0:
            self.get_logger().info(
                f"Phase: {self.motion.current_phase:.2f} | "
                f"FL: x={leg_targets[Leg.FL][0]:.2f} | "
                f"FR: x={leg_targets[Leg.FR][0]:.2f}",
                throttle_duration_sec=0.2)

# ------------------- 主函数 -------------------
def main(args=None):
    rclpy.init(args=args)
    controller = LocomotionController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("控制器安全停止")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
