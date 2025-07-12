import os
from typing import Dict, List

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, Vector3, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ============================
#  Utility math helpers
# ============================

def quat_inv(q: np.ndarray) -> np.ndarray:
    """Return quaternion inverse.

    Args:
        q: Quaternion in ``[w, x, y, z]`` order.

    Returns:
        Inverse quaternion.
    """
    norm_sq = np.dot(q, q)
    if norm_sq < 1e-12:
        raise ValueError("Zero-norm quaternion has no inverse.")
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32) / norm_sq


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-D vector *v* by quaternion *q*.

    The implementation avoids constructing an intermediate 4×4 matrix for speed.
    """
    w, x, y, z = q / np.linalg.norm(q)
    x2, y2, z2 = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    xw, yw, zw = x * w, y * w, z * w

    rot = np.array([
        [1 - 2 * (y2 + z2), 2 * (xy + zw), 2 * (xz - yw)],
        [2 * (xy - zw), 1 - 2 * (x2 + z2), 2 * (yz + xw)],
        [2 * (xz + yw), 2 * (yz - xw), 1 - 2 * (x2 + y2)],
    ], dtype=np.float32)
    return rot @ v


# ============================
#  Model controller
# ============================

class ModelController:
    """Wrap an ONNX policy network for quadruped control."""

    OBS_DIM: int = 36  # (3 + 3 + 3 + 9 + 9 + 9)
    ACTION_DIM: int = 9
    NUM_JOINTS_FOR_OBS: int = 9

    ACTION_CLIP_RANGE: float = 1.22
    ACTION_SCALE_FACTOR: float = 0.25

    COMMAND_LIN_RANGE: float = 3.5  # m/s
    COMMAND_ANG_RANGE: float = 2.0  # rad/s

    def __init__(self, model_path: str, ros_node: Node | None = None) -> None:
        self._node = ros_node
        self._session = self._load_onnx_model(model_path)

        self.command = np.array([2.0, 0.0, 0.0], dtype=np.float32)
        self.last_action = np.zeros(self.ACTION_DIM, dtype=np.float32)

        if self._node:
            self._node.get_logger().info("ModelController initialised successfully.")

    # ------------------------------------------------------------------
    #  Private helpers
    # ------------------------------------------------------------------

    def _load_onnx_model(self, model_path: str) -> ort.InferenceSession:
        if not os.path.exists(model_path):
            msg = f"ONNX model not found at {model_path}"
            if self._node:
                self._node.get_logger().error(msg)
            raise FileNotFoundError(msg)

        providers: List[str] = ["CPUExecutionProvider"]
        if ort.get_device() == "GPU":
            providers.insert(0, "CUDAExecutionProvider")
            if self._node:
                self._node.get_logger().info("CUDA available – using GPU for inference.")

        opts = ort.SessionOptions()
        session = ort.InferenceSession(model_path, sess_options=opts, providers=providers)
        return session

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def set_command(self, lin_x: float, lin_y: float, ang_z: float) -> None:
        self.command = np.array([lin_x, lin_y, ang_z], dtype=np.float32)
        if self._node and self._node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self._node.get_logger().debug(f"Updated command: {self.command}")

    def forward(self, state: Dict[str, np.ndarray]) -> np.ndarray:
        """Run inference and return scaled joint targets."""
        obs = self._build_observation(state)
        raw_action = self._inference(obs)
        self.last_action = raw_action
        scaled_action = raw_action * self.ACTION_SCALE_FACTOR
        return scaled_action

    # ------------------------------------------------------------------
    #  Internal computation
    # ------------------------------------------------------------------

    def _build_observation(self, state: Dict[str, np.ndarray]) -> np.ndarray:
        base_rot = state["base_rotation"]
        base_ang_vel = state["base_angular_velocity"]
        joint_pos = state["joint_positions"][: self.NUM_JOINTS_FOR_OBS]
        joint_vel = state["joint_velocities"][: self.NUM_JOINTS_FOR_OBS]

        gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        gravity_body = quat_rotate(quat_inv(base_rot), gravity_world)

        obs = np.zeros(self.OBS_DIM, dtype=np.float32)
        obs[0:3] = base_ang_vel
        obs[3:6] = gravity_body
        obs[6:9] = self.command
        obs[9:18] = joint_pos
        obs[18:27] = joint_vel
        obs[27:36] = self.last_action
        return obs

    def _inference(self, obs: np.ndarray) -> np.ndarray:
        obs_batch = obs[None, :]  # Add batch dimension
        input_name = self._session.get_inputs()[0].name
        output_name = self._session.get_outputs()[0].name
        action = self._session.run([output_name], {input_name: obs_batch})[0][0]
        return action


# ============================
#  ROS 2 Node wrapper
# ============================

class RobotControllerNode(Node):
    """ROS 2 node providing real-time joint targets using an ONNX policy."""

    CONTROLLED_JOINT_NAMES: List[str] = [
        "front_waist_fl_i_up",
        "front_waist_fl_o_up",
        "front_waist_fr_i_up",
        "front_waist_fr_o_up",
        "front_waist_waist",
        "back_waist_hl_i_up",
        "back_waist_hl_o_up",
        "back_waist_hr_i_up",
        "back_waist_hr_o_up",
    ]

    def __init__(self) -> None:
        super().__init__("robot_controller")

        # Resolve ONNX model path via package share directory for portability
        pkg_share_dir = get_package_share_directory("robot_controller")
        onnx_path = os.path.join(pkg_share_dir, "policy.onnx")

        self._controller = ModelController(model_path=onnx_path, ros_node=self)

        # Internal state placeholders
        self._latest_joint_state: JointState | None = None
        self._latest_orientation: Quaternion | None = None
        self._latest_angular_velocity: np.ndarray | None = None
        self._joint_index_map: List[int] | None = None

        # QoS profiles
        sensor_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cmd_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self.create_subscription(JointState, "joint_state", self._joint_state_cb, sensor_qos)
        self.create_subscription(Quaternion, "orient", self._orient_cb, sensor_qos)
        self.create_subscription(Vector3, "angvel", self._angvel_cb, sensor_qos)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, cmd_qos)

        # Publisher
        self._action_pub = self.create_publisher(JointState, "action", cmd_qos)

        # Timer (50 Hz)
        self.create_timer(0.02, self._timer_cb)

        self.get_logger().info("RobotControllerNode successfully started.")

    # ------------------------------------------------------------------
    #  Subscription callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        if self._joint_index_map is None:
            self._joint_index_map = []
            for name in self.CONTROLLED_JOINT_NAMES:
                try:
                    idx = msg.name.index(name)
                    self._joint_index_map.append(idx)
                except ValueError:
                    self.get_logger().error(f"Required joint '{name}' not found in JointState message.")
            if len(self._joint_index_map) != len(self.CONTROLLED_JOINT_NAMES):
                self.get_logger().fatal("Not all required joints are present – shutting down node.")
                raise RuntimeError("Missing joints in JointState message")

    def _orient_cb(self, msg: Quaternion) -> None:
        self._latest_orientation = msg

    def _angvel_cb(self, msg: Vector3) -> None:
        self._latest_angular_velocity = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        lin_x = float(msg.linear.x)
        lin_y = float(msg.linear.y)
        ang_z = float(msg.angular.z)

        # Clip to policy range
        lin_x = np.clip(lin_x, 0.0, self._controller.COMMAND_LIN_RANGE)
        lin_y = np.clip(lin_y, 0.0, 0.0)
        ang_z = np.clip(ang_z, -self._controller.COMMAND_ANG_RANGE, self._controller.COMMAND_ANG_RANGE)

        self._controller.set_command(lin_x, lin_y, ang_z)

    # ------------------------------------------------------------------
    #  Timer callback
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        if (
            self._latest_joint_state is None
            or self._latest_orientation is None
            or self._latest_angular_velocity is None
            or self._joint_index_map is None
        ):
            self.get_logger().debug("Waiting for all sensor data …")
            return

        # Assemble state arrays for the policy
        joint_positions = np.zeros(9, dtype=np.float32)
        joint_velocities = np.zeros(9, dtype=np.float32)

        for i, idx in enumerate(self._joint_index_map):
            if idx < len(self._latest_joint_state.position):
                joint_positions[i] = self._latest_joint_state.position[idx]
            if idx < len(self._latest_joint_state.velocity):
                joint_velocities[i] = self._latest_joint_state.velocity[idx]

        state = {
            "base_rotation": np.array([
                self._latest_orientation.w,
                self._latest_orientation.x,
                self._latest_orientation.y,
                self._latest_orientation.z,
            ], dtype=np.float32),
            "base_angular_velocity": self._latest_angular_velocity,
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
        }

        action = self._controller.forward(state)

        # Publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.CONTROLLED_JOINT_NAMES
        msg.position = action.tolist()
        self._action_pub.publish(msg)


# ============================
#  Entrypoint
# ============================

def main(args: List[str] | None = None) -> None:  # noqa: D401 – keep signature ros-friendly
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped via Ctrl-C.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 