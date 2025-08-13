## ARESdog 控制仓库（ROS2）

本仓库用于控制四足机器人 ARESdog 的动作与状态管理，包含基于 ROS2 的状态机控制器、逆运动学、行走/跳跃/空翻等多种动作实现，以及一组独立的调试与演示脚本。

- **核心模块**: `state_machine_package`（模块化状态机控制器）
- **运行方式**: 直接以 Python 脚本运行，或集成至 ROS2 工作区
- **适用场景**: 机器人步态控制、动作序列、姿态管理、导航占位

## 仓库结构

```text
control_respository/
├── package.xml
├── state_machine_package/              # 模块化状态机控制器（核心）
│   ├── constants.py                    # 常量/枚举/参数
│   ├── kinematics.py                   # 逆运动学
│   ├── base_state.py                   # 状态基类
│   ├── state_machine_controller.py     # 主控制器（ROS2 Node）
│   ├── run_controller.py               # 启动脚本（推荐）
│   ├── main.py                         # 模块入口
│   ├── states/                         # 具体状态（idle/walk/flip/jump/...）
│   ├── README.md
│   └── REFACTORING_SUMMARY.md
├── gait_test.py / flip_forward.py / ... # 若干实验/演示脚本
├── .gitignore
└── README.md                           # 本文档
```

更多细节请参考子模块文档：`state_machine_package/README.md`。

## 主要功能

- **状态机控制**: Idle、Walk、Flip、Jump、Recovery、Error、Stair Jump、Navigation 等
- **逆运动学**: `LegKinematics` 提供腿部极坐标 IK 计算
- **ROS2 接口**: 话题订阅/发布，控制关节、参数调整、状态切换
- **可扩展性**: 通过继承 `State` 快速添加新状态

## 快速开始

### 环境要求
- ROS2（建议 Humble）
- Python 3.8+
- 依赖包：`rclpy`、`numpy`、`tf_transformations`、`sensor_msgs`、`geometry_msgs`、`std_msgs`

### 获取源码
将本仓库置于 ROS2 工作区，例如：`~/ARESdog_ws/src/control_respository`

### 安装依赖（Python）
可按需创建虚拟环境，然后安装：
```bash
pip install numpy tf_transformations
# rclpy 及 ROS 消息类型由 ROS2 提供
```

### 运行控制器（推荐）
直接以 Python 启动脚本运行：
```bash
cd state_machine_package
python run_controller.py
```
或以模块运行：
```bash
python -m state_machine_package.main
```
在完整 ROS2 环境中建议先加载环境：
```bash
source /opt/ros/humble/setup.bash
source ~/ARESdog_ws/install/setup.bash
```

## ROS 话题与交互
- **状态切换**（订阅 `/state`，类型 `std_msgs/Int8`）
  - `1`: Flip（空翻）
  - `2`: Jump（前跳）
  - `3`: Recovery（恢复）
  - `4`: Navigation（导航）
  - `5`: Stair Jump（台阶跳跃）
- **速度控制**（订阅 `/cmd_vel`，类型 `geometry_msgs/Twist`）
  - `linear.x`：前进/后退速度
  - `angular.z`：转向速度
- **参数调整**（订阅 `/params`，类型 `std_msgs/Float32MultiArray`）
  - `[0]` 腿长；`[1]` 俯仰角；`[2]` 步态周期；`[3]` 步幅；`[4]` 摆动高度
- **关节控制发布**
  - `/action`（`sensor_msgs/JointState`）关节位置
  - `/kp_cmd`、`/kd_cmd`（`sensor_msgs/JointState`）控制参数

## 开发指南
- 新增状态：
  1. 在 `states/` 新建状态类文件，继承 `State`
  2. 在 `constants.py` 的 `StateType` 中登记（如需要）
  3. 在 `state_machine_controller.py` 的 `self.states` 中注册
  4. 在 `states/__init__.py` 中导出类
- 调试建议：
  - 先用 `state_machine_package/test_package.py` 做导入与基础逻辑验证
  - 运行 `run_controller.py`，通过话题交互验证状态切换与动作输出

## 常见问题
- 导入错误：确保从 `state_machine_package` 目录运行或以模块方式运行
- ROS 环境：确保已 `source` ROS2 与工作区 `install` 环境

## 许可证
若无特别声明，本仓库默认以私有或内部用途为主。若需开源许可证，请在此处补充。

## 致谢
感谢贡献者对状态机模块化重构与动作实现所做的工作，详细重构说明见 `state_machine_package/REFACTORING_SUMMARY.md`。
