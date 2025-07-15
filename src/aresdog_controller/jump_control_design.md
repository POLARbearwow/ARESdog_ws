# 跳跃控制设计文档（原地起跳）

*草稿 – 2025-07-16*

---

## 1. 总体目标
1. 让四足机器人在原地完成 **蹲下 → 弹跳离地 → 空中飞行 → 平稳落地 → 恢复站立** 全流程。
2. 复用已实现的 **摆线步态生成器**、**BalanceController** 与 **LandingController**，最小化新增代码。
3. 所有控制逻辑在上位机 ROS 2 中完成，实时性目标：控制循环 ≥ 200 Hz。

---

## 2. 状态机
```
            ┌────────┐  cmd_jump  ┌──────────┐  detect_off_ground ┌─────────┐
 STANDING ─▶│ CROUCH │──────────▶│  THRUST  │───────────────────▶│ FLIGHT  │
   ▲  ▲     └────────┘            └──────────┘                   │   │     
   │  └───────── touchdown_done ◀────────────── touchdown_detect ┘   │
   │                                                               │
   └────────────────────── STANDING  ◀────────────── landing_done ─┘
```
* **STANDING**：默认状态，由 `BalanceWalkNode` 或静态平衡控制维持。
* **CROUCH**：缓慢降低质心，腿部折叠储能。
* **THRUST**：短时间大力矩伸腿，产生向上冲量。
* **FLIGHT**：空中阶段，腿部收起以避免碰撞，并等待触地检测。
* **LANDING**：调用 `LandingController` 吸收冲击，完成站立过渡。

状态切换触发条件：
| 从 → 到 | 触发条件 |
|---------|---------|
| STANDING → CROUCH | `/jump_cmd` action 或参数自动循环 |
| CROUCH → THRUST   | 达到设定最小高度或时间上限 |
| THRUST → FLIGHT   | 足端力矩急剧下降（离地）或 IMU z 加速度为负 |
| FLIGHT → LANDING  | `LandingController` 触地检测（0x0203 力矩阈值） |
| LANDING → STANDING| `LandingController` 状态机返回 `STANDING` |

---

## 3. 关键阶段细节
### 3.1 CROUCH – 蹲下储能
* 使用摆线生成器的 **stance 相**，但逐步降低 `nominal_z_`（目标足端 z）到 `crouch_z = nominal_z_ - Δcrouch`。
* 时间常数 `t_crouch ≈ 0.3 s`，确保电机不过流。
* 腳步轨迹仍保持四足支撑（腿不抬起），仅 z 方向调整。

### 3.2 THRUST – 爆发伸腿
* 改变摆线参数：
  * 设定 `τ_peak` 前馈力矩，通过 `/torque_cmd` 发送至下位机。
  * 同时在 **1–2 个控制周期 (≈10 ms)** 将 `nominal_z_` 从 `crouch_z` 快速过渡回 `nominal_z_` 甚至 `nominal_z_+Δboost`，形成跃升冲量。
* 可选：在伸腿过程中取消 `BalanceController` 的 Δz 以获得对称推力。

### 3.3 FLIGHT – 空中收腿
* 检测离地后：
  * 设定腿部角度到 **收缩位姿**（例如 `theta1 = 0 rad`, `theta4 = 90°`），或继续原 gait swing 曲线但保持 `z` 大于地面安全高度。
  * 关闭 `/torque_cmd`，避免空打。
* 使用 IMU `orient` 判断机体姿态，必要时小增益 BalanceController 保姿。

### 3.4 LANDING – 平稳落地
* 交由已设计的 `LandingController`：
  * 订阅 `/joint_state.effort` 检测触地。
  * 生成冲击吸收前馈力矩曲线并发布 `/torque_cmd`。
  * 当落地完毕（Phase-3）后切回 STANDING。

---

## 4. 节点/模块
| 节点            | 角色 | 备注 |
|-----------------|------|------|
| `JumpController`| 高层状态机 & 参数管理 | 负责发布 `/jump_state`, `/jump_phase`，调用子控制器 |
| `BalanceWalkNode` | 站立 & 步态 | 在 STANDING / CROUCH / THRUST 前段复用其 IK & gait |
| `LandingController` | 落地吸收 | 已设计，复用 |
| `usb_bridge_node` | 帧发送/接收 | 已支持角度 & 力矩帧 |

---

## 5. ROS 2 接口
| 话题 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/jump_cmd` | std_msgs/Bool 或自定义 | App → JumpController | 触发一次跳跃 |
| `/jump_state` | std_msgs/String | JumpController → GUI | 当前状态机阶段 |
| `/action` | sensor_msgs/JointState | 各控制器 → usb_bridge | 关节目标角度 |
| `/torque_cmd` | sensor_msgs/JointState | 各控制器 → usb_bridge | 前馈力矩 |

---

## 6. 参数
| 参数 | 默认 | 说明 |
|------|------|------|
| `Δcrouch` | 0.05 m | 蹲下深度 |
| `t_crouch` | 0.3 s | 蹲下时长 |
| `τ_peak`   | 12 N·m | 弹跳最高前馈力矩 |
| `boost_z`  | 0.02 m | 伸腿时额外上抬量 |

---

## 7. 实施路线
1. **新增 `JumpController.cpp`**：实现状态机、计时与参数读取。
2. **修改 `BalanceWalkNode`**：允许动态调节 `nominal_z_`。
3. **复用 `LandingController`** 完成落地。
4. Gazebo 验证 → 硬件测试。

---

> 完成本设计后，跳跃功能可通过发布一次 `/jump_cmd` = true 触发，机器人将自动完成全流程并恢复站立。 

## 变更记录（2025-07-17）
* **取消前馈力矩控制**：下位机仅支持位置模式，故所有阶段仅通过 `/action` 发送关节角度，不再发布 `/torque_cmd`。
* **THRUST 阶段实现**：通过快速改变足端目标高度（`nominal_z_` + `boost_z`）来产生向上冲量。
* **LandingController**：沿用触地检测，但同样只输出位置指令（缓慢下沉吸收冲击）。

---

## 更新后关键调整
1. **接口简化**
   | 话题 | 类型 | 方向 | 用途 |
   |------|------|------|------|
   | `/jump_cmd` | std_msgs/Bool | App → JumpController | 触发跳跃 |
   | `/action`   | sensor_msgs/JointState | 控制器 → usb_bridge | 关节目标角度（position）|
   | `/jump_state` | std_msgs/String | JumpController → GUI | 状态机阶段 |

2. **THRUST 阶段**
   * 在 10 ms 内将 `nominal_z_` 从 `crouch_z` 快速升至 `nominal_z_ + boost_z`。
   * 依据机器人质量手动整定 `boost_z`（通常 0.02–0.03 m）。

3. **LANDING 阶段**
   * 改为**位置阻尼**策略：
     * Phase-0：保持足端在地面高度 `nominal_z_ + dz_land`（略低）
     * Phase-1：用指数插值让 `nominal_z_` 回到正常值，时间常数 0.2 s。

4. **实现影响**
   * 删除 `/torque_cmd` 发布、协议 `0x0102/0x0203` 将暂不使用。
   * `usb_bridge_node` 保持兼容，但在 Jump/ Landing 流程中不会发送力矩帧。 