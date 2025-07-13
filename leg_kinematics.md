
# 并联腿运动学公式 (Parallel Leg Kinematics)

## 链节长度 (Link Lengths)

| 符号 | 描述 | 长度 (m) |
|------|------|-----------|
| **L3** | 上连杆 / 大腿段 (upper link / proximal segment) | **0.224** |
| **L4** | 下连杆 / 小腿段 (lower link / distal segment)  | **0.090** |

> 说明：`L3` 与 `L4` 即代码中 `#define L3 0.224f` 与 `#define L4 0.09f`，分别对应两段刚性连杆的物理长度。

## 关节角定义 (Joint Angle Definitions)

* **θ1 (theta1)**  
  上连杆与水平 **X 轴** 的夹角，逆时针为正方向。代码变量 `theta1`，对应 **Motor-0 (Hip)**。
* **θ4 (theta4)**  
  下连杆与水平 **X 轴** 的夹角，逆时针为正方向。代码变量 `theta4`，对应 **Motor-1 (Knee)**。在实际驱动前会加上 π (180°) 的机械安装偏置。

> 其余电机控制参数（P、D、F、Torque 等）与运动学无关，故未在此记录。

---

## 逆运动学 (Inverse Kinematics)

给定足端目标位置 \(P = (x, z)\)（单位：米），计算关节角 \(\theta_1\)、\(\theta_4\)：

1. **连杆末端距原点距离**  
   $$ l_0 = \sqrt{x^2 + z^2} $$

2. **可达性约束**  
   $$ |L_3 - L_4| \; \le \; l_0 \; \le \; L_3 + L_4 $$
   若不满足，则目标超出工作空间。

3. **辅助角**  
   $$
     \theta_{inside} = \arccos \left( \frac{L_4^2 + l_0^2 - L_3^2}{2 L_4 l_0} \right) , \quad
     \theta = \operatorname{atan2}(z,\,x)
   $$

4. **关节解**  
   $$
     \theta_1 = \theta + \theta_{inside}, \qquad
     \theta_4 = \theta - \theta_{inside}
   $$

---
<!-- 
## 正运动学 (Forward Kinematics)

在满足并联约束的前提下（即两段连杆末端重合于同一点），足端位置 \((x, z)\) 可由下式得到：

$$
  x = L_3 \cos \theta_1 = L_4 \cos \theta_4 , \qquad
  z = L_3 \sin \theta_1 = L_4 \sin \theta_4
$$

实际实现中，可任选其中一组等式（通常使用 \(L_3\)）进行数值计算：

$$
  x = L_3 \cos \theta_1 , \qquad
  z = L_3 \sin \theta_1
$$

---

## 参考 (Reference)

本公式对应于固件函数 `inverse_kinematic()` 的实现，角度以 **弧度 (rad)** 为单位。  -->