/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#include "gait/LegKinematics.h"
#include <cmath>

LegKinematics::LegKinematics()
    : _emergencyStop(false),
      _currentTheta1(0.0f),
      _currentTheta4(0.0f)
{
    // 构造函数，初始化成员变量
}

LegKinematics::~LegKinematics()
{
    // 析构函数
}

bool LegKinematics::inverseKinematic(float x, float z, float* theta1, float* theta4)
{
    float l0 = std::sqrt(x * x + z * z);

    // 检查是否可解
    if (std::fabs(L3 - L4) >= l0 || l0 >= (L3 + L4))
    {
        return false; // 返回错误标志
    }

    float theta_inside = std::acos((L4 * L4 + l0 * l0 - L3 * L3) / (2.0f * L4 * l0));
    float theta = std::atan2(z, x);

    *theta1 = theta + theta_inside;
    *theta4 = theta - theta_inside;
    
    // 更新当前角度
    _currentTheta1 = *theta1;
    _currentTheta4 = *theta4;
    
    return true; // 成功标志
}

void LegKinematics::forwardKinematic(float theta1, float theta4, float* x, float* z)
{
    // 从关节角度计算末端执行器位置
    float l3_x = L3 * std::cos(theta1);
    float l3_z = L3 * std::sin(theta1);
    
    float l4_x = L4 * std::cos(theta4);
    float l4_z = L4 * std::sin(theta4);
    
    // 计算末端位置
    *x = l3_x + l4_x;
    *z = l3_z + l4_z;
    
    // 更新当前角度
    _currentTheta1 = theta1;
    _currentTheta4 = theta4;
}

void LegKinematics::generateCycloidTrajectory(Point2D start, Point2D end, float height, Point2D* traj, int num_points)
{
    float delta_x = end.x - start.x;
    float delta_z = end.z - start.z; // 起点与落点可能存在高度差

    for (int i = 0; i < num_points; ++i)
    {
        float t = 2.0f * M_PI * i / (num_points - 1);

        // X方向摆线
        traj[i].x = start.x + delta_x * (t - std::sin(t)) / (2.0f * M_PI);

        // Z方向摆线
        traj[i].z = start.z + delta_z * (i / static_cast<float>(num_points)) + height * (1.0f - std::cos(t)) / 2.0f;
    }
}

void LegKinematics::setEmergencyStop(bool stop)
{
    _emergencyStop = stop;
}

bool LegKinematics::isEmergencyStop() const
{
    return _emergencyStop;
} 