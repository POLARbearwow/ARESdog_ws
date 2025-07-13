/**********************************************************************
Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#include "gait/CycloidTrajectory.h"
#include <cmath>

CycloidTrajectory::CycloidTrajectory()
{
    // 构造函数，初始化成员变量
}

CycloidTrajectory::~CycloidTrajectory()
{
    // 析构函数
}

float CycloidTrajectory::cycloidXYPosition(float start, float end, float phase)
{
    // 标准摆线参数方程: x = r(t - sin(t)), 其中t为参数
    // 这里将phase映射到[0, 2π]
    float phasePI = 2.0 * M_PI * phase;
    
    // 计算位置：start + (end-start) * 归一化位置
    return (end - start) * (phasePI - sin(phasePI)) / (2.0 * M_PI) + start;
}

float CycloidTrajectory::cycloidXYVelocity(float start, float end, float phase, float swingTime)
{
    // 摆线位置对时间的导数
    float phasePI = 2.0 * M_PI * phase;
    
    // 计算速度：(end-start) * 归一化速度 / 摆动时间
    return (end - start) * (1.0 - cos(phasePI)) / swingTime;
}

float CycloidTrajectory::cycloidZPosition(float start, float height, float phase)
{
    // 摆线高度参数方程: z = r(1 - cos(t)), 其中t为参数
    // 这里将phase映射到[0, 2π]
    float phasePI = 2.0 * M_PI * phase;
    
    // 计算高度：start + height * 归一化高度
    return height * (1.0 - cos(phasePI)) / 2.0 + start;
}

float CycloidTrajectory::cycloidZVelocity(float height, float phase, float swingTime)
{
    // 摆线高度对时间的导数
    float phasePI = 2.0 * M_PI * phase;
    
    // 计算高度变化速度：height * 归一化速度 / 摆动时间
    return height * M_PI * sin(phasePI) / swingTime;
} 