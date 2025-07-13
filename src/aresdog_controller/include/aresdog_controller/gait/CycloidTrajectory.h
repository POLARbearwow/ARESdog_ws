/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#ifndef CYCLOID_TRAJECTORY_H
#define CYCLOID_TRAJECTORY_H

#include "common/mathTypes.h"
#include "common/enumClass.h"
#include <cmath>
#include "gait/LegKinematics.h" // 引入新的LegKinematics头文件

/**
 * @brief 摆线轨迹生成器类
 * 
 * 为串联腿四足机器人生成摆线轨迹，可用于控制足端位置和速度
 */
class CycloidTrajectory {
public:
    /**
     * @brief 构造函数
     */
    CycloidTrajectory();
    
    /**
     * @brief 析构函数
     */
    ~CycloidTrajectory();

    /**
     * @brief 计算XY平面上的摆线位置
     * 
     * @param start 起始位置
     * @param end 终止位置
     * @param phase 相位 [0,1]
     * @return float 当前位置
     */
    float cycloidXYPosition(float start, float end, float phase);

    /**
     * @brief 计算XY平面上的摆线速度
     * 
     * @param start 起始位置
     * @param end 终止位置
     * @param phase 相位 [0,1]
     * @param swingTime 摆动相时间(秒)
     * @return float 当前速度
     */
    float cycloidXYVelocity(float start, float end, float phase, float swingTime);

    /**
     * @brief 计算Z方向的摆线位置
     * 
     * @param start 起始高度
     * @param height 抬腿高度
     * @param phase 相位 [0,1]
     * @return float 当前高度
     */
    float cycloidZPosition(float start, float height, float phase);

    /**
     * @brief 计算Z方向的摆线速度
     * 
     * @param height 抬腿高度
     * @param phase 相位 [0,1]
     * @param swingTime 摆动相时间(秒)
     * @return float 当前速度
     */
    float cycloidZVelocity(float height, float phase, float swingTime);
};

#endif  // CYCLOID_TRAJECTORY_H 