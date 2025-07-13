/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "gait/WaveGenerator.h"
#include "gait/CycloidTrajectory.h"
#include "gait/LegKinematics.h"
#include "common/aresRobot.h"
#include "message/LowlevelState.h"

/**
 * @brief 控制组件前向声明
 */
class CtrlComponents;

/**
 * @brief 步态生成器类
 * 
 * 基于摆线轨迹和波形生成器，为串联腿四足机器人生成步态
 */
class GaitGenerator {
public:
    /**
     * @brief 构造函数
     * 
     * @param ctrlComp 控制组件
     */
    GaitGenerator(CtrlComponents *ctrlComp);
    
    /**
     * @brief 析构函数
     */
    ~GaitGenerator();

    /**
     * @brief 设置步态参数
     * 
     * @param vxyGoalGlobal 全局XY平面目标速度
     * @param dYawGoal 目标偏航角速度
     * @param gaitHeight 步态高度
     */
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);

    /**
     * @brief 运行步态生成器
     * 
     * @param feetPos 输出 - 四足位置
     * @param feetVel 输出 - 四足速度
     */
    void run(Vec34 &feetPos, Vec34 &feetVel);

    /**
     * @brief 获取指定腿的足端位置
     * 
     * @param i 腿的索引 (0-3)
     * @return Vec3 足端位置
     */
    Vec3 getFootPos(int i);

    /**
     * @brief 获取指定腿的足端速度
     * 
     * @param i 腿的索引 (0-3)
     * @return Vec3 足端速度
     */
    Vec3 getFootVel(int i);

    /**
     * @brief 重新启动步态生成器
     */
    void restart();
    
    /**
     * @brief 为指定腿生成自定义摆线轨迹
     * 
     * @param legID 腿的ID (0-3)
     * @param start 起始位置
     * @param end 目标位置
     * @param height 抬腿高度
     * @param traj 输出轨迹点数组
     * @param num_points 轨迹点数量
     */
    void generateLegTrajectory(int legID, Point2D start, Point2D end, float height, Point2D* traj, int num_points);
    
    /**
     * @brief 计算指定腿的逆运动学
     * 
     * @param legID 腿的ID (0-3)
     * @param x X坐标
     * @param z Z坐标
     * @param theta1 输出角度1
     * @param theta4 输出角度4
     * @return bool 计算是否成功
     */
    bool calculateLegIK(int legID, float x, float z, float* theta1, float* theta4);
    
    /**
     * @brief 设置紧急停止状态
     * 
     * @param stop 是否停止
     */
    void setEmergencyStop(bool stop);

private:
    // 外部引用
    WaveGenerator *_waveG;       // 波形生成器
    Vec4 *_phase;                // 相位指针
    VecInt4 *_contact;           // 接触状态指针
    QuadrupedRobot *_robModel;   // 机器人模型
    LowlevelState *_state;       // 底层状态

    // 本地变量
    CycloidTrajectory _cycloid;  // 摆线轨迹生成器
    LegKinematics _legKin[4];    // 四条腿的运动学器
    float _gaitHeight;           // 步态高度
    Vec2 _vxyGoal;               // 目标XY速度
    float _dYawGoal;             // 目标偏航角速度
    Vec4 _phasePast;             // 过去的相位
    Vec34 _startP, _endP;        // 起点和终点位置
    Vec34 _idealP, _pastP;       // 理想位置和过去位置
    bool _firstRun;              // 是否首次运行

private:
    /**
     * @brief 计算足端目标位置
     * 
     * @param legID 腿的ID (0-3)
     * @param vxyGoal 目标XY速度
     * @param dYawGoal 目标偏航角速度
     * @param phase 相位
     * @return Vec3 足端目标位置
     */
    Vec3 calculateFootTargetPosition(int legID, Vec2 vxyGoal, float dYawGoal, float phase);
};

#endif  // GAITGENERATOR_H 