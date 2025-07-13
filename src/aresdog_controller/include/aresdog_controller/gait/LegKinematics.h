/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <cmath>

// 机械参数配置
#define L1 0.09f
#define L2 0.224f
#define L3 0.224f
#define L4 0.09f
#define CONTROL_CYCLE 10 // 控制周期10ms
#define TRAJ_POINTS 30   // 轨迹点数
#define BACK_TRAJ_POINTS 30
#define P_MOTOR  10
#define D_MOTOR  1
#define F_MOTOR  7
#define Torque 10

/**
 * @brief 二维点结构
 */
struct Point2D {
    float x;
    float z;
    
    Point2D() : x(0.0f), z(0.0f) {}
    Point2D(float _x, float _z) : x(_x), z(_z) {}
};

/**
 * @brief 腿部运动学类
 * 
 * 包含四足机器人单腿的正逆运动学计算
 */
class LegKinematics {
public:
    /**
     * @brief 构造函数
     */
    LegKinematics();
    
    /**
     * @brief 析构函数
     */
    ~LegKinematics();

    /**
     * @brief 逆运动学计算 (X-Z平面)
     * 
     * @param x X坐标
     * @param z Z坐标
     * @param theta1 输出角度1
     * @param theta4 输出角度4
     * @return bool 计算是否成功
     */
    bool inverseKinematic(float x, float z, float* theta1, float* theta4);
    
    /**
     * @brief 正运动学计算 (X-Z平面)
     * 
     * @param theta1 关节角度1 (弧度)
     * @param theta4 关节角度4 (弧度)
     * @param x 输出X坐标
     * @param z 输出Z坐标
     */
    void forwardKinematic(float theta1, float theta4, float* x, float* z);
    
    /**
     * @brief 生成摆线轨迹点序列
     * 
     * @param start 起始点
     * @param end 终止点
     * @param height 轨迹高度
     * @param traj 输出轨迹点数组
     * @param num_points 轨迹点数量
     */
    void generateCycloidTrajectory(Point2D start, Point2D end, float height, Point2D* traj, int num_points);
    
    /**
     * @brief 设置紧急停止状态
     * 
     * @param stop 是否紧急停止
     */
    void setEmergencyStop(bool stop);
    
    /**
     * @brief 获取紧急停止状态
     * 
     * @return bool 是否处于紧急停止状态
     */
    bool isEmergencyStop() const;

private:
    // 紧急停止标志
    bool _emergencyStop;
    
    // 当前关节角度
    float _currentTheta1;
    float _currentTheta4;
};

#endif  // LEG_KINEMATICS_H 