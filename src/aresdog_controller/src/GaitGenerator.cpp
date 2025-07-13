
#include "gait/GaitGenerator.h"
#include "control/CtrlComponents.h"
#include "control/Estimator.h"
#include <cmath>

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
    : _waveG(ctrlComp->waveGen), 
      _phase(ctrlComp->phase), 
      _contact(ctrlComp->contact),
      _robModel(ctrlComp->robotModel), 
      _state(ctrlComp->lowState),
      _gaitHeight(0.05),  // 默认步态高度5cm
      _dYawGoal(0.0),
      _firstRun(true)
{
    _vxyGoal.setZero();
    _phasePast.setZero();
    _startP.setZero();
    _endP.setZero();
    _idealP = _robModel->getFeetPosIdeal();
    _pastP.setZero();
}

GaitGenerator::~GaitGenerator()
{
    // 析构函数
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight)
{
    // 设置步态参数
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart()
{
    // 重置步态生成器状态
    _firstRun = true;
    _vxyGoal.setZero();
    _dYawGoal = 0.0;
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel)
{
    if (_firstRun) {
        // 首次运行，初始化起点位置
        for (int i = 0; i < 4; ++i) {
            _startP.col(i) = _robModel->getFootPosition(*_state, i, FrameType::BODY);
        }
        _firstRun = false;
    }

    // 处理每条腿的轨迹
    for (int i = 0; i < 4; ++i) {
        if ((*_contact)(i) == 1) {
            // 支撑相：保持接触地面，位置保持不变，速度为零
            if ((*_phase)(i) < 0.5) {
                // 支撑相前半段，更新起始位置
                _startP.col(i) = _robModel->getFootPosition(*_state, i, FrameType::BODY);
            }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        } else {
            // 摆动相：计算目标位置
            _endP.col(i) = calculateFootTargetPosition(i, _vxyGoal, _dYawGoal, (*_phase)(i));
            
            // 计算足端位置和速度
            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }
    
    // 保存当前状态
    _pastP = feetPos;
    _phasePast = *_phase;
}

Vec3 GaitGenerator::getFootPos(int i)
{
    // 使用摆线轨迹计算XYZ方向上的位置
    Vec3 footPos;
    
    // 计算XY平面位置
    footPos(0) = _cycloid.cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = _cycloid.cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    
    // 计算Z方向位置（高度）
    footPos(2) = _cycloid.cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i)
{
    // 使用摆线轨迹计算XYZ方向上的速度
    Vec3 footVel;
    
    // 计算XY平面速度
    footVel(0) = _cycloid.cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i), _waveG->getTswing());
    footVel(1) = _cycloid.cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i), _waveG->getTswing());
    
    // 计算Z方向速度
    footVel(2) = _cycloid.cycloidZVelocity(_gaitHeight, (*_phase)(i), _waveG->getTswing());
    
    return footVel;
}

Vec3 GaitGenerator::calculateFootTargetPosition(int legID, Vec2 vxyGoal, float dYawGoal, float phase)
{
    // 获取机器人理想足端位置
    Vec3 defaultPos = _idealP.col(legID);
    Vec3 targetPos;
    
    // 计算步态周期内的位移
    float totalTime = _waveG->getT();
    float dx = vxyGoal(0) * totalTime;
    float dy = vxyGoal(1) * totalTime;
    
    // 计算旋转角度
    float dTheta = dYawGoal * totalTime;
    
    // 构建旋转矩阵
    float cosTheta = cos(dTheta);
    float sinTheta = sin(dTheta);
    Mat3 R;
    R << cosTheta, -sinTheta, 0,
         sinTheta,  cosTheta, 0,
                0,         0, 1;
    
    // 计算目标位置，考虑线性运动和旋转
    targetPos = R * defaultPos;
    targetPos(0) += dx;
    targetPos(1) += dy;
    
    return targetPos;
}

void GaitGenerator::generateLegTrajectory(int legID, Point2D start, Point2D end, float height, Point2D* traj, int num_points)
{
    // 确保legID有效
    if (legID < 0 || legID >= 4) {
        return;
    }
    
    // 使用LegKinematics类生成轨迹
    _legKin[legID].generateCycloidTrajectory(start, end, height, traj, num_points);
}

bool GaitGenerator::calculateLegIK(int legID, float x, float z, float* theta1, float* theta4)
{
    // 确保legID有效
    if (legID < 0 || legID >= 4) {
        return false;
    }
    
    // 使用LegKinematics类进行逆运动学计算
    return _legKin[legID].inverseKinematic(x, z, theta1, theta4);
}

void GaitGenerator::setEmergencyStop(bool stop)
{
    // 设置所有腿的紧急停止状态
    for (int i = 0; i < 4; ++i) {
        _legKin[i].setEmergencyStop(stop);
    }
} 