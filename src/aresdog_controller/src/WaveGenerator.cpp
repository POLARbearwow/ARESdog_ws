/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#include "gait/WaveGenerator.h"
#include <iostream>
#include <cmath>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{
    // 检查支撑相比例参数有效性
    if ((_stRatio >= 1) || (_stRatio <= 0)) {
        std::cout << "[错误] WaveGenerator的支撑相比例应在(0, 1)范围内" << std::endl;
        exit(-1);
    }

    // 检查相位偏差参数有效性
    for (int i = 0; i < bias.rows(); ++i) {
        if ((bias(i) > 1) || (bias(i) < 0)) {
            std::cout << "[错误] WaveGenerator的相位偏差应在[0, 1]范围内" << std::endl;
            exit(-1);
        }
    }

    // 初始化
    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
}

WaveGenerator::~WaveGenerator()
{
    // 析构函数
}

void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status)
{
    // 计算当前波形
    calcWave(_phase, _contact, status);

    // 处理状态切换
    if (status != _statusPast) {
        if (_switchStatus.sum() == 0) {
            _switchStatus.setOnes();
        }
        
        calcWave(_phasePast, _contactPast, _statusPast);
        
        // 处理特殊状态转换
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL)) {
            _contactPast.setOnes();
        } else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL)) {
            _contactPast.setZero();
        }
    }

    // 处理状态平滑过渡
    if (_switchStatus.sum() != 0) {
        for (int i = 0; i < 4; ++i) {
            if (_contact(i) == _contactPast(i)) {
                _switchStatus(i) = 0;
            } else {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        
        if (_switchStatus.sum() == 0) {
            _statusPast = status;
        }
    }

    // 返回结果
    phaseResult = _phase;
    contactResult = _contact;
}

float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

float WaveGenerator::getT()
{
    return _period;
}

void WaveGenerator::reset()
{
    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
    _switchStatus.setZero();
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL) {
        // 计算波形运行状态下的相位和接触状态
        _passT = (double)(getSystemTime() - _startT) * 1e-6;
        
        for (int i = 0; i < 4; ++i) {
            // 计算归一化时间，考虑相位偏差
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            
            // 根据归一化时间计算相位和接触状态
            if (_normalT(i) < _stRatio) {
                // 支撑相
                contact(i) = 1;
                phase(i) = _normalT(i) / _stRatio;
            } else {
                // 摆动相
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    } else if (status == WaveStatus::SWING_ALL) {
        // 全摆动状态
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5;
    } else if (status == WaveStatus::STANCE_ALL) {
        // 全支撑状态
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
} 