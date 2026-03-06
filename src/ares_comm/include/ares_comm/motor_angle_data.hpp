#pragma once
#include <cstdint>
#include <array>

namespace ares_comm {

/**
 * @brief 电机角度数据结构
 * 
 * 用于存储下位机发送的电机编码器角度数据
 */
struct MotorAngleData {
    static constexpr size_t MOTOR_COUNT = 12;  // 4条腿，每条腿3个电机
    
    // 原始编码器数据
    std::array<int32_t, MOTOR_COUNT> encoder_values;
    
    // 转换后的角度数据（弧度）
    std::array<float, MOTOR_COUNT> angle_values;
    
    // 用于标识每个电机的状态
    std::array<uint8_t, MOTOR_COUNT> motor_status;
    
    // 构造函数
    MotorAngleData() {
        encoder_values.fill(0);
        angle_values.fill(0.0f);
        motor_status.fill(0);
    }
    
    // 将编码器值转换为角度值(弧度)
    void convert_to_radians(double encoder_to_radians_factor = 0.01) {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            angle_values[i] = static_cast<float>(encoder_values[i] * encoder_to_radians_factor);
        }
    }
};

} // namespace ares_comm 