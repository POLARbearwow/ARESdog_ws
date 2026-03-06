#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "ares_comm/serial_interface.hpp"
#include "ares_comm/protocol_parser.hpp"
#include "ares_comm/motor_angle_data.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ares_comm {

class MotorAngleNode : public rclcpp::Node {
public:
    MotorAngleNode() : Node("motor_angle_node") {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("read_timeout_ms", 100);
        this->declare_parameter<int>("motor_angle_data_id", 0x0003);  // 电机角度数据ID
        this->declare_parameter<double>("encoder_to_radians_factor", 0.01);  // 编码器值转换为弧度的系数
        this->declare_parameter<int>("motor_count_per_frame", 10);  // 每帧包含的电机数量
        this->declare_parameter<bool>("debug_output", false);  // 是否输出调试信息

        // 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        read_timeout_ms_ = this->get_parameter("read_timeout_ms").as_int();
        motor_angle_data_id_ = this->get_parameter("motor_angle_data_id").as_int();
        encoder_to_radians_factor_ = this->get_parameter("encoder_to_radians_factor").as_double();
        motor_count_per_frame_ = this->get_parameter("motor_count_per_frame").as_int();
        debug_output_ = this->get_parameter("debug_output").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "监听电机角度数据ID: 0x%04X, 每帧最多包含 %d 个电机", 
                   motor_angle_data_id_, motor_count_per_frame_);

        // 创建角度发布者
        motor_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "motors/angles", 10);
            
        // 创建原始编码器值发布者
        motor_encoder_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "motors/encoders", 10);

        // 打开串口
        try {
            serial_ = SerialInterface::create(port, baudrate);
            RCLCPP_INFO(this->get_logger(), "成功打开串口 %s，波特率 %d", 
                        port.c_str(), baudrate);
        } catch (const SerialException& e) {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s", e.what());
            return;
        }

        // 创建定时器，定期从串口读取数据
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MotorAngleNode::read_serial, this));
    }
    
    ~MotorAngleNode() {
        if (serial_ && serial_->is_open()) {
            serial_->close();
        }
    }

private:
    void read_serial() {
        if (!serial_ || !serial_->is_open()) {
            return;
        }

        try {
            // 尝试读取一个完整的帧
            std::vector<uint8_t> frame = serial_->read_frame();
            if (frame.empty()) {
                return;
            }

            // 打印接收到的原始帧（十六进制格式）
            if (debug_output_) {
                std::stringstream hex_data;
                hex_data << "接收到数据帧: ";
                for (uint8_t byte : frame) {
                    hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                }
                RCLCPP_DEBUG(this->get_logger(), "%s", hex_data.str().c_str());
            }

            try {
                FrameType frame_type = ProtocolParser::parse_frame(frame);
                if (frame_type == FrameType::DataFrame) {
                    // 解析数据帧
                    DataFrame data_frame = ProtocolParser::parse_data(frame);
                    
                    // 检查数据ID
                    if (data_frame.data_id == motor_angle_data_id_) {
                        size_t expected_data_size = motor_count_per_frame_ * sizeof(int32_t);
                        if (data_frame.data.size() < expected_data_size) {
                            RCLCPP_WARN(this->get_logger(), 
                                "数据大小不足，期望 %zu 字节，实际 %zu 字节", 
                                expected_data_size, data_frame.data.size());
                            return;
                        }

                        // 处理电机角度数据
                        MotorAngleData angle_data;
                        
                        // 解析数据（每4字节一个电机的int32_t编码器值）
                        size_t actual_motor_count = std::min(
                            static_cast<size_t>(motor_count_per_frame_), 
                            data_frame.data.size() / sizeof(int32_t));
                        
                        // 记录哪些电机得到了更新
                        std::vector<bool> updated_motors(MotorAngleData::MOTOR_COUNT, false);
                        
                        // 解析电机数据
                        for (size_t i = 0; i < actual_motor_count && i < MotorAngleData::MOTOR_COUNT; ++i) {
                            // 检查数据有效性
                            if (i * sizeof(int32_t) + 3 < data_frame.data.size()) {
                                angle_data.encoder_values[i] = ProtocolParser::read_le32(
                                    &data_frame.data[i * sizeof(int32_t)]);
                                updated_motors[i] = true;
                            }
                        }
                        
                        // 将编码器值转换为角度值（弧度）
                        angle_data.convert_to_radians(encoder_to_radians_factor_);
                        
                        // 创建标准消息用于发布
                        auto angle_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
                        auto encoder_msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
                        
                        // 设置消息数据
                        angle_msg->data.resize(MotorAngleData::MOTOR_COUNT);
                        encoder_msg->data.resize(MotorAngleData::MOTOR_COUNT);
                        
                        for (size_t i = 0; i < MotorAngleData::MOTOR_COUNT; ++i) {
                            angle_msg->data[i] = angle_data.angle_values[i];
                            encoder_msg->data[i] = angle_data.encoder_values[i];
                        }
                        
                        // 打印电机角度数据
                        std::stringstream angle_stream;
                        angle_stream << "电机角度数据(弧度): ";
                        for (size_t i = 0; i < MotorAngleData::MOTOR_COUNT; ++i) {
                            if (updated_motors[i]) {
                                angle_stream << "M" << i << "=" << angle_data.angle_values[i] << " ";
                            }
                        }
                        RCLCPP_INFO(this->get_logger(), "%s", angle_stream.str().c_str());
                        
                        // 发布消息
                        motor_angle_publisher_->publish(*angle_msg);
                        motor_encoder_publisher_->publish(*encoder_msg);
                        
                        // 计算一下帧的利用率
                        double frame_efficiency = (actual_motor_count * sizeof(int32_t)) * 100.0 / data_frame.data.size();
                        RCLCPP_DEBUG(this->get_logger(), "帧利用率: %.1f%% (%zu/%zu 字节)", 
                            frame_efficiency, actual_motor_count * sizeof(int32_t), data_frame.data.size());
                    }
                }
            } catch (const ProtocolException& e) {
                RCLCPP_ERROR(this->get_logger(), "协议解析错误: %s", e.what());
            }
        } catch (const SerialException& e) {
            RCLCPP_ERROR(this->get_logger(), "串口读取错误: %s", e.what());
        }
    }

    std::unique_ptr<SerialInterface> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_encoder_publisher_;
    
    int read_timeout_ms_;
    int motor_angle_data_id_;
    double encoder_to_radians_factor_;
    int motor_count_per_frame_;
    bool debug_output_;
};

} // namespace ares_comm

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ares_comm::MotorAngleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 