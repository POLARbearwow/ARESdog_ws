#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ares_usb_comm/msg/motor_cmd.hpp"
#include "ares_usb_comm/msg/motor_state.hpp"

#include "ares_protocol.hpp"  // 来自 ares_comm/ARES_bulk_library
#include "ares_usb_comm/helpers.hpp"

#include <array>
#include <mutex>
#include <chrono>

using namespace std::chrono_literals;

namespace ares_usb_comm
{
/**
 * @class UsbBridgeNode
 * @brief ROS2 节点：将 ROS 话题与 USB 协议相互桥接。
 */
class UsbBridgeNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，完成协议初始化与 ROS 接口创建。
     */
    UsbBridgeNode() : Node("usb_bridge_node")
    {
        RCLCPP_INFO(this->get_logger(), "UsbBridgeNode starting...");

        // 创建 ROS 接口 --------------------------------------------------
        motor_state_pub_ = this->create_publisher<ares_usb_comm::msg::MotorState>(
            "motor_state", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", 50);
        motor_cmd_sub_ = this->create_subscription<ares_usb_comm::msg::MotorCmd>(
            "motor_cmd", 10,
            std::bind(&UsbBridgeNode::motor_cmd_callback, this, std::placeholders::_1));

        // 尝试连接 USB 设备 ----------------------------------------------
        if (!protocol_.connect())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect USB device, node will keep running but inactive.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "USB device connected.");
        }

        // 注册协议回调 ----------------------------------------------------
        protocol_.register_sync_callback(
            [this](uint16_t data_id, const uint8_t *data, size_t len)
            { handle_sync_frame(data_id, data, len); });

        // 关闭安全：在析构函数中处理 USB 断开。
    }

    /**
     * @brief 析构函数，确保 USB 设备断开。
     */
    ~UsbBridgeNode() override
    {
        RCLCPP_INFO(this->get_logger(), "UsbBridgeNode destructing, disconnecting USB device...");
        protocol_.disconnect();
    }

private:
    // ---------------- USB 回调处理 ---------------------------------------
    /**
     * @brief 处理下位机同步帧，分发至对应话题。
     */
    void handle_sync_frame(uint16_t data_id, const uint8_t *data, size_t len)
    {
        switch (data_id)
        {
        case DATAID_MOTORSTATE_ANGLE:
            if (len >= 36)
            {
                std::array<float, 9> angles;
                unpack_float_array_be<9>(data, angles);
                {
                    std::lock_guard<std::mutex> lock(motor_state_mutex_);
                    latest_angles_ = angles;
                    angles_received_ = true;
                }
                publish_motor_state_if_ready();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "MotorState-A frame length mismatch: %zu", len);
            }
            break;
        case DATAID_MOTORSTATE_SPEED:
            if (len >= 36)
            {
                std::array<float, 9> speeds;
                unpack_float_array_be<9>(data, speeds);
                {
                    std::lock_guard<std::mutex> lock(motor_state_mutex_);
                    latest_speeds_ = speeds;
                    speeds_received_ = true;
                }
                publish_motor_state_if_ready();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "MotorState-S frame length mismatch: %zu", len);
            }
            break;
        case DATAID_IMU6:
            if (len >= 24)
            {
                std::array<float, 6> imu_raw;
                unpack_float_array_be<6>(data, imu_raw);
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header.stamp = this->now();
                imu_msg.linear_acceleration.x = imu_raw[0];
                imu_msg.linear_acceleration.y = imu_raw[1];
                imu_msg.linear_acceleration.z = imu_raw[2];
                imu_msg.angular_velocity.x = imu_raw[3];
                imu_msg.angular_velocity.y = imu_raw[4];
                imu_msg.angular_velocity.z = imu_raw[5];
                // orientation 留空
                imu_pub_->publish(imu_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "IMU frame length mismatch: %zu", len);
            }
            break;
        default:
            // 其他 DataID 可在此扩展
            break;
        }
    }

    /**
     * @brief 当 angle & speed 都收到时，合并并发布 MotorState。
     */
    void publish_motor_state_if_ready()
    {
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        if (!(angles_received_ && speeds_received_))
            return;

        auto msg = ares_usb_comm::msg::MotorState();
        msg.angle = latest_angles_;
        msg.speed = latest_speeds_;
        motor_state_pub_->publish(msg);
        angles_received_ = speeds_received_ = false; // 重置标志
    }

    // ---------------- 发送 MotorCmd --------------------------------------
    /**
      * @brief 订阅回调：将 MotorCmd 转为单帧并发送。
      */
    void motor_cmd_callback(const ares_usb_comm::msg::MotorCmd::SharedPtr msg)
    {
        uint8_t payload[36];
        pack_float_array_be<9>(msg->angle, payload);
        if (!protocol_.send_sync(DATAID_MOTORCMD_ANGLE, payload, sizeof(payload)))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send MotorCmd angle frame.");
        }
    }

    // ---------------- 成员变量 ------------------------------------------
    ares::Protocol protocol_{}; ///< 底层 USB 协议封装

    // ROS 通信
    rclcpp::Subscription<ares_usb_comm::msg::MotorCmd>::SharedPtr motor_cmd_sub_;
    rclcpp::Publisher<ares_usb_comm::msg::MotorState>::SharedPtr motor_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // MotorState 缓冲
    std::array<float, 9> latest_angles_{};
    std::array<float, 9> latest_speeds_{};
    bool angles_received_{false};
    bool speeds_received_{false};
    std::mutex motor_state_mutex_;
};

} // namespace ares_usb_comm

// ------------------------------ main ---------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<ares_usb_comm::UsbBridgeNode>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
} 