#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ares_protocol.hpp" // 来自 ares_comm/ARES_bulk_library
#include "ares_usb_comm/helpers.hpp"

#include <array>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <vector>

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

        // 将日志记录器级别设置为 DEBUG，以便所有调试信息都可见
        // 注意：在生产环境中，建议将其改回 INFO 并使用 ros args 进行调试
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);

        // 创建 ROS 接口 --------------------------------------------------
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_state", 10);
        orient_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
            "orient", 50);
        angvel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "angvel", 50);
        joint_state_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "action", 10,
            std::bind(&UsbBridgeNode::joint_state_cmd_callback, this, std::placeholders::_1));
        // 新增：订阅前馈力矩指令
        torque_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "torque_cmd", 10,
            std::bind(&UsbBridgeNode::torque_cmd_callback, this, std::placeholders::_1));

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
    /**
     * @brief (新增) 将浮点数组打包为小端序字节流。
     * @tparam N 数组元素个数
     * @param src 源浮点数组
     * @param dst 目标字节缓冲区
     */
    template <size_t N>
    static void pack_float_array_le(const std::array<float, N> &src, uint8_t *dst)
    {
        for (size_t i = 0; i < N; ++i)
        {
            const uint8_t *float_bytes = reinterpret_cast<const uint8_t *>(&src[i]);
            dst[i * 4 + 0] = float_bytes[0];
            dst[i * 4 + 1] = float_bytes[1];
            dst[i * 4 + 2] = float_bytes[2];
            dst[i * 4 + 3] = float_bytes[3];
        }
    }

    /**
     * @brief (新增) 小端序字节流解包为浮点数组。
     * @tparam N 数组元素个数
     * @param src 源字节缓冲区
     * @param dst 目标浮点数组
     */
    template <size_t N>
    static void unpack_float_array_le(const uint8_t *src, std::array<float, N> &dst)
    {
        for (size_t i = 0; i < N; ++i)
        {
            uint32_t temp = (uint32_t)src[i * 4 + 3] << 24 |
                            (uint32_t)src[i * 4 + 2] << 16 |
                            (uint32_t)src[i * 4 + 1] << 8 |
                            (uint32_t)src[i * 4 + 0];
            dst[i] = *reinterpret_cast<float *>(&temp);
        }
    }

    // ---------------- USB 回调处理 ---------------------------------------
    /**
     * @brief 处理下位机同步帧，分发至对应话题。
     */
    void handle_sync_frame(uint16_t data_id, const uint8_t *data, size_t len)
    {
        // 关键修复：由于协议库按小端序解析了 DataID，而下位机发送的是大端序，
        // 在此处进行字节序反转以修正。例如，大端序 0x0201 会被误读为 0x0102，
        // 反转后恢复为 0x0201。
        // data_id = (data_id >> 8) | (data_id << 8);

        switch (data_id)
        {
        case DATAID_MOTORSTATE_ANGLE:
            if (len >= 36)
            {
                std::array<float, 9> angles;
                unpack_float_array_le<9>(data, angles); // 使用小端序解包
                {
                    // 打印接收到的角度数据
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2);
                    for (size_t i = 0; i < angles.size(); ++i)
                    {
                        ss << angles[i];
                        if (i < angles.size() - 1)
                            ss << ", ";
                    }
                    // RCLCPP_DEBUG(this->get_logger(), "MotorState-Angles: [%s]", ss.str().c_str());

                    std::lock_guard<std::mutex> lock(motor_state_mutex_);
                    latest_angles_ = angles;
                    angles_received_ = true;
                }
                publish_joint_state_if_ready();
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
                unpack_float_array_le<9>(data, speeds); // 使用小端序解包
                {
                    // 打印接收到的速度数据
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2);
                    for (size_t i = 0; i < speeds.size(); ++i)
                    {
                        ss << speeds[i];
                        if (i < speeds.size() - 1)
                            ss << ", ";
                    }
                    // RCLCPP_DEBUG(this->get_logger(), "MotorState-Speeds: [%s]", ss.str().c_str());

                    std::lock_guard<std::mutex> lock(motor_state_mutex_);
                    latest_speeds_ = speeds;
                    speeds_received_ = true;
                }
                publish_joint_state_if_ready();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "MotorState-S frame length mismatch: %zu", len);
            }
            break;
        case DATAID_MOTORSTATE_TORQUE: // 新增：实际力矩反馈
            if (len >= 36)
            {
                std::array<float, 9> torques;
                unpack_float_array_le<9>(data, torques);
                {
                    std::lock_guard<std::mutex> lock(motor_state_mutex_);
                    latest_torques_ = torques;
                    torques_received_ = true;
                }
                publish_joint_state_if_ready();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "MotorState-T frame length mismatch: %zu", len);
            }
            break;
        case DATAID_IMU6:
            if (len >= 28) // 长度从 24 (6*4) 更新为 28 (7*4)
            {
                std::array<float, 7> imu_data; // 从 6-float 更新为 7-float
                unpack_float_array_le<7>(data, imu_data); // 解包 7 个浮点数

                // 发布方向 (orient), 来自协议帧的前 4 个 float
                auto orient_msg = geometry_msgs::msg::Quaternion();
                orient_msg.x = imu_data[0];
                orient_msg.y = imu_data[1];
                orient_msg.z = imu_data[2];
                orient_msg.w = imu_data[3];
                orient_pub_->publish(orient_msg);

                // 发布角速度 (angvel), 来自协议帧的后 3 个 float
                auto angvel_msg = geometry_msgs::msg::Vector3();
                angvel_msg.x = imu_data[4];
                angvel_msg.y = imu_data[5];
                angvel_msg.z = imu_data[6];
                angvel_pub_->publish(angvel_msg);

                {
                    // 打印接收到的 IMU 原始数据
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(3);
                    ss << "Quat(x,y,z,w): " << imu_data[0] << ", " << imu_data[1] << ", " << imu_data[2] << ", " << imu_data[3]
                       << ", Gyro(x,y,z): " << imu_data[4] << ", " << imu_data[5] << ", " << imu_data[6];
                    // RCLCPP_DEBUG(this->get_logger(), "IMU Data: %s", ss.str().c_str());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "IMU frame length mismatch: %zu, expected 28", len);
            }
            break;
        default:
            // 如果收到未知的 DataID，打印警告，帮助调试
            RCLCPP_WARN(this->get_logger(), "Received unknown DataID: 0x%04X", data_id);
            break;
        }
    }

    /**
     * @brief 当 angle & speed 都收到时，合并并发布 JointState。
     */
    void publish_joint_state_if_ready()
    {
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        if (!(angles_received_ && speeds_received_))
            return; // 等待力矩到齐再发布

        if (!torques_received_)
            return;

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = joint_names_; // 填充关节名称，符合 JointState 标准用法
        msg.position.resize(9);
        msg.velocity.resize(9);
        msg.effort.resize(9);

        // 从 C++11 的 std::array 转换到 std::vector
        for (size_t i = 0; i < 9; ++i)
        {
            msg.position[i] = latest_angles_[i];
            msg.velocity[i] = latest_speeds_[i];
            msg.effort[i]   = latest_torques_[i];
        }

        RCLCPP_DEBUG(this->get_logger(), "Publish JointState (with torque)");
        joint_state_pub_->publish(msg);
        angles_received_ = speeds_received_ = torques_received_ = false; // 重置标志
    }

    // ---------------- 发送电机指令 --------------------------------------
    /**
      * @brief 订阅回调：将 JointState 命令中的 position 转为协议帧并发送。
      * @detail 此回调函数是鲁棒的，它会根据消息中的 `name` 字段来匹配正确的关节索引，
      *         而不是盲目依赖 `position` 数组的顺序。
      */
    void joint_state_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() != msg->position.size())
        {
            RCLCPP_WARN(this->get_logger(), "Received 'action' with mismatched name (%zu) and position (%zu) sizes. Ignoring.",
                        msg->name.size(), msg->position.size());
            return;
        }

        std::array<float, 9> angles;
        // 使用 std::vector<bool> 来追踪每个关节是否已接收到指令
        std::vector<bool> received(9, false);

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            // 查找接收到的关节名在我们预定义列表中的位置
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
            if (it != joint_names_.end())
            {
                // 如果找到了，计算出它在目标数组中的索引
                size_t index = std::distance(joint_names_.begin(), it);
                angles[index] = static_cast<float>(msg->position[i]);
                received[index] = true;
            }
        }

        // 检查是否所有 9 个关节的指令都已收到
        if (std::find(received.begin(), received.end(), false) != received.end())
        {
            RCLCPP_WARN(this->get_logger(), "Received 'action' message does not contain all 9 required joints. Ignoring.");
            return;
        }


        // 打印即将发送的电机指令角度
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        for (size_t i = 0; i < 9; ++i)
        {
            ss << angles[i];
            if (i < 9 - 1)
                ss << ", ";
        }
        // RCLCPP_DEBUG(this->get_logger(), "Send MotorCmd-Angle from 'action' topic: [%s]", ss.str().c_str());

        uint8_t payload[36];
        pack_float_array_le<9>(angles, payload);

        // 关键: 为确保 DataID 在传输时为大端序 (以匹配下位机)，
        // 在此反转字节序，以抵消协议库可能存在的默认小端序序列化。
        // uint16_t data_id_be = (DATAID_MOTORCMD_ANGLE >> 8) | (DATAID_MOTORCMD_ANGLE << 8);
        // if (!protocol_.send_sync(data_id_be, payload, sizeof(payload)))

  
        if (!protocol_.send_sync(DATAID_MOTORCMD_ANGLE, payload, sizeof(payload)))
        {   
            RCLCPP_ERROR(this->get_logger(), "Failed to send MotorCmd angle frame.");
        }
    }

    // ---------------- 发送力矩指令 --------------------------------------
    /**
     * @brief 订阅回调：将 JointState.effort 转为 TorqueCmd 帧并发送。
     */
    void torque_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() != msg->effort.size())
        {
            RCLCPP_WARN(this->get_logger(), "Received 'torque_cmd' with mismatched name (%zu) and effort (%zu) sizes. Ignoring.",
                        msg->name.size(), msg->effort.size());
            return;
        }

        std::array<float, 9> torques;
        std::vector<bool> received(9, false);

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
            if (it != joint_names_.end())
            {
                size_t index = std::distance(joint_names_.begin(), it);
                torques[index] = static_cast<float>(msg->effort[i]);
                received[index] = true;
            }
        }

        if (std::find(received.begin(), received.end(), false) != received.end())
        {
            RCLCPP_WARN(this->get_logger(), "'torque_cmd' message does not contain all 9 joints. Ignoring.");
            return;
        }

        uint8_t payload[36];
        pack_float_array_le<9>(torques, payload);
        if (!protocol_.send_sync(DATAID_MOTORCMD_TORQUE, payload, sizeof(payload)))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send TorqueCmd frame.");
        }
    }

    // ---------------- 成员变量 ------------------------------------------
    ares::Protocol protocol_{}; ///< 底层 USB 协议封装

    // ROS 通信
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orient_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angvel_pub_;

    // 电机索引映射 (与 ares_usb_comm_design.md 保持一致)
    const std::vector<std::string> joint_names_{
        "FL_thigh_joint_i",
        "FL_thigh_joint_o",
        "FR_thigh_joint_i",
        "FR_thigh_joint_o",
        "waist_joint",
        "RL_thigh_joint_i",
        "RL_thigh_joint_o",
        "RR_thigh_joint_i",
        "RR_thigh_joint_o"};

    // MotorState 缓冲
    std::array<float, 9> latest_angles_{};
    std::array<float, 9> latest_speeds_{};
    std::array<float, 9> latest_torques_{};      // 新增：实际力矩
    bool angles_received_{false};
    bool speeds_received_{false};
    bool torques_received_{false};
    std::mutex motor_state_mutex_;

    // 新增：TorqueCmd 订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torque_cmd_sub_;
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