/**
 * @file test_motor_cmd_node.cpp
 * @brief Simple ROS 2 node to publish desired joint angles for all motors.
 *
 * This node is intended for bench testing. It cycles through a pre-defined
 * sequence of joint angle commands and repeatedly publishes a
 * `sensor_msgs::msg::JointState` message on the `/action` topic.
 *
 * The command sequence is defined directly in the source code.
 *
 * Parameters
 *  ──────────────────────────────────────────────────────────────
 *  • pub_rate        (double)     ‑ Publish frequency in Hz. Default: 20.
 *  • pose_hold_time  (double)     ‑ Time to hold each pose in seconds. Default: 4.
 *
 * Usage example:
 *  ros2 run aresdog_controller test_motor_cmd_node --ros-args -p pose_hold_time:=2.5
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <vector>
#include <string>
#include <array>
#include <chrono>

namespace aresdog_controller
{

/**
 * @class TestMotorCmdNode
 * @brief Publishes a cycling sequence of fixed joint angles for motor tests.
 */
class TestMotorCmdNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node, define the command sequence, and start the timer.
   */
  TestMotorCmdNode()
  : Node("test_motor_cmd_node")
  {
    // Declare parameters
    pub_rate_ = this->declare_parameter("pub_rate", 20.0); // Hz
    pose_hold_time_ = this->declare_parameter("pose_hold_time", 4.0); // seconds

    // ======================================================================
    // ===                 INPUT YOUR COMMANDS HERE                       ===
    // ======================================================================
    // Define the sequence of joint angle commands (in radians).
    // The node will cycle through this vector of arrays.
    cmd_sequence_ = {
      // Pose 1: Standstill (all joints at zero)
      {1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
      // Pose 2: Front-Left leg raised slightly
      // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      // Pose 3: Front-Right leg raised slightly
      // {0.0, 0.0, 0.4, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0},
      // // Pose 4: Back to standstill before looping
      // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };
    // ======================================================================

    // Prepare JointState message template
    msg_.name = {
      "FL_thigh_joint_i", "FL_thigh_joint_o", "FR_thigh_joint_i", "FR_thigh_joint_o",
      "waist_joint", "RL_thigh_joint_i", "RL_thigh_joint_o", "RR_thigh_joint_i", "RR_thigh_joint_o"};

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/action", 10);

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / pub_rate_);
    timer_ = this->create_wall_timer(period, std::bind(&TestMotorCmdNode::on_timer, this));

    // Initial state
    last_switch_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(),
      "test_motor_cmd_node started. Cycling through %zu poses, holding each for %.1f s.",
      cmd_sequence_.size(), pose_hold_time_);
  }

private:
  /**
   * @brief Timer callback – cycles poses and publishes the current command.
   */
  void on_timer()
  {
    auto now = this->get_clock()->now();

    // Check if it's time to switch to the next command in the sequence
    if ((now - last_switch_time_).seconds() >= pose_hold_time_) {
      seq_idx_ = (seq_idx_ + 1) % cmd_sequence_.size();
      last_switch_time_ = now;
      RCLCPP_INFO(this->get_logger(), "Switching to pose #%zu", seq_idx_ + 1);
    }

    // Populate and publish the message with the current command
    msg_.header.stamp = now;
    const auto& current_cmd = cmd_sequence_[seq_idx_];
    msg_.position.assign(current_cmd.begin(), current_cmd.end());
    pub_->publish(msg_);
  }

  //------------------------------------------------------------------
  // Parameters / state
  double pub_rate_{};
  double pose_hold_time_{};

  std::vector<std::array<double, 9>> cmd_sequence_{};
  size_t seq_idx_{0};
  rclcpp::Time last_switch_time_;

  sensor_msgs::msg::JointState msg_;

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                               timer_;
};

} // namespace aresdog_controller

//--------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aresdog_controller::TestMotorCmdNode>());
  rclcpp::shutdown();
  return 0;
} 