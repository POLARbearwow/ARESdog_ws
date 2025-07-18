/**********************************************************************
 * @file aresdog_controller_node.cpp
 * @brief ROS 2 node that fuses BalanceController + LegKinematics and publishes JointState commands.
 **********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "aresdog_controller/balance_controller.hpp"
#include "aresdog_controller/leg_kinematics.hpp"

namespace aresdog_controller
{

class AresDogControllerNode : public rclcpp::Node
{
public:
  AresDogControllerNode()
  : Node("aresdog_controller_node")
  {
    // Declare parameters with defaults
    kp_roll_  = this->declare_parameter("kp_roll", 0.08);   // m / rad
    kp_pitch_ = this->declare_parameter("kp_pitch", 0.15);  // m / rad
    nominal_z_ = this->declare_parameter("nominal_z", -0.20);
    nominal_x_ = this->declare_parameter("nominal_x",  0.00);

    balance_controller_ = std::make_unique<BalanceController>(static_cast<float>(kp_roll_), static_cast<float>(kp_pitch_));
    leg_kinematics_     = std::make_unique<LegKinematics>(0.224f, 0.090f);

    RCLCPP_INFO(this->get_logger(), "AresDog Static Balance Controller started.");
    RCLCPP_INFO(this->get_logger(), "Params: kp_roll=%.3f, kp_pitch=%.3f, nominal_z=%.2f",
      kp_roll_, kp_pitch_, nominal_z_);

    // Publisher – JointState to /action
    action_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/action", 10);

    // Prepare fixed joint name list (must match usb_bridge_node expectation)
    joint_names_ = {
      "FL_thigh_joint_i", "FL_thigh_joint_o", "FR_thigh_joint_i", "FR_thigh_joint_o",
      "waist_joint",
      "RL_thigh_joint_i", "RL_thigh_joint_o", "RR_thigh_joint_i", "RR_thigh_joint_o"};

    // Subscriber – IMU orientation
    orient_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/orient", 10, std::bind(&AresDogControllerNode::orient_callback, this, std::placeholders::_1));

    // Initialise last valid angles to zero
    last_cmd_.fill(0.0);
  }

private:
  //------------------------------------------------------------------
  void orient_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    // 1. Get Δz from balance controller
    LegZAdjustments dz = balance_controller_->compute_leg_adjustments(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Balance dz: fl=%.3f, fr=%.3f, rl=%.3f, rr=%.3f",
      dz.fl, dz.fr, dz.rl, dz.rr);

    // 2. Compute target points for each leg
    std::array<Point2D, 4> targets;
    targets[0] = {static_cast<float>(nominal_x_), static_cast<float>(nominal_z_ + dz.fl)}; // FL
    targets[1] = {static_cast<float>(nominal_x_), static_cast<float>(nominal_z_ + dz.fr)}; // FR
    targets[2] = {static_cast<float>(nominal_x_), static_cast<float>(nominal_z_ + dz.rl)}; // RL
    targets[3] = {static_cast<float>(nominal_x_), static_cast<float>(nominal_z_ + dz.rr)}; // RR

    // --- SAFETY CLAMP: Ensure all targets are within reachable workspace ---
    const float min_reach_sq = 0.134f * 0.134f;
    const float max_reach_sq = 0.314f * 0.314f;

    for (auto& p : targets) {
      float dist_sq = p.x * p.x + p.z * p.z;
      if (dist_sq < min_reach_sq) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Target (%.2f, %.2f) is too close. Clamping to min reach.", p.x, p.z);
        float dist = std::sqrt(dist_sq);
        if (dist > 1e-6f) {
          float scale = std::sqrt(min_reach_sq) / dist;
          p.x *= scale;
          p.z *= scale;
        } else {
          p.x = 0.0f;
          p.z = -std::sqrt(min_reach_sq);
        }
      } else if (dist_sq > max_reach_sq) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Target (%.2f, %.2f) is too far. Clamping to max reach.", p.x, p.z);
        float dist = std::sqrt(dist_sq);
        float scale = std::sqrt(max_reach_sq) / dist;
        p.x *= scale;
        p.z *= scale;
      }
    }
    // --- END SAFETY CLAMP ---

    // 3. Compute IK for each leg (x is constant, z varies)
    float theta1_fl, theta4_fl;
    float theta1_fr, theta4_fr;
    float theta1_rl, theta4_rl;
    float theta1_rr, theta4_rr;

    bool ok = true;
    ok &= leg_kinematics_->inverseKinematics(targets[0].x, targets[0].z, theta1_fl, theta4_fl);
    ok &= leg_kinematics_->inverseKinematics(targets[1].x, targets[1].z, theta1_fr, theta4_fr);
    ok &= leg_kinematics_->inverseKinematics(targets[2].x, targets[2].z, theta1_rl, theta4_rl);
    ok &= leg_kinematics_->inverseKinematics(targets[3].x, targets[3].z, theta1_rr, theta4_rr);

    if (!ok)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "IK failed – reusing last command.");
      publish_joint_state(last_cmd_);
      return;
    }

    // 4. Map θ → motor command per latest hardware table
    std::array<double, 9> cmd{};

    cmd[0] = -static_cast<double>(theta1_fl);                   // FL_i  (index 0)
    cmd[1] =  static_cast<double>(theta4_fl + M_PI);            // FL_o  (index 1)
    cmd[2] =  static_cast<double>(theta1_fr);                   // FR_i  (index 2)
    cmd[3] = -static_cast<double>(theta4_fr + M_PI);            // FR_o  (index 3)
    cmd[4] = 0.0;                                              // waist (index 4)
    cmd[5] = -static_cast<double>(theta4_rl + M_PI);            // RL_i  (index 5)
    cmd[6] =  static_cast<double>(theta1_rl);                   // RL_o  (index 6)
    cmd[7] =  static_cast<double>(theta4_rr + M_PI);            // RR_i  (index 7)
    cmd[8] = -static_cast<double>(theta1_rr);                   // RR_o  (index 8)

    last_cmd_ = cmd;
    publish_joint_state(cmd);
  }

  //------------------------------------------------------------------
  void publish_joint_state(const std::array<double, 9> & positions)
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.assign(positions.begin(), positions.end());

    action_pub_->publish(msg);
  }

  //------------------------------------------------------------------
  // Parameters
  double kp_roll_;
  double kp_pitch_;
  double nominal_z_;
  double nominal_x_;

  // Core components
  std::unique_ptr<BalanceController> balance_controller_;
  std::unique_ptr<LegKinematics>     leg_kinematics_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orient_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr      action_pub_;

  // Data helpers
  std::vector<std::string> joint_names_;
  std::array<double, 9>    last_cmd_{};
};

} // namespace aresdog_controller

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aresdog_controller::AresDogControllerNode>());
  rclcpp::shutdown();
  return 0;
} 