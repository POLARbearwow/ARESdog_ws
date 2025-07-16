/**********************************************************************
 * @file aresdog_balance_walk_node.cpp
 * @brief ROS 2 node that fuses cycloid gait generation with BalanceController to
 *        keep the robot balanced while walking.
 *
 * This implementation introduces a very lightweight open-loop gait generator
 * (trot) based on cycloidal trajectories.  Each leg follows the same stride
 * length and height but with a 180-deg phase offset between diagonal pairs
 * (FL+RR vs FR+RL) so that at any time exactly two legs are in swing while the
 * other two support the body.
 *
 * Key features
 *   • Adjustable stride length, step height, cycle time and duty factor via
 *     ROS parameters.
 *   • Uses BalanceController to convert roll / pitch error from the IMU into
 *     per-leg vertical offsets (Δz) which are added on-the-fly to the gait
 *     trajectory.
 *   • Runs at a fixed control rate (default 100 Hz) and publishes the final
 *     joint targets to the `/action` topic as `sensor_msgs/msg/JointState`.
 *
 * NOTE: This node *adds* functionality – the existing `aresdog_controller_node`
 * remains untouched.
 **********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "aresdog_controller/balance_controller.hpp"
#include "aresdog_controller/leg_kinematics.hpp"

#include <array>
#include <chrono>
#include <cmath>

namespace aresdog_controller
{

/**
 * @enum LegIndex
 * @brief Convenient enumeration for leg array indexing.
 */
enum LegIndex : std::size_t { FL = 0, FR = 1, RL = 2, RR = 3 };

/**
 * @class BalanceWalkNode
 * @brief Main ROS 2 node that generates walking gait and keeps balance.
 */
class BalanceWalkNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node, declare parameters, and set up ROS interfaces.
   */
  BalanceWalkNode()
  : Node("aresdog_balance_walk_node")
  {
    //--------------------------------------------------------------
    // Parameter declaration (with sensible defaults)
    //--------------------------------------------------------------
    kp_roll_      = this->declare_parameter("kp_roll",  0.0);    // [m/rad] - Smaller gain for smoother correction
    kp_pitch_     = this->declare_parameter("kp_pitch", 0.0);    // [m/rad] - Smaller gain for smoother correction

    nominal_z_    = this->declare_parameter("nominal_z",   -0.215);// [m] Height during stance phase
    swing_base_z_ = this->declare_parameter("swing_base_z", -0.21); // [m] Base height for swing trajectory
    nominal_x_    = this->declare_parameter("nominal_x",    0.00);// [m]

    stride_length_= this->declare_parameter("stride_length", 0.12);// [m]
    step_height_  = this->declare_parameter("step_height",  0.025);// [m]
    cycle_time_   = this->declare_parameter("cycle_time",   0.2); // [s]
    duty_factor_  = this->declare_parameter("duty_factor",  0.5); // swing ratio (0-1)

    control_rate_ = this->declare_parameter("control_rate", 100.0); // [Hz]

    //--------------------------------------------------------------
    // Core components
    
    //--------------------------------------------------------------
    balance_controller_ = std::make_unique<BalanceController>(static_cast<float>(kp_roll_), static_cast<float>(kp_pitch_));
    leg_kinematics_     = std::make_unique<LegKinematics>(0.224f, 0.090f);

    //--------------------------------------------------------------
    // Publishers / Subscribers
    //--------------------------------------------------------------
    action_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/action", 10);

    orient_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/orient", 20, std::bind(&BalanceWalkNode::orient_callback, this, std::placeholders::_1));

    //--------------------------------------------------------------
    // Joint name list MUST match usb_bridge_node order
    //--------------------------------------------------------------
    joint_names_ = {
      "FL_thigh_joint_i", "FL_thigh_joint_o", "FR_thigh_joint_i", "FR_thigh_joint_o",
      "waist_joint", /* not actuated but preserved for compatibility */
      "RL_thigh_joint_i", "RL_thigh_joint_o", "RR_thigh_joint_i", "RR_thigh_joint_o"};

    //--------------------------------------------------------------
    // Phase offsets for trot gait (diagonal legs in phase)
    //--------------------------------------------------------------
    phase_offset_ = {0.0, 0.5, 0.5, 0.0}; // FL & RR lead   |   FR & RL lag by 180°

    //--------------------------------------------------------------
    // Start control loop timer
    //--------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = this->create_wall_timer(period, std::bind(&BalanceWalkNode::control_loop, this));

    start_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "aresdog_balance_walk_node started – cycle_time=%.2f s stride=%.2f m", cycle_time_, stride_length_);
  }

private:
  //--------------------------------------------------------------
  /**
   * @brief Callback – stores the latest IMU quaternion.
   * @param msg Shared pointer to Quaternion message.
   */
  void orient_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    latest_orientation_ = *msg; // store for use in control loop
  }

  //--------------------------------------------------------------
  /**
   * @brief Main control loop executed at fixed rate.
   */
  void control_loop()
  {
    const rclcpp::Time now = this->get_clock()->now();
    const double t = (now - start_time_).seconds();

    //----------------------------------------------------------
    // 1. Compute gait phase for each leg (0-1)
    //----------------------------------------------------------
    std::array<double, 4> phase{{0,0,0,0}};
    for (std::size_t i = 0; i < phase.size(); ++i) {
      double raw = std::fmod(t + phase_offset_[i] * cycle_time_, cycle_time_);
      phase[i] = raw / cycle_time_;
    }

    //----------------------------------------------------------
    // 2. Balance corrections (Δz per leg)
    //----------------------------------------------------------
    LegZAdjustments dz = balance_controller_->compute_leg_adjustments(latest_orientation_);

    //----------------------------------------------------------
    // 3. Target foot positions & IK
    //----------------------------------------------------------
    // Arrays to hold joint commands in hardware order
    std::array<double, 9> cmd{};
    bool ik_ok = true;

    // Define kinematic limits based on L3=0.224, L4=0.090
    const float min_reach_sq = 0.134f * 0.134f;
    const float max_reach_sq = 0.314f * 0.314f;

    for (std::size_t leg = 0; leg < 4; ++leg) {
      // Calculate desired (x, z) in body frame for this leg
      Point2D p = foot_target(static_cast<LegIndex>(leg), phase[leg]);
      p.z += leg_dz_offset(leg, dz); // Re-enable balance correction

      // --- SAFETY CLAMP: Ensure target is within reachable workspace ---
      float dist_sq = p.x * p.x + p.z * p.z;

      if (dist_sq < min_reach_sq) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Target (%.2f, %.2f) is too close. Clamping to min reach.", p.x, p.z);
        float dist = std::sqrt(dist_sq);
        if (dist > 1e-6f) { // Avoid division by zero
          float scale = std::sqrt(min_reach_sq) / dist;
          p.x *= scale;
          p.z *= scale;
        } else { // At origin, push down vertically to a safe default
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
      // --- END SAFETY CLAMP ---


      // Solve IK
      float theta1{}, theta4{};
      if (!leg_kinematics_->inverseKinematics(p.x, p.z, theta1, theta4)) {
        ik_ok = false;
        break;
      }

      // Map to array indices as per hardware mapping table
      switch (leg) {
        case FL:
          cmd[0] = -static_cast<double>(theta1);
          cmd[1] =  static_cast<double>(theta4 + M_PI);
          break;
        case FR:
          cmd[2] =  static_cast<double>(theta1);
          cmd[3] = -static_cast<double>(theta4 + M_PI);
          break;
        case RL:
          cmd[5] = -static_cast<double>(theta4 + M_PI);
          cmd[6] =  static_cast<double>(theta1);
          break;
        case RR:
          cmd[7] =  static_cast<double>(theta4 + M_PI);
          cmd[8] = -static_cast<double>(theta1);
          break;
      }
    }

    // Waist joint remains 0
    cmd[4] = 0.0;

    if (!ik_ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "IK failed – skipping cycle");
      return;
    }

    publish_joint_state(cmd);
  }

  //--------------------------------------------------------------
  /**
   * @brief Publish a JointState message containing commanded positions.
   * @param positions Joint positions in hardware order (size = 9).
   */
  void publish_joint_state(const std::array<double, 9>& positions)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.assign(positions.begin(), positions.end());
    action_pub_->publish(msg);
  }

  //--------------------------------------------------------------
  /**
   * @brief Compute nominal foot target for a given leg & phase (without Δz).
   *
   * The leg follows a cycloidal trajectory in the X-direction.  During the
   * swing phase (phase < duty_factor_) the foot moves forward while lifting to
   * `step_height_`.  During stance (phase ≥ duty_factor_) the foot moves
   * backward on the ground (z ≈ nominal_z_).
   *
   * @warning If `nominal_z_` and `swing_base_z_` are different, a discontinuity
   *          will occur at the stance/swing transitions. For smooth motion,
   *          they should typically be set to the same value.
   *
   * @param leg   Leg index.
   * @param phase Normalised phase ∈ [0, 1].
   * @return 2-D point (x, z) in body frame.
   */
  Point2D foot_target(LegIndex leg, double phase) const
  {
    (void)leg; // Currently gait is symmetric, leg id not used for trajectory shape

    const double half_stride = stride_length_ / 2.0;
    Point2D p;

    if (phase < duty_factor_) {
      // Swing phase — use cycloid
      const double s   = phase / duty_factor_;           // 0->1
      const double t   = 2.0 * M_PI * s;
      p.x = -half_stride + stride_length_ * (s - std::sin(t) / (2.0 * M_PI));
      p.z = swing_base_z_  + static_cast<double>(step_height_) * (1.0 - std::cos(t)) / 2.0;
    } else {
      // Stance phase — use cycloid for smooth backward motion, stay on ground
      const double s = (phase - duty_factor_) / (1.0 - duty_factor_); // 0->1
      const double t   = 2.0 * M_PI * s;
      p.x =  half_stride - stride_length_ * (s - std::sin(t) / (2.0 * M_PI));
      p.z = nominal_z_ -  0.1*static_cast<double>(step_height_) * (1.0 - std::cos(t)) / 2.0;
    }

    return p;
  }

  //--------------------------------------------------------------
  /**
   * @brief Helper: select Δz for the given leg from BalanceController output.
   */
  static float leg_dz_offset(std::size_t leg, const LegZAdjustments& dz)
  {
    switch (leg) {
      case FL: return dz.fl;
      case FR: return dz.fr;
      case RL: return dz.rl;
      case RR: return dz.rr;
      default: return 0.0f;
    }
  }

  //--------------------------------------------------------------
  // Parameters / gait settings
  double kp_roll_{};
  double kp_pitch_{};
  double nominal_z_{};
  double swing_base_z_{};
  double nominal_x_{}; // Reserved for future lateral sway support
  double stride_length_{};
  double step_height_{};
  double cycle_time_{};
  double duty_factor_{};
  double control_rate_{};

  //--------------------------------------------------------------
  // Core components
  std::unique_ptr<BalanceController> balance_controller_;
  std::unique_ptr<LegKinematics>     leg_kinematics_;

  //--------------------------------------------------------------
  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orient_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr      action_pub_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  //--------------------------------------------------------------
  // Helpers / state
  geometry_msgs::msg::Quaternion latest_orientation_{}; // Updated by orient_callback()
  rclcpp::Time                   start_time_;
  std::array<double, 4>          phase_offset_{};       // [0-1] fractions of cycle_time_
  std::vector<std::string>       joint_names_;
};

} // namespace aresdog_controller

//--------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aresdog_controller::BalanceWalkNode>());
  rclcpp::shutdown();
  return 0;
} 