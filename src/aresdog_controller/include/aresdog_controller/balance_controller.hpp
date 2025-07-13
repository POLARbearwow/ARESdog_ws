/**
 * @file balance_controller.hpp
 * @brief Declaration of the BalanceController class used for torso attitude self-balancing.
 */

#ifndef ARESDOG_CONTROLLER__BALANCE_CONTROLLER_HPP_
#define ARESDOG_CONTROLLER__BALANCE_CONTROLLER_HPP_

#include <geometry_msgs/msg/quaternion.hpp>

namespace aresdog_controller
{

/// Holds the Z-axis adjustment (in metres) required for each leg.
struct LegZAdjustments
{
  float fl{0.0f}; ///< Front-Left leg Z offset
  float fr{0.0f}; ///< Front-Right leg Z offset
  float rl{0.0f}; ///< Rear-Left leg Z offset
  float rr{0.0f}; ///< Rear-Right leg Z offset
};

/**
 * @class BalanceController
 * @brief Simple P controller that converts roll / pitch error into leg height offsets.
 *
 * The controller maintains the robot torso at a level attitude (roll = pitch = 0).
 * Given the current IMU quaternion, the class calculates the necessary vertical
 * displacement for each foot so that the generated inverse-kinematics targets push
 * the body back to the desired attitude.
 */
class BalanceController
{
public:
  /**
   * @brief Construct a BalanceController with the given proportional gains.
   * @param kp_roll  P-gain [m/rad] for roll error.
   * @param kp_pitch P-gain [m/rad] for pitch error.
   */
  BalanceController(float kp_roll, float kp_pitch);

  /**
   * @brief Compute vertical adjustments for all four legs.
   *
   * @param current_orientation IMU quaternion (geometry_msgs::msg::Quaternion).
   * @return LegZAdjustments Structure containing Δz for FL/FR/RL/RR (metres).
   */
  LegZAdjustments compute_leg_adjustments(const geometry_msgs::msg::Quaternion & current_orientation);

private:
  float kp_roll_{};  ///< Proportional gain for roll
  float kp_pitch_{}; ///< Proportional gain for pitch
};

} // namespace aresdog_controller

#endif // ARESDOG_CONTROLLER__BALANCE_CONTROLLER_HPP_ 