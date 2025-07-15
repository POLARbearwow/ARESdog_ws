/**
 * @file balance_controller.cpp
 * @brief Implementation of BalanceController – converts body attitude error to leg Z adjustments.
 */

#include "aresdog_controller/balance_controller.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>

namespace aresdog_controller
{

//--------------------------------------------------------------------
BalanceController::BalanceController(float kp_roll, float kp_pitch)
: kp_roll_(kp_roll), kp_pitch_(kp_pitch)
{
}

//--------------------------------------------------------------------
LegZAdjustments BalanceController::compute_leg_adjustments(const geometry_msgs::msg::Quaternion & q_msg)
{
  // 1. Convert quaternion → roll & pitch (radians)
  static float last_pitch_err = 0.0f;
  static float last_roll_err = 0.0f;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(yaw, pitch, roll); // roll around X, pitch around Y, yaw around Z (ENU convention)

  pitch = -pitch;
  yaw = -yaw;

  printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);
  // 2. Errors (desired = 0)

  // 2. Errors (desired = 0)
  const double roll_err   = -roll;   // target is zero
  const double pitch_err  = -pitch;  // target is zero

  // Not a ROS 2 node – use printf for debug output
  printf("roll_err: %f, pitch_err: %f\n", roll_err, pitch_err);

  // 3. P-controller → correction (in metres)
  const float corr_roll  = static_cast<float>(kp_roll_  * roll_err);
  const float corr_pitch = static_cast<float>(kp_pitch_ * pitch_err);

  printf("corr_roll: %f, corr_pitch: %f\n", corr_roll, corr_pitch);

  LegZAdjustments dz;

  // Mapping: positive corr → leg should extend (increase Z) to push torso back.
  // Pitch component: front legs +, rear legs -
  dz.fl =  corr_pitch;
  dz.fr =  corr_pitch;
  dz.rl = -corr_pitch;
  dz.rr = -corr_pitch;

  // last_pitch_err += 0.03f*corr_pitch;
  // last_pitch_err = std::clamp(last_pitch_err, -0.02f, 0.02f);
  // printf("last_pitch_err: %f\n", last_pitch_err);
  // dz.fl += last_pitch_err;
  // dz.fr += last_pitch_err;
  // dz.rl += -last_pitch_err;
  // dz.rr += -last_pitch_err;
  

  //Roll component: right legs +, left legs -  (roll > 0 → lean right, so right side down)
  dz.fl += -corr_roll;
  dz.rl += -corr_roll;
  dz.fr +=  corr_roll;
  dz.rr +=  corr_roll;

  last_roll_err += 0.03f*corr_roll;
  last_roll_err = std::clamp(last_roll_err, -0.02f, 0.02f);
  printf("last_roll_err: %f\n", last_roll_err);
  dz.fl += -last_roll_err;
  dz.rl += -last_roll_err;
  dz.fr +=  last_roll_err;
  dz.rr +=  last_roll_err;
  printf("dz: fl: %f, fr: %f, rl: %f, rr: %f\n", dz.fl, dz.fr, dz.rl, dz.rr);

  // Optionally clamp corrections to reasonable range (e.g., ±0.05 m)
  const float limit = 0.05f;
  dz.fl = std::clamp(dz.fl, -limit, limit);
  dz.fr = std::clamp(dz.fr, -limit, limit);
  dz.rl = std::clamp(dz.rl, -limit, limit);
  dz.rr = std::clamp(dz.rr, -limit, limit);

  return dz;
}

} // namespace aresdog_controller 