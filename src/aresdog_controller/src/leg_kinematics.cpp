#include "aresdog_controller/leg_kinematics.hpp"
#include <iostream>

namespace aresdog_controller
{

/**
 * @brief Constructs a LegKinematics instance.
 * @param l3 The length of the upper link (proximal segment).
 * @param l4 The length of the lower link (distal segment).
 */
LegKinematics::LegKinematics(float l3, float l4) : l3_(l3), l4_(l4)
{
  // Initialization complete in the initializer list.
}

/**
 * @brief Solves the inverse kinematics.
 * @param x The target x-coordinate.
 * @param z The target z-coordinate.
 * @param[out] theta1 The calculated angle for the first joint.
 * @param[out] theta4 The calculated angle for the second joint.
 * @return True if a solution exists, false otherwise.
 */
bool LegKinematics::inverseKinematics(float x, float z, float& theta1, float& theta4)
{
  float l0 = sqrtf(x * x + z * z);

  // Check if the target is within the reachable workspace
  if (fabsf(l3_ - l4_) > l0 || l0 > (l3_ + l4_))
  {
    // Target is unreachable, return failure
    return false;
  }

  float theta_inside = acosf((l4_ * l4_ + l0 * l0 - l3_ * l3_) / (2 * l4_ * l0));
  float theta = atan2f(z, x);

  theta1 = theta + theta_inside;
  theta4 = theta - theta_inside;

  return true; // Solution found successfully
}

/**
 * @brief Solves the forward kinematics.
 * @param theta1 The angle of the first joint.
 * @return The calculated (x, z) coordinates of the foot-tip.
 */
Point2D LegKinematics::forwardKinematics(float theta1)
{
    Point2D foot_tip_position;
    foot_tip_position.x = l3_ * cosf(theta1);
    foot_tip_position.z = l3_ * sinf(theta1);
    return foot_tip_position;
}


} // namespace aresdog_controller 