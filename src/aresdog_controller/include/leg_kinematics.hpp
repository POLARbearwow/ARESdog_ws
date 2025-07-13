#ifndef LEG_KINEMATICS_HPP_
#define LEG_KINEMATICS_HPP_

#include <cmath>
#include <cstdint>

namespace aresdog_controller
{

/**
 * @struct Point2D
 * @brief Represents a point in a 2D Cartesian plane.
 */
struct Point2D
{
  float x;
  float z;
};

/**
 * @class LegKinematics
 * @brief Handles the forward and inverse kinematics for a single parallel leg.
 *
 * This class encapsulates the geometric parameters and kinematic calculations
 * for a two-link parallel leg operating in the X-Z plane.
 */
class LegKinematics
{
public:
  /**
   * @brief Constructs a LegKinematics instance.
   * @param l3 The length of the upper link (proximal segment), in meters.
   * @param l4 The length of the lower link (distal segment), in meters.
   */
  LegKinematics(float l3, float l4);

  /**
   * @brief Solves the inverse kinematics for a given target foot-tip position.
   * @param x The target x-coordinate of the foot-tip.
   * @param z The target z-coordinate of the foot-tip.
   * @param[out] theta1 The calculated angle for the first joint (hip/motor0), in radians.
   * @param[out] theta4 The calculated angle for the second joint (knee/motor1), in radians.
   * @return True if a valid solution is found (target is reachable), false otherwise.
   */
  bool inverseKinematics(float x, float z, float& theta1, float& theta4);

  /**
   * @brief Solves the forward kinematics for given joint angles.
   * 
   * @param theta1 The angle of the first joint (hip/motor0), in radians.
   * @return The calculated (x, z) coordinates of the foot-tip.
   */
  Point2D forwardKinematics(float theta1);


private:
  const float l3_; // Length of the upper link (m)
  const float l4_; // Length of the lower link (m)
};

} // namespace aresdog_controller

#endif // LEG_KINEMATICS_HPP_ 