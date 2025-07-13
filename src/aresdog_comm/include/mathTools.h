/**********************************************************************
 Copyright (c) 2023-2024, Ares Robotics. All rights reserved.
***********************************************************************/
#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <stdio.h>
#include <iostream>
#include "common/mathTypes.h"

inline RotMat rotx(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << 1, 0, 0, 0, c, -s, 0, s, c;
    return R;
}

inline RotMat roty(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, 0, s, 0, 1, 0, -s, 0, c;
    return R;
}

inline RotMat rotz(const double &theta) {
    double s = std::sin(theta);
    double c = std::cos(theta);

    RotMat R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

inline RotMat rpyToRotMat(const double& row, const double& pitch, const double& yaw) {
    RotMat m = rotz(yaw) * roty(pitch) * rotx(row);
    return m;
}

inline Vec3 rotMatToRPY(const Mat3& R) {
    Vec3 rpy;
    rpy(0) = atan2(R(2,1),R(2,2));
    rpy(1) = asin(-R(2,0));
    rpy(2) = atan2(R(1,0),R(0,0));
    return rpy;
}

inline RotMat quatToRotMat(const Quat& q) {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

#endif  // MATHTOOLS_H 