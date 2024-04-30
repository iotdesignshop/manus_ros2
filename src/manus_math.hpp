#pragma once

#include "SDKMinimalClient.hpp"

ManusQuaternion operator*(const ManusQuaternion &q1, const ManusQuaternion &q2);
ManusQuaternion InverseQuaternion(const ManusQuaternion &q);
void QuaternionToEulerAngles(const double *quaternion, double &roll, double &pitch, double &yaw);

