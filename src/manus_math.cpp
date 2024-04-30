// Local Includes
#include <cmath> // Required for quaternionToEulerAngles

#include "manus_math.hpp"

// Quaternion Multiplication operator needed for wrist transform
ManusQuaternion operator*(const ManusQuaternion &q1, const ManusQuaternion &q2)
{
	ManusQuaternion result;
	result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
	return result;
}

// Inverse Quaternaion needed for wrist transform
ManusQuaternion InverseQuaternion(const ManusQuaternion &q)
{
	double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	if (norm == 0.0)
	{
		// Avoid division by zero
		return {1.0, 0.0, 0.0, 0.0}; // Identity quaternion
	}

	double invNorm = 1.0 / norm;
	return {q.w * invNorm, -q.x * invNorm, -q.y * invNorm, -q.z * invNorm};
}

// QuaternionToEulerAngles - Converts a quaternion to Euler angles
void QuaternionToEulerAngles(const double *quaternion, double &roll, double &pitch, double &yaw)
{
	double qx = quaternion[0];
	double qy = quaternion[1];
	double qz = quaternion[2];
	double qw = quaternion[3];

	// Roll (x-axis rotation)
	double sinr_cosp = 2.0 * (qw * qx + qy * qz);
	double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
	roll = std::atan2(sinr_cosp, cosr_cosp);

	// Pitch (y-axis rotation)
	double sinp = 2.0 * (qw * qy - qz * qx);
	if (std::abs(sinp) >= 1)
		pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = std::asin(sinp);

	// Yaw (z-axis rotation)
	double siny_cosp = 2.0 * (qw * qz + qx * qy);
	double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
	yaw = std::atan2(siny_cosp, cosy_cosp);
}