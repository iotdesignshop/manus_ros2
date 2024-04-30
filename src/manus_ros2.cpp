// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// DexHand ROS2 Includes
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Manus SDK includes
#include "SDKMinimalClient.hpp"
#include <fstream>
#include <iostream>
#include <thread>

// Local Includes
#include <cmath> // Required for quaternionToEulerAngles

// Needed for isKeyPressed
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using namespace std;

// Scan for a key press - Used to reset wrist position to center
bool KeyDown()
{
	struct termios oldt, newt;
	int oldf;

	// Get the current terminal settings
	tcgetattr(STDIN_FILENO, &oldt);

	// Save the current terminal settings so we can restore them later
	newt = oldt;

	// Disable canonical mode and echo
	newt.c_lflag &= ~(ICANON | ECHO);

	// Apply the new terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	// Set the file descriptor for stdin to non-blocking
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	// Try to read a character from stdin
	char ch;
	ssize_t nread = read(STDIN_FILENO, &ch, 1);

	// Restore the old terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	// Restore the file descriptor flags
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	// Check if a character was read
	if (nread == 1)
		return true;
	else
		return false;
}

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

// Global for now to pass joint data between the Manus SDK and ROS2
double g_euler_joint[24][3] = {0.0};

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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher() : Node("dexhand_manus"), count_(0)
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

		// timer_ = this->create_wall_timer(33ms, std::bind(&MinimalPublisher::timer_callback, this)); // ~30Hz
		timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::timer_callback, this));
	}

	// print_joint - prints global joint data from Manus Hand after transform
	void print_joint(int joint)
	{
		std::cout << std::fixed << std::setprecision(5) << "Joint: " << joint << " " << (g_euler_joint[joint][0] >= 0 ? " " : "") << g_euler_joint[joint][0] << " " << (g_euler_joint[joint][1] >= 0 ? " " : "") << g_euler_joint[joint][1] << " " << (g_euler_joint[joint][2] >= 0 ? " " : "") << g_euler_joint[joint][2] << " "
				  << "\n";
	}

private:
	// timer_callback - publishes joint states to ROS2
	void timer_callback()
	{
		auto joint_state = sensor_msgs::msg::JointState();

		joint_state.name.push_back("wrist_pitch_lower");
		joint_state.name.push_back("wrist_pitch_upper");
		joint_state.name.push_back("wrist_yaw");
		joint_state.name.push_back("index_yaw");
		joint_state.name.push_back("middle_yaw");
		joint_state.name.push_back("ring_yaw");
		joint_state.name.push_back("pinky_yaw");
		joint_state.name.push_back("index_pitch");
		joint_state.name.push_back("index_knuckle");
		joint_state.name.push_back("index_tip");
		joint_state.name.push_back("middle_pitch");
		joint_state.name.push_back("middle_knuckle");
		joint_state.name.push_back("middle_tip");
		joint_state.name.push_back("ring_pitch");
		joint_state.name.push_back("ring_knuckle");
		joint_state.name.push_back("ring_tip");
		joint_state.name.push_back("pinky_pitch");
		joint_state.name.push_back("pinky_knuckle");
		joint_state.name.push_back("pinky_tip");
		joint_state.name.push_back("thumb_yaw");
		joint_state.name.push_back("thumb_roll");
		joint_state.name.push_back("thumb_pitch");
		joint_state.name.push_back("thumb_knuckle");
		joint_state.name.push_back("thumb_tip");

		const std::string t_FingerNames[NUM_FINGERS_ON_HAND] = {"[thumb] ", "[index] ", "[middle]", "[ring]  ", "[pinky] "};
		const std::string t_FingerJointNames[NUM_FINGERS_ON_HAND] = {"mcp", "pip", "dip"};
		const std::string t_ThumbJointNames[NUM_FINGERS_ON_HAND] = {"cmc", "mcp", "ip "};

		// alignment assumes wrist flat on table so testing the fingers is easier
		// joint_state.position.push_back(0.0); // "wrist_pitch_lower"
		// joint_state.position.push_back(0.0); // "wrist_pitch_upper"
		// joint_state.position.push_back(0.0); // "wrist_yaw"

		// print_joint(0);

		joint_state.position.push_back(-g_euler_joint[0][1]); // "wrist_pitch_lower"
		joint_state.position.push_back(-g_euler_joint[0][1]); // "wrist_pitch_upper"
		joint_state.position.push_back(g_euler_joint[0][2]);  // "wrist_yaw"
		joint_state.position.push_back(g_euler_joint[5][2]);  // "index_yaw"
		joint_state.position.push_back(g_euler_joint[9][2]);  // "middle_yaw"
		joint_state.position.push_back(g_euler_joint[13][2]); // "ring_yaw"
		joint_state.position.push_back(g_euler_joint[17][2]); // "pinky_yaw"
		joint_state.position.push_back(g_euler_joint[5][1]);  // "index_pitch"
		joint_state.position.push_back(g_euler_joint[6][1]);  // "index_knuckle"
		joint_state.position.push_back(g_euler_joint[7][1]);  // "index_tip"
		joint_state.position.push_back(g_euler_joint[9][1]);  // "middle_pitch"
		joint_state.position.push_back(g_euler_joint[10][1]); // "middle_knuckle"
		joint_state.position.push_back(g_euler_joint[11][1]); // "middle_tip"
		joint_state.position.push_back(g_euler_joint[13][1]); // "ring_pitch"
		joint_state.position.push_back(g_euler_joint[14][1]); // "ring_knuckle"
		joint_state.position.push_back(g_euler_joint[15][1]); // "ring_tip"
		joint_state.position.push_back(g_euler_joint[17][1]); // "pinky_pitch"
		joint_state.position.push_back(g_euler_joint[18][1]); // "pinky_knuckle"
		joint_state.position.push_back(g_euler_joint[19][1]); // "pinky_tip"
		joint_state.position.push_back(g_euler_joint[1][2]);  // "thumb_yaw"
		joint_state.position.push_back(g_euler_joint[1][0]);  // "thumb_roll"
		joint_state.position.push_back(g_euler_joint[1][1]);  // "thumb_pitch"
		joint_state.position.push_back(-g_euler_joint[2][2]); // "thumb_knuckle"
		joint_state.position.push_back(-g_euler_joint[3][2]); // "thumb_tip"

		// RCLCPP_INFO(this->get_logger(), "Publishing: %lf", joint_state.position[0]);

		// Set timestamp to current ROS time
		joint_state.header.stamp = this->now();

		// Command the robot to move
		publisher_->publish(joint_state);

		// std::this_thread::sleep_for(std::chrono::milliseconds(33)); // or roughly 30fps, but good enough to show the results.
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	size_t count_;
};

void convertSkeletonDataToROS()
{
  ClientSkeletonCollection* csc = SDKMinimalClient::GetInstance()->CurrentSkeletons();
	if (csc != nullptr && csc->skeletons.size() != 0)
	{
    cout << "Skeletons count " << csc->skeletons.size() << endl;
    for (int i=0; i < csc->skeletons.size(); ++i) {
      cout << i << ": Skeleton " << csc->skeletons[i].info.id << " nodes count " << csc->skeletons[i].info.nodesCount << endl;
    }
    double roll, pitch, yaw;
    static bool initialized = false;
		static ManusQuaternion referenceRotation = {1.0, 0.0, 0.0, 0.0}; // Initialize with identity quaternion

		// Put the skeleton data in the bridge structure
		for (uint32_t i = 0; i < csc->skeletons[0].info.nodesCount; i++)
		{
			const auto &joint = csc->skeletons[0].nodes[i];

			// Calculate delta rotation only for the wrist (joint 0)
			if (i == 0)
			{
				if (!initialized)
				{
					referenceRotation = joint.transform.rotation;
					initialized = true;
				}
				else if (KeyDown())
				{
					initialized = false;
				}

				// Calculate the inverse of the reference rotation
				ManusQuaternion referenceRotationInverse = InverseQuaternion(referenceRotation);

				// Calculate the delta rotation
				ManusQuaternion newRotationInverse = referenceRotationInverse * joint.transform.rotation;

				double quaternion[4] = {0.0};
				quaternion[0] = newRotationInverse.x;
				quaternion[1] = newRotationInverse.y;
				quaternion[2] = newRotationInverse.z;
				quaternion[3] = newRotationInverse.w;

				QuaternionToEulerAngles(quaternion, roll, pitch, yaw);

				// Assign roll, pitch, and yaw directly to g_dexhand_joint
				g_euler_joint[i][0] = roll;
				g_euler_joint[i][1] = pitch;
				g_euler_joint[i][2] = yaw;
			}
			else
			{
				// For child joints, perform regular quaternion to Euler angle conversion
				double quaternion[4] = {0.0};
				quaternion[0] = joint.transform.rotation.x;
				quaternion[1] = joint.transform.rotation.y;
				quaternion[2] = joint.transform.rotation.z;
				quaternion[3] = joint.transform.rotation.w;

				QuaternionToEulerAngles(quaternion, roll, pitch, yaw);

				// Assign roll, pitch, and yaw directly to g_dexhand_joint
				g_euler_joint[i][0] = roll;
				g_euler_joint[i][1] = pitch;
				g_euler_joint[i][2] = yaw;
			}
		}

#if DEBUG_PRINT
		std::cout << "Node: " << i << " Rotation X, Y, Z, W "
				  << joint.transform.rotation.x << ", "
				  << joint.transform.rotation.y << ", "
				  << joint.transform.rotation.z << ", "
				  << joint.transform.rotation.w << ")" << std::endl;

		// Print Euler angles
		std::cout << "Node: " << i << " Roll, Pitch, Yaw: "
				  << roll << ", " << pitch << ", " << yaw << std::endl;
#endif
  }
        

}

// Main function - Initializes the minimal client and starts the ROS2 node
int main(int argc, char *argv[])
{
	std::cout << "Starting minimal client!\n";
	SDKMinimalClient t_Client;
	t_Client.Initialize();
	rclcpp::init(argc, argv);
	std::cout << "minimal client is initialized.\n";

	t_Client.ConnectToHost();

	auto minimal_publisher = std::make_shared<MinimalPublisher>();

	// Create an executor to spin the minimal_publisher
	auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	executor->add_node(minimal_publisher);

	// Setup a timer to call t_Client.Run at a fixed rate
	auto timer = minimal_publisher->create_wall_timer(1ms, [&t_Client]()
													  { if (t_Client.Run()) convertSkeletonDataToROS(); });

	// Spin the executor
	executor->spin();

	// Shutdown ROS 2
	rclcpp::shutdown();

	// Shutdown the Manus client
	std::cout << "minimal client is done, shutting down.\n";
	t_Client.ShutDown();

	return 0;
}
