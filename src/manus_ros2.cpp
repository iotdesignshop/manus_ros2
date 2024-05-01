/// @file manus_ros2.cpp
/// @brief This file contains the main function for the manus_ros2 node, which interfaces with the Manus SDK to
/// receive animated skeleton data, and republishes the events as ROS 2 messages.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "SDKMinimalClient.hpp"
#include "manus_math.hpp"
#include <fstream>
#include <iostream>
#include <thread>


using namespace std::chrono_literals;
using namespace std;

// Global for now to pass joint data between the Manus SDK and ROS2
float g_euler_joint[24][3] = {0.0};


/// @brief ROS2 publisher class for the manus_ros2 node
class ManusROS2Publisher : public rclcpp::Node
{
public:
	ManusROS2Publisher() : Node("manus_ros2")
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    	manus_left_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("manus_left", 10);
    	manus_right_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("manus_right", 10);
    	publishtimer_ = this->create_wall_timer(33ms, std::bind(&ManusROS2Publisher::timer_callback, this)); // 30Hz publish rate
	}

	// print_joint - prints global joint data from Manus Hand after transform
	void print_joint(int joint)
	{
    RCLCPP_INFO_STREAM(this->get_logger(), 
		  std::fixed << std::setprecision(5) << "Joint: " << joint << " " 
        << (g_euler_joint[joint][0] >= 0 ? " " : "") << g_euler_joint[joint][0] << " " 
        << (g_euler_joint[joint][1] >= 0 ? " " : "") << g_euler_joint[joint][1] << " " 
        << (g_euler_joint[joint][2] >= 0 ? " " : "") << g_euler_joint[joint][2] << " "
			  << "\n");
	}

  void publish_left(geometry_msgs::msg::PoseArray::SharedPtr pose_array) {
    manus_left_publisher_->publish(*pose_array);
  }

  void publish_right(geometry_msgs::msg::PoseArray::SharedPtr pose_array) {
    manus_right_publisher_->publish(*pose_array);
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

	rclcpp::TimerBase::SharedPtr publishtimer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr manus_left_publisher_;
  	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr manus_right_publisher_;
};


// Scan for a key press - Used to reset wrist position to center
bool KeyDown()
{
	return false;
}


void convertSkeletonDataToROS(auto &publisher)
{
  ClientSkeletonCollection* csc = SDKMinimalClient::GetInstance()->CurrentSkeletons();
	if (csc != nullptr && csc->skeletons.size() != 0)
	{
    for (int i=0; i < csc->skeletons.size(); ++i) {
      
      // Prepare a new PoseArray message for the data
      auto pose_array = std::make_shared<geometry_msgs::msg::PoseArray>();
      pose_array->header.stamp = publisher->now();

      // Set the poses for the message
      for (int j=0; j < csc->skeletons[i].info.nodesCount; ++j) {
        const auto &joint = csc->skeletons[i].nodes[j];
        geometry_msgs::msg::Pose pose;
        pose.position.x = joint.transform.position.x;
        pose.position.y = joint.transform.position.y;
        pose.position.z = joint.transform.position.z;
        pose.orientation.x = joint.transform.rotation.x;
        pose.orientation.y = joint.transform.rotation.y;
        pose.orientation.z = joint.transform.rotation.z;
        pose.orientation.w = joint.transform.rotation.w;
        pose_array->poses.push_back(pose);
      }

      // Which hand is this?
      if (csc->skeletons[i].info.id == SDKMinimalClient::GetInstance()->GetRightHandID()) {
        pose_array->header.frame_id = "manus_right";
        publisher->publish_right(pose_array);
      } else {
        pose_array->header.frame_id = "manus_left";
        publisher->publish_left(pose_array);
      }

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
	rclcpp::init(argc, argv);
	
	auto publisher = std::make_shared<ManusROS2Publisher>();

	RCLCPP_INFO(publisher->get_logger(), "Starting manus_ros2 node");
	SDKMinimalClient t_Client(publisher);
	ClientReturnCode status = t_Client.Initialize();

	if (status != ClientReturnCode::ClientReturnCode_Success)
	{
		RCLCPP_ERROR_STREAM(publisher->get_logger(), "Failed to initialize the Manus SDK. Error code: " << (int)status);
		return 1;
	}
	
	RCLCPP_INFO(publisher->get_logger(), "Connecting to Manus SDK");
	t_Client.ConnectToHost();

	
	// Create an executor to spin the minimal_publisher
	auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	executor->add_node(publisher);

	// Publish the poses at 50hz
	auto timer = publisher->create_wall_timer(20ms, [&t_Client,&publisher]()
													  { if (t_Client.Run()) convertSkeletonDataToROS(publisher); });

	// Spin the executor
	executor->spin();

	// Shutdown the Manus client
	t_Client.ShutDown();

	// Shutdown ROS 2
	rclcpp::shutdown();

	return 0;
}
