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
#include <fstream>
#include <iostream>
#include <thread>


using namespace std::chrono_literals;
using namespace std;


/// @brief ROS2 publisher class for the manus_ros2 node
class ManusROS2Publisher : public rclcpp::Node
{
public:
	ManusROS2Publisher() : Node("manus_ros2")
	{
		manus_left_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("manus_left", 10);
    	manus_right_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("manus_right", 10);
	}

	void publish_left(geometry_msgs::msg::PoseArray::SharedPtr pose_array) {
    	manus_left_publisher_->publish(*pose_array);
  	}

  	void publish_right(geometry_msgs::msg::PoseArray::SharedPtr pose_array) {
    	manus_right_publisher_->publish(*pose_array);
  	}

private:
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr manus_left_publisher_;
  	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr manus_right_publisher_;
};



void convertSkeletonDataToROS(std::shared_ptr<ManusROS2Publisher> publisher)
{
	ClientSkeletonCollection* csc = SDKMinimalClient::GetInstance()->CurrentSkeletons();
	if (csc != nullptr && csc->skeletons.size() != 0) {
    	for (size_t i=0; i < csc->skeletons.size(); ++i) {
      
			// Prepare a new PoseArray message for the data
			auto pose_array = std::make_shared<geometry_msgs::msg::PoseArray>();
			pose_array->header.stamp = publisher->now();

			// Set the poses for the message
      		for (size_t j=0; j < csc->skeletons[i].info.nodesCount; ++j) {
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
