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
#include "ManusSDKTypes.h"
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

const bool RIGHT_HAND = true;

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
													  { t_Client.Run(); });

	// Spin the executor
	executor->spin();

	// Shutdown ROS 2
	rclcpp::shutdown();

	// Shutdown the Manus client
	std::cout << "minimal client is done, shutting down.\n";
	t_Client.ShutDown();

	return 0;
}

// Manus Hand functionality from here down
// Initialize the static member variable
SDKMinimalClient *SDKMinimalClient::s_Instance = nullptr;

SDKMinimalClient::SDKMinimalClient()
{
	s_Instance = this;
}

SDKMinimalClient::~SDKMinimalClient()
{
	s_Instance = nullptr;
}

/// @brief Initialize the sample console and the SDK.
/// This function attempts to resize the console window and then proceeds to initialize the SDK's interface.
ClientReturnCode SDKMinimalClient::Initialize()
{
	if (!PlatformSpecificInitialization())
	{
		return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificInitialization;
	}

	const ClientReturnCode t_IntializeResult = InitializeSDK();
	if (t_IntializeResult != ClientReturnCode::ClientReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

/// @brief Initialize the sdk, register the callbacks and set the coordinate system.
/// This needs to be done before any of the other SDK functions can be used.
ClientReturnCode SDKMinimalClient::InitializeSDK()
{
	// before we can use the SDK, some internal SDK bits need to be initialized.
	// however after initializing, the SDK is not yet connected to a host or doing anything network related just yet.
	const SDKReturnCode t_InitializeResult = CoreSdk_Initialize(SessionType::SessionType_CoreSDK);
	if (t_InitializeResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	const ClientReturnCode t_CallBackResults = RegisterAllCallbacks();
	if (t_CallBackResults != ::ClientReturnCode::ClientReturnCode_Success)
	{
		return t_CallBackResults;
	}

	// after everything is registered and initialized as seen above
	// we must also set the coordinate system being used for the data in this client.
	// (each client can have their own settings. unreal and unity for instance use different coordinate systems)
	// if this is not set, the SDK will not connect to any Manus core host.
	CoordinateSystemVUH t_VUH;
	CoordinateSystemVUH_Init(&t_VUH);
	// t_VUH.handedness = Side::Side_Left; // this is currently set to unreal mode.
	// t_VUH.up = AxisPolarity::AxisPolarity_PositiveY;
	// t_VUH.view = AxisView::AxisView_ZFromViewer;
	t_VUH.handedness = Side::Side_Right; // this is currently set to ROS mode.
	t_VUH.up = AxisPolarity::AxisPolarity_PositiveZ;
	t_VUH.view = AxisView::AxisView_XFromViewer;
	t_VUH.unitScale = 1.0f; // 1.0 is meters, 0.01 is cm, 0.001 is mm.

	const SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, false);

	if (t_CoordinateResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

/// @brief When you are done with the SDK, don't forget to nicely shut it down
/// this will close all connections to the host, close any threads and clean up after itself
/// after this is called it is expected to exit the client program. If not it needs to call initialize again.
ClientReturnCode SDKMinimalClient::ShutDown()
{
	const SDKReturnCode t_Result = CoreSdk_ShutDown();
	if (t_Result != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;
	}

	if (!PlatformSpecificShutdown())
	{
		return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificShutdown;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

/// @brief Used to register the callbacks between sdk and core.
/// Callbacks that are registered functions that get called when a certain 'event' happens, such as data coming in from Manus Core.
/// All of these are optional, but depending on what data you require you may or may not need all of them.
ClientReturnCode SDKMinimalClient::RegisterAllCallbacks()
{
	// Register the callback for when manus core is sending Skeleton data
	// it is optional, but without it you can not see any resulting skeleton data.
	// see OnSkeletonStreamCallback for more details.
	const SDKReturnCode t_RegisterSkeletonCallbackResult = CoreSdk_RegisterCallbackForSkeletonStream(*OnSkeletonStreamCallback);
	if (t_RegisterSkeletonCallbackResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToInitialize;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

/// @brief This function is called when the SDK has connected to a host.
/// This function is called when the SDK has connected to a host.  Split out from the original Run function.
void SDKMinimalClient::ConnectToHost()
{
	// first loop until we get a connection
	std::cout << "minimal client is connecting to host. (make sure it is running)\n";
	while (Connect() != ClientReturnCode::ClientReturnCode_Success)
	{
		std::cout << "minimal client could not connect. Trying again in a second.\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	std::cout << "minimal client is connected, setting up skeletons.\n";
	// Upload a simple skeleton with a chain. This will just be a left hand for the first user index.
	LoadTestSkeleton();
}

/// @brief Main loop that receives data from the SDK and processes it.
void SDKMinimalClient::Run()
{
	double roll, pitch, yaw;

	// Check if there is new data, otherwise, we just wait.
	m_SkeletonMutex.lock();
	if (m_NextSkeleton != nullptr)
	{
		if (m_Skeleton != nullptr)
			delete m_Skeleton;
		m_Skeleton = m_NextSkeleton;
		m_NextSkeleton = nullptr;
	}
	m_SkeletonMutex.unlock();

	if (m_Skeleton != nullptr && m_Skeleton->skeletons.size() != 0)
	{
		static bool initialized = false;
		static ManusQuaternion referenceRotation = {1.0, 0.0, 0.0, 0.0}; // Initialize with identity quaternion

		// Put the skeleton data in the bridge structure
		for (uint32_t i = 0; i < m_Skeleton->skeletons[0].info.nodesCount; i++)
		{
			const auto &joint = m_Skeleton->skeletons[0].nodes[i];

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

		m_FrameCounter++;
	}

	// TODO: Either turn this on or change the frequency in Main based on its value
	// std::this_thread::sleep_for(std::chrono::milliseconds(33)); // or roughly 30fps, but good enough to show the results.
}

/// @brief the client will now try to connect to manus core via the SDK.
ClientReturnCode SDKMinimalClient::Connect()
{
	SDKReturnCode t_StartResult = CoreSdk_LookForHosts(1, false);
	if (t_StartResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	uint32_t t_NumberOfHostsFound = 0;
	SDKReturnCode t_NumberResult = CoreSdk_GetNumberOfAvailableHostsFound(&t_NumberOfHostsFound);
	if (t_NumberResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	if (t_NumberOfHostsFound == 0)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	std::unique_ptr<ManusHost[]> t_AvailableHosts;
	t_AvailableHosts.reset(new ManusHost[t_NumberOfHostsFound]);

	SDKReturnCode t_HostsResult = CoreSdk_GetAvailableHostsFound(t_AvailableHosts.get(), t_NumberOfHostsFound);
	if (t_HostsResult != SDKReturnCode::SDKReturnCode_Success)
	{
		return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
	}

	SDKReturnCode t_ConnectResult = CoreSdk_ConnectToHost(t_AvailableHosts[0]);

	if (t_ConnectResult == SDKReturnCode::SDKReturnCode_NotConnected)
	{
		return ClientReturnCode::ClientReturnCode_FailedToConnect;
	}

	return ClientReturnCode::ClientReturnCode_Success;
}

/// @brief This function sets up a very minimalistic hand skeleton.
/// In order to have any 3d positional/rotational information from the gloves or body,
/// one needs to setup a skeleton on which this data can be applied.
/// In the case of this sample we create a Hand skeleton in order to get skeleton information
/// in the OnSkeletonStreamCallback function. This sample does not contain any 3D rendering, so
/// we will not be applying the returned data on anything.
void SDKMinimalClient::LoadTestSkeleton()
{
	uint32_t t_SklIndex = 0;

	SkeletonSetupInfo t_SKL;
	SkeletonSetupInfo_Init(&t_SKL);
	t_SKL.type = SkeletonType::SkeletonType_Hand;
	t_SKL.settings.scaleToTarget = true;
	t_SKL.settings.targetType = SkeletonTargetType::SkeletonTargetType_UserIndexData;

	// The user index is the index of the user that the skeleton is attached to.
	// If the glove does not exist then the added skeleton will not be animated.
	// Same goes for any other skeleton made for invalid users/gloves.
	t_SKL.settings.skeletonTargetUserIndexData.userIndex = 0; // Just take the first index. make sure this matches in the landscape.

	CopyString(t_SKL.name, sizeof(t_SKL.name), std::string("Hand"));

	SDKReturnCode t_Res = CoreSdk_CreateSkeletonSetup(t_SKL, &t_SklIndex);
	if (t_Res != SDKReturnCode::SDKReturnCode_Success)
	{
		return;
	}

	// setup nodes and chains for the skeleton hand
	if (!SetupHandNodes(t_SklIndex, RIGHT_HAND))
	{
		return;
	}
	if (!SetupHandChains(t_SklIndex, RIGHT_HAND))
	{
		return;
	}

	// load skeleton
	uint32_t t_ID = 0;
	t_Res = CoreSdk_LoadSkeleton(t_SklIndex, &t_ID);
	if (t_Res != SDKReturnCode::SDKReturnCode_Success)
	{
		return;
	}
}

/// @brief Skeletons are pretty extensive in their data setup
/// so we have several support functions so we can correctly receive and parse the data,
/// this function helps setup the data.
/// @param p_Id the id of the created node setup
/// @param p_ParentId the id of the node parent
/// @param p_PosX X position of the node, this is defined with respect to the global coordinate system or the local one depending on
/// the parameter p_UseWorldCoordinates set when initializing the sdk,
/// @param p_PosY Y position of the node this is defined with respect to the global coordinate system or the local one depending on
/// the parameter p_UseWorldCoordinates set when initializing the sdk,
/// @param p_PosZ Z position of the node this is defined with respect to the global coordinate system or the local one depending on
/// the parameter p_UseWorldCoordinates set when initializing the sdk,
/// @param p_Name the name of the node setup
/// @return the generated node setup
NodeSetup SDKMinimalClient::CreateNodeSetup(uint32_t p_Id, uint32_t p_ParentId, float p_PosX, float p_PosY, float p_PosZ, std::string p_Name)
{
	NodeSetup t_Node;
	NodeSetup_Init(&t_Node);
	t_Node.id = p_Id; // Every ID needs to be unique per node in a skeleton.
	CopyString(t_Node.name, sizeof(t_Node.name), p_Name);
	t_Node.type = NodeType::NodeType_Joint;
	// Every node should have a parent unless it is the Root node.
	t_Node.parentID = p_ParentId; // Setting the node ID to its own ID ensures it has no parent.
	t_Node.settings.usedSettings = NodeSettingsFlag::NodeSettingsFlag_None;

	t_Node.transform.position.x = p_PosX;
	t_Node.transform.position.y = p_PosY;
	t_Node.transform.position.z = p_PosZ;
	return t_Node;
}

ManusVec3 SDKMinimalClient::CreateManusVec3(float p_X, float p_Y, float p_Z)
{
	ManusVec3 t_Vec;
	t_Vec.x = p_X;
	t_Vec.y = p_Y;
	t_Vec.z = p_Z;
	return t_Vec;
}

/// @brief This support function sets up the nodes for the skeleton hand
/// In order to have any 3d positional/rotational information from the gloves or body,
/// one needs to setup the skeleton on which this data should be applied.
/// In the case of this sample we create a Hand skeleton for which we want to get the calculated result.
/// The ID's for the nodes set here are the same IDs which are used in the OnSkeletonStreamCallback,
/// this allows us to create the link between Manus Core's data and the data we enter here.
bool SDKMinimalClient::SetupHandNodes(uint32_t p_SklIndex, bool isRightHand)
{
	// Define number of fingers per hand and number of joints per finger
	const uint32_t t_NumFingers = 5;
	const uint32_t t_NumJoints = 4;

	// Create an array with the initial position of each hand node.
	ManusVec3 t_Fingers_Right[t_NumFingers * t_NumJoints] = {
		CreateManusVec3(0.025320f, 0.024950f, 0.000000f), // Thumb CMC joint
		CreateManusVec3(0.032742f, 0.000000f, 0.000000f), // Thumb MCP joint
		CreateManusVec3(0.028739f, 0.000000f, 0.000000f), // Thumb IP joint
		CreateManusVec3(0.028739f, 0.000000f, 0.000000f), // Thumb Tip joint

		CreateManusVec3(0.052904f, -0.011181f, 0.000000f), // Index MCP joint
		CreateManusVec3(0.038257f, 0.000000f, 0.000000f),  // Index PIP joint
		CreateManusVec3(0.020884f, 0.000000f, 0.000000f),  // Index DIP joint
		CreateManusVec3(0.018759f, 0.000000f, 0.000000f),  // Index Tip joint

		CreateManusVec3(0.051287f, 0.000000f, 0.000000f), // Middle MCP joint
		CreateManusVec3(0.041861f, 0.000000f, 0.000000f), // Middle PIP joint
		CreateManusVec3(0.024766f, 0.000000f, 0.000000f), // Middle DIP joint
		CreateManusVec3(0.019683f, 0.000000f, 0.000000f), // Middle Tip joint

		CreateManusVec3(0.049802f, -0.011274f, 0.000000f), // Ring MCP joint
		CreateManusVec3(0.039736f, 0.000000f, 0.000000f),  // Ring PIP joint
		CreateManusVec3(0.023564f, 0.000000f, 0.000000f),  // Ring DIP joint
		CreateManusVec3(0.019868f, 0.000000f, 0.000000f),  // Ring Tip joint

		CreateManusVec3(0.047309f, -0.020145f, 0.000000f), // Pinky MCP joint
		CreateManusVec3(0.033175f, 0.000000f, 0.000000f),  // Pinky PIP joint
		CreateManusVec3(0.018020f, 0.000000f, 0.000000f),  // Pinky DIP joint
		CreateManusVec3(0.019129f, 0.000000f, 0.000000f)   // Pinky Tip joint
	};

	ManusVec3 t_Fingers_Left[t_NumFingers * t_NumJoints] = {
		CreateManusVec3(-0.025320f, 0.024950f, 0.000000f), // Thumb CMC joint
		CreateManusVec3(-0.032742f, 0.000000f, 0.000000f), // Thumb MCP joint
		CreateManusVec3(-0.028739f, 0.000000f, 0.000000f), // Thumb IP joint
		CreateManusVec3(-0.028739f, 0.000000f, 0.000000f), // Thumb Tip joint

		CreateManusVec3(-0.052904f, -0.011181f, 0.000000f), // Index MCP joint
		CreateManusVec3(-0.038257f, 0.000000f, 0.000000f),	// Index PIP joint
		CreateManusVec3(-0.020884f, 0.000000f, 0.000000f),	// Index DIP joint
		CreateManusVec3(-0.018759f, 0.000000f, 0.000000f),	// Index Tip joint

		CreateManusVec3(-0.051287f, 0.000000f, 0.000000f), // Middle MCP joint
		CreateManusVec3(-0.041861f, 0.000000f, 0.000000f), // Middle PIP joint
		CreateManusVec3(-0.024766f, 0.000000f, 0.000000f), // Middle DIP joint
		CreateManusVec3(-0.019683f, 0.000000f, 0.000000f), // Middle Tip joint

		CreateManusVec3(-0.049802f, 0.011274f, 0.000000f), // Ring MCP joint
		CreateManusVec3(-0.039736f, 0.000000f, 0.000000f), // Ring PIP joint
		CreateManusVec3(-0.023564f, 0.000000f, 0.000000f), // Ring DIP joint
		CreateManusVec3(-0.019868f, 0.000000f, 0.000000f), // Ring Tip joint

		CreateManusVec3(-0.047309f, 0.020145f, 0.000000f), // Pinky MCP joint
		CreateManusVec3(-0.033175f, 0.000000f, 0.000000f), // Pinky PIP joint
		CreateManusVec3(-0.018020f, 0.000000f, 0.000000f), // Pinky DIP joint
		CreateManusVec3(-0.019129f, 0.000000f, 0.000000f)  // Pinky Tip joint
	};

	// skeleton entry is already done. just the nodes now.
	// setup a very simple node hierarchy for fingers
	// first setup the root node
	//
	// root, This node has ID 0 and parent ID 0, to indicate it has no parent.
	SDKReturnCode t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(0, 0, 0, 0, 0, "Hand"));
	if (t_Res != SDKReturnCode::SDKReturnCode_Success)
	{
		return false;
	}

	// then loop for 5 fingers
	int t_FingerId = 0;
	for (uint32_t i = 0; i < t_NumFingers; i++)
	{
		uint32_t t_ParentID = 0;
		// then the digits of the finger that are linked to the root of the finger.
		for (uint32_t j = 0; j < t_NumJoints; j++)
		{
			// Setup the handeness of the Manus Glove
			if (isRightHand) // Right Hand
				t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(1 + t_FingerId + j, t_ParentID, t_Fingers_Right[i * 4 + j].x, t_Fingers_Right[i * 4 + j].y, t_Fingers_Right[i * 4 + j].z, "fingerdigit"));
			else // Left Hand
				t_Res = CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(1 + t_FingerId + j, t_ParentID, t_Fingers_Left[i * 4 + j].x, t_Fingers_Left[i * 4 + j].y, t_Fingers_Left[i * 4 + j].z, "fingerdigit"));
			if (t_Res != SDKReturnCode::SDKReturnCode_Success)
			{
				printf("Failed to Add Node To Skeleton Setup. The error given %d.", t_Res);
				return false;
			}
			t_ParentID = 1 + t_FingerId + j;
		}
		t_FingerId += t_NumJoints;
	}
	return true;
}

/// @brief This function sets up some basic hand chains.
/// Chains are required for a Skeleton to be able to be animated, it basically tells Manus Core
/// which nodes belong to which body part and what data needs to be applied to which node.
/// @param p_SklIndex The index of the temporary skeleton on which the chains will be added.
/// @return Returns true if everything went fine, otherwise returns false.
bool SDKMinimalClient::SetupHandChains(uint32_t p_SklIndex, bool isRightHand)
{
	// Add the Hand chain, this identifies the wrist of the hand
	{
		ChainSettings t_ChainSettings;
		ChainSettings_Init(&t_ChainSettings);
		t_ChainSettings.usedSettings = ChainType::ChainType_Hand;
		t_ChainSettings.hand.handMotion = HandMotion::HandMotion_IMU;
		t_ChainSettings.hand.fingerChainIdsUsed = 5; // We have 5 fingers
		t_ChainSettings.hand.fingerChainIds[0] = 1;	 // Links to the other chains we will define further down
		t_ChainSettings.hand.fingerChainIds[1] = 2;
		t_ChainSettings.hand.fingerChainIds[2] = 3;
		t_ChainSettings.hand.fingerChainIds[3] = 4;
		t_ChainSettings.hand.fingerChainIds[4] = 5;

		ChainSetup t_Chain;
		ChainSetup_Init(&t_Chain);
		t_Chain.id = 0; // Every ID needs to be unique per chain in a skeleton.
		t_Chain.type = ChainType::ChainType_Hand;
		t_Chain.dataType = ChainType::ChainType_Hand;
		t_Chain.side = isRightHand ? Side::Side_Right : Side::Side_Left; // Set the proper hand side
		t_Chain.dataIndex = 0;
		t_Chain.nodeIdCount = 1;
		t_Chain.nodeIds[0] = 0; // This links to the hand node created in the SetupHandNodes
		t_Chain.settings = t_ChainSettings;

		SDKReturnCode t_Res = CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain);
		if (t_Res != SDKReturnCode::SDKReturnCode_Success)
		{
			return false;
		}
	}

	// Add the 5 finger chains
	const ChainType t_FingerTypes[5] = {ChainType::ChainType_FingerThumb,
										ChainType::ChainType_FingerIndex,
										ChainType::ChainType_FingerMiddle,
										ChainType::ChainType_FingerRing,
										ChainType::ChainType_FingerPinky};
	for (int i = 0; i < 5; i++)
	{
		ChainSettings t_ChainSettings;
		ChainSettings_Init(&t_ChainSettings);
		t_ChainSettings.usedSettings = t_FingerTypes[i];
		t_ChainSettings.finger.handChainId = 0; // This links to the wrist chain above.

		// This identifies the metacarpal bone, if none exists, or the chain is a thumb it should be set to -1.
		// The metacarpal bone should not be part of the finger chain, unless you are defining a thumb which does need it.
		t_ChainSettings.finger.metacarpalBoneId = -1;
		t_ChainSettings.finger.useLeafAtEnd = false; // This is set to true if there is a leaf bone to the tip of the finger.
		ChainSetup t_Chain;
		ChainSetup_Init(&t_Chain);
		t_Chain.id = i + 1; // Every ID needs to be unique per chain in a skeleton.
		t_Chain.type = t_FingerTypes[i];
		t_Chain.dataType = t_FingerTypes[i];
		t_Chain.side = isRightHand ? Side::Side_Right : Side::Side_Left; // Set the proper hand side
		t_Chain.dataIndex = 0;
		if (i == 0) // Thumb
		{
			t_Chain.nodeIdCount = 4; // The amount of node id's used in the array
			t_Chain.nodeIds[0] = 1;	 // this links to the hand node created in the SetupHandNodes
			t_Chain.nodeIds[1] = 2;	 // this links to the hand node created in the SetupHandNodes
			t_Chain.nodeIds[2] = 3;	 // this links to the hand node created in the SetupHandNodes
			t_Chain.nodeIds[3] = 4;	 // this links to the hand node created in the SetupHandNodes
		}
		else // All other fingers
		{
			t_Chain.nodeIdCount = 4;		  // The amount of node id's used in the array
			t_Chain.nodeIds[0] = (i * 4) + 1; // this links to the hand node created in the SetupHandNodes
			t_Chain.nodeIds[1] = (i * 4) + 2; // this links to the hand node created in the SetupHandNodes
			t_Chain.nodeIds[2] = (i * 4) + 3; // this links to the hand node created in the SetupHandNodes
			t_Chain.nodeIds[3] = (i * 4) + 4; // this links to the hand node created in the SetupHandNodes
		}
		t_Chain.settings = t_ChainSettings;

		SDKReturnCode t_Res = CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain);
		if (t_Res != SDKReturnCode::SDKReturnCode_Success)
		{
			return false;
		}
	}
	return true;
}

/// @brief This gets called when the client is connected to manus core
/// @param p_SkeletonStreamInfo contains the meta data on how much data regarding the skeleton we need to get from the SDK.
void SDKMinimalClient::OnSkeletonStreamCallback(const SkeletonStreamInfo *const p_SkeletonStreamInfo)
{
	if (s_Instance)
	{
		ClientSkeletonCollection *t_NxtClientSkeleton = new ClientSkeletonCollection();
		t_NxtClientSkeleton->skeletons.resize(p_SkeletonStreamInfo->skeletonsCount);

		for (uint32_t i = 0; i < p_SkeletonStreamInfo->skeletonsCount; i++)
		{
			CoreSdk_GetSkeletonInfo(i, &t_NxtClientSkeleton->skeletons[i].info);
			t_NxtClientSkeleton->skeletons[i].nodes = new SkeletonNode[t_NxtClientSkeleton->skeletons[i].info.nodesCount];
			CoreSdk_GetSkeletonData(i, t_NxtClientSkeleton->skeletons[i].nodes, t_NxtClientSkeleton->skeletons[i].info.nodesCount);
		}
		s_Instance->m_SkeletonMutex.lock();
		if (s_Instance->m_NextSkeleton != nullptr)
			delete s_Instance->m_NextSkeleton;
		s_Instance->m_NextSkeleton = t_NxtClientSkeleton;
		s_Instance->m_SkeletonMutex.unlock();
	}
}