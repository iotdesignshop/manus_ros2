#ifndef _SDK_MINIMAL_CLIENT_HPP_
#define _SDK_MINIMAL_CLIENT_HPP_


// Set up a Doxygen group.
/** @addtogroup SDKMinimalClient
 *  @{
 */


#include "ClientPlatformSpecific.hpp"
#include "ManusSDK.h"
#include <mutex>
#include <vector>

/// @brief Values that can be returned by this application.
enum class ClientReturnCode : int
{
	ClientReturnCode_Success = 0,
	ClientReturnCode_FailedPlatformSpecificInitialization,
	ClientReturnCode_FailedToResizeWindow,
	ClientReturnCode_FailedToInitialize,
	ClientReturnCode_FailedToFindHosts,
	ClientReturnCode_FailedToConnect,
	ClientReturnCode_UnrecognizedStateEncountered,
	ClientReturnCode_FailedToShutDownSDK,
	ClientReturnCode_FailedPlatformSpecificShutdown,
	ClientReturnCode_FailedToRestart,
	ClientReturnCode_FailedWrongTimeToGetData,

	ClientReturnCode_MAX_CLIENT_RETURN_CODE_SIZE
};

/// @brief Used to store the information about the final animated skeletons.
class ClientSkeleton
{
public:
	SkeletonInfo info;
	SkeletonNode* nodes = nullptr;

	~ClientSkeleton()
	{
		if (nodes != nullptr)delete[] nodes;
	}
};

/// @brief Used to store all the final animated skeletons received from Core.
class ClientSkeletonCollection
{
public:
	std::vector<ClientSkeleton> skeletons;
};

class SDKMinimalClient : public SDKClientPlatformSpecific
{
public:
	SDKMinimalClient();
	~SDKMinimalClient();
	ClientReturnCode Initialize();
	ClientReturnCode InitializeSDK();
	void ConnectToHost();
	ClientReturnCode ShutDown();
	ClientReturnCode RegisterAllCallbacks();
    ClientReturnCode Update();
    void Run();

    static void OnConnectedCallback(const ManusHost* const p_Host);

	static void OnSkeletonStreamCallback(const SkeletonStreamInfo* const p_SkeletonStreamInfo);

protected:

	ClientReturnCode Connect();
	bool SetupHandNodes(uint32_t p_SklIndex, bool isRightHand);
	bool SetupHandNodesLeft(uint32_t p_SklIndex);
	bool SetupHandNodesRight(uint32_t p_SklIndex);
	bool SetupHandChains(uint32_t p_SklIndex, bool isRightHand);
	void LoadTestSkeleton();
	NodeSetup CreateNodeSetup(uint32_t p_Id, uint32_t p_ParentId, float p_PosX, float p_PosY, float p_PosZ, std::string p_Name);
	static ManusVec3 CreateManusVec3(float p_X, float p_Y, float p_Z);

	static SDKMinimalClient* s_Instance;
	bool m_Running = true;

	std::mutex m_SkeletonMutex;

	ClientSkeletonCollection* m_NextSkeleton = nullptr;
	ClientSkeletonCollection* m_Skeleton = nullptr;

	std::mutex m_LandscapeMutex;
	Landscape* m_NewLandscape = nullptr;
	Landscape* m_Landscape = nullptr;
	std::vector<GestureLandscapeData> m_NewGestureLandscapeData;
	std::vector<GestureLandscapeData> m_GestureLandscapeData;

	uint32_t m_FirstLeftGloveID = 0;
	uint32_t m_FirstRightGloveID = 0;

	uint32_t m_FrameCounter = 0;
};

// Close the Doxygen group.
/** @} */
#endif
