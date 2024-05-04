# manus_ros2
ROS 2 Node for Publishing Manus VR Glove Messages to ROS 2

## Background and Objectives
This package is intended to be a simple ROS 2 Node that is able to read data from MANUS VR Gloves and the MANUS SDK and to publish the data as ROS messages for use as a robot controller. It is largely based on the SDK_MinimalClient_Linux project provided by MANUS.

We originally developed the package for use with the DexHand http://www.dexhand.org and http://www.dexhand.com. Please check out our projects if you are interested in Dexterous Humanoid Robot Hands.

That said, this package should work in a generic fashion for any projects wanting to interface Manus VR Gloves to ROS 2 for controls or other purposes. If you find a novel use for it, we'd love to hear what you are doing!

## Intended Platform and Limitations
On the DexHand project, we primarily use ROS 2 Humble as this is a LTS release of ROS 2. There shouldn't be any great difficulty in getting this package to compile against other versions of ROS. Also, note, we only build against Linux, and are using the MANUS SDK_MinimalClient_Linux project as the base. YMMV on other platforms, but if you do get them working, feel free to send us a PR to incorporate the changes.

## Authors and Credits
Michael Heilemann originally developed the code for this node, and Trent Shumay has contributed to it's distribution.

## Dependency on the MANUS SDK
Note that this project does not include the MANUS SDK itself - you must download this yourself with a developer account from https://www.manus-meta.com/resources/downloads. We are not affiliated with MANUS directly, nor do we publish or maintain the official SDK. 

Extract the ZIP into the /ext folder of this source tree. 

Properly installed, the MANUS SDK should be in:

/ext/MANUS_Core_2.3.X.X_SDK

**Note: At the time we published this project, we were linking against MANUS_Core_2.3.0.1_SDK**

**Also Note: If you haven't already configured the Linux SDK and dependencies on your machine, you should refer to the process described here https://www.manus-meta.com/knowledge-articles/software-core-2-3-linux-sdk**

## ROS 2 Messages and Node Functions
This node is super simple. It does two things:

1) Launches a minimal client to connect to Manus Core (either running on the same machine, or the same network) and begins receiving hand poses for the left and right gloves via the Manus SDK.
2) Re-broadcasts these poses as ROS2 PoseArray messages containing the positions and orientations reported by the Manus client.

These messages are as follows:

- `manus_left`: A ROS 2 PoseArray message containing positions and rotations for the left hand
- `manus_right`: A ROS 2 PoseArray message containing positions and rotations for the right hand

This data is provided verbatim in exactly the same order and format as received from the Manus client with no additional transforms or logic. If needed, you can modify your own fork of this node to do that, but we'd actually recommend just doing it in the subscriber to these messages so that you are not convoluting the data being reported from the Manus SDK. 

Currently, the node is on a 50hz timer, but you can experiment with increasing or decreasing this as needed.

