# manus_ros2
ROS 2 Node for Publishing Manus VR Glove Messages to ROS 2

## Background and Objectives
This package is intended to be a simple ROS 2 Node that is able to read data from MANUS VR Gloves and the MANUS SDK and to publish the data as ROS messages for use as a robot controller. It is largely based on the SDK_MinimalClient_Linux project provided by MANUS.

We originally developed the package for use with the DexHand http://www.dexhand.org and http://www.dexhand.com. Please check out our projects if you are interested in Dexterous Humanoid Robot Hands.

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
