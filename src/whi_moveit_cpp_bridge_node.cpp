/******************************************************************
node to bridge moveitcpp commands

Features:
- moveitcpp api
- message and service of plan and execute
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-08-03: Initial version
2022-xx-xx: xxx
******************************************************************/
#include "whi_moveit_cpp_bridge/whi_moveit_cpp_bridge.h"

#include <ros/ros.h>

#include <iostream>
#include <signal.h>
#include <functional>

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI MoveItCpp bridge VERSION 00.12.5" << std::endl;
	std::cout << "Copyright © 2023-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    const std::string nodeName("whi_moveit_cpp_bridge"); 
	ros::init(argc, argv, nodeName);
	auto nodeHandle = std::make_shared<ros::NodeHandle>(nodeName);

	/// node logic
	auto instance = std::make_unique<whi_moveit_cpp_bridge::MoveItCppBridge>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		instance = nullptr;

		// all the default sigint handler does is call shutdown()
		ros::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
#else
	ros::MultiThreadedSpinner spinner(0);
	spinner.spin();
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
