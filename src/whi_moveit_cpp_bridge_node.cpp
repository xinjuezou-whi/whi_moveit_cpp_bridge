/******************************************************************
node to bridge moveitcpp commands

Features:
- moveitcpp api
- message and service of plan and execute
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-08-03: Initial version
2026-04-24: Migrate to ROS 2
2026-xx-xx: xxx
******************************************************************/
#include "whi_moveit_cpp_bridge/whi_moveit_cpp_bridge.h"

#include <rclcpp/rclcpp.hpp>

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
	std::cout << "\nWHI MoveItCpp bridge VERSION 02.13.4" << std::endl;
	std::cout << "Copyright © 2023-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    const std::string nodeName("whi_moveit_cpp_bridge");

	rclcpp::init(argc, argv);
	rclcpp::NodeOptions nodeOptions;
	nodeOptions.automatically_declare_parameters_from_overrides(true);
	auto nodeHandle = rclcpp::Node::make_shared(nodeName, "", nodeOptions);

	/// node logic
	auto instance = std::make_unique<whi_moveit_cpp_bridge::MoveItCppBridge>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		instance = nullptr;

		// all the default sigint handler does is call shutdown()
		rclcpp::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(nodeHandle);
	std::thread spinThread(
		[&]()
		{
			executor->spin();
		}
	);
#else
	std::thread spinThread(
		[&]()
		{
			rclcpp::spin(nodeHandle);
		}
	);
#endif

	// give time for subscriptions start
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	instance->initMoveitCpp();

    // keep main thread alive
    spinThread.join();

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
