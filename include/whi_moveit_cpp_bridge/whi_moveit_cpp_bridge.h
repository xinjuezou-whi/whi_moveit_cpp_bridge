/******************************************************************
MoveItCpp bridge to handle moveit commands under ROS 1

Features:
- advertise command service
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-08-03: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include <memory>

namespace whi_moveit_cpp_bridge
{
	class MoveItCppBridge
	{
    public:
        MoveItCppBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~MoveItCppBridge() = default;

    protected:
        void init();
        void update(const ros::TimerEvent & Event);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
	};
} // namespace whi_moveit_cpp_bridge
