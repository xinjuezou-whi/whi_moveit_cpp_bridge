/******************************************************************
MoveItCpp bridge to handle moveit commands under ROS 1

Features:
- advertise command service
- xxx

Dependencies:
- whi_interfaces::WhiTcpPose
- whi_interfaces::WhiSrvTcpPos
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-08-03: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/WhiTcpPose.h"
#include "whi_interfaces/WhiSrvTcpPose.h"

#include <ros/ros.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

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
        bool execute(const std::string& PoseGroup, const geometry_msgs::Pose& Pose);
        void callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg);
        bool onServiceTcpPose(whi_interfaces::WhiSrvTcpPose::Request& Req, whi_interfaces::WhiSrvTcpPose::Response& Res);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_{ nullptr };
        std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_{ nullptr };
        moveit::core::RobotModelConstPtr robot_model_{ nullptr };
        const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
        std::unique_ptr<ros::Subscriber> target_sub_{ nullptr };
        std::unique_ptr<ros::ServiceServer> target_srv_{ nullptr };
	};
} // namespace whi_moveit_cpp_bridge
