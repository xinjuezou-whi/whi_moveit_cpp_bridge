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
#include "whi_interfaces/WhiSrvTcpPose.h"
#include "whi_interfaces/WhiMotionState.h"

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
        bool execute(const whi_interfaces::WhiTcpPose& Pose);
        void callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg);
        void callbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& Msg);
        bool onServiceTcpPose(whi_interfaces::WhiSrvTcpPose::Request& Req,
            whi_interfaces::WhiSrvTcpPose::Response& Res);
        bool trans2TargetFrame(const std::string& DstFrame,
            const geometry_msgs::PoseStamped& PoseIn, geometry_msgs::PoseStamped& PoseOut);
        void loadInitPlanParams();
        bool checkPlanned(const moveit::core::RobotState& CurrentState,
            const moveit::core::RobotState& LastPlannedWaypointState);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::shared_ptr<ros::NodeHandle> node_handle_ns_free_{ nullptr };
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_{ nullptr };
        std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_{ nullptr };
        moveit::core::RobotModelConstPtr robot_model_{ nullptr };
        const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
        std::unique_ptr<ros::Subscriber> target_sub_{ nullptr };
        std::unique_ptr<ros::ServiceServer> target_srv_{ nullptr };
        std::unique_ptr<ros::Subscriber> motion_state_sub_{ nullptr };
        moveit_cpp::PlanningComponent::PlanRequestParameters init_plan_parameters_;
        int max_ik_try_cout_{ 3 };
        int motion_state_{ whi_interfaces::WhiMotionState::STA_STANDBY };
        std::unique_ptr<ros::ServiceClient> client_arm_ready_{ nullptr };
        std::string tf_prefix_;
        double wait_duration_{ 1.0 };
	};
} // namespace whi_moveit_cpp_bridge
