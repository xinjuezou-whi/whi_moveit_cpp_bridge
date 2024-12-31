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
#include <whi_interfaces/WhiSrvTcpPose.h>
#include <whi_interfaces/WhiSrvJointPose.h>
#include <whi_interfaces/WhiSrvJointNames.h>
#include <whi_interfaces/WhiSrvTcpDifference.h>
#include <whi_interfaces/WhiSrvCurrentTcpPose.h>
#include <whi_interfaces/WhiMotionState.h>

#include <ros/ros.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <std_msgs/Bool.h>

#include <memory>
#include <atomic>

namespace whi_moveit_cpp_bridge
{
	class MoveItCppBridge
	{
    public:
        MoveItCppBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~MoveItCppBridge();

    protected:
        void init();
        bool preExecution() const;
        bool execute(const whi_interfaces::WhiTcpPose& Pose);
        bool execute(const whi_interfaces::WhiJointPose& Pose);
        void callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg);
        void callbackJointPose(const whi_interfaces::WhiJointPose::ConstPtr& Msg);
        void callbackArmMotionState(const whi_interfaces::WhiMotionState::ConstPtr& Msg);
        void callbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& Msg);
        void callbackEstop(const std_msgs::Bool::ConstPtr& Msg);
        bool onServiceTcpPose(whi_interfaces::WhiSrvTcpPose::Request& Req,
            whi_interfaces::WhiSrvTcpPose::Response& Res);
        bool onServiceJointPose(whi_interfaces::WhiSrvJointPose::Request& Req,
            whi_interfaces::WhiSrvJointPose::Response& Res);
        bool onServiceJointNames(whi_interfaces::WhiSrvJointNames::Request& Req,
            whi_interfaces::WhiSrvJointNames::Response& Res);
        bool onServiceTcpDifference(whi_interfaces::WhiSrvTcpDifference::Request& Req,
            whi_interfaces::WhiSrvTcpDifference::Response& Res);
        bool onServiceCurrentTcpPose(whi_interfaces::WhiSrvCurrentTcpPose::Request& Req,
            whi_interfaces::WhiSrvCurrentTcpPose::Response& Res);
        bool trans2TargetFrame(const std::string& DstFrame,
            const geometry_msgs::PoseStamped& PoseIn, geometry_msgs::PoseStamped& PoseOut);
        void loadInitPlanParams();
        bool checkPlanned(const moveit::core::RobotState& CurrentState,
            const moveit::core::RobotState& LastPlannedWaypointState);
        void executeInitPoseGroup();

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::shared_ptr<ros::NodeHandle> node_handle_ns_free_{ nullptr };
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_{ nullptr };
        std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_{ nullptr };
        std::string planning_group_{ "whi_arm" };
        moveit::core::RobotModelConstPtr robot_model_{ nullptr };
        const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
        std::unique_ptr<ros::Subscriber> target_tcp_sub_{ nullptr };
        std::unique_ptr<ros::Subscriber> target_joint_sub_{ nullptr };
        std::unique_ptr<ros::ServiceServer> target_tcp_srv_{ nullptr };
        std::unique_ptr<ros::ServiceServer> target_joint_srv_{ nullptr };
        std::unique_ptr<ros::ServiceServer> joint_names_srv_{ nullptr };
        std::unique_ptr<ros::ServiceServer> tcp_difference_srv_{ nullptr };
        std::unique_ptr<ros::ServiceServer> current_tcp_pose_srv_{ nullptr };
        std::unique_ptr<ros::Subscriber> arm_state_sub_{ nullptr };
        std::unique_ptr<ros::Subscriber> estop_sub_{ nullptr };
        std::unique_ptr<ros::Subscriber> motion_state_sub_{ nullptr };
        std::unique_ptr<ros::Publisher> state_pub_{ nullptr };
        moveit_cpp::PlanningComponent::PlanRequestParameters init_plan_parameters_;
        std::atomic_bool is_arm_fault_{ false };
        std::unique_ptr<ros::ServiceClient> client_arm_ready_{ nullptr };
        std::string tf_prefix_;
        double wait_duration_{ 1.0 };
        int max_try_count_{ 10 };
        double cartesian_fraction_{ 1.0 };
        double cartesian_traj_max_step_{ 0.01 };
        std::vector<double> cartesian_precision_{ 0.01, 0.01 };
        std::string eef_link_{ "eef" };
        bool estopped_{ false };
        std::atomic_bool executing_{ false };
        std::map<std::string, double> init_pose_groups_;
        std::map<std::string, int> link_index_map_;
	};
} // namespace whi_moveit_cpp_bridge
