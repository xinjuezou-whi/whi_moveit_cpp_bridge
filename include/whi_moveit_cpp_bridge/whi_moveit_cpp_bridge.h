/******************************************************************
MoveItCpp bridge to handle moveit commands under ROS 2

Features:
- advertise command service
- xxx

Dependencies:
- whi_interfaces::WhiTcpPose
- whi_interfaces::WhiSrvTcpPos
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-08-03: Initial version
2026-04-23: Migrate to ROS 2
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <whi_interfaces/srv/whi_srv_tcp_pose.hpp>
#include <whi_interfaces/srv/whi_srv_joint_pose.hpp>
#include <whi_interfaces/srv/whi_srv_joint_names.hpp>
#include <whi_interfaces/srv/whi_srv_tcp_difference.hpp>
#include <whi_interfaces/srv/whi_srv_current_tcp_pose.hpp>
#include <whi_interfaces/msg/whi_motion_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <atomic>

namespace whi_moveit_cpp_bridge
{
	class MoveItCppBridge
	{
    public:
        MoveItCppBridge(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~MoveItCppBridge();

    public:
        void initMoveitCpp();

    protected:
        bool isFakeHardware() const;
        void init();
        bool preExecution() const;
        bool execute(const whi_interfaces::msg::WhiTcpPose& Pose);
        bool execute(const whi_interfaces::msg::WhiJointPose& Pose);
        void callbackTcpPose(const whi_interfaces::msg::WhiTcpPose::SharedPtr Msg);
        void callbackJointPose(const whi_interfaces::msg::WhiJointPose::SharedPtr Msg);
        void callbackArmMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg);
        void callbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg);
        void callbackSwEstop(const std_msgs::msg::Bool::SharedPtr Msg);
        void onServiceTcpPose(const std::shared_ptr<whi_interfaces::srv::WhiSrvTcpPose::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvTcpPose::Response> Response);
        void onServiceJointPose(const std::shared_ptr<whi_interfaces::srv::WhiSrvJointPose::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvJointPose::Response> Response);
        void onServiceJointNames(const std::shared_ptr<whi_interfaces::srv::WhiSrvJointNames::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvJointNames::Response> Response);
        void onServiceTcpDifference(const std::shared_ptr<whi_interfaces::srv::WhiSrvTcpDifference::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvTcpDifference::Response> Response);
        void onServiceCurrentTcpPose(const std::shared_ptr<whi_interfaces::srv::WhiSrvCurrentTcpPose::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvCurrentTcpPose::Response> Response);
        void onServiceAbort(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> Response);
        bool trans2TargetFrame(const std::string& DstFrame,
            const geometry_msgs::msg::PoseStamped& PoseIn, geometry_msgs::msg::PoseStamped& PoseOut);
        void loadInitPlanParams();
        bool checkPlanned(const moveit::core::RobotState& CurrentState,
            const moveit::core::RobotState& LastPlannedWaypointState);
        void executeInitPoseGroup();

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_{ nullptr };
        std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_{ nullptr };
        std::string planning_group_{ "whi_arm" };
        moveit::core::RobotModelConstPtr robot_model_{ nullptr };
        const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
        rclcpp::Subscription<whi_interfaces::msg::WhiTcpPose>::SharedPtr target_tcp_sub_{ nullptr };
        rclcpp::Subscription<whi_interfaces::msg::WhiJointPose>::SharedPtr target_joint_sub_{ nullptr };
        rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr arm_state_sub_{ nullptr };
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_{ nullptr };
        rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr motion_state_sub_{ nullptr };
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvTcpPose>::SharedPtr target_tcp_srv_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvJointPose>::SharedPtr target_joint_srv_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvJointNames>::SharedPtr joint_names_srv_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvTcpDifference>::SharedPtr tcp_difference_srv_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvCurrentTcpPose>::SharedPtr current_tcp_pose_srv_{ nullptr };
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_{ nullptr };
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_arm_ready_{ nullptr };
        moveit_cpp::PlanningComponent::PlanRequestParameters init_plan_parameters_;
        std::atomic_bool is_arm_fault_{ false };
        std::string tf_prefix_;
        double wait_duration_{ 1.0 };
        int max_try_count_{ 10 };
        double cartesian_fraction_{ 1.0 };
        double cartesian_traj_max_step_{ 0.01 };
        std::vector<double> cartesian_precision_{ 0.01, 0.01 };
        bool estopped_{ false };
        bool sw_estopped_{ false };
        std::atomic_bool executing_{ false };
        std::map<std::string, double> init_pose_groups_;
        std::map<std::string, int> link_index_map_;
        rclcpp::CallbackGroup::SharedPtr async_callback_group_;
	};
} // namespace whi_moveit_cpp_bridge
