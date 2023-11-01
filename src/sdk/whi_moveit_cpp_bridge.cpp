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

******************************************************************/
#include "whi_moveit_cpp_bridge/whi_moveit_cpp_bridge.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace whi_moveit_cpp_bridge
{
    MoveItCppBridge::MoveItCppBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    void MoveItCppBridge::init()
    {
        // params
        std::string planningGroup;
        node_handle_->param("planning_group", planningGroup, std::string("whi_arm"));
        loadInitPlanParams();
        node_handle_->param("max_ik_try_count", max_ik_try_cout_, 3);

        try
        {
            moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(*node_handle_);
            if (moveit_cpp_)
            {
                moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
                planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planningGroup, moveit_cpp_);
                robot_model_ = moveit_cpp_->getRobotModel();
                joint_model_group_ = robot_model_->getJointModelGroup(planningGroup);
                planning_components_->setStartStateToCurrentState();
            }
        }
        catch (const std::exception& e)
        {
            ROS_FATAL_STREAM("failed to init moveitcpp instance: " << e.what());
            return;
        }

        // providing the tcp pose service, and it will preempt publisher
        std::string service;
        node_handle_->param("tcp_pose_service", service, std::string());
        if (!service.empty())
        {
            target_srv_ = std::make_unique<ros::ServiceServer>(
                node_handle_->advertiseService(service, &MoveItCppBridge::onServiceTcpPose, this));
        }
        else
        {
            // subscriber
            std::string topic;
            node_handle_->param("tcp_pose_topic", topic, std::string(""));
            if (!topic.empty())
            {
                target_sub_ = std::make_unique<ros::Subscriber>(
			        node_handle_->subscribe<whi_interfaces::WhiTcpPose>(topic, 10,
			        std::bind(&MoveItCppBridge::callbackTcpPose, this, std::placeholders::_1)));
            }
        }

        // subscribe to arm motion state
        std::string stateTopic;
        node_handle_->param("motion_state_topic", stateTopic, std::string("arm_moton_state"));
        motion_state_sub_ = std::make_unique<ros::Subscriber>(
		    node_handle_->subscribe<whi_interfaces::WhiMotionState>(stateTopic, 10,
		    std::bind(&MoveItCppBridge::callbackMotionState, this, std::placeholders::_1)));
    }

    bool MoveItCppBridge::execute(const whi_interfaces::WhiTcpPose& Pose)
    {
        auto startState = moveit_cpp_->getCurrentState();
        planning_components_->setStartStateToCurrentState();

        bool foundIk = true;

        if (Pose.pose_group.empty())
        {
            auto state = *startState;

            geometry_msgs::PoseStamped targetPose = Pose.tcp_pose;
            std::string armRoot = robot_model_->getRootLinkName();
            if (Pose.tcp_pose.header.frame_id != armRoot &&
                Pose.tcp_pose.header.frame_id != "world" && !Pose.tcp_pose.header.frame_id.empty())
            {
                if (!trans2TargetFrame(armRoot, Pose.tcp_pose, targetPose))
                {
                    ROS_WARN_STREAM("failed to get pose transform");
                    return false;
                }
#ifdef DEBUG
                std::cout << "pose from msg x:" << Pose.tcp_pose.pose.position.x << ",y:" <<
                    Pose.tcp_pose.pose.position.y << ",z:" << Pose.tcp_pose.pose.position.z << std::endl;
                std::cout << "transformed pose from " << Pose.tcp_pose.header.frame_id << " to " << armRoot <<
                    " with pose x:" << targetPose.pose.position.x << ",y:" <<
                    targetPose.pose.position.y << ",z:" << targetPose.pose.position.z <<
                    "-orientation x:" << targetPose.pose.orientation.x <<
                    ",y:" << targetPose.pose.orientation.y <<
                    ",z:" << targetPose.pose.orientation.z <<
                    ",w:" << targetPose.pose.orientation.w << std::endl;
#endif
            }

            int tryCount = 0;
            do
            {
                foundIk = state.setFromIK(joint_model_group_, targetPose.pose);
                ++tryCount;
            } while (tryCount < max_ik_try_cout_ && !foundIk);

            if (foundIk)
            {
                planning_components_->setGoal(state);
            }
        }
        else
        {
            planning_components_->setGoal(Pose.pose_group);
        }

        if (foundIk)
        {
            moveit_cpp::PlanningComponent::PlanRequestParameters params = init_plan_parameters_;
            if (Pose.velocity_scale > 0.0)
            {
                params.max_velocity_scaling_factor = Pose.velocity_scale;
            }
            if (Pose.acceleration_scale > 0.0)
            {
                params.max_acceleration_scaling_factor = Pose.acceleration_scale;
            }
            auto solution = planning_components_->plan(params);
            if (solution)
            {
                bool res = planning_components_->execute();
                if (motion_state_ == whi_interfaces::WhiMotionState::STA_FAULT)
                {
                    motion_state_ = whi_interfaces::WhiMotionState::STA_STANDBY;
                    res = false;
                }
                return res;
            }
            else
            {
                ROS_WARN_STREAM("failed to find path solution");
                return false;
            }
        }
        else
        {
            ROS_WARN_STREAM("failed to find IK solution");
            return false;
        }
    }

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg)
    {
        execute(*Msg);
    }

    void MoveItCppBridge::callbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& Msg)
    {
        motion_state_ = Msg->state;
    }

    bool MoveItCppBridge::onServiceTcpPose(whi_interfaces::WhiSrvTcpPose::Request& Req,
        whi_interfaces::WhiSrvTcpPose::Response& Res)
    {
        Res.result = execute(Req.pose);

        return Res.result;
    }

    bool MoveItCppBridge::trans2TargetFrame(const std::string& DstFrame,
        const geometry_msgs::PoseStamped& PoseIn, geometry_msgs::PoseStamped& PoseOut)
    {
        try
        {
            PoseOut = moveit_cpp_->getTFBuffer()->transform(PoseIn, DstFrame, ros::Duration(0.0));
            return true;
        }
        catch (tf2::TransformException &e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
    }

    void MoveItCppBridge::loadInitPlanParams()
    {
        node_handle_->param("plan_request_params/planner_id", init_plan_parameters_.planner_id, std::string(""));
        node_handle_->param("plan_request_params/planning_pipeline",
            init_plan_parameters_.planning_pipeline, std::string(""));
        node_handle_->param("plan_request_params/planning_time", init_plan_parameters_.planning_time, 1.0);
        node_handle_->param("plan_request_params/planning_attempts", init_plan_parameters_.planning_attempts, 5);
        node_handle_->param("plan_request_params/max_velocity_scaling_factor",
            init_plan_parameters_.max_velocity_scaling_factor, 1.0);
        node_handle_->param("plan_request_params/max_acceleration_scaling_factor",
            init_plan_parameters_.max_acceleration_scaling_factor, 1.0);
#ifdef DEBUG
        std::cout << "request params:" << init_plan_parameters_.planner_id << ","
            << init_plan_parameters_.planning_pipeline<< ","
            << init_plan_parameters_.planning_time << "," << init_plan_parameters_.planning_attempts << ","
            << init_plan_parameters_.max_velocity_scaling_factor << ","
            << init_plan_parameters_.max_acceleration_scaling_factor << std::endl;
#endif
    }

    bool MoveItCppBridge::checkPlanned(const moveit::core::RobotState& CurrentState,
        const moveit::core::RobotState& LastPlannedWaypointState)
    {
        const auto current = CurrentState.getVariablePositions();
        const auto plannedLast = LastPlannedWaypointState.getVariablePositions();
        
        bool succeed = false;
        for (size_t i = 0; i < CurrentState.getVariableCount(); ++i)
        {
            succeed |= fabs(current[i] - plannedLast[i]) > 1e-3;
        }

        return succeed;
    }

} // namespace whi_moveit_cpp_bridge
