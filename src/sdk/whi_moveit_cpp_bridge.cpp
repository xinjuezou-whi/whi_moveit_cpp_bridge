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
    }

    bool MoveItCppBridge::execute(const whi_interfaces::WhiTcpPose& Pose)
    {
        auto startState = moveit_cpp_->getCurrentState();
        planning_components_->setStartStateToCurrentState();
        
        if (Pose.pose_group.empty())
        {
            auto state = *startState;

            std::string armRoot = robot_model_->getRootLinkName();
            if (Pose.tcp_pose.header.frame_id == armRoot ||
                Pose.tcp_pose.header.frame_id == "world" || Pose.tcp_pose.header.frame_id.empty())
            {
                state.setFromIK(joint_model_group_, Pose.tcp_pose.pose);
            }
            else
            {
                geometry_msgs::PoseStamped transformedPose;
                if (trans2TargetFrame(armRoot, Pose.tcp_pose, transformedPose))
                {
#ifdef DEBUG
                    std::cout << "pose from msg x:" << Pose.pose.position.x << ",y:" <<
                        Pose.pose.position.y << ",z:" << Pose.pose.position.z << std::endl;
                    std::cout << "transformed pose x:" << transformedPose.pose.position.x << ",y:" <<
                        transformedPose.pose.position.y << ",z:" << transformedPose.pose.position.z << std::endl;
#endif
                    state.setFromIK(joint_model_group_, transformedPose.pose);
                }
            }

            planning_components_->setGoal(state);
        }
        else
        {
            planning_components_->setGoal(Pose.pose_group);
        }
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
#ifndef DEBUG
            std::cout << "trajectory waypoints count " << solution.trajectory_->getWayPointCount() << std::endl;
#endif
            return planning_components_->execute();
        }
        else
        {
            ROS_WARN_STREAM("failed to find solution");
            return false;
        }
    }

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg)
    {
        execute(*Msg);
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
#ifndef DEBUG
        std::cout << "request params:" << init_plan_parameters_.planner_id << ","
            << init_plan_parameters_.planning_pipeline<< ","
            << init_plan_parameters_.planning_time << "," << init_plan_parameters_.planning_attempts << ","
            << init_plan_parameters_.max_velocity_scaling_factor << ","
            << init_plan_parameters_.max_acceleration_scaling_factor << std::endl;
#endif
    }
} // namespace whi_moveit_cpp_bridge
