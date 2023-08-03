/******************************************************************
MoveItCpp bridge to handle moveit commands under ROS 1

Features:
- advertise command service
- xxx

Dependencies:
- whi_interfaces::WhiTcpPose
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_moveit_cpp_bridge/whi_moveit_cpp_bridge.h"

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
        node_handle_->param("whi_moveit_cpp_bridge/planning_group", planningGroup, std::string("chin_arm"));

        try
        {
            moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(*node_handle_);
            if (moveit_cpp_)
            {
                moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
                planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planningGroup, moveit_cpp_);
                robot_model_ = moveit_cpp_->getRobotModel();
                joint_model_group_ = robot_model_->getJointModelGroup(planningGroup);
            }
        }
        catch (const std::exception& e)
        {
            ROS_FATAL_STREAM("failed to init moveitcpp instance: " << e.what());
            return;
        }

        // subscriber
        target_sub_ = std::make_unique<ros::Subscriber>(
			node_handle_->subscribe<whi_interfaces::WhiTcpPose>("tcp_pose", 10,
			std::bind(&MoveItCppBridge::callbackTcpPose, this, std::placeholders::_1)));
    }

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg)
    {
        if (Msg->pose_group.empty())
        {
            auto state = *(moveit_cpp_->getCurrentState());
            state.setFromIK(joint_model_group_, Msg->tcp_pose.pose);
            planning_components_->setStartState(state);
        }
        else
        {
            planning_components_->setGoal(Msg->pose_group);
        }
        auto solution = planning_components_->plan();
        if (solution)
        {
            planning_components_->execute();
            std::cout << "succeed" << std::endl;
        }
    }
} // namespace whi_moveit_cpp_bridge
