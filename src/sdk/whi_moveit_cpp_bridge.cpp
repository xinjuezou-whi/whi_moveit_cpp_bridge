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
        // providing the tcp pose service
        target_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("tcp_pose", &MoveItCppBridge::onServiceTcpPose, this));
    }

    bool MoveItCppBridge::execute(const std::string& PoseGroup, const geometry_msgs::Pose& Pose)
    {
        if (PoseGroup.empty())
        {
            auto state = *(moveit_cpp_->getCurrentState());
            state.setFromIK(joint_model_group_, Pose);
            planning_components_->setStartState(state);
        }
        else
        {
            planning_components_->setGoal(PoseGroup);
        }
        auto solution = planning_components_->plan();
        if (solution)
        {
            return planning_components_->execute();
        }
        else
        {
            return false;
        }
    }

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg)
    {
        execute(Msg->pose_group, Msg->tcp_pose.pose);
    }

    bool MoveItCppBridge::onServiceTcpPose(whi_interfaces::WhiSrvTcpPose::Request& Req,
        whi_interfaces::WhiSrvTcpPose::Response& Res)
    {
        Res.result = execute(Req.pose_group, Req.tcp_pose.pose);

        return Res.result;
    }
} // namespace whi_moveit_cpp_bridge
