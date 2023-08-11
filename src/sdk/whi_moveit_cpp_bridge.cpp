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
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);

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

    bool MoveItCppBridge::execute(const std::string& PoseGroup, const geometry_msgs::PoseStamped& Pose)
    {
        if (PoseGroup.empty())
        {
            auto state = *(moveit_cpp_->getCurrentState());

            std::string armRoot = robot_model_->getRootLinkName();
            if (Pose.header.frame_id == armRoot || Pose.header.frame_id == "world" || Pose.header.frame_id.empty())
            {
                state.setFromIK(joint_model_group_, Pose.pose);
            }
            else
            {
                geometry_msgs::PoseStamped transformedPose;
                if (trans2TargetFrame(armRoot, Pose, transformedPose))
                {
                    state.setFromIK(joint_model_group_, transformedPose.pose);
                }                
            }

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
        execute(Msg->pose_group, Msg->tcp_pose);
    }

    bool MoveItCppBridge::onServiceTcpPose(whi_interfaces::WhiSrvTcpPose::Request& Req,
        whi_interfaces::WhiSrvTcpPose::Response& Res)
    {
        Res.result = execute(Req.pose_group, Req.tcp_pose);

        return Res.result;
    }

    bool MoveItCppBridge::trans2TargetFrame(const std::string& DstFrame,
        const geometry_msgs::PoseStamped& PoseIn, geometry_msgs::PoseStamped& PoseOut)
    {
        try
        {
            PoseOut = buffer_.transform(PoseIn, DstFrame, ros::Duration(0.0));
            return true;
        }
        catch (tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
    }
} // namespace whi_moveit_cpp_bridge
