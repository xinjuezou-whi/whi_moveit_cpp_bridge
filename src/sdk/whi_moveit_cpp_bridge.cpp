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
#include <std_srvs/Trigger.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <std_msgs/Bool.h>

#include <thread>
#include <iterator>

namespace whi_moveit_cpp_bridge
{
    MoveItCppBridge::MoveItCppBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    void MoveItCppBridge::init()
    {
        // check if controller is fake
        bool isFake = false;
        XmlRpc::XmlRpcValue controllerList;
        node_handle_->getParam("controller_list", controllerList);
        for (int i = 0; i < controllerList.size(); ++i)
        {
            if (static_cast<std::string>(controllerList[i]["name"]).find("fake") != std::string::npos)
            {
                isFake = true;
                break;
            }
        }
        state_pub_ = std::make_unique<ros::Publisher>(
        	node_handle_->advertise<std_msgs::Bool>("moveit_cpp_state", 10));

        node_handle_ns_free_ = std::make_shared<ros::NodeHandle>();
        // initiate arm ready service client if not fake
        if (!isFake)
        {
            node_handle_->param("wait_duration", wait_duration_, 1.0);
            node_handle_->param("max_try_count", max_try_count_, 10);
            std::string serviceReady;
            node_handle_->param("arm_ready_service", serviceReady, std::string("arm_ready"));
            if (!serviceReady.empty())
            {
                client_arm_ready_ = std::make_unique<ros::ServiceClient>(
                    node_handle_ns_free_->serviceClient<std_srvs::Trigger>(serviceReady));
            }
            // wait for service active
            while (!client_arm_ready_->waitForExistence(ros::Duration(wait_duration_)))
            {
                ROS_WARN_STREAM("wait for arm service...");
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
            }
            // wait for arm ready
            std_srvs::Trigger srv;
            while (!client_arm_ready_->call(srv))
            {
                ROS_WARN_STREAM("wait for arm ready...");
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
            }
        }

        // other params
        node_handle_->param("tf_prefix", tf_prefix_, std::string(""));
        node_handle_->param("planning_group", planning_group_, std::string("whi_arm"));
        loadInitPlanParams();
        node_handle_->param("cartesian_fraction", cartesian_fraction_, 1.0);
        node_handle_->param("cartesian_traj_max_step", cartesian_traj_max_step_, 0.01);
        node_handle_->param("eef_link", eef_link_, std::string("eef"));

        try
        {
            moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(*node_handle_);
            if (moveit_cpp_)
            {
                moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
                planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_);
                robot_model_ = moveit_cpp_->getRobotModel();
                joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
                planning_components_->setStartStateToCurrentState();
            }
        }
        catch (const std::exception& e)
        {
            ROS_FATAL_STREAM("failed to init moveitcpp instance: " << e.what());
            return;
        }

        // providing the tcp_pose service
        std::string poseAction("tcp_pose");
        target_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService(poseAction, &MoveItCppBridge::onServiceTcpPose, this));
        target_sub_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiTcpPose>(poseAction, 10,
            std::bind(&MoveItCppBridge::callbackTcpPose, this, std::placeholders::_1)));

        // subscribe to arm motion state
        std::string stateTopic;
        node_handle_->param("arm_state_topic", stateTopic, std::string("arm_motion_state"));
        arm_state_sub_ = std::make_unique<ros::Subscriber>(
		    node_handle_ns_free_->subscribe<whi_interfaces::WhiMotionState>(stateTopic, 10,
		    std::bind(&MoveItCppBridge::callbackArmMotionState, this, std::placeholders::_1)));
        // subscribe estop topic
        std::string estopTopic;
        node_handle_->param("estop_topic", estopTopic, std::string("estop"));
        estop_sub_ = std::make_unique<ros::Subscriber>(
		    node_handle_ns_free_->subscribe<std_msgs::Bool>(estopTopic, 10,
		    std::bind(&MoveItCppBridge::callbackEstop, this, std::placeholders::_1)));
        // subscribe motion state topic
        std::string motionStateTopic;
        node_handle_->param("motion_state_topic", motionStateTopic, std::string("motion_state"));
        motion_state_sub_ = std::make_unique<ros::Subscriber>(
		    node_handle_ns_free_->subscribe<whi_interfaces::WhiMotionState>(motionStateTopic, 10,
		    std::bind(&MoveItCppBridge::callbackMotionState, this, std::placeholders::_1)));

        // publish state for notifying nodes that depend on me
        std_msgs::Bool msg;
        msg.data = true;
        state_pub_->publish(msg);
    }

    bool MoveItCppBridge::execute(const whi_interfaces::WhiTcpPose& Pose)
    {
        if (estopped_)
        {
            ROS_WARN_STREAM("failed to execute TCP pose, EStop is active");

            return false;
        }

        int tryCount = 0;
        std_srvs::Trigger srv;
        while (client_arm_ready_ && !client_arm_ready_->call(srv))
        {
            if (++tryCount > max_try_count_)
            {
                ROS_ERROR_STREAM("failed to execute TCP pose, arm is not ready");

                return false;
            }
            else
            {
                ROS_WARN_STREAM("wait for arm ready... in " << max_try_count_ << " seconds");
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
            }
        }

        auto startState = moveit_cpp_->getCurrentState();
        planning_components_->setStartStateToCurrentState();

        bool foundIk = false;
        if (Pose.pose_group.empty())
        {
            geometry_msgs::PoseStamped targetPose = Pose.tcp_pose;
            std::string armRoot(tf_prefix_.empty() ? "" : tf_prefix_ + "/");
            armRoot += robot_model_->getRootLinkName();
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

            if (Pose.is_cartesian)
            {
                // convert from geometry_msgs::Pose to Eigen::Isometry3d
                Eigen::Isometry3d target;
                tf2::fromMsg(targetPose.pose, target);

                // compute the Cartesian path
                const moveit::core::LinkModel* linkModel = joint_model_group_->getLinkModel(eef_link_);
                if (linkModel != nullptr)
                {
                    std::vector<moveit::core::RobotStatePtr> trajState;
                    tryCount = 0;
                    double fraction = 0.0;
                    do
                    {
                        fraction = startState->computeCartesianPath(joint_model_group_, trajState, linkModel, target,
                            true, cartesian_traj_max_step_, 0.0);
#ifndef DEBUG
                        std::cout << "Cartersian fraction " << fraction << ", trajectory size " <<
                            trajState.size() << std::endl;
#endif
                    } while (++tryCount < max_try_count_ && fraction < cartesian_fraction_);

                    if (fraction - cartesian_fraction_ >= 0.0)
                    {
                        // get the robot_trajectory::RobotTrajectory from RobotStatePtr
                        robot_trajectory::RobotTrajectoryPtr traj = std::make_shared<robot_trajectory::RobotTrajectory>(
                            robot_model_, planning_group_);
                        for (const moveit::core::RobotStatePtr& it : trajState)
                        {
                            traj->addSuffixWayPoint(it, 0.0);
                        }
                        // apply the velocity and acceleration scale
                        trajectory_processing::IterativeParabolicTimeParameterization iptp;
                        if (iptp.computeTimeStamps(*traj, Pose.velocity_scale, Pose.acceleration_scale))
                        {
                            // execute path
                            bool res = moveit_cpp_->execute(planning_group_, traj);
                            if (is_arm_fault_.load())
                            {
                                is_arm_fault_.store(false);
                                res = false;

                                ROS_ERROR_STREAM("protective stop encountered");
                            }
                            return res;
                        }
                        else
                        {
                            ROS_WARN_STREAM("failed to apply time parameters");
                            return false;
                        }
                    }
                    else
                    {
                        ROS_WARN_STREAM("failed to find solution");
                        return false;
                    }
                }
                else
                {
                    ROS_WARN_STREAM("link " << eef_link_ << " doesn't exit, please check the config!");
                    return false;
                }
            }
            else
            {
                tryCount = 0;
                do
                {
                    foundIk = startState->setFromIK(joint_model_group_, targetPose.pose);
                } while (++tryCount < max_try_count_ && !foundIk);

                if (foundIk)
                {
                    // depending on the planning problem MoveIt chooses between
                    // ``joint space`` and ``cartesian space`` for problem representation.
                    // Setting the planner group parameter ``enforce_joint_model_state_space:true`` in
                    // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
                    //
                    // by default planning requests with orientation path constraints
                    // are sampled in ``cartesian space`` so that invoking IK serves as a
                    // generative sampler.
                    //
                    // by enforcing ``joint space`` the planning process will use rejection
                    // sampling to find valid requests. Please note that this might
                    // increase planning time considerably.
                    moveit_msgs::Constraints constraints;
                    // // first let the pose of contraint meet the target pose
                    // std::vector<double> currentJointPositions;
                    // startState->copyJointGroupPositions(joint_model_group_, currentJointPositions);
                    // auto jointsName = joint_model_group_->getJointModelNames();
                    constraints.joint_constraints = Pose.joint_constraints;
                    // for (auto& it : constraints.joint_constraints)
                    // {
                    //     auto found = std::find(jointsName.begin(), jointsName.end(), it.joint_name);
                    //     if (found != jointsName.end())
                    //     {
                    //         int index = std::distance(jointsName.begin(), found);
                    //         it.position = currentJointPositions[index];
                    //     }
                    // }
                    // // then set the constraints
                    planning_components_->setPathConstraints(constraints);

                    planning_components_->setGoal(*startState);
                }
                else
                {
                    ROS_ERROR_STREAM("failed to find the IK solution");
                }
            }
        }
        else
        {
            foundIk = true;
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
                if (is_arm_fault_.load())
                {
                    is_arm_fault_.store(false);
                    res = false;

                    ROS_ERROR_STREAM("protective stop encountered");
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
            ROS_WARN_STREAM("failed to find solution");
            return false;
        }
    }

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg)
    {
        execute(*Msg);
    }

    void MoveItCppBridge::callbackArmMotionState(const whi_interfaces::WhiMotionState::ConstPtr& Msg)
    {
        if (Msg->state == whi_interfaces::WhiMotionState::STA_FAULT)
        {
            is_arm_fault_.store(true);
        }
    }

    void MoveItCppBridge::callbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& Msg)
    {
        if (Msg->state == whi_interfaces::WhiMotionState::STA_ESTOP)
        {
            moveit_cpp_->getTrajectoryExecutionManagerNonConst()->stopExecution();
            estopped_ = true;
        }
        else if (Msg->state == whi_interfaces::WhiMotionState::STA_ESTOP_CLEAR)
        {
            estopped_ = false;
        }
    }

    void MoveItCppBridge::callbackEstop(const std_msgs::Bool::ConstPtr& Msg)
    {
        estopped_ = Msg->data;
        if (estopped_)
        {
            moveit_cpp_->getTrajectoryExecutionManagerNonConst()->stopExecution();
        }
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
