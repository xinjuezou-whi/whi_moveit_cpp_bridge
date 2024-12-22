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
#include <tf2_eigen/tf2_eigen.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/cartesian_interpolator.h> // comment if old moveitcore is required

#include <thread>
#include <iterator>

namespace whi_moveit_cpp_bridge
{
    MoveItCppBridge::MoveItCppBridge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    MoveItCppBridge::~MoveItCppBridge()
    {
        // execute init pose
        executeInitPoseGroup();
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
        node_handle_->getParam("cartesian_precision", cartesian_precision_);
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

        // providing the tcp_pose/joint_pose service
        std::string tcpAction("tcp_pose");
        target_tcp_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService(tcpAction, &MoveItCppBridge::onServiceTcpPose, this));
        target_tcp_sub_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiTcpPose>(tcpAction, 10,
            std::bind(&MoveItCppBridge::callbackTcpPose, this, std::placeholders::_1)));
        std::string jointAction("joint_pose");
        target_joint_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService(jointAction, &MoveItCppBridge::onServiceJointPose, this));
        target_joint_sub_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiJointPose>(jointAction, 10,
            std::bind(&MoveItCppBridge::callbackJointPose, this, std::placeholders::_1)));
        // providing joint model names service
        joint_names_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("joint_names", &MoveItCppBridge::onServiceJointNames, this));
        // advertise tcp offset service
        tcp_difference_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("tcp_difference", &MoveItCppBridge::onServiceTcpDifference, this));
        // advertise current tcp pose
        current_tcp_pose_srv_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("tcp_current", &MoveItCppBridge::onServiceCurrentTcpPose, this));

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

        // execute init pose
        node_handle_->getParam("init_pose_groups", init_pose_groups_);
        executeInitPoseGroup();
    }

    bool MoveItCppBridge::preExecution() const
    {
        if (estopped_)
        {
            ROS_WARN_STREAM("cannot execute pose action, EStop is active");
            return false;
        }
        if (executing_.load())
        {
            ROS_WARN_STREAM("there is motion executing");
            return false;
        }

        int tryCount = 0;
        std_srvs::Trigger srv;
        while (client_arm_ready_ && !client_arm_ready_->call(srv))
        {
            if (++tryCount > max_try_count_)
            {
                ROS_ERROR_STREAM("cannot execute pose action, arm is not ready");

                return false;
            }
            else
            {
                ROS_WARN_STREAM("wait for arm ready... in " << max_try_count_ << " seconds");
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
            }
        }

        return true;
    }

    bool MoveItCppBridge::execute(const whi_interfaces::WhiTcpPose& Pose)
    {
        if (!preExecution())
        {
            return false;
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
                    int tryCount = 0;
                    double fraction = 0.0;
                    do
                    {
                        // uncomment if old moveitcore is required
                        // fraction = startState->computeCartesianPath(joint_model_group_, trajState, linkModel, target,
                        //     true, cartesian_traj_max_step_, 0.0);
                        // comment if old moveitcore is required
                        fraction = moveit::core::CartesianInterpolator::computeCartesianPath(startState.get(),
                            joint_model_group_, trajState, linkModel, target, true,
                            moveit::core::MaxEEFStep(cartesian_traj_max_step_),
                            moveit::core::CartesianPrecision{ cartesian_precision_[0], cartesian_precision_[1] },
                            moveit::core::GroupStateValidityCallbackFn(), kinematics::KinematicsQueryOptions(),
                            Eigen::Isometry3d::Identity());
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
                int tryCount = 0;
                do
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

                    // set the constraints
                    moveit_msgs::Constraints jc;
                    jc.joint_constraints = Pose.joint_constraints;
                    planning_components_->setPathConstraints(jc);
                    // trajectory constraints has no effect so far
                    // moveit_msgs::TrajectoryConstraints constraints;
                    // constraints.constraints.push_back(jc);
                    // planning_components_->setTrajectoryConstraints(constraints);
                    foundIk = startState->setFromIK(joint_model_group_, targetPose.pose);
                } while (++tryCount < max_try_count_ && !foundIk);

                if (foundIk)
                {
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
            // set the constraints
            moveit_msgs::Constraints jc;
            jc.joint_constraints = Pose.joint_constraints;
            planning_components_->setPathConstraints(jc);
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
                executing_.store(true);
                bool res = planning_components_->execute();
                executing_.store(false);
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

    bool MoveItCppBridge::execute(const whi_interfaces::WhiJointPose& Pose)
    {
        if (!preExecution())
        {
            return false;
        }

        auto startState = moveit_cpp_->getCurrentState();
        planning_components_->setStartStateToCurrentState();

        if (Pose.is_relative)
        {
            std::vector<double> currentJointPositions;
            startState->copyJointGroupPositions(joint_model_group_, currentJointPositions);

            for (int i = 0; i < std::min(currentJointPositions.size(), Pose.joint_pose.position.size()); ++i)
            {
                currentJointPositions[i] += Pose.joint_pose.position[i];
            }
            startState->setJointGroupPositions(joint_model_group_, currentJointPositions);
        }
        else
        {
            startState->setJointGroupPositions(joint_model_group_, Pose.joint_pose.position);
        }

        moveit_msgs::Constraints constraints;
        constraints.joint_constraints = Pose.joint_constraints;
        planning_components_->setPathConstraints(constraints);
        planning_components_->setGoal(*startState);

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
            executing_.store(true);
            bool res = planning_components_->execute();
            executing_.store(false);
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

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::WhiTcpPose::ConstPtr& Msg)
    {
        execute(*Msg);
    }

    void MoveItCppBridge::callbackJointPose(const whi_interfaces::WhiJointPose::ConstPtr& Msg)
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

    bool MoveItCppBridge::onServiceJointPose(whi_interfaces::WhiSrvJointPose::Request& Req,
        whi_interfaces::WhiSrvJointPose::Response& Res)
    {
        Res.result = execute(Req.pose);

        return Res.result;
    }

    bool MoveItCppBridge::onServiceJointNames(whi_interfaces::WhiSrvJointNames::Request& Req,
        whi_interfaces::WhiSrvJointNames::Response& Res)
    {
        Res.joint_names = joint_model_group_->getJointModelNames();
        Res.result = Res.joint_names.empty() ? false : true;

        return Res.result;
    }

    bool MoveItCppBridge::onServiceTcpDifference(whi_interfaces::WhiSrvTcpDifference::Request& Req,
        whi_interfaces::WhiSrvTcpDifference::Response& Res)
    {
        auto state = moveit_cpp_->getCurrentState();
        geometry_msgs::Pose currentTcpPose = tf2::toMsg(state->getGlobalLinkTransform(eef_link_));
        tf2::Quaternion currentQ(currentTcpPose.orientation.x, currentTcpPose.orientation.y,
            currentTcpPose.orientation.z, currentTcpPose.orientation.w);

        if (!Req.pose_group.empty())
        {
            auto jointValues = planning_components_->getNamedTargetStateValues(Req.pose_group);
            if (!jointValues.empty())
            {
                std::vector<double> jointPositions;
                for (const auto& it : jointValues)
                {
                    jointPositions.push_back(it.second);
                }

                state->setJointGroupPositions(joint_model_group_, jointPositions);

                // forward kinematics
                auto transform = state->getGlobalLinkTransform(eef_link_);
                auto reference = tf2::toMsg(transform);
                tf2::Quaternion referenceQ(reference.orientation.x, reference.orientation.y,
                    reference.orientation.z, reference.orientation.w);

                Res.result = true;
                Res.difference.position.x = currentTcpPose.position.x - reference.position.x;
                Res.difference.position.y = currentTcpPose.position.y - reference.position.y;
                Res.difference.position.z = currentTcpPose.position.z - reference.position.z;
                Res.difference.orientation = tf2::toMsg(currentQ * referenceQ.inverse());
            }
            else
            {
                Res.result = false;
            }
        }
        else if (!Req.joint_pose.position.empty())
        {
            state->setJointGroupPositions(joint_model_group_, Req.joint_pose.position);

            // forward kinematics
            auto transform = state->getGlobalLinkTransform(eef_link_);
            auto reference = tf2::toMsg(transform);
            tf2::Quaternion referenceQ(reference.orientation.x, reference.orientation.y,
                reference.orientation.z, reference.orientation.w);

            Res.result = true;
            Res.difference.position.x = currentTcpPose.position.x - reference.position.x;
            Res.difference.position.y = currentTcpPose.position.y - reference.position.y;
            Res.difference.position.z = currentTcpPose.position.z - reference.position.z;
            Res.difference.orientation = tf2::toMsg(currentQ * referenceQ.inverse());
        }
        else
        {
            tf2::Quaternion referenceQ(Req.tcp_pose.pose.orientation.x, Req.tcp_pose.pose.orientation.y,
                Req.tcp_pose.pose.orientation.z, Req.tcp_pose.pose.orientation.w);

            Res.result = true;
            Res.difference.position.x = currentTcpPose.position.x - Req.tcp_pose.pose.position.x;
            Res.difference.position.y = currentTcpPose.position.y - Req.tcp_pose.pose.position.y;
            Res.difference.position.z = currentTcpPose.position.z - Req.tcp_pose.pose.position.z;
            Res.difference.orientation = tf2::toMsg(currentQ * referenceQ.inverse());
        }

        return Res.result;
    }

    bool MoveItCppBridge::onServiceCurrentTcpPose(whi_interfaces::WhiSrvCurrentTcpPose::Request& Req,
        whi_interfaces::WhiSrvCurrentTcpPose::Response& Res)
    {
        auto state = moveit_cpp_->getCurrentState();
        Res.pose = tf2::toMsg(state->getGlobalLinkTransform(eef_link_));
        Res.result = true;

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

    void MoveItCppBridge::executeInitPoseGroup()
    {
        for (const auto& it : init_pose_groups_)
        {
            whi_interfaces::WhiTcpPose poseGroup;
            poseGroup.pose_group = it;
            execute(poseGroup);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
} // namespace whi_moveit_cpp_bridge
