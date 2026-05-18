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

******************************************************************/
#include "whi_moveit_cpp_bridge/whi_moveit_cpp_bridge.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/robot_state/cartesian_interpolator.h> // comment if old moveitcore is required

#include <thread>
#include <iterator>

namespace whi_moveit_cpp_bridge
{
    MoveItCppBridge::MoveItCppBridge(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    MoveItCppBridge::~MoveItCppBridge()
    {
        // execute init pose
        executeInitPoseGroup();
    }

    void MoveItCppBridge::initMoveitCpp()
    {
        // check if controller is fake
        bool isFake = false;
        // XmlRpc::XmlRpcValue controllerList;
        // node_handle_->getParam("controller_list", controllerList);
        // for (int i = 0; i < controllerList.size(); ++i)
        // {
        //     if (static_cast<std::string>(controllerList[i]["name"]).find("fake") != std::string::npos)
        //     {
        //         isFake = true;
        //         break;
        //     }
        // }

        // initiate arm ready service client if not fake
        if (!isFake)
        {
            if (!node_handle_->has_parameter("wait_duration"))
            {
                node_handle_->declare_parameter("wait_duration", 1.0);
            }
            wait_duration_ = node_handle_->get_parameter("wait_duration").as_double();

            if (!node_handle_->has_parameter("max_try_count"))
            {
                node_handle_->declare_parameter("max_try_count", 10);
            }
            max_try_count_ = node_handle_->get_parameter("max_try_count").as_int();

            if (!node_handle_->has_parameter("arm_ready_service"))
            {
                node_handle_->declare_parameter("arm_ready_service", std::string("arm_ready"));
            }
            std::string serviceReady = node_handle_->get_parameter("arm_ready_service").as_string();
            // arm ready service client
            if (!serviceReady.empty())
            {
                client_arm_ready_ = node_handle_->create_client<std_srvs::srv::Trigger>(serviceReady);
            }

            // wait for service active
            while (!client_arm_ready_->wait_for_service(std::chrono::duration<double>(wait_duration_)))
            {
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), "wait for arm service...");
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
            }
            // wait for arm ready
            bool armReady = false;
            do
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                client_arm_ready_->async_send_request(
                    request,
                    [this, request, &armReady](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
                    {
                        if (future.get()->success)
                        {
                            armReady = true;
                        }
                        else
                        {
                            armReady = false;
                        }
                    });

                RCLCPP_WARN_STREAM(node_handle_->get_logger(), "wait for arm ready...");
                std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
            } while (!armReady);
        }

        try
        {
            moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_handle_);
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
            RCLCPP_FATAL_STREAM(node_handle_->get_logger(), "failed to init moveitcpp instance: " << e.what());
            return;
        }

        // providing the tcp_pose/joint_pose service
        std::string tcpAction("tcp_pose");
        target_tcp_srv_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvTcpPose>(tcpAction,
            std::bind(&MoveItCppBridge::onServiceTcpPose, this, std::placeholders::_1, std::placeholders::_2));
        target_tcp_sub_ = node_handle_->create_subscription<whi_interfaces::msg::WhiTcpPose>(
            tcpAction, 10, std::bind(&MoveItCppBridge::callbackTcpPose, this, std::placeholders::_1));

        std::string jointAction("joint_pose");
        target_joint_srv_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvJointPose>(jointAction,
            std::bind(&MoveItCppBridge::onServiceJointPose, this, std::placeholders::_1, std::placeholders::_2));
        target_joint_sub_ = node_handle_->create_subscription<whi_interfaces::msg::WhiJointPose>(
            jointAction, 10, std::bind(&MoveItCppBridge::callbackJointPose, this, std::placeholders::_1));

        // providing joint model names service
        joint_names_srv_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvJointNames>("joint_names",
            std::bind(&MoveItCppBridge::onServiceJointNames, this, std::placeholders::_1, std::placeholders::_2));

        // advertise tcp offset service
        tcp_difference_srv_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvTcpDifference>("tcp_difference",
            std::bind(&MoveItCppBridge::onServiceTcpDifference, this, std::placeholders::_1, std::placeholders::_2));

        // advertise current tcp pose
        current_tcp_pose_srv_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvCurrentTcpPose>("tcp_current",
            std::bind(&MoveItCppBridge::onServiceCurrentTcpPose, this, std::placeholders::_1, std::placeholders::_2));

        // execute init pose
        executeInitPoseGroup();

        state_pub_ = node_handle_->create_publisher<std_msgs::msg::Bool>("moveit_cpp_state", 10);
        // publish state for notifying nodes that depend on me
        std_msgs::msg::Bool msg;
        msg.data = true;
        state_pub_->publish(msg);
    }

    void MoveItCppBridge::init()
    {
        // other params
        if (!node_handle_->has_parameter("tf_prefix"))
        {
            node_handle_->declare_parameter("tf_prefix", std::string(""));
        }
        tf_prefix_ = node_handle_->get_parameter("tf_prefix").as_string();

        if (!node_handle_->has_parameter("planning_group"))
        {
            node_handle_->declare_parameter("planning_group", std::string("whi_arm"));
        }
        planning_group_ = node_handle_->get_parameter("planning_group").as_string();

        if (!node_handle_->has_parameter("cartesian_fraction"))
        {
            node_handle_->declare_parameter("cartesian_fraction", 1.0);
        }
        cartesian_fraction_ = node_handle_->get_parameter("cartesian_fraction").as_double();

        if (!node_handle_->has_parameter("cartesian_traj_max_step"))
        {
            node_handle_->declare_parameter("cartesian_traj_max_step", 0.01);
        }
        cartesian_traj_max_step_ = node_handle_->get_parameter("cartesian_traj_max_step").as_double();

        if (!node_handle_->has_parameter("cartesian_precision"))
        {
            node_handle_->declare_parameter("cartesian_precision", std::vector<double>{});
        }
        cartesian_precision_ = node_handle_->get_parameter("cartesian_precision").as_double_array();

        if (!node_handle_->has_parameter("link_index_map"))
        {
            node_handle_->declare_parameters<int>("link_index_map", std::map<std::string, int>{});
        }
        node_handle_->get_parameters<int>("link_index_map", link_index_map_);

        if (!node_handle_->has_parameter("init_pose_groups"))
        {
            node_handle_->declare_parameters<double>("init_pose_groups", std::map<std::string, double>{});
        }
        node_handle_->get_parameters<double>("init_pose_groups", init_pose_groups_);

        loadInitPlanParams();

        // subscribe to arm motion state
        if (!node_handle_->has_parameter("arm_state_topic"))
        {
            node_handle_->declare_parameter("arm_state_topic", std::string("arm_motion_state"));
        }
        std::string stateTopic = node_handle_->get_parameter("arm_state_topic").as_string();
        arm_state_sub_ = node_handle_->create_subscription<whi_interfaces::msg::WhiMotionState>(
            stateTopic, 10, std::bind(&MoveItCppBridge::callbackArmMotionState, this, std::placeholders::_1));

        // subscribe estop topic
        if (!node_handle_->has_parameter("estop_topic"))
        {
            node_handle_->declare_parameter("estop_topic", std::string("estop"));
        }
        std::string swEstopTopic = node_handle_->get_parameter("estop_topic").as_string();

        estop_sub_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
            swEstopTopic, 10, std::bind(&MoveItCppBridge::callbackSwEstop, this, std::placeholders::_1));

        // subscribe motion state topic
        if (!node_handle_->has_parameter("motion_state_topic"))
        {
            node_handle_->declare_parameter("motion_state_topic", std::string("motion_state"));
        }
        std::string motionStateTopic = node_handle_->get_parameter("motion_state_topic").as_string();
        motion_state_sub_ = node_handle_->create_subscription<whi_interfaces::msg::WhiMotionState>(
            motionStateTopic, 10, std::bind(&MoveItCppBridge::callbackMotionState, this, std::placeholders::_1));
    }

    bool MoveItCppBridge::preExecution() const
    {
        return true;

        if (estopped_ || sw_estopped_)
        {
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "cannot execute pose action, EStop is active");
            return false;
        }
        if (executing_.load())
        {
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "there is motion executing");
            return false;
        }

        int tryCount = 0;
        bool armReady = false;
        do
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            client_arm_ready_->async_send_request(
                request,
                [this, request, &armReady](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
                {
                    if (future.get()->success)
                    {
                        armReady = true;
                    }
                    else
                    {
                        armReady = false;
                    }
                });

            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "wait for arm ready... in " << max_try_count_ << " seconds");
            std::this_thread::sleep_for(std::chrono::milliseconds(int(wait_duration_ * 1000.0)));
        } while (!armReady && ++tryCount < max_try_count_);

        if (!armReady)
        {
            RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "cannot execute pose action, arm is not ready");
        }

        return armReady;
    }

    bool MoveItCppBridge::execute(const whi_interfaces::msg::WhiTcpPose& Pose)
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
            geometry_msgs::msg::PoseStamped targetPose = Pose.tcp_pose;
            std::string armRoot(tf_prefix_.empty() ? "" : tf_prefix_ + "/");
            armRoot += robot_model_->getRootLinkName();
            if (Pose.tcp_pose.header.frame_id != armRoot &&
                Pose.tcp_pose.header.frame_id != "world" && !Pose.tcp_pose.header.frame_id.empty())
            {
                if (!trans2TargetFrame(armRoot, Pose.tcp_pose, targetPose))
                {
                    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "failed to get pose transform");
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
                const moveit::core::LinkModel* linkModel = joint_model_group_->getLinkModel(Pose.tcp_pose.header.frame_id);
                if (linkModel != nullptr)
                {
                    std::vector<moveit::core::RobotStatePtr> trajState;
                    int tryCount = 0;
                    double fraction = 0.0;
                    do
                    {
                        fraction = moveit::core::CartesianInterpolator::computeCartesianPath(startState.get(),
                            joint_model_group_, trajState, linkModel, target, true,
                            moveit::core::MaxEEFStep(cartesian_traj_max_step_),
                            moveit::core::CartesianPrecision{ cartesian_precision_[0], cartesian_precision_[1] },
                            moveit::core::GroupStateValidityCallbackFn(), kinematics::KinematicsQueryOptions());
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
                        trajectory_processing::TimeOptimalTrajectoryGeneration totp;
                        if (totp.computeTimeStamps(*traj, Pose.velocity_scale, Pose.acceleration_scale))
                        {
                            // execute path
                            bool res = moveit_cpp_->execute(traj);
                            if (is_arm_fault_.load())
                            {
                                is_arm_fault_.store(false);
                                res = false;

                                RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "protective stop encountered");
                            }
                            return res;
                        }
                        else
                        {
                            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "failed to apply time parameters");
                            return false;
                        }
                    }
                    else
                    {
                        RCLCPP_WARN_STREAM(node_handle_->get_logger(), "failed to find solution");
                        return false;
                    }
                }
                else
                {
                    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "link " << Pose.tcp_pose.header.frame_id << " doesn't exit, please check the config!");
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
                    moveit_msgs::msg::Constraints jc;
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
                    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "failed to find the IK solution");
                }
            }
        }
        else
        {
            foundIk = true;
            // set the constraints
            moveit_msgs::msg::Constraints jc;
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
                bool res = moveit_cpp_->execute(solution.trajectory);
                executing_.store(false);
                if (is_arm_fault_.load())
                {
                    is_arm_fault_.store(false);
                    res = false;

                    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "protective stop encountered");
                }
                return res;
            }
            else
            {
                RCLCPP_WARN_STREAM(node_handle_->get_logger(), "failed to find path solution");
                return false;
            }
        }
        else
        {
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "failed to find solution");
            return false;
        }
    }

    bool MoveItCppBridge::execute(const whi_interfaces::msg::WhiJointPose& Pose)
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

        moveit_msgs::msg::Constraints constraints;
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
            bool res = moveit_cpp_->execute(solution.trajectory);
            executing_.store(false);
            if (is_arm_fault_.load())
            {
                is_arm_fault_.store(false);
                res = false;

                RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "protective stop encountered");
            }

            return res;
        }
        else
        {
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "failed to find path solution");
            return false;
        }
    }

    void MoveItCppBridge::callbackTcpPose(const whi_interfaces::msg::WhiTcpPose::SharedPtr Msg)
    {
        execute(*Msg);
    }

    void MoveItCppBridge::callbackJointPose(const whi_interfaces::msg::WhiJointPose::SharedPtr Msg)
    {
        execute(*Msg);
    }

    void MoveItCppBridge::callbackArmMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg)
    {
        if (Msg->state == whi_interfaces::msg::WhiMotionState::STA_FAULT)
        {
            is_arm_fault_.store(true);
        }
    }

    void MoveItCppBridge::callbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg)
    {
        if (Msg->state == whi_interfaces::msg::WhiMotionState::STA_ESTOP)
        {
            moveit_cpp_->getTrajectoryExecutionManagerNonConst()->stopExecution();
            estopped_ = true;
        }
        else if (Msg->state == whi_interfaces::msg::WhiMotionState::STA_STANDBY)
        {
            estopped_ = false;
        }
    }

    void MoveItCppBridge::callbackSwEstop(const std_msgs::msg::Bool::SharedPtr Msg)
    {
        sw_estopped_ = Msg->data;
        if (sw_estopped_)
        {
            moveit_cpp_->getTrajectoryExecutionManagerNonConst()->stopExecution();
        }
    }

    void MoveItCppBridge::onServiceTcpPose(const std::shared_ptr<whi_interfaces::srv::WhiSrvTcpPose::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvTcpPose::Response> Response)
    {
        Response->result = execute(Request->pose);
    }

    void MoveItCppBridge::onServiceJointPose(const std::shared_ptr<whi_interfaces::srv::WhiSrvJointPose::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvJointPose::Response> Response)
    {
        Response->result = execute(Request->pose);
    }

    void MoveItCppBridge::onServiceJointNames(const std::shared_ptr<whi_interfaces::srv::WhiSrvJointNames::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvJointNames::Response> Response)
    {
        Response->joint_names = joint_model_group_->getJointModelNames();
        Response->result = Response->joint_names.empty() ? false : true;
    }

    void MoveItCppBridge::onServiceTcpDifference(const std::shared_ptr<whi_interfaces::srv::WhiSrvTcpDifference::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvTcpDifference::Response> Response)
    {
        std::string frameId;
        if (!Request->pose_group.pose_group.empty())
        {
            frameId = Request->pose_group.header.frame_id;
        }
        else if (!Request->joint_pose.position.empty())
        {
            frameId = Request->joint_pose.header.frame_id;
        }
        else
        {
            frameId = Request->tcp_pose.header.frame_id;
        }

        auto state = moveit_cpp_->getCurrentState();
        geometry_msgs::msg::Pose currentTcpPose = tf2::toMsg(state->getGlobalLinkTransform(frameId));
        tf2::Quaternion currentQ(currentTcpPose.orientation.x, currentTcpPose.orientation.y,
            currentTcpPose.orientation.z, currentTcpPose.orientation.w);

        if (!Request->pose_group.pose_group.empty())
        {
            auto jointValues = planning_components_->getNamedTargetStateValues(Request->pose_group.pose_group);
            if (!jointValues.empty())
            {
                std::vector<double> jointPositions;

                // sort joint values with ascending order
                if (link_index_map_.size() == jointValues.size())
                {
                    jointPositions.resize(link_index_map_.size());
                    for (const auto& it : link_index_map_)
                    {
                        jointPositions[it.second] = jointValues[it.first];
                    }
                }
                else
                {
                    for (const auto& it : jointValues)
                    {
                        jointPositions.push_back(it.second);
                    }
                }

                state->setJointGroupPositions(joint_model_group_, jointPositions);

                // forward kinematics
                auto transform = state->getGlobalLinkTransform(frameId);
                auto reference = tf2::toMsg(transform);
                tf2::Quaternion referenceQ(reference.orientation.x, reference.orientation.y,
                    reference.orientation.z, reference.orientation.w);

                Response->result = true;
                Response->difference.position.x = currentTcpPose.position.x - reference.position.x;
                Response->difference.position.y = currentTcpPose.position.y - reference.position.y;
                Response->difference.position.z = currentTcpPose.position.z - reference.position.z;
                Response->difference.orientation = tf2::toMsg(currentQ * referenceQ.inverse());
            }
            else
            {
                Response->result = false;
            }
        }
        else if (!Request->joint_pose.position.empty())
        {
            state->setJointGroupPositions(joint_model_group_, Request->joint_pose.position);

            // forward kinematics
            auto transform = state->getGlobalLinkTransform(frameId);
            auto reference = tf2::toMsg(transform);
            tf2::Quaternion referenceQ(reference.orientation.x, reference.orientation.y,
                reference.orientation.z, reference.orientation.w);

            Response->result = true;
            Response->difference.position.x = currentTcpPose.position.x - reference.position.x;
            Response->difference.position.y = currentTcpPose.position.y - reference.position.y;
            Response->difference.position.z = currentTcpPose.position.z - reference.position.z;
            Response->difference.orientation = tf2::toMsg(currentQ * referenceQ.inverse());
        }
        else
        {
            tf2::Quaternion referenceQ(Request->tcp_pose.pose.orientation.x, Request->tcp_pose.pose.orientation.y,
                Request->tcp_pose.pose.orientation.z, Request->tcp_pose.pose.orientation.w);

            Response->result = true;
            Response->difference.position.x = currentTcpPose.position.x - Request->tcp_pose.pose.position.x;
            Response->difference.position.y = currentTcpPose.position.y - Request->tcp_pose.pose.position.y;
            Response->difference.position.z = currentTcpPose.position.z - Request->tcp_pose.pose.position.z;
            Response->difference.orientation = tf2::toMsg(currentQ * referenceQ.inverse());
        }
    }

    void MoveItCppBridge::onServiceCurrentTcpPose(const std::shared_ptr<whi_interfaces::srv::WhiSrvCurrentTcpPose::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvCurrentTcpPose::Response> Response)
    {
        auto state = moveit_cpp_->getCurrentState();
        Response->pose = tf2::toMsg(state->getGlobalLinkTransform(Request->header.frame_id));
        Response->result = true;
    }

    bool MoveItCppBridge::trans2TargetFrame(const std::string& DstFrame,
        const geometry_msgs::msg::PoseStamped& PoseIn, geometry_msgs::msg::PoseStamped& PoseOut)
    {
        try
        {
            PoseOut = moveit_cpp_->getTFBuffer()->transform(PoseIn, DstFrame);
            return true;
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_ERROR_STREAM(node_handle_->get_logger(), e.what());
            return false;
        }
    }

    void MoveItCppBridge::loadInitPlanParams()
    {        
        if (!node_handle_->has_parameter("plan_request_params.planner_id"))
        {
            node_handle_->declare_parameter("plan_request_params.planner_id", std::string("RRTConnectkConfigDefault"));
        }
        init_plan_parameters_.planner_id = node_handle_->get_parameter("plan_request_params.planner_id").as_string();

        if (!node_handle_->has_parameter("plan_request_params.planning_pipeline"))
        {
            node_handle_->declare_parameter("plan_request_params.planning_pipeline", std::string("ompl"));
        }
        init_plan_parameters_.planning_pipeline = node_handle_->get_parameter("plan_request_params.planning_pipeline").as_string();

        if (!node_handle_->has_parameter("plan_request_params.planning_time"))
        {
            node_handle_->declare_parameter("plan_request_params.planning_time", 2.0);
        }
        init_plan_parameters_.planning_time = node_handle_->get_parameter("plan_request_params.planning_time").as_double();

        if (!node_handle_->has_parameter("plan_request_params.planning_attempts"))
        {
            node_handle_->declare_parameter("plan_request_params.planning_attempts", 5);
        }
        init_plan_parameters_.planning_attempts = node_handle_->get_parameter("plan_request_params.planning_attempts").as_int();

        if (!node_handle_->has_parameter("plan_request_params.max_velocity_scaling_factor"))
        {
            node_handle_->declare_parameter("plan_request_params.max_velocity_scaling_factor", 1.0);
        }
        init_plan_parameters_.max_velocity_scaling_factor = node_handle_->get_parameter("plan_request_params.max_velocity_scaling_factor").as_double();

        if (!node_handle_->has_parameter("plan_request_params.max_acceleration_scaling_factor"))
        {
            node_handle_->declare_parameter("plan_request_params.max_acceleration_scaling_factor", 1.0);
        }
        init_plan_parameters_.max_acceleration_scaling_factor = node_handle_->get_parameter("plan_request_params.max_acceleration_scaling_factor").as_double();
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
            whi_interfaces::msg::WhiTcpPose poseGroup;
            poseGroup.pose_group = it.first;
            poseGroup.velocity_scale = it.second;
            poseGroup.acceleration_scale = it.second;
            execute(poseGroup);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
} // namespace whi_moveit_cpp_bridge
