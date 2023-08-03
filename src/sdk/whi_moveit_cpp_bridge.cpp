/******************************************************************
MoveItCpp bridge to handle moveit commands under ROS 1

Features:
- advertise command service
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
        // // params
        // node_handle_->param("whi_hik_uvc/frame_id", frame_id_, std::string("camera"));
        // std::string topicImg, topicInfo;
        // node_handle_->param("whi_hik_uvc/topic_image", topicImg, std::string("image"));
        // node_handle_->param("whi_hik_uvc/cam_info", topicInfo, std::string("cam_info"));

        // // publisher
        // image_transport_ = std::make_unique<image_transport::ImageTransport>(*node_handle_);
        // pub_image_ = std::make_unique<image_transport::Publisher>(image_transport_->advertise(topicImg, 1));
        // pub_info_ = std::make_unique<image_transport::CameraPublisher>();

        // spinner
        node_handle_->param("whi_moveit_cpp_bridge/loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq,
            std::bind(&MoveItCppBridge::update, this, std::placeholders::_1)));
    }

    void MoveItCppBridge::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        // TODO
        //std::cout << "elapsed " << elapsed_time_.toSec() << std::endl;
    }
} // namespace whi_moveit_cpp_bridge
