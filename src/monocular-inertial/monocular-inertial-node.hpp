#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"


#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;


class MonocularInertialNode : public rclcpp::Node
{
public:
    // keep simple for now, don't take in options that aren't needed yet
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);

    ~MonocularInertialNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void GrabImu(const ImuMsg::SharedPtr msg);

    // turn image into openCV matrix that we can use
    cv::Mat GetImage(const ImageMsg::SharedPtr msg, bool flip = true);

    // keep imu and image timepoints stable
    void SyncWithImu();

    // message subscriptions
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    // SLAM parameters
    ORB_SLAM3::System* SLAM_;
    std::thread *syncThread_;

    // IMU buffer
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex imuBufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex imgBufMutex_;

};

#endif
