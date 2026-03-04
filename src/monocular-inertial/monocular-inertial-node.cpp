#include "monocular-inertial-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    SLAM_ = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/image_raw",
        10,
        std::bind(&MonocularInertialNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularInertialNode::~MonocularInertialNode()
{
    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // flip each frame before sending since our camera is mounted upside down.
    cv::Mat flipped;
    cv::flip(m_cvImPtr->image, flipped, -1); // -1 indicates both axes to flip
    std::cout<<"one flipped frame has been sent"<<std::endl;

    SLAM_->TrackMonocular(flipped, Utility::StampToSec(msg->header.stamp));
}
