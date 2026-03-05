#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2")
{
    SLAM_ = pSLAM;
    // subscribe to image topic
    subImg_ = this->create_subscription<ImageMsg>(
        "/image_raw",
        10, // QOS
        // Bind callback handler with GrabImage
        std::bind(&MonocularInertialNode::GrabImage, this, std::placeholders::_1));

    // subscribe to IMU topic
    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));

    // Start a separate thread to synchronize IMU and image data
    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);
}

MonocularInertialNode::~MonocularInertialNode()
{
    // Join and delete the synchronization thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("MonoInertialKeyFrameTrajectory.txt");
}

// Function to handle incoming IMU messages
void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    imuBufMutex_.lock();   // Lock the mutex to ensure thread safety
    imuBuf_.push(msg);     // Add the IMU message to the buffer
    imuBufMutex_.unlock(); // Unlock the mutex
}

// Handle incoming image messages, but do not process yet
void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    imgBufMutex_.lock();

    // remove oldest image if buffer is not empty
    if (!imgBuf_.empty())
    {
        imgBuf_.pop();
    }

    imgBuf_.push(msg);

    imgBufMutex_.unlock();
}

// Convert ROS image message to OpenCV mat, optionally transforming image
cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg, bool flip)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        // optionally flip based on parameter
        if (flip)
        {
            // flip in place (source and destination are the same)
            // TODO: make sure this works
            cv::flip(cv_ptr->image, cv_ptr->image, -1); // -1 indicates both axes to flip
        }

        return cv_ptr->image.clone(); // Return a copy of the image
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

// Synchronize IMU and image data
// ideally, both sensors would utilize an external sync pin that could be pulsed, but for now manual rectification should work
// This is based on the stereo version, but adjusted for only one camera source
void MonocularInertialNode::SyncWithImu()
{
    while (1)
    {
        cv::Mat img;
        double tImg = 0;
        // proceed if both buffers have data
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);

            imgBufMutex_.lock();
            img = GetImage(imgBuf_.front());
            imgBuf_.pop();
            imgBufMutex_.unlock();

            // store list of IMU measurements since last image taken
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            imuBufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load IMU measurements from the buffer since last image
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImg)
                {
                    // extract parameters from message to OpenCV 3d point datatype
                    double tImu = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tImu));
                    imuBuf_.pop();
                }
            }
            imuBufMutex_.unlock();

            // TODO: Maybe introduce CLAHE image enhancement - currently we assume we don't need to rectify`

            // DEBUG: get idea of what our IMU vector looks like on each run
            RCLCPP_INFO(this->get_logger(),
                        R"(Num IMU Measurements: %f
First imu time: %f
Last IMU time: %f
Image time: %f)",
                        vImuMeas.size(), vImuMeas.front().t, vImuMeas.back().t, tImg);

            // Pass the synchronized data to the SLAM system
            SLAM_->TrackMonocular(img, tImg, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep); // Sleep for a short duration to reduce CPU usage
        }
    }
}