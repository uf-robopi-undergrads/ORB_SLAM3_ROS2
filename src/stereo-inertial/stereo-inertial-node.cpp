#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

// Comments added by Copilot to make it easier to modify
StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"), // Initialize the ROS2 node with the name "ORB_SLAM3_ROS2"
    SLAM_(SLAM) // Store the pointer to the ORB_SLAM3 system instance
{
    // Parse the string input for rectification and convert it to a boolean
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    // Parse the string input for histogram equalization and convert it to a boolean
    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    // Set the CLAHE (Contrast Limited Adaptive Histogram Equalization) flag based on the input
    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load stereo calibration settings from the provided YAML file
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        // Extract camera calibration parameters for left and right cameras
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        // Validate that all required parameters are loaded
        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        // Initialize rectification maps for left and right cameras
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    // Subscribe to IMU data topic
    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    // Subscribe to left camera image topic
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    // Subscribe to right camera image topic
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    // Start a separate thread to synchronize IMU and image data
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Join and delete the synchronization thread
    syncThread_->join();
    delete syncThread_;

    // Shutdown the SLAM system
    SLAM_->Shutdown();

    // Save the keyframe trajectory to a file
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

// Function to handle incoming IMU messages
void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock(); // Lock the mutex to ensure thread safety
    imuBuf_.push(msg); // Add the IMU message to the buffer
    bufMutex_.unlock(); // Unlock the mutex
}

// Function to handle incoming left camera images
void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop(); // Remove the oldest image if the buffer is not empty
    imgLeftBuf_.push(msgLeft); // Add the new image to the buffer

    bufMutexLeft_.unlock();
}

// Function to handle incoming right camera images
void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop(); // Remove the oldest image if the buffer is not empty
    imgRightBuf_.push(msgRight); // Add the new image to the buffer

    bufMutexRight_.unlock();
}

// Function to convert ROS image messages to OpenCV Mat format
cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
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
        return cv_ptr->image.clone(); // Return a copy of the image
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

// Function to synchronize IMU and image data
void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01; // Maximum allowed time difference between  left and right image timestamps

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop(); // Remove older right images if the time difference is too large
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop(); // Remove older left images if the time difference is too large
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load IMU measurements from the buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft); // Apply CLAHE to the left image
                clahe_->apply(imRight, imRight); // Apply CLAHE to the right image
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR); // Rectify the left image
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR); // Rectify the right image
            }

            // Pass the synchronized data to the SLAM system
            SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep); // Sleep for a short duration to reduce CPU usage
        }
    }
}
