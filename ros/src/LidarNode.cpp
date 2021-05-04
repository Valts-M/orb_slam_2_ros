#include "LidarNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Lidar");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);

    LidarNode node (ORB_SLAM2::System::LIDAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}

LidarNode::LidarNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
    lidar_subscriber_ = node_handle.subscribe("/kitti/velo/pointcloud", 3, &LidarNode::LidarCallback, this);
}

void LidarNode::LidarCallback (const sensor_msgs::PointCloud2ConstPtr& msgLidar) {
    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptrRGB;
//    try {
//        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//
//    current_frame_time_ = msgRGB->header.stamp;
//
//    orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    orb_slam_->TrackLidar(msgLidar);
    Update ();
}


LidarNode::~LidarNode () {
    delete lidar_subscriber_;
}


