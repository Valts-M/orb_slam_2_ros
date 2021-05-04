//
// Created by valts on 17.04.21.
//

#ifndef ORB_SLAM2_ROS_LIDARNODE_H
#define ORB_SLAM2_ROS_LIDARNODE_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
//#include <pcl>

#include "System.h"
#include "Node.h"
#include "Cloud2Img.h"


class LidarNode : public Node
{
public:
    LidarNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~LidarNode ();
    void LidarCallback (const sensor_msgs::PointCloud2ConstPtr& msgLidar);

private:
    ros::Subscriber lidar_subscriber_;
};
#endif //ORB_SLAM2_ROS_LIDARNODE_H
