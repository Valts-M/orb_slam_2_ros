/*
 * MIT License
 * Copyright (c) 2020 Anirudh Topiwala
 * Author: Anirudh Topiwala
 * Create Date: 2020-03
 * Last Edit Date: 2020-03
 *
 * @brief This class is used to load a point cloud using pcl and convert it into
 * spherical view or front view projection. More information about the theory
 * can be found on the medium article here:
 *
 */

#include "Cloud2Img.h"

#include <math.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

Cloud2Img::Cloud2Img(){
    cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
};

void Cloud2Img::LoadCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);
}

void Cloud2Img::MakeImages(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input, cv::Mat& gs_img, cv::Mat& d_img) {
    LoadCloud(input);

    if (cloud_->size() == 0) {
        std::cerr << "Empty Point Cloud_" << std::endl;
        return;
    }

    gs_img = cv::Mat::zeros(gs_img.rows, gs_img.cols, gs_img.type());
    d_img = cv::Mat::zeros(d_img.rows, d_img.cols, d_img.type());

    for (auto point : *cloud_) {
        // Getting Pixel from Point
        int pixel_v = 0;
        int pixel_u = 0;
        float range = 0.0;
        GetProjection(point, &pixel_v, &pixel_u, &range);
        gs_img.at<unsigned char >(pixel_u, pixel_v)= point.intensity * 255;
        d_img.at<float>(pixel_u, pixel_v)= sqrt(point.x * point.x + point.y * point.y)/config_.max_range;
    }
    cv::GaussianBlur(gs_img, gs_img, cv::Size(1.0, 7.0), 2.0, 2.0);
    //cv::GaussianBlur(d_img, d_img, cv::Size(1.0, 7.0), 2.0, 2.0);

    //FillHoles(d_img);
    //FillHoles(gs_img);
    //ShowImg(gs_img, d_img);
}

void Cloud2Img::GetProjection(const pcl::PointXYZI& point,
                               int* pixel_v, int* pixel_u,
                               float* range) const {
    // range of Point from Lidar
    *range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    //  Getting the angle of all the Points
    auto yaw = atan2(point.y, point.x);
    auto pitch = asin(point.z / *range);
//    std::cout << "OLD:" << std::endl;
//    std::cout << "\tyaw = " << yaw << std::endl;
//    std::cout << "\tpitch = " << pitch << std::endl;
//    std::cout << "\tx = " << point.x << std::endl;
//    std::cout << "\ty = " << point.y << std::endl;
//    std::cout << "\tz = " << point.z << std::endl << std::endl;

    // Get projections in image coords and normalizing
    double v = 0.5 * (yaw / M_PI + 1.0);
    double u = 1.0 - (pitch + std::abs(fov_down_rad)) / fov_rad;
    // Scaling as per the lidar config given
    v *= config_.img_length;
    u *= config_.num_lasers;
    // round and clamp for use as index
    v = floor(v);
    v = std::min(config_.img_length - 1, v);
    v = std::max(0.0, v);
    *pixel_v = int(v);

    u = floor(u);
    u = std::min(config_.num_lasers - 1, u);
    u = std::max(0.0, u);
    *pixel_u = int(u);

//    const float nyaw = ((v/config_.img_length)/ 0.5 - 1) * M_PI;
//    const float npitch = (1.0 - (u/config_.num_lasers)) * fov_rad - std::abs(fov_down_rad);
//
//    double nrange = sqrt(point.x * point.x + point.y * point.y);
//    double z = nrange * tan(npitch);
//    double x = nrange * cos(nyaw);
//    double y = nrange * sin(nyaw);
//
//    std::cout << "New:" << std::endl;
//    std::cout << "\tyaw = " << nyaw << std::endl;
//    std::cout << "\tpitch = " << npitch << std::endl;
//    std::cout << "\tx = " << x << std::endl;
//    std::cout << "\ty = " << y << std::endl;
//    std::cout << "\tz = " << z << std::endl << std::endl;
}

void Cloud2Img::GetReprojection(const float &u, const float &v, float &x, float &y, float &z){
    const float yaw = ((u/config_.img_length)/ 0.5 - 1) * M_PI;
    const float pitch = (1.0 - (v/config_.num_lasers)) * fov_rad - std::abs(fov_down_rad);

    z *= config_.max_range;

    x = -z * sin(yaw);
    y = -z * tan(pitch);
    z = z * cos(yaw);

//    std::cout << "\tyaw = " << yaw << std::endl;
//    std::cout << "\tpitch = " << pitch << std::endl;
//    std::cout << "\tx = " << x << std::endl;
//    std::cout << "\ty = " << y << std::endl;
//    std::cout << "\tz = " << z << std::endl << std::endl;
}

void Cloud2Img::ShowImg(cv::Mat &img, cv::Mat &dimg) const{
    //cv::imwrite("IntensityImg.png", img);
    //cv::imwrite("DepthImg.png", dimg);
    cv::namedWindow("Intensity image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Intensity image", img);
    cv::imshow("Depth image", dimg);
    cv::waitKey(50);
}

void Cloud2Img::FillHoles(cv::Mat &img) {
    cv::rgbd::DepthCleaner depthCleaner{img.type(), 7, cv::rgbd::DepthCleaner::DEPTH_CLEANER_NIL};
    depthCleaner.operator()(img, img);
    cv::resize(img, img, cv::Size(), 0.5, 0.5);
    cv::inpaint(img, (img == 0), img, 2.0, cv::INPAINT_TELEA);
    cv::resize(img, img, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
    cv::imshow("sa", img);
    cv::waitKey();
}
