#ifndef ORB_SLAM2_ROS_CLOUD2IMG_H
#define ORB_SLAM2_ROS_CLOUD2IMG_H

#include <pcl/point_cloud.h>  // For PCL Point Cloud
#include <pcl/point_types.h>  // For PCL different csloud types
#include <opencv2/opencv.hpp>  // For visualizing image
#include <opencv2/rgbd.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
/**
 * @brief this wraps all the configurations needed for doing spherical
 * projection.
 *
 */
struct Configuration {
    /**
     * @brief The upper filed of view of the lidar (degrees)
     *
     */
    double fov_up = 0.0;
    /**
     * @brief The lower filed of view of the lidar (degrees)
     *
     */
    double fov_down = 0.0;
    /**
     * @brief The number of lasers in the lidar. This will make up the width of
     * the projection image.
     *
     */
    double num_lasers = 0.0;
    /**
     * @brief the length of the projection image.
     *
     */
    double img_length = 0.0;

    double max_range = 0.1;
};

class Cloud2Img {
public:
    /**
     * @brief Constructor to initialize the configuration and the projection
     * image.
     *
     * @param[in] configurations to be set for the projection image
     */
    Cloud2Img();
    /**
     * @brief Loads the point cloud of type .pcd in the cloud_ class variable.
     *
     * @param[in] path Absolute path to the .pcd file
     * @return -1 The path is invalid
     * @return 1 Cloud Loaded Successfully
     */
    void LoadCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    /**
     * @brief Function to iterate over each point of cloud and make the projection
     * image.
     *
     * @return -1 The cloud is empty
     * @return 1 spherical image formed successfully
     */
    void MakeImages(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input, cv::Mat& gs_img, cv::Mat& d_img);
    /**
     * @brief Convert 3D point to 2D pixel cooredinated by doing Spherical
     * Projection
     *
     * @param[in] point
     * @param[in] fov_rad
     * @param[in] fov_down_rad
     * @param[out] pixel_x
     * @param[out] pixel_y
     * @param[out] depth
     *
     */
    void GetProjection(const pcl::PointXYZI& point, int* pixel_x, int* pixel_y,
                       float* depth) const;

    void GetReprojection(const float &u, const float &v, float &x, float &y, float &range);
    /**
     * @brief Use OpenCv to view the spherical image formed
     *
     * @param img
     */
    void ShowImg(cv::Mat &img, cv::Mat &dimg) const;

private:
    /**
     * @brief The configuration required for forming the spherical projection
     *
     */
    const Configuration config_{2, -24.8, 64, 1800, 80.0};
    const float fov_up_rad = (config_.fov_up / 180) * M_PI;
    const float fov_down_rad = (config_.fov_down / 180) * M_PI;
    const float fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);

    /**
     * @brief the point cloud will be loaded and accesible to the class through
     * this variable.
     *
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

    void FillHoles(cv::Mat &img);
};
#endif //ORB_SLAM2_ROS_CLOUD2IMG_H
