#ifndef PCL_FILTER_H
#define PCL_FILTER_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <sstream>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/filters/radius_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

using namespace std;

//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
#define DEBUG 1

class PclFilter
{

    ros::NodeHandle* nh_p_;

    //Functions
    void register_all_subscribers();
    void initialize_private_params();
    void advertise_all_publishers();


    ros::Subscriber point_cloud_subscribe;
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_outlier_removal_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filtering_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through_filtering_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // declare parameters
    float voxel_size_param_ ;
    float radius_threshold_param_;
    float min_neighbors_param_;
    float pass_z_param_,pass_y_param_, pass_x_param_;

    pcl::PointCloud<pcl::PointXYZ>  filtered_cloud;
    ros::Publisher filtered_point_cloud;

public:

    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        return (radius_outlier_removal_(cloud));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        return voxel_grid_filtering_(cloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        return pass_through_filtering_(cloud);
    }

    PclFilter();
    ~PclFilter();
};

#endif
