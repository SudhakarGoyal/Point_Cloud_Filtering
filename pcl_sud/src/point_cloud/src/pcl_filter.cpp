#include "point_cloud/pcl_filter.h"

/**
 * @brief constructor
 * initialize all params mentioned in the launch file
 * subscribing to velodyne point cloud dataset or any other point cloud whose data you wish to use
 * advertise all publishers -- in this case the filtered point cloud
 */
PclFilter::PclFilter()
{
    nh_p_ = new ros::NodeHandle("~");

    // Initialize Private Parameters
    initialize_private_params();

    //Register Subscribers
    register_all_subscribers();

    //Advertise Publishers
    advertise_all_publishers();

    ros::Rate loop_rate(20);

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * @brief Destructor Method
 */
PclFilter::~PclFilter()
{

}

void PclFilter::initialize_private_params()
{
    nh_p_->param<float>("radius_threshold", radius_threshold_param_, 0.5);
    nh_p_->param<float>("min_neighbors", min_neighbors_param_, 0.5);
    nh_p_->param<float>("voxel_size", voxel_size_param_, 0.2);
    nh_p_->param<float>("pass_through_z", pass_z_param_, 1);
    nh_p_->param<float>("pass_through_y", pass_y_param_, 1);
    nh_p_->param<float>("pass_through_x", pass_x_param_, 1);

}

/**
 * @brief PclFilter::radius_outlier_removal_
 * removing points/outliers on basis of number of points is the neighborhood given a certain radius threshold
 * @param cloud - input cloud
 * @return filtered cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilter::radius_outlier_removal_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(radius_threshold_param_);
    outrem.setMinNeighborsInRadius (min_neighbors_param_);
    outrem.filter (*temp_cloud);

    return temp_cloud;
}

/**
 * @brief PclFilter::voxel_grid_filtering_
 * downsampling a point cloud
 * @param
 * input is our cloud wither a filter or non filtered one depending when the function is called
 * @return  downsampled point cloud on the basis of voxel_grid_param passed in the launch file
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr  PclFilter::voxel_grid_filtering_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (voxel_size_param_, voxel_size_param_, voxel_size_param_);
    sor.filter (*temp_cloud);

    return temp_cloud;
}

void PclFilter::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);

    if(DEBUG)
        cout<<cloud->size()<<endl;

    cloud = PclFilter::radius_outlier_removal(cloud);
    cloud = PclFilter::voxel_grid_filtering(cloud);
    filtered_point_cloud.publish(cloud);

}

/**
 * @brief PclFilter::pass_through_filtering_
 * filtering points on basis of disctance and axis
 * @param cloud
 * @return filtered point cloud depending whether we wish to return the filtered cloud or the rest of the cloud
 * for filtered cloud set setFilterLimitsNegative param as false or don't write it
 * for removal of filtered cloud from rest of the cloud set setFilterLimitsNegative as True
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr  PclFilter::pass_through_filtering_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, pass_y_param_);
    pass.setFilterLimitsNegative (true);
    pass.filter (*temp_cloud);

    return temp_cloud;
}



/**
 * @brief Method to register all subscribers with their callbacks
 */
void PclFilter::register_all_subscribers()
{
    point_cloud_subscribe = nh_p_->subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &PclFilter::point_cloud_callback, this); //any topic on which we are subscribing pcl
}

/**
 * @brief Method to advertise all publishers
 */
void PclFilter::advertise_all_publishers()
{
    filtered_point_cloud = nh_p_->advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 10);
}




