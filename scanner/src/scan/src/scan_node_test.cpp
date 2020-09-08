#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


ros::Publisher pub_combined_image;
laser_geometry::LaserProjection projector;


void scan_callback(const sensor_msgs::LaserScan laser_msg){
    // cout << "hi" << endl;

    // Laserscan -> ROS PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    projector.projectLaser(laser_msg, cloud_msg);
    //pub_combined_image.publish(cloud_msg);


    // ROS PointCloud2 -> PCL Pointcloud
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);

    PointCloudXYZPtr cloud_filtered(new PointCloudXYZ);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_raw);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.5, 0.5);
    pass.setFilterLimitsNegative(false);
    pass.filter (*cloud_filtered);
    

    // PCL Pointcloud -> ROS PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub_combined_image.publish(output);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_node");
    ros::NodeHandle nh;
    
    // ROS related
    pub_combined_image = nh.advertise<sensor_msgs::PointCloud2>("scan_pointcloud", 1);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scan_callback);

    ros::spin();
    return 0;
}
