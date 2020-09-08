#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


ros::Publisher pub_combined_image;
laser_geometry::LaserProjection projector;


void scan_callback(const sensor_msgs::LaserScan laser_msg){
    

    // Laserscan -> ROS PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    projector.projectLaser(laser_msg, cloud_msg);
    


    // ROS PointCloud2 -> PCL Pointcloud
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);

    //line finding
    std::vector<int> inliers;
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
    model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud_raw));
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);



    
    PointCloudXYZPtr cloud_line(new PointCloudXYZ);
    pcl::copyPointCloud (*cloud_raw, inliers, *cloud_line);
    cout << cloud_line->points.size() << endl;
    // PCL Pointcloud -> ROS PointCloud2
    sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud_line, output);
    pcl::toROSMsg(*cloud_line, output);
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