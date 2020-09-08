#include <iostream>
#include <Eigen/Dense>


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
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
//merging point clouds
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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
    ransac.setDistanceThreshold (0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);
    PointCloudXYZPtr cloud_line(new PointCloudXYZ);
    pcl::copyPointCloud (*cloud_raw, inliers, *cloud_line);
    //get firstlinecoefficients
    Eigen::VectorXf coef = Eigen::VectorXf::Zero(6 , 1);
    ransac.getModelCoefficients(coef);
    cout << coef[0] << " " << coef[1] << " " << coef[2] << " " << coef[3] << " " << coef[4] << " " << coef[5] << endl;
 
    // Filter out line
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
    inliers_ptr->indices = inliers;
    PointCloudXYZPtr cloud_extracted(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_raw);
    extractor.setIndices(inliers_ptr);  
    extractor.setNegative(true);
    extractor.filter(*cloud_extracted);

    //second line finding
    std::vector<int> inliers2;
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
    model_l2(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud_extracted));
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model_l2);
    ransac2.setDistanceThreshold (0.05);
    ransac2.computeModel();
    ransac2.getInliers(inliers2);
    //get secondlinecoefficients
    Eigen::VectorXf coef2 = Eigen::VectorXf::Zero(6 , 1);
    ransac2.getModelCoefficients(coef2);
    cout << coef2[0] << " " << coef2[1] << " " << coef2[2] << " " << coef2[3] << " " << coef2[4] << " " << coef2[5] << endl;
    

    //filter again
    pcl::PointIndices::Ptr inliers_ptr2(new pcl::PointIndices());
    inliers_ptr2->indices = inliers2;

    


    PointCloudXYZPtr cloud_second_line(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extractor2;
    extractor2.setInputCloud(cloud_extracted);
    extractor2.setIndices(inliers_ptr2);
    extractor2.setNegative(false);
    extractor2.filter(*cloud_second_line);

    //merge two point clouds
    PointCloudXYZPtr cloud_final(new PointCloudXYZ);
    *cloud_final=*cloud_line+*cloud_second_line;
    
    //PointCloudXYZPtr cloud_line(new PointCloudXYZ);
    //pcl::copyPointCloud (*cloud_raw, inliers, *cloud_line);

    // PCL Pointcloud -> ROS PointCloud2
    sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud_line, output);
    pcl::toROSMsg(*cloud_final, output);
    //cout << cloud_second_line->points.size() << endl;
    pub_combined_image.publish(output);

    //environment detect
    if(abs(coef[3]*coef2[3]+coef[4]*coef2[4])<0.1)
    {
        cout << "corner" <<endl;
    }
    else if (abs(abs(coef[3])-abs(coef2[3])+abs(coef[4])-abs(coef2[4]))<0.1)
    {
        if((cloud_line->points.size()-cloud_second_line->points.size())>(cloud_line->points.size())/3)
        {
            cout << "corner" <<endl;
        }
        else
        {
            cout << "corridor" <<endl;
        }
    }
    else
    {
        cout << "open environment" <<endl; 
    }
    
    



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
