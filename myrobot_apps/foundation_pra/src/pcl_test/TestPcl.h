#ifndef _TEST_PCL_H
#define _TEST_PCL_H
#include <ros/ros.h>
#include <sensor_msgs//PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>


class TestPcl
{
public:
    TestPcl();
    ~TestPcl();

    
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Publisher scanPub_;
    ros::Subscriber scanSub_;
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
    void laserScanCb (const sensor_msgs::LaserScanConstPtr& laserScan);
};

#endif