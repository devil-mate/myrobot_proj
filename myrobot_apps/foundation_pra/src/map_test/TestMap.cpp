

#include "TestPcl.h"
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//map接收，map的发布可以直接调用map_server
using namespace std;
TestPcl::TestPcl(){
    
    sub = n.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, &TestPcl::cloud_cb,this);
    pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);
    scanPub_ = n.advertise<sensor_msgs::PointCloud2>("laserPointCloud_out",1);
    // scanPub_ = n.advertise<sensor_msgs::LaserScan>("laserPointCloud_out",1);
    scanSub_ = n.subscribe<sensor_msgs::LaserScan>("/scan",1, &TestPcl::laserScanCb,this);
}
TestPcl::~TestPcl(){
    
}



int main(int argc, char** argv){
    try{
        ROS_INFO("hello !");
        ros::init (argc, argv, "TestPcl_node");
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);
        TestPcl testPcl;
        ros::spin();
    }
    catch(...){
        ROS_ERROR("[error] exit!");
        return -1;
    }
    
    return 0;
}