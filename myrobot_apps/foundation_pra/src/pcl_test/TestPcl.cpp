

#include "TestPcl.h"
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
 
void TestPcl::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//   // Do data processing here...  
  //auto output = *input;
  // 声明存储原始数据与滤波后的数据的点云的 格式
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;     //存储滤波后的数据格式  
// 转化为PCL中的点云的数据格式
 pcl_conversions::toPCL(*input, *cloud);  // 进行一个滤波处理
  
 pcl::VoxelGrid<pcl::PCLPointCloud2> sor;   //实例化滤波
 sor.setInputCloud (cloudPtr);     //设置输入的滤波
 sor.setLeafSize (0.02, 0.02, 0.02);   //设置体素网格的大小
 sor.filter (cloud_filtered);      //存储滤波后的点云  

// 再将滤波后的点云的数据格式转换为ROS 下的数据格式发布出去
 sensor_msgs::PointCloud2 output;   //声明的输出的点云的格式
 pcl_conversions::fromPCL(cloud_filtered, output);    //第一个参数是输入，后面的是输出  
//发布命令 
  ROS_INFO_STREAM( "003===");
  pub.publish (output);

}
// 激光雷达数据订阅，转换成点云数据，利用pcl库处理后，再转化成ros消息发布（用于显示/或判断障碍物后给出结论等应用），
void TestPcl::laserScanCb (const sensor_msgs::LaserScanConstPtr& laserScan){
    //根据激光雷达数据转换成xy点，然后放到pcl点云中
    pcl::PointXYZ currentPoint;
    static pcl::PointCloud<pcl::PointXYZ> laser_pointXYZ_;
    static sensor_msgs::PointCloud2 laserPoint2_output;
    double currentAngle;

    for(uint32_t i =0; i< laserScan->ranges.size();i++){
        currentAngle = laserScan->angle_min +laserScan->angle_increment*i;
        currentPoint.x = laserScan->ranges[i]*cos(currentAngle);
        currentPoint.y = laserScan->ranges[i]*sin(currentAngle);
        //currentPoint.intensity = laserScan->intensities[i];
        currentPoint.z =0;
        laser_pointXYZ_.push_back(currentPoint);

    }
    pcl::PCLPointCloud2 laser_pointCloud_;
    // laser_pointCloud_.header.stamp = ros::Time::now();
    //得到PCL中的点云数据 PCLPointCloud2
    pcl::toPCLPointCloud2(laser_pointXYZ_,laser_pointCloud_);
    // pcl::PCLPointCloud2ConstPtr cloudPtr(*laser_pointCloud_);

    // pcl::PCLPointCloud2 cloud_filtered; 
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;   //实例化滤波
    // sor.setInputCloud (cloudPtr);     //设置输入的滤波
    // sor.setLeafSize (0.02, 0.02, 0.02);   //设置体素网格的大小
    // sor.filter (cloud_filtered);      //存储滤波后的点云 
    //pcl_conversions::fromPCL(cloud_filtered, laserPoint2_output);
    pcl_conversions::fromPCL(laser_pointCloud_, laserPoint2_output);
    laserPoint2_output.header.frame_id = "laser";
    scanPub_.publish(laserPoint2_output);
    //TODO还差transform
    // static tf::TransformListener listener;
    // static tf::StampedTransform transform;
    // listener.lookupTransform("/odom","/base_link",ros::Time(0), transform);

    //发送laserScan
    // sensor_msgs::LaserScan outLaserScan;
    // outLaserScan = *laserScan;
    // scanPub_.publish(outLaserScan);
    ROS_INFO_STREAM ("laserCloud out");
    

    
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