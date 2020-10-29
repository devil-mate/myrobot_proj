
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    // ROS_INFO("angle_range, %f, %f", scan->angle_min, scan->angle_max);
    // ROS_INFO("angle_increment: %f ",scan->angle_increment);
    // ROS_INFO("ranges size: %d",scan->ranges.size());
    count  = scan->ranges.size();
    float degree=0,dis=0;
    for(int i = 0; i < count; i++) {
        degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        dis = scan->ranges[i];
        if(degree>0 && dis>1){
            ROS_INFO(" [%f, %f]", degree, dis);
        }
        
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
