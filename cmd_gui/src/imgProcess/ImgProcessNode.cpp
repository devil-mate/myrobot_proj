//
// Created by jl on 2020/6/5.
//

/*
 *
 */

#include "ImgProcessNode.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

using namespace cv;
ImgProcessNode::ImgProcessNode() {
    imgPublisher = n.advertise<sensor_msgs::Image>("imgTopic",1);
    preProcessImg = new PreProcessImg();
//    blurServer = n.advertiseService("blurService",&ImgProcessNode::blurServiceCallback,this);
    pubImgTimer_ = n.createTimer(ros::Duration(1.0/std::max(6.0,1.0)),&ImgProcessNode::pubImgTick,this);
}

ImgProcessNode::~ImgProcessNode() {

}

void ImgProcessNode::pubImgTick(const ros::TimerEvent &e) {
    cv::Mat tempImg;
//    opencv 的Mat 数据转换成 sensor_msgs::Image 发送
    sensor_msgs::ImagePtr imageMsg;
    tempImg=preProcessImg->getImage();
    imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tempImg).toImageMsg();
    imgPublisher.publish(imageMsg);
}

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"xx");
    ImgProcessNode imgProcessNode;

//    ros::Rate loop_rate(10);
//    while (ros::ok()){
//
//        ROS_INFO_STREAM("imgProcess");
//        ros::spinOnce();
//    }
    ros::spin();
}
