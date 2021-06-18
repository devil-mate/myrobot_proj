//
// Created by jl on 2020/6/5.
//

#ifndef SRC_IMGPROCESSNODE_H
#define SRC_IMGPROCESSNODE_H

#include "ros/ros.h"
#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "PreProcessImg.h"
class ImgProcessNode {
public:
    ImgProcessNode();
    ~ImgProcessNode();

private:
    ros::NodeHandle n;
    ros::Publisher imgPublisher;
    ros::ServiceServer blurServer;
    PreProcessImg *preProcessImg;
    ros::Timer pubImgTimer_;
    void pubImgTick(const ros::TimerEvent& e);

};


#endif //SRC_IMGPROCESSNODE_H
