//
// Created by jl on 2020/5/28.
//

#ifndef SRC_GUI_NODE_H
#define SRC_GUI_NODE_H
#include "ros/ros.h"

#include <QtWidgets/QApplication>
#include "mainwindow.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "pabi_gui/ControlBoxMsg.h"
#include "geometry_msgs/Twist.h"
#include "pabi_gui/ParamSet.h"
//多线程--条件变量
#include <thread>
#include <condition_variable>
#include <mutex>
// #include "pabi/CameraControlSrv.h"

class gui_node {
public:
    gui_node();
    ~gui_node();

    cv::Mat dealPic_;
    // pabi::CameraControlSrv cameraControlSrv;

private:
    ros::NodeHandle n;
    ros::ServiceClient cameraControlClient;

    ros::Publisher cmdPublisher;
    ros::Publisher cmd_vel_Publisher;
    ros::Publisher dynParam_Publisher;
    ros::Subscriber dataShowSubscriber;
    ros::Subscriber imgSubscriber;
    void initParam();
    void imageSubCallback(const sensor_msgs::Image::ConstPtr& msg);
    void guiThread();
    void sendCmdTick(const ros::TimerEvent& e);
    void setToMsg();
    double freq_;
    std::thread *gui_Thread;
    std_msgs::Float32 Msg_count_;
    pabi_gui::ControlBoxMsg controlBoxMsg_;
    geometry_msgs::Twist twistMsg_;
    //pabi_gui::ParamSet paramMsg_;
    std_msgs::Float32MultiArray paramMsg_;

    ros::Timer pubTimer;

    MainWindow *mainWindow;
    ControlBox *controlBox_;
    std::mutex mutex_;
    std::condition_variable conditionVariable_;

    float manualSpeedMax_,autoSpeedMax_,omegaMax_;


};


#endif //SRC_GUI_NODE_H
