//
// Created by jl on 2020/5/28.
//

#include "gui_node.h"
#include <QtWidgets/QApplication>
#include <cv_bridge/cv_bridge.h>



gui_node::gui_node():freq_(20.0),mainWindow(NULL),manualSpeedMax_(10),autoSpeedMax_(10),
        omegaMax_(1) {

// 显示界面线程gui
    gui_Thread= new std::thread(&gui_node::guiThread,this);
// 后台处理消息
//    TODO 等待gui加载完成.线程同步.{建立了maiwindow后，才能使用}
    sleep(1);
    //mainWindow指针赋初值,可用于之后判断指针是否指向需要的内存（new）,这种方式不行？
    dynParam_Publisher =n.advertise<std_msgs::Float32MultiArray>("param_set",10);
    // dynParam_Publisher =n.advertise<pabi_gui::ParamSet>("param_set",10);
    cmdPublisher = n.advertise<cmd_gui::ControlBoxMsg>("cmd_gui",10);
    cmd_vel_Publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",5);
    pubTimer = n.createTimer(ros::Duration(1.0/std::max(freq_,1.0)),&gui_node::sendCmdTick,this);
//     imgSubscriber = n.subscribe<sensor_msgs::Image>("imgTopic",1,&gui_node::imageSubCallback,this);
//     cameraControlClient = n.serviceClient<ros_gui_qt::CameraControlSrv>("cameraControlService");
// //    sleep(2);

    //
    initParam();

    ROS_INFO_STREAM("cmd_gui start--");
}

gui_node::~gui_node() {
    //gui_Thread->join();
    delete mainWindow;
//TODO 结束另一个进程（界面进程）
    delete gui_Thread;



}
void gui_node::guiThread() {
    int ax=0;
    char *aa[]={0};
    QApplication a(ax,aa);
    //std::unique_lock<std::mutex> guardLock(mutex_);
    mainWindow = new MainWindow(NULL);
    mainWindow->initParamParent(this);
//    guardLock.unlock();
    mainWindow->show();
    conditionVariable_.notify_one();
    a.exec();
}
void gui_node::initParam(){
     n.getParam("manualSpeedMax",manualSpeedMax_);
     n.getParam("autoSpeedMax",autoSpeedMax_);
     n.getParam("omegaMax",omegaMax_);
}
void gui_node::sendCmdTick(const ros::TimerEvent& e) {

    static bool flag=false;
    static int8_t tempx=0;
    static float extime=0;
//    while (!flag){
//        std::unique_lock<std::mutex> guardLock(mutex_);
//        conditionVariable_.wait(guardLock);
//        guardLock.unlock();
//        flag=true;
//    }

    setToMsg();
    //TODO 两个线程同时调用updateState(this)段错误问题
    cmdPublisher.publish(controlBoxMsg_);
    cmd_vel_Publisher.publish(twistMsg_);
    dynParam_Publisher.publish (paramMsg_);
    //ROS_INFO_STREAM("gui_Node pub msg : "<< controlBoxMsg_.autoSpeed);

}

void gui_node::imageSubCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv::Mat tempMat = cv_bridge::toCvShare(msg, "bgr8")->image;
        dealPic_ = tempMat;
//        mainWindow->updateLabelPic(tempMat);
//        cv::imshow( "video", tempMat );
//        cv::waitKey(10);

    }
    catch (...){
        ROS_ERROR_STREAM("callback error,exit");
        throw -1;
    }
}

void gui_node::setToMsg() {

//    //方式1： 直接调用mainWindow类的成员label；
//    //方式2：mainWindow->update 方法，统一更新控件状态,参数传递。
////    mainWindow->label->setText(QString::number(controlDataS.testData));
    ControlBox_Send_S tempControlDataS;
    if(mainWindow ==NULL){ 
        return;
    }
    mainWindow->getControlData(tempControlDataS);
    // controlBox 给出速度百分比；
     controlBoxMsg_.autoSpeed = tempControlDataS.autoSpeed * autoSpeedMax_;
    
    controlBoxMsg_.forward =  tempControlDataS.forward;
    controlBoxMsg_.backward =  tempControlDataS.backward;
    controlBoxMsg_.controlMode =  tempControlDataS.mode; //手自动复位
    twistMsg_.linear.x =controlBoxMsg_.manualSpeed = tempControlDataS.manualSpeed * manualSpeedMax_;
    twistMsg_.angular.z =controlBoxMsg_.omega =tempControlDataS.omega*omegaMax_;
    static Params_s tempParams;
    mainWindow->getDynParam(tempParams);
    paramMsg_.data.resize(5);
    for(int8_t i=0;i<5;i++){
        // paramMsg_.data.push_back(tempParams.pidParam[i]);
        paramMsg_.data[i]=tempParams.pidParam[i];
    }
    
    


}


int main(int argc,char *argv[])
{
    try{
        ros::init(argc,argv,"xy");
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);
        gui_node guiNode;
        ros::spin();
    }catch(...){
        ROS_ERROR("error,exit");
        ros::shutdown();
    }
    return 0;

}



