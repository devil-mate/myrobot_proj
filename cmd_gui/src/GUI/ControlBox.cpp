//
// Created by jl on 2020/6/12.
//


#include "gui_node.h"
#include "ControlBox.h"

ControlBox::ControlBox(QWidget *parent):QDialog(parent) {
    memset(&controlBoxSendS_,0,sizeof(controlBoxSendS_));
    initWidget();
}

ControlBox::~ControlBox() {

}

void ControlBox::initWidget() {
    this->resize(QSize(800,500));
    this->setWindowTitle("ControlWin");
//    显示
    label=new QLabel(this);
    label->setText(QObject::tr("this is  controlBoxForm"));
    label->resize(100,50);
    label->move(10,10);

    backMainButton = new QPushButton(this);
    backMainButton->setText("BackMain");
    backMainButton->move(40,50);
    connect(backMainButton,SIGNAL(clicked()),this,SLOT(on_backMainButton_click()));

    gamePad_ = new GamePad(this);
//    gamePad->resize(100,100);
    gamePad_->move(400,200);
//speed
    autoSpeedSlider_ =new QSlider(Qt::Horizontal,this);
    autoSpeedSlider_->setWindowTitle("autoSpeed");
    autoSpeedSlider_->move(500,50);
    autoSpeedSlider_->setMinimum(-100);
    autoSpeedSlider_->setMaximum(100);
    autoSpeedSlider_->setValue(0);
    int sliderPos = autoSpeedSlider_->value();
    float autoSpeed = (float)sliderPos/100.0;
    controlBoxSendS_.manualSpeed = autoSpeed; //用manualSpeed表示线速度
    connect(autoSpeedSlider_,SIGNAL(valueChanged(int)),this,SLOT(on_autoSpeedSlider_ValueChanged(int)));
//speed w
    Speed_W_Slider_ =new QSlider(Qt::Horizontal,this);
    Speed_W_Slider_->move(500,100);
    Speed_W_Slider_->setMinimum(-100);
    Speed_W_Slider_->setMaximum(100);
    Speed_W_Slider_->setValue(0);
    int sliderPos_w = autoSpeedSlider_->value();
    float speedW = (float)sliderPos_w/100.0;
    controlBoxSendS_.omega = speedW; 
    connect(Speed_W_Slider_,SIGNAL(valueChanged(int)),this,SLOT(on_speedWSlider_ValueChanged(int)));
// //
//     picLightSlider_ =new QSlider(Qt::Horizontal,this);
//     picLightSlider_->setWindowTitle("pic Light");
//     picLightSlider_->move(500,100);
//     picLightSlider_->setMinimum(0);
//     picLightSlider_->setMaximum(2550);
//     picLightSlider_->setValue(1000);
//     int sliderPos1 = picLightSlider_->value();
//     float brightnessValue = (float)sliderPos1/10.0;
//     controlBoxSendS_.cameraParamS.brightnessValue =brightnessValue;
//     connect(picLightSlider_,SIGNAL(valueChanged(int)),this,SLOT(on_picLightSlider_ValueChanged(int)));
// 参数自动调整使能
//     autoParamAdjCheck_ =new QCheckBox(this);
//     autoParamAdjCheck_->setText("autoParamAdj");
//     autoParamAdjCheck_->move(600,100);
//     connect(autoParamAdjCheck_,SIGNAL(clicked(bool)),this,SLOT(on_autoParamAdjCheck_click()));
// // 相机触发模式
//     cameraMode_ =new QCheckBox(this);
//     cameraMode_->setText("cameraMode");
//     cameraMode_->move(600,150);
//     connect(cameraMode_,SIGNAL(clicked(bool)),this,SLOT(on_cameraMode_click()));
// // 曝光时间
//     exposureTimeLineEd_ =new QLineEdit(this);
//     QDoubleValidator * exTimeValidator = new QDoubleValidator();
//     exTimeValidator->setRange(0,30000);
//     exposureTimeLineEd_->setValidator(exTimeValidator);
//     exposureTimeLineEd_->setText("9000");
//     exposureTimeLineEd_->move(600,200);
//     connect(exposureTimeLineEd_,SIGNAL(editingFinished()),this,SLOT(finished_exposureTime()));
// //增益
//     picGainLineEd_ =  new QLineEdit(this);
//     picGainLineEd_->move(600,230);
//     QDoubleValidator * gainValidator = new QDoubleValidator();
//     gainValidator->setRange(0,15);
//     picGainLineEd_->setValidator(gainValidator);
//     picGainLineEd_->setText("2");

//     connect(picGainLineEd_,SIGNAL(editingFinished()),this,SLOT(finished_picGain()));
// //  cameraIndex
//     cameraIndexBox_ = new QComboBox(this);
//     cameraIndexBox_->setValidator(new QIntValidator(0,5));
//     cameraIndexBox_->move(500,150);
//     cameraIndexBox_->addItem("0",0);
//     cameraIndexBox_->addItem("camera1",1);
//     cameraIndexBox_->addItem("camera2",2);
//     cameraIndexBox_->addItem("camera3",3);
// //    cameraIndexBox_->set
//     connect(cameraIndexBox_,SIGNAL(currentIndexChanged(int)),this,SLOT(currentChanged_cameraIndex()));
//控制运动平台相关控件
    initControlWidget();
}
void ControlBox::initControlWidget() {

    QButtonGroup *pButtonGroup = new QButtonGroup(this);
    pButtonGroup->setExclusive(true);
    manualButton_ = new QPushButton(this);
    manualButton_->setCheckable(true); // 设置可选中
    manualButton_->setText(QString("manual"));
    autoModeButton_ = new QPushButton(this);
    autoModeButton_->move(200,320);
    autoModeButton_->setCheckable(true); // 设置可选中
    autoModeButton_->setText(QString("auto"));
    resetButton_ = new QPushButton(this);
    resetButton_->move(200,340);
    resetButton_->setCheckable(true); // 设置可选中
    resetButton_->setText(QString("reset"));
    pButtonGroup->addButton(manualButton_,1);
    pButtonGroup->addButton(autoModeButton_,2);
    pButtonGroup->addButton(resetButton_,3);

    QVBoxLayout *modeVBoxLayout = new QVBoxLayout();
    modeVBoxLayout->addStretch(1);//添加占位符 占的比例是2
    modeVBoxLayout->addWidget(manualButton_);
//    modeVBoxLayout->setStretchFactor(manualButton_,2);//设置控件的比例
    modeVBoxLayout->addWidget(autoModeButton_);
    modeVBoxLayout->addWidget(resetButton_);
    modeVBoxLayout->addStretch(1);//添加占位符 占的比例是2
    modeVBoxLayout->setMargin(10);//设置外围边框
//    setLayout(modeVBoxLayout);//将三个按钮的控件布局到 widget 上

    QHBoxLayout *qhBoxLayout = new QHBoxLayout();
    qhBoxLayout->addStretch(1);
    qhBoxLayout->addLayout(modeVBoxLayout);
    qhBoxLayout->addStretch(2);
    setLayout(qhBoxLayout);
    connect(pButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(on_ButtonGroup_Clicked(int)));

//前进后退
    forwardButton_ =new QPushButton(this);
    forwardButton_->move(200,300);
    forwardButton_->setCheckable(true); // 设置可选中
    forwardButton_->setText(QString("Forward"));
    connect(forwardButton_, SIGNAL(pressed()), this, SLOT(pressed_forward_clicked()));
    connect(forwardButton_, SIGNAL(released()), this, SLOT(release_forward_clicked()));

    backButton_ =new QPushButton(this);
    backButton_->move(200,330);
    backButton_->setCheckable(true); // 设置可选中
    backButton_->setText(QString("backward"));
    connect(backButton_, SIGNAL(pressed()), this, SLOT(pressed_backward_clicked()));
    connect(backButton_, SIGNAL(released()), this, SLOT(release_backward_clicked()));
// //    上下左右/前进后退+角速度
//     upButton_ =new QPushButton(this);
//     upButton_->move(500,300);
//     upButton_->setText("up");
//     downButton_ =new QPushButton(this);
//     downButton_->move(500,400);
//     downButton_->setText("down");
//     leftButton_ =new QPushButton(this);
//     leftButton_->move(450,350);
//     leftButton_->setText("left");
//     rightButton_ =new QPushButton(this);
//     rightButton_->move(550,350);
//     rightButton_->setText("right");




}

void ControlBox::on_backMainButton_click() {
    this->hide();
}
void ControlBox::on_picLightSlider_ValueChanged(int){
    int sliderPos = picLightSlider_->value();
    float brightnessValue = (float)sliderPos/10.0;
    controlBoxSendS_.cameraParamS.brightnessValue =brightnessValue;
    // guiNode_->cameraControlSrv.request.brightnessValue= controlBoxSendS_.cameraParamS.brightnessValue;
    // ROS_DEBUG_STREAM("brightnessValue: "<<guiNode_->cameraControlSrv.request.brightnessValue );
    serverCall();


}


void ControlBox::on_autoParamAdjCheck_click(){
    if(autoParamAdjCheck_->isChecked()){
        controlBoxSendS_.cameraParamS.autoParamAdj= true;
    } else{
        controlBoxSendS_.cameraParamS.autoParamAdj= false;
    }
    // guiNode_->cameraControlSrv.request.autoParamAdj = controlBoxSendS_.cameraParamS.autoParamAdj;
    // ROS_DEBUG("autoParamAdj: %d",guiNode_->cameraControlSrv.request.autoParamAdj );
    serverCall();
}

void ControlBox::initParamParent(gui_node *guiNode) {
    guiNode_= guiNode;

}

void ControlBox::on_cameraMode_click() {
    if(cameraMode_->isChecked()){
        controlBoxSendS_.cameraParamS.cameraTriggerMode = true;
    } else{
        controlBoxSendS_.cameraParamS.cameraTriggerMode= false;
    }
    // guiNode_->cameraControlSrv.request.triggerMode =  controlBoxSendS_.cameraParamS.cameraTriggerMode;
    serverCall();
    // ROS_DEBUG("triggerMode: %d",guiNode_->cameraControlSrv.request.triggerMode );
}

int ControlBox::serverCall() {
    // guiNode_->cameraControlClient.call(guiNode_->cameraControlSrv);
    return 0;
}

void ControlBox::finished_exposureTime() {
    double extime = exposureTimeLineEd_->text().toDouble();
    // guiNode_->cameraControlSrv.request.exposureTime = extime;
    serverCall();
    ROS_DEBUG_STREAM("exposureTime: "<<extime );

}

void ControlBox::finished_picGain() {
    double picGain = picGainLineEd_->text().toDouble();
    // guiNode_->cameraControlSrv.request.gain = picGain;
    serverCall();
    ROS_DEBUG_STREAM("gain: "<<picGain );
}

void ControlBox::currentChanged_cameraIndex() {
    int camIndex= cameraIndexBox_->currentData().toInt();
    // guiNode_->cameraControlSrv.request.cameraIndex = camIndex;
    serverCall();
    ROS_DEBUG_STREAM("====cameraIndex: "<<camIndex );

}
void ControlBox::on_ButtonGroup_Clicked(int buttonId){
    switch (buttonId){
        default:
            break;
        case 1:
            controlBoxSendS_.mode = 1;
            ROS_DEBUG("manual mode: %d",controlBoxSendS_.mode);
            break;
        case 2:
            controlBoxSendS_.mode = 2;
            ROS_DEBUG("auto mode: %d",controlBoxSendS_.mode);
            break;
        case 3:
            controlBoxSendS_.mode = 3;
            controlBoxSendS_.forward = false;
            controlBoxSendS_.backward = false;
            ROS_DEBUG("reset mode: %d",controlBoxSendS_.mode);
            break;
    }
}
// 控制相关用topic
void ControlBox::on_autoSpeedSlider_ValueChanged(int) {
    int sliderPos = autoSpeedSlider_->value();
    float autoSpeed = (float)sliderPos/100.0; //-1~1 比例
//    float autoSpeed = (float)sliderPos/1.0;
    controlBoxSendS_.autoSpeed=autoSpeed;
    controlBoxSendS_.manualSpeed =autoSpeed;
////    label->setText(QString::number(autoSpeed));
//    guiNode_->cameraControlSrv.request.autoSpeed = controlBoxSendS_.autoSpeed;
//    ROS_DEBUG_STREAM("autoSpeed: "<<guiNode_->cameraControlSrv.request.autoSpeed );
//    serverCall();
}
void ControlBox::on_speedWSlider_ValueChanged(int){
    int sliderPos = autoSpeedSlider_->value();
    float omegaSpeed = (float)sliderPos/100.0; //-1~1 比例
    controlBoxSendS_.omega=omegaSpeed;

}

void ControlBox::pressed_forward_clicked() {
    if(controlBoxSendS_.mode==1){
        int sliderPos = autoSpeedSlider_->value();
        float manualSpeed = (float)sliderPos/10.0;
        controlBoxSendS_.manualSpeed=manualSpeed;
        ROS_DEBUG_STREAM("forward: manualSpeed= "<<controlBoxSendS_.manualSpeed );
    }
    else if(controlBoxSendS_.mode==2){
        controlBoxSendS_.forward = true;
        controlBoxSendS_.backward = false;
        ROS_DEBUG("autoMode: start = %d ",controlBoxSendS_.forward );
    }
}

void ControlBox::release_forward_clicked() {
    controlBoxSendS_.manualSpeed = 0;
    ROS_DEBUG_STREAM("forward: manualSpeed= "<<controlBoxSendS_.manualSpeed );
}

void ControlBox::pressed_backward_clicked() {
    if(controlBoxSendS_.mode==1){
        int sliderPos = autoSpeedSlider_->value();
        float manualSpeed = -(float)sliderPos/10.0;
        controlBoxSendS_.manualSpeed=manualSpeed;
        ROS_DEBUG_STREAM("backward: manualSpeed= "<<controlBoxSendS_.manualSpeed );
    }
    else if(controlBoxSendS_.mode==2){
        controlBoxSendS_.forward = false;
        controlBoxSendS_.backward = true;
        ROS_DEBUG("autoMode: start = %d ",controlBoxSendS_.backward );
    }
}

void ControlBox::release_backward_clicked() {
    controlBoxSendS_.manualSpeed = 0;
    ROS_DEBUG_STREAM("backward: manualSpeed= "<<controlBoxSendS_.manualSpeed );
}
void ControlBox::keyPressEvent(QKeyEvent *event){

    if(event->key()==Qt::Key_Up){
        upButton_->setFocus();
        label->setText("key up");
    }
    else if(event->key()==Qt::Key_Down){
        downButton_->setFocus();
        label->setText("key down");
    }
    else if(event->key()==Qt::Key_Left){
        label->setText("key left");
        leftButton_->setFocus();
    }
    else if(event->key()==Qt::Key_Right){
        rightButton_->setFocus();
        label->setText("key right");
    }
    else if(event->key()==Qt::Key_Up && event->key()==Qt::Key_Left ){
        upButton_->setFocus();
        leftButton_->setFocus();
        label->setText("key up and left");
    }
    else if(event->key()==Qt::Key_Up && event->key()==Qt::Key_Right){
        upButton_->setFocus();
        rightButton_->setFocus();
        label->setText("key up and right");
    }
    else if(event->key()==Qt::Key_Down && event->key()==Qt::Key_Right){

    }
}

void ControlBox::getControlData(ControlBox_Send_S &controlData)
{
    controlData.autoSpeed = controlBoxSendS_.autoSpeed;
    if(gamePad_->throttle_!=0){
        controlData.manualSpeed = gamePad_->throttle_;
        controlBoxSendS_.manualSpeed =0;
    } else if(controlBoxSendS_.manualSpeed!=0){
        controlData.manualSpeed = controlBoxSendS_.manualSpeed;
        gamePad_->throttle_=0;
    }else{
        controlData.manualSpeed =0;
    }

    if(gamePad_->omega_!=0){
        controlData.omega = gamePad_->omega_;
        controlBoxSendS_.omega=0;
    }else if(controlBoxSendS_.omega!=0){
        controlData.omega = controlBoxSendS_.omega;
        gamePad_->omega_=0;
    }else{
        controlData.omega =0;
    }
    controlData.forward = controlBoxSendS_.forward;
    controlData.backward = controlBoxSendS_.backward;
    controlData.mode= controlBoxSendS_.mode;
}
