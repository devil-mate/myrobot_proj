//
// Created by jl on 2020/6/24.
//

#include <iostream>
#include "GamePad.h"
#include <sys/time.h> //linux 定时器
#include <math.h>
#define x_min 0
#define x_max 200
#define y_min 0
#define y_max 200
//omega边界10°~80°
#define PI  3.1415926
#define OMEGA_MAX 80*PI/180.0
#define OMEGA_MIN 10*PI/180.0
#define cMIN(a, b) (((a) > (b)) ? (b) : (a))
#define cMAX(a, b) (((a) > (b)) ? (a) : (b))
#define cABS(a)	   (((a)>=0)?(a):(-(a)))

//#define BoundaryLimit(a,b,c)  do {                                          \  
// #define BoundaryLimit(a,b,c) do {                                         \                                    
//     // if(a<b){\
//     //     a=b;\
//     // }\   
//     // if(a>c){ \
//     //     a=c;\
//     // }\
//     // a=a;\   
//     ;    \                                 
//     } while(false);

#define BoundaryLimit(a,b,c) do { \
    if(a<b) a=b;\
    if(a>c) a=c;\
    a=a;\
  } while(false)

GamePad::GamePad(QWidget *parent)
        : QWidget(parent)
{
    omega_ = 0;
    throttle_ =0;
    setAutoFillBackground(true);
    setPalette(QPalette(Qt::white));
    resize(200,200);
    setMinimumSize(100,100);
    mouseX=width()/2;
    mouseY=height()/2;
    tim=new QTimer(this);
    connect(tim,&QTimer::timeout,this,[=]{
        emit keyNumchanged(getKeyNum());
    });
    connect(this,&GamePad::keyNumchanged,this,[=](int num){
        //qDebug()<<num<<endl;
    });
    // 油门和角速度计算
    calculateTimer = new QTimer(this);
    connect(calculateTimer,SIGNAL(timeout()),this,SLOT(updateCmdData()));
    calculateTimer->start(100);

}

GamePad::~GamePad()
{
    // delete QTimer;  //QT类不需要delete，删除基类的时候会自动删除？
}
void GamePad::paintEvent(QPaintEvent *){

    QPainter painter(this);

    int side = qMin(width(), height());

    padR=side/2; //底盘半径
    padX=padR;//底盘圆心
    padY=padR;//底盘圆心
    handleR=padR/4;//摇杆圆半径
    int handleMaxR=padR-handleR;
    QColor handleColor(Qt::gray);
    //加载底盘图像
    painter.save();

    painter.scale(side / 400.0, side / 400.0);//坐标会随窗口缩放
    painter.drawPixmap(0, 0, QPixmap(":/image/pad.png"));
    painter.restore();

    //自绘底盘
    painter.save();
    QRadialGradient RadialGradient(padR,padR,padR*3,padR,padR);//圆心2，半径1，焦点2
    RadialGradient.setColorAt(0,QColor(90,90,90,127));//渐变
    RadialGradient.setColorAt(1,QColor(255,255,255,190));//渐变
    painter.setBrush(RadialGradient);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPoint(padR,padR),side/2,side/2);//大圆盘
    painter.restore();

    //   painter.drawText(20,20,tr("%1,%2,%3").arg(mouseX).arg(mouseY).arg(handPaddis));

    if(!mousePressed){//鼠标没按下则摇杆恢复到底盘中心
        mouseX=padX;
        mouseY=padY;
    }
    handPadDis=Pointdis(padR,padR,mouseX,mouseY);
    if(handPadDis<=handleMaxR){
        handleX=mouseX;
        handleY=mouseY;
    }
    else {
        handleX=(int)(handleMaxR*(mouseX-padX)/handPadDis+padX);
        handleY=(int)(handleMaxR*(mouseY-padY)/handPadDis+padY);
    }
    // painter.drawText(200,200,tr("%1,%2,%3").arg(handleX).arg(handleY).arg(handPaddis));
    painter.setPen(Qt::NoPen);
    painter.setBrush(handleColor);
    painter.drawEllipse(QPoint(handleX,handleY),handleR,handleR);//摇杆
}
void GamePad::mouseMoveEvent(QMouseEvent* event){
    static bool r=false;
    mouseX=event->pos().x();
    mouseY=event->pos().y();
    // std::cout<<"mouseX:  " <<mouseX <<", mouseY: "<<mouseY<<std::endl;
    caclulateCmdData();
    if(r==true){
        update(); //qwidget的方法。
        r=false;
    }
    else{
        r=true;
    }
}
void GamePad::mouseReleaseEvent(QMouseEvent* event){
    mouseX=width()/2;
    mouseY=height()/2;
    tim->stop();
    mousePressed=false;
    emit keyNumchanged(GamePad::stop);
    // std::cout<<"mouseX:  " <<mouseX <<", mouseY: "<<mouseY<<std::endl;
    caclulateCmdData();
    update();
}
void GamePad::mousePressEvent(QMouseEvent* event){
    // mouseX=event->pos().x();
    // mouseY=event->pos().y();
    tim->start(100);
    mousePressed=true;
    //std::cout<<"mouseX:  " <<mouseX <<", mouseY: "<<mouseY<<std::endl;
    update();
}

double GamePad::Pointdis(int a,int b,int x,int y){
    return sqrt((double)((x-a)*(x-a)+(y-b)*(y-b)));
}
int GamePad::getKeyNum(){
    int x,y;
    int keynum;
    x=(int)(handleX*3.0/(padR*2));
    y=(int)(handleY*3.0/(padR*2));
    keynum=3*y+x;
    return keynum;
}
void  GamePad::updateCmdData(){
    // BoundaryLimit(mouseX,x_min,x_max);
    // BoundaryLimit(mouseY,y_min,y_max);
    // if(mouseX<x_min)
    //     mouseX = x_min;
    // if(mouseX>x_max)
    //     mouseX = x_max;
    // if(mouseY<y_min)
    //     mouseY = y_min;
    // if(mouseY>y_max)
    //     mouseY = y_max;
    //  std::cout<<"====mouseX:  " <<mouseX <<", mouseY: "<<mouseY<<std::endl;
}
bool GamePad::caclulateCmdData()
{
    BoundaryLimit(mouseX,x_min,x_max);
    BoundaryLimit(mouseY,y_min,y_max);
    // std::cout<<"mouseX:  " <<mouseX <<", mouseY: "<<mouseY<<std::endl;
    // 模
    throttle_ = (float)sqrt(pow(mouseX-100,2)+pow(mouseY-100,2))/100.0;
    if(mouseY>100){
        throttle_ = -throttle_;
    }
    //方向
    if((mouseY-100)!=0){
        omega_ = (float)atan((float)(mouseX-100)/(float)(100-mouseY));
    }else if(mouseX!=100){
        omega_=PI/2;
    }else{
        omega_=0;
    }
    // std::cout<<"----atan_omega= "<<omega_<<std::endl;
    //原地旋转
    if(cABS(omega_)>OMEGA_MAX){
        throttle_ =0;
        omega_=OMEGA_MAX;
    }

    //角速度为0
    if(cABS(omega_)<OMEGA_MIN){
        omega_ =OMEGA_MIN;
    }
     //std::cout<<"----Min_atan_omega= "<<omega_<<std::endl;
    //归一化,浮点计算，结果不会是刚好为0；10°到80°（1.39~0.17弧度）
    omega_ =(omega_-OMEGA_MIN)/(OMEGA_MAX-OMEGA_MIN);
    //std::cout<<"----omega_-OMEGA_MIN= "<<omega_-OMEGA_MIN<<"M_x"<<std::endl;
    BoundaryLimit(throttle_,-1,1);//
    //忽略小于阈值的角速度，阈值0.05
    if(cABS(omega_)<0.05){
        omega_=0;
    }
    // std::cout<<"====throttle_:  " <<throttle_ 
    //     <<", omega_:"<<omega_<<std::endl;

}