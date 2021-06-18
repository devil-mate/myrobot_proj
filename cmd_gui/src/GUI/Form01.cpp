//
// Created by jl on 2020/6/3.
//

#include "ros/ros.h"

#include "Form01.h"
#include "gui_node.h"
using namespace cv;
Form01::Form01(QWidget *parent):QWidget(parent) {
    // preProcessImg_ = new PreProcessImg();
    initWidget();
}
//Form01::Form01(QWidget *parent,gui_node *guiNode ):QWidget(parent),guiNode_(NULL) {
//    preProcessImg_ = new PreProcessImg();
//    guiNode_= guiNode;
//    initWidget();
//}
Form01::~Form01() {

}

void Form01::initWidget() {
    this->resize(QSize(1280,1024));

    //
    label=new QLabel(this);
    label->setText(QObject::tr("this is  Form01,picture"));
    label->move(1000,10);
//    showPic();
    //


    backMainButton = new QPushButton(this);
    backMainButton->setText("openPic");
    backMainButton->move(1000,30);
    connect(backMainButton,SIGNAL(clicked()),this,SLOT(on_backMainButton_click()));
//
    pictureLabel_ = new QLabel(this);
    pictureLabel_->resize(1280,1024);

    //TODO 显示opencv图像
    scrollArea_=new QScrollArea(this);
    scrollArea_->setBackgroundRole(QPalette::Dark);
    scrollArea_->setWidget(pictureLabel_);
    scrollArea_->resize(800,600);
    scrollArea_->widget()->resize(1280,1024);
    scrollArea_->move(0,00);
    //滑动条
    lightSlider_ =new QSlider(Qt::Horizontal,this);
    lightSlider_->move(1000,100);
    lightSlider_->setMinimum(0);
    lightSlider_->setMaximum(255);
    lightSlider_->setValue(60);
    connect(lightSlider_,SIGNAL(valueChanged(int)),this,SLOT(on_lightSlider_ValueChanged(int)));


}
void Form01::on_backMainButton_click() {
//    this->hide();
    showPic();
}

void Form01::showPic() {
    QString filename;

    //文件选择
    filename=QFileDialog::getOpenFileName(this,
                                          tr("选择图像"),
                                          "",
                                          tr("Images (*.png *.bmp *.jpg *.tif *.GIF )"));
    if(filename.isEmpty()){
        return;
    }

    QImage* img=new QImage;

    if(! ( img->load(filename) ) ) //加载图像
    {
        QMessageBox::information(this,
                                 tr("打开图像失败"),
                                 tr("打开图像失败!"));
        delete img;
        return;
    }
//    QLabel *pictureLabel;
//    pictureLabel = new QLabel(this);
//    pictureLabel->resize(1280,1024);
//
//    //TODO 显示opencv图像
//    QScrollArea *scrollArea;
//    scrollArea=new QScrollArea(this);
//    scrollArea->setBackgroundRole(QPalette::Dark);
//    scrollArea->setWidget(pictureLabel_);
//    scrollArea->resize(800,600);
//    scrollArea->widget()->resize(1280,1024);
//    scrollArea->move(0,00);
////        scrollArea->setWidgetResizable(true);
    pictureLabel_->setPixmap(QPixmap::fromImage(*img));
    scrollArea_->widget()->resize(1280,1024);


}
void Form01::showOpencvPic(){
//    QString filename;
//
//    filename = "/home/jl/share_jl/LBASTest/LBAS_110_4_01.bmp";
    cv::Mat tempReadImg = cv::imread("/home/jl/share_jl/LBASTest/LBAS_150_5_01.bmp");
    cv::Mat sourceImg= tempReadImg.clone();
    dealImg_ = tempReadImg.clone();
    //文件选择
//    filename=QFileDialog::getOpenFileName(this,
//                                          tr("选择图像"),
//                                          "",
//                                          tr("Images (*.png *.bmp *.jpg *.tif *.GIF )"));
//    if(filename.isEmpty()){
//        return;
//    }

    QImage* img=new QImage;
    QImage tempImg((const unsigned char*)(sourceImg.data),
                   sourceImg.cols, sourceImg.rows, sourceImg.step, QImage::Format_RGB888);

    if( sourceImg.empty() ) //加载图像
    {
        QMessageBox::information(this,
                                 tr("打开图像失败"),
                                 tr("打开图像失败!"));
        delete img;
        return;
    }
    pictureLabel_->setPixmap(QPixmap::fromImage(tempImg));
    scrollArea_->widget()->resize(sourceImg.cols,sourceImg.rows);

}

void Form01::on_lightSlider_ValueChanged(int) {
    int sliderPos = lightSlider_->value();
    int param = (int)sliderPos/10;

    label->setText(QString::number(sliderPos));
//  TODO 服务请求，处理图像
    Mat dstImg;
    QImage image;
    image = pictureLabel_->pixmap()->toImage();
    cv::Mat mat(image.height(), image.width(),
                CV_8UC4,
                (void*)image.constBits(),
                image.bytesPerLine());

    //preProcessImg_->imageBlur(mat,dstImg,param);


    QImage tempImg((const unsigned char*)(dstImg.data),
                   dstImg.cols, dstImg.rows, dstImg.step, QImage::Format_RGB888);
    pictureLabel_->setPixmap(QPixmap::fromImage(tempImg));

}

void Form01::updatePic(const cv::Mat &pic) {
    if (pic.empty()){
        ROS_WARN_STREAM("pic empty");
        return;
    }
    QImage tempImg((const unsigned char*)(pic.data),
                   pic.cols, pic.rows, pic.step, QImage::Format_RGB888);
    pictureLabel_->setPixmap(QPixmap::fromImage(tempImg));

}

void Form01::updateState(const gui_node *guiNode) {
    if(guiNode==NULL){
        return;
    }
    cv::Mat pic= guiNode->dealPic_;
    QImage tempImg((const unsigned char*)(pic.data),
                   pic.cols, pic.rows, pic.step, QImage::Format_RGB888);
    pictureLabel_->setPixmap(QPixmap::fromImage(tempImg));
}
