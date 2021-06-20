//
// Created by jl on 2020/4/17.
//

#include "PreProcessImg.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

PreProcessImg::PreProcessImg() {
    Path_s sPath;
    init(sPath);
//    readOneImg(sPath.picName);
    readFolderImg_gui(sPath.folderPath);
//    frameCapture(sPath.folderPath);
}

PreProcessImg::~PreProcessImg() {

}
bool PreProcessImg::readOneImg(std::string picName) {


    cv::Mat sourceImg=imread(picName,IMREAD_COLOR);
    if(!sourceImg.size){
        cout<<"read img fail! "<<picName<<endl;
        throw -1;
    }
    Mat image=Mat(cv::Size(sourceImg.cols,sourceImg.rows),CV_8UC1);
    image=sourceImg.clone(); //复制一张,不损坏原有照片
    //picProcess(image);
    if(image.empty()){
        cout<<"image is empty!"<<endl;
        return false;
    }
    Mat preImg,imgROI;
    generalProcess(image, preImg);
//    waitKey(0);
    // getROI(image,dst);
    getAutoROI(preImg,imgROI);
    // picShow(sourceImg);


   // picShow(sourceImg);
    //picShow(dst);

    return true;
}



void PreProcessImg::picProcess(cv::Mat &image) {
    //算平均值
    double_t matMean;
    cv::Scalar scalar = cv::mean(image);
    matMean = scalar.val[0];
    cout<<"matMean:"<<matMean<<endl;
    //滤波，{滤波后变模糊，清晰度评价指标当然变小}
    Mat dst=Mat(cv::Size(image.cols,image.rows),CV_8UC1);
//    imshow("source ",image);

    dst=image.clone(); //复制一张,不损坏原有照片
    GaussianBlur(image,dst,Size(3,3),0,0);
//    imshow("GaussianBlur ",dst);

    //清晰度评价1
    articulation_sobel(dst);
    //清晰度评价2
//    articulation_StdDev(image);


}


void PreProcessImg::picShow(cv::Mat &image) {
    namedWindow("Display",WINDOW_AUTOSIZE);
    imshow("Display",image);
    waitKey(0);
}
void PreProcessImg::init(Path_s &sPath) {
//    TODO 获得yaml文件的相对路径
    string yamlPath="/home/jl/catkin_ws/src/myROS_Proj/ros_gui_qt/config/config.yaml";
    string tempName;
    YAML::Node yamNode;
    try {
        yamNode = YAML::LoadFile(yamlPath);
        if (yamNode.IsNull()) {
            cout << "yamlNode is None !path" << yamlPath << endl;
            throw 1;
        }
        sPath.folderPath = yamNode["folderPath"].as<std::string>() + "/";
        tempName = yamNode["picName"].as<std::string>();
        sPath.picName = sPath.folderPath + tempName;
    }
    catch (...){
        cout<<"[Error] read yaml file fail: "<< yamlPath<<endl;
        throw -1;
    }

}

void PreProcessImg::readFolderImg(std::string folderPath) {
    vector<cv::String> picName;
    int num=0;
    Mat image;

    glob(folderPath, picName, false);
    num=picName.size();
    cout<<picName.size()<<endl;
    namedWindow("Display",WINDOW_AUTOSIZE);
    for(size_t i=0;i<num;i++){
        image=imread(picName[i],IMREAD_COLOR);
        if(!image.data){
            cout<<"图片数据错误"<<endl;
            continue;
        }
        picProcess(image);
        imshow("Display",image);
        if(waitKey(100)==27){
            cout<< "exit"<<endl;
            break;
        }
    }




}

void PreProcessImg::frameCapture(std::string folderPath) {
    VideoCapture videoCap(folderPath);//错误，不能这样用？只能是视频文件？
    if(!videoCap.isOpened()){
        cout<<"open faild! "<<folderPath<<endl;
        return;
    }
    namedWindow("videoCap", CV_WINDOW_AUTOSIZE);
    Mat capFrame;
    bool capOK;
    while(1){
        capOK=videoCap.read(capFrame);
        if(!capOK){
            cout<<"read Frame fail"<<endl;
            continue;
        }
        imshow("videoCap",capFrame);
        if(waitKey(20)==27){
            cout<< "exit"<<endl;
            break;
        }

    }
}

void PreProcessImg::articulation_sobel(cv::Mat &image) {
    Mat imageSobel;
    //方法1 sobel算子
    Sobel(image,imageSobel,CV_8UC1,1,1);
    //方法2 Laplacian 梯度{应用效果不好}
    //Laplacian(image,imageSobel,CV_8UC1,1,1);
    double meanValue = 0.0;
    meanValue = mean(imageSobel)[0];//
    cout<<"imageSobel imageSobelmatMean:"<<meanValue<<endl;

    stringstream meanValueStream;
    string meanValueString;
    //double 转string,方法1 借助stringstream ，它和std::cout效果一样；方法2，C++11中提供了方法to_string
//    meanValueStream << meanValue;
//    meanValueStream >> meanValueString;
//    meanValueString = "Articulation(Sobel Method): " + meanValueString;
    meanValueString="Articulation(Sobel Method): "+std::to_string(meanValue);
    putText(image, meanValueString, Point(20, 50), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 25), 2);

}

void PreProcessImg::articulation_StdDev(cv::Mat &image) {
    Mat meanValueImg,stdDevImg;
    meanStdDev(image,meanValueImg,stdDevImg);
    double meanValue = 0.0;
    meanValue = stdDevImg.at<double_t >(0,0);//
    string meanValueString;
    meanValueString="Articulation(stdDev Method): "+to_string(meanValue*meanValue);
    putText(image, meanValueString, Point(20, 50), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 25), 2);


}

void PreProcessImg::getROI(cv::Mat Img,cv::Mat &dstImg){
    cout<<"cols: "<<Img.cols<<"rows: "<<Img.rows<<endl;
    cv::Rect img_ROI=Rect(Img.cols/6,0,Img.cols-Img.cols/3,Img.rows);
    dstImg=Img(img_ROI);

}

void PreProcessImg::getAutoROI(Mat &Img, cv::Mat &dstImg){
    Mat grayImage,edge,tempImage;
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;
    ///////////////找轮廓
     //threshold(grayImage, grayImage, 50, 200, THRESH_OTSU);
//    Img = Img > 200;////////////？？
//    findContours(Img, contours, hierarchy, RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//   cout<<"contours num: "<<contours.size()<<endl;
//    cout<<"hierarchy size: "<<hierarchy.size()<<endl;
//    tempImage = Mat::zeros(Img.size(), CV_8UC1);
//    //绘制轮廓图
//    drawContours(tempImage, contours, -1, Scalar(100,50,100));
////     for (int i = 0; i < contours.size(); i++)
////     {
////         Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
////          drawContours(tempImage, contours, i, Scalar(100,50,100), CV_FILLED, 8, hierarchy);
////     }
//    imshow("轮廓图", tempImage);
    ////////////////直线边缘检测
    edgeDetection(Img);
    waitKey(0);

}

void PreProcessImg::generalProcess(cv::Mat &Img, cv::Mat &dstImg) {
    Mat grayImage,blurImg;
//    cvtColor(Img,grayImage,CV_BGR2GRAY);
//    imshow("grayImage",grayImage);

    blur(Img,dstImg,Size(3,3));
//    imshow("blur Img",blurImg);

//    imgEnhance(blurImg, dstImg);
//    imshow("imgEnhance Img",dstImg);

}

void PreProcessImg::imgEnhance(cv::Mat &Img, cv::Mat &dstImg) {
    Mat imgEnhance;
    Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, 0, 4, 0, 0, -1, 0);
    filter2D(Img, dstImg, CV_8UC1, kernel);

}

void PreProcessImg::edgeDetection(cv::Mat &Img) {
    Mat edgeCanny;
    cv::Canny(Img, edgeCanny, 100, 200, 3);
    imshow("canny",edgeCanny);
    vector<Vec4f> pLines;
    HoughLinesP(edgeCanny, pLines, 1, CV_PI / 180.0, 30, 60, 5);
    Scalar color = Scalar(0, 0, 255);
    cout<<"lines size:"<<pLines.size()<<endl;
    for (size_t i = 0; i < pLines.size(); i++) {
        Vec4f hline = pLines[i];
        float tempK=(hline[3]-hline[1])/(hline[2]-hline[1]+0.01);
        if(tempK<1 && tempK>=-1) {
            line(Img, Point(hline[0], hline[1]), Point(hline[2], hline[3]), color, 3, LINE_AA);
        }
    }
      imshow("效果图",Img);
}
//

void PreProcessImg::imageBlur(cv::Mat &Img, cv::Mat &dstImg,int param) {
    cv::Mat grayImage,blurImg;
    if(param<=0){
        param =1;
    }
    blur(Img,dstImg,Size(param,param));
}

void PreProcessImg::readFolderImg_gui(std::string folderPath) {
    int num;
    glob(folderPath, picName_, false);
    num=picName_.size();
    cout<<"file num:"<<picName_.size()<<endl;
    for(size_t i=0;i<num;i++){
        cout<< "picName"<<picName_[i]<<endl;
    }
}

cv::Mat PreProcessImg::getImage() {
    Mat tempImg(200,150,CV_8UC1);
    static int32_t picCounter=0;
    picCounter++;
    if (picCounter>=picName_.size()){
        return image_;
    }

    image_=imread(picName_[picCounter],IMREAD_COLOR);
    if(image_.empty()){
        cout<<"图片数据错误"<<endl;
        return tempImg;
    }
    return image_;
}
