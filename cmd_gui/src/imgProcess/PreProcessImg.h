//
// Created by jl on 2020/4/17.
//

#ifndef PRAC_OPENCV_PREPROCESSIMG_H
#define PRAC_OPENCV_PREPROCESSIMG_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include"opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;

typedef struct{
    string folderPath;
    string picName;
}Path_s;

class PreProcessImg {
public:
    PreProcessImg();
    ~PreProcessImg();
    void imageBlur(cv::Mat &Img, cv::Mat &dstImg,int param);
    cv::Mat getImage();
private:
    bool readOneImg(std::string picName);//读取单张图片
    void picShow(cv::Mat &image);
    void picProcess(cv::Mat &image);//
    void readFolderImg(std::string folderPath);//读取文件夹下所有照片
    void frameCapture(std::string folderPath);//读取文件夹，做视频显示
    void init(Path_s &sPath);
    void articulation_sobel(cv::Mat &image); //清晰度评价1，Sobel算子处理后的图像的平均灰度值，值越大，代表图像越清晰。
    void articulation_StdDev(cv::Mat &image);//
    void getROI(cv::Mat Img,cv::Mat &dstImg); //计算感兴趣区域
    void getAutoROI(Mat &Img, cv::Mat &dstImg); //自动识别感兴趣区域
    void generalProcess(cv::Mat &Img, cv::Mat &dstImg);// 图像处理：调用图像增强等
    void imgEnhance(cv::Mat &Img, cv::Mat &dstImg);//图像增强
    void edgeDetection(cv::Mat &Img);

    // gui 命令处理图像
    void readFolderImg_gui(std::string folderPath);//读取文件夹下所有照片

    //
    Mat image_;
    vector<cv::String> picName_;


};


#endif //PRAC_OPENCV_PREPROCESSIMG_H
