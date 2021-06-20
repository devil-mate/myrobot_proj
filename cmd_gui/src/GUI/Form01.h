//
// Created by jl on 2020/6/3.
//
//图片显示与操作窗口
#ifndef SRC_FORM01_H
#define SRC_FORM01_H

#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "guiCommon.h"
#include "src/imgProcess/PreProcessImg.h"
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
class gui_node;
class Form01: public QWidget {
Q_OBJECT
public:
    explicit Form01(QWidget *parent= nullptr);
//    explicit Form01(QWidget *parent= nullptr,gui_node *guiNode= nullptr);
    ~Form01() override;
    void showPic();
    void showOpencvPic();
    void updatePic(const cv::Mat &pic);
    void updateState(const gui_node *guiNode= nullptr);

private:
    void initWidget();

    QLabel *label;
    QPushButton *backMainButton;
    QLabel *pictureLabel_;
    QScrollArea *scrollArea_;
    QSlider *lightSlider_;

    cv::Mat dealImg_;
    PreProcessImg *preProcessImg_;
    gui_node *guiNode_;

private slots:
    void on_backMainButton_click();
    void on_lightSlider_ValueChanged(int);
};


#endif //SRC_FORM01_H
