#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "guiCommon.h"
#include <QtWidgets/QMainWindow>
#include <QStackedWidget>
#include "ParamSetWin.h"
#include "Form01.h"
#include "Form02.h"
#include "ControlBox.h"

class gui_node;
typedef struct
{
    float     manualSpeed;
    float     autoSpeed;
    int16_t   controlDigit;
    int16_t     testData;
}ControlData_s;
//TODO 同时做一个ros标准cmd命令.
class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
     //MainWindow(QWidget *parent = nullptr,gui_node *guiNode);
    ~MainWindow();
    void getControlData(ControlBox_Send_S &controlData);//其他类获取界面的输入数据。
    void getDynParam(Params_s &sParam);
    void initParamParent(gui_node *guiNode);
//    void updateState();
    void updateLabelPic(const cv::Mat &pic);


private:
    void addWidget();

    QLabel *label;
    QPushButton *showTxtPushButton;
    QPushButton *showForm02Button;
    QPushButton *showControlBoxButton;
    
    ControlData_s tempData;
    ControlBox *controlBox_;
    //
    QStackedWidget *qStackedWidget;
    //其他窗口
    ParamSetWin paramSetWin;
    Form01 *form01;
    Form02 *paramSetForm_;
    gui_node *guiNode_;

private slots:
    void on_showTxtButton_click();
    void on_showForm02Button_click();
    void on_showControlBoxButton_click();

};

#endif // MAINWINDOW_H
