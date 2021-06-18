#include "mainwindow.h"
#include "gui_node.h"
#include <QDesktopWidget>
MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent)
{
    memset(&tempData,0,sizeof(tempData));
    addWidget();

}

MainWindow::~MainWindow()
{
    delete controlBox_;
}




void MainWindow::getControlData(ControlBox_Send_S &controlData) {
    controlBox_->getControlData(controlData);
    // controlData.autoSpeed = controlBox_->controlBoxSendS_.autoSpeed;
    // controlData.manualSpeed = controlBox_->controlBoxSendS_.manualSpeed;
    // controlData.forward = controlBox_->controlBoxSendS_.forward;
    // controlData.backward = controlBox_->controlBoxSendS_.backward;
    // controlData.mode= controlBox_->controlBoxSendS_.mode;


}
void MainWindow::getDynParam(Params_s &sParam) {
    //controlBox_->getControlData(controlData);
    paramSetForm_->getDynParam(sParam);

}
void MainWindow::initParamParent(gui_node *guiNode) {
    guiNode_ = guiNode;
    controlBox_->initParamParent(guiNode);

}

void MainWindow::updateLabelPic(const cv::Mat &pic) {
    form01->updatePic(pic);

}

void MainWindow::addWidget() {
    //窗口
    // QDesktopWidget* desktopWidget = QApplication::desktop(); // QApplication::desktop()返回一个QDesktopWidget类型
    int ScreenWid = QApplication::desktop()->width();
    int ScreenHei  = QApplication::desktop()->height();
    this->resize(QSize(1280,1024));
    // this->resize(QSize(ScreenWid,ScreenHei));
    setWindowTitle("主窗口");
//
    qStackedWidget = new QStackedWidget(this);
    form01 = new Form01(this);
    paramSetForm_ = new Form02(this);
    qStackedWidget->addWidget(form01);
    qStackedWidget->addWidget(paramSetForm_);
    qStackedWidget->move(30,30);
    qStackedWidget->resize(1280,800);
    qStackedWidget->setCurrentWidget(form01);
//
    controlBox_ = new ControlBox(this);
//    qStackedWidget->setCurrentIndex(0);
    //
    label=new QLabel(this);
    //
    showTxtPushButton = new QPushButton(this);
    showTxtPushButton->setText("showTxt");
    showTxtPushButton->move(40,50);
    connect(showTxtPushButton,SIGNAL(clicked()),this,SLOT(on_showTxtButton_click()));

    showForm02Button =new QPushButton(this);
    showForm02Button->setText("paramSet");
    showForm02Button->move(40,150);
    connect(showForm02Button,SIGNAL(clicked()),this,SLOT(on_showForm02Button_click()));

    showControlBoxButton =new QPushButton(this);
    showControlBoxButton->setText("openControlBox");
    showControlBoxButton->move(40,250);
    connect(showControlBoxButton,SIGNAL(clicked()),this,SLOT(on_showControlBoxButton_click()));
//




}



void MainWindow::on_showTxtButton_click() {
    label->setText(QObject::tr("hello world!"));
    showTxtPushButton->setText("xxxxxxxxxxx");
    tempData.testData++;
//    this->hide();
    //paramSetWin.show();
//    paramSetWin.exec();
    form01->showOpencvPic();
    qStackedWidget->setCurrentWidget(form01);


//    qStackedWidget->setCurrentIndex(0);
//    form01->show();
//    this->show();

}
void MainWindow::on_showForm02Button_click() {

    //this->hide();
    //paramSetWin.show();
//    paramSetWin.exec();
    qStackedWidget->setCurrentWidget(paramSetForm_);
    //controlBox_->show();
//    qStackedWidget->setCurrentIndex(1);
//    form02->show();
//    this->show();

}
void MainWindow::on_showControlBoxButton_click() {

    this->hide();
    controlBox_->show();
    

}
