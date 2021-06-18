//
// Created by jl on 2020/6/12.
//

#ifndef SRC_CONTROLBOX_H
#define SRC_CONTROLBOX_H


#include <include/communication_struct.h>


#include "guiCommon.h"
#include <QKeyEvent>
#include "GamePad.h"
class gui_node;
class ControlBox: public QDialog {
Q_OBJECT
public:
    explicit ControlBox(QWidget *parent = nullptr);
    ~ControlBox() override;
    void getControlData(ControlBox_Send_S &controlData);
    
    float tempX=102;
    void initParamParent(gui_node *guiNode);

protected:
    void keyPressEvent(QKeyEvent *event);
private:
    void initWidget(); //相机相关控件
    void initControlWidget(); //控制相关控件
    ControlBox_Send_S controlBoxSendS_;
    QLabel *label;
    QPushButton *backMainButton;
    QSlider *autoSpeedSlider_;
    QSlider *Speed_W_Slider_;
    QSlider *picLightSlider_;
    QCheckBox *autoParamAdjCheck_, *cameraMode_;
    QLineEdit *picGainLineEd_,*exposureTimeLineEd_;
    QComboBox *cameraIndexBox_;
    gui_node *guiNode_;
    QAbstractButton *manualButton_,*autoModeButton_,*resetButton_;
    QAbstractButton *forwardButton_,*backButton_;

    QPushButton *upButton_,*downButton_,*leftButton_,*rightButton_;
    GamePad *gamePad_;
    int serverCall();

private slots:
    void on_backMainButton_click();
    void on_picLightSlider_ValueChanged(int);
    void on_autoSpeedSlider_ValueChanged(int);
    void on_speedWSlider_ValueChanged(int);
    void on_autoParamAdjCheck_click();
    void on_cameraMode_click();
    void finished_exposureTime();
    void finished_picGain();
    void currentChanged_cameraIndex();
    void on_ButtonGroup_Clicked(int buttonId);
    void pressed_forward_clicked();
    void release_forward_clicked();
    void pressed_backward_clicked();
    void release_backward_clicked();

};

#endif //SRC_CONTROLBOX_H
