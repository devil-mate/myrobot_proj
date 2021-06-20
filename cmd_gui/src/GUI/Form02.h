//
// Created by jl on 2020/6/3.
//

#ifndef SRC_FORM02_H
#define SRC_FORM02_H


#include "guiCommon.h"
#include "SwitchButton.h"
#include <QKeyEvent>
#include <QLineEdit>
#include <iostream>
typedef struct{
    float pidParam[5];
}Params_s;
using namespace  std;
class Form02: public QWidget {
Q_OBJECT
public:
    explicit Form02(QWidget *parent= nullptr);
    ~Form02() override;
    void getDynParam(Params_s &param);
    
private:
    void initWidget();
    Params_s sParam;
    QLabel *label;
    QLabel *edit_ki_label;
    QLabel *edit_kp_label;
    QLabel *edit_kd_label;
    QLabel *iGate_label;
    QLabel *outGate_label;

    QLineEdit *edit_kp;
    QLineEdit *edit_ki;
    QLineEdit *edit_kd;
    QLineEdit *edit_iGate;
    QLineEdit *edit_outGate;
    QPushButton *backMainButton;
    SwitchButton * switchButton;
protected:
    void keyPressEvent(QKeyEvent *event);

private slots:
    void on_backMainButton_click();
    void check_switchButton_click();
    void kp_Finished_slot();
    void ki_Finished_slot();
    void kd_Finished_slot();
    void iGate_Finished_slot();
    void outGate_Finished_slot();
};


#endif //SRC_FORM02_H
