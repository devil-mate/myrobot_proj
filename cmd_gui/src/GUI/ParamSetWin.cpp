//
// Created by jl on 2020/6/3.
//

#include "ParamSetWin.h"
//ParamSetWin::ParamSetWin(QWidget *parent) :
//        QDialog(parent){
//
//        }
ParamSetWin::ParamSetWin(QWidget *parent):QDialog(parent) {
    initWidget();
}

ParamSetWin::~ParamSetWin() {

}

void ParamSetWin::initWidget() {
    this->resize(QSize(500,200));
    backMainButton = new QPushButton(this);
    backMainButton->setText("BackMain");
    backMainButton->move(40,50);
    //
    label=new QLabel(this);
    label->setText(QObject::tr("this is a DialogWindow"));

    //signal and slots
    connect(backMainButton,SIGNAL(clicked()),this,SLOT(on_backMainButton_click()));
}

void ParamSetWin::on_backMainButton_click() {
    this->hide();
}
