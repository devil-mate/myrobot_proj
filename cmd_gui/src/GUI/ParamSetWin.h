//
// Created by jl on 2020/6/3.
//

#ifndef SRC_PARAMSETWIN_H
#define SRC_PARAMSETWIN_H


#include <QtWidgets/QDialog>
#include <QLabel>
#include <QPushButton>
class ParamSetWin: public QDialog {
Q_OBJECT
public:
    explicit ParamSetWin(QWidget *parent = nullptr);
    ~ParamSetWin() override;

private:
    void initWidget();
    QLabel *label;
    QPushButton *backMainButton;
private slots:
    void on_backMainButton_click();
};



#endif //SRC_PARAMSETWIN_H
