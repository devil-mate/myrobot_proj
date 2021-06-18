//
// Created by jl on 2020/6/24.
//

#ifndef SRC_GAMEPAD_H
#define SRC_GAMEPAD_H

#include <QWidget>
#include <QPainter>
#include <QDrag>
#include <QMouseEvent>
#include <QtMath>
#include <QTimer>
#include <QDebug>
class GamePad : public QWidget
{
    Q_OBJECT

public:
    GamePad(QWidget *parent = 0);
    ~GamePad();
    enum {upleft=0,up,upright,left,stop,right,downleft,down,downright};
    // 压力大小/距离；角速度
    float throttle_, omega_;
    signals:
            void keyNumchanged(int num);
protected:
    void paintEvent(QPaintEvent *event)override;
    void mouseMoveEvent(QMouseEvent *event)override;
    void mouseReleaseEvent(QMouseEvent *event)override;
    void mousePressEvent(QMouseEvent *event)override;
    //  void resizeEvent(QResizeEvent *event)override;
private:
    int mouseX;
    int mouseY;
    int handleX;//摇杆
    int handleY;
    int handleR;
    int padX;//底盘
    int padY;
    int padR;
    double handPadDis;//两圆圆心距离
    bool mousePressed;
    QTimer *tim;
    QTimer *calculateTimer;
private:
    double Pointdis(int a,int b,int x,int y);//两点距离
    int getKeyNum();
    bool caclulateCmdData();
private slots:
    void updateCmdData();

};




#endif //SRC_GAMEPAD_H
