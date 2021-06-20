//
// Created by jl on 2020/6/3.
//

#include "Form02.h"

Form02::Form02(QWidget *parent):QWidget(parent) {
    memset(&sParam,0,sizeof(sParam));
    sParam.pidParam[0] =6;
    sParam.pidParam[1] =1;
    sParam.pidParam[2] =0;
    sParam.pidParam[3] =1000;
    sParam.pidParam[4] =5000;
    
    initWidget();
}

Form02::~Form02() {

}

void Form02::initWidget() {
    this->resize(QSize(1024,800));
    //
    label=new QLabel(this);
    label->setText(QObject::tr("this is  Form02"));
    label->move(200,600);
    //
    QLabel *label1= new QLabel(this);
//    QLabel label1;
    label1->setText(QObject::tr("this is  Form02,picture Label2"));
    label1->move(300,50);

    edit_kp_label=new QLabel("Kp：",this);
    int8_t deltax=50,deltay=50;
    edit_kp_label->setGeometry(500,30,30,30);
    edit_kp=new QLineEdit("6",this);
    edit_kp->setGeometry(550,30,80,40);
    edit_kp->setEchoMode(QLineEdit::Normal);             // 显示模式：与默认类型一致
    // edit_ki->setEchoMode(QLineEdit::NoEcho);            // 显示模式：隐藏输入文本，不改变光标位置
    //edit_ki->setEchoMode(QLineEdit::Password);             // 显示模式：用*表示文本
    // edit_kp->setEchoMode(QLineEdit::PasswordEchoOnEdit);    // 显示模式：若编辑文本与默认相同，失去焦点显示
    edit_kp->setValidator(new QDoubleValidator(0, 100, 2,this)); //限制输入浮点数
    connect(edit_kp,SIGNAL(editingFinished()),this,SLOT(kp_Finished_slot())); 
    
    edit_ki_label=new QLabel("Ki：",this);
    edit_ki_label->setGeometry(500,30+deltay,30,30);
    edit_ki=new QLineEdit("1",this);
    edit_ki->setGeometry(550,30+deltay,80,40);
    edit_ki->setEchoMode(QLineEdit::Normal);              
    edit_ki->setValidator(new QDoubleValidator(0, 100, 2,this)); 
    connect(edit_ki,SIGNAL(editingFinished()),this,SLOT(ki_Finished_slot())); 

    edit_kd_label=new QLabel("Kd：",this);
    edit_kd_label->setGeometry(500,30+2*deltay,30,30);
    edit_kd=new QLineEdit("0.0",this);
    edit_kd->setGeometry(550,30+2*deltay,80,40);
    edit_kd->setEchoMode(QLineEdit::Normal);              
    edit_kd->setValidator(new QDoubleValidator(0, 100, 2,this)); 
    connect(edit_kd,SIGNAL(editingFinished()),this,SLOT(kd_Finished_slot())); 

    iGate_label=new QLabel("iGate：",this);
    iGate_label->setGeometry(500,30+3*deltay,30,30);
    edit_iGate=new QLineEdit("1000",this);
    edit_iGate->setGeometry(550,30+3*deltay,80,40);
    edit_iGate->setEchoMode(QLineEdit::Normal);             
    // edit_iGate->setEchoMode(QLineEdit::PasswordEchoOnEdit);    
    edit_iGate->setValidator(new QDoubleValidator(0, 5000, 2,this)); 
    connect(edit_iGate,SIGNAL(editingFinished()),this,SLOT(iGate_Finished_slot())); 

    outGate_label=new QLabel("outGate：",this);
    outGate_label->setGeometry(500,30+4*deltay,30,30);
    edit_outGate=new QLineEdit("5000",this);
    edit_outGate->setGeometry(550,30+4*deltay,80,40);
    edit_outGate->setEchoMode(QLineEdit::Normal);             
    // edit_outGate->setEchoMode(QLineEdit::PasswordEchoOnEdit);    
    edit_outGate->setValidator(new QDoubleValidator(0, 5000, 2,this)); 
    connect(edit_outGate,SIGNAL(editingFinished()),this,SLOT(outGate_Finished_slot())); 

    // backMainButton = new QPushButton(this);
    // backMainButton->setText("BackMain02");
    // backMainButton->move(500,500);
    // connect(backMainButton,SIGNAL(clicked()),this,SLOT(on_backMainButton_click()));
    //选择开关
//     switchButton = new SwitchButton(this);
// //    switchButton->resize(300,200);
//     switchButton->move(100,500);
//     connect(switchButton,SIGNAL(checkedChanged(bool)),this,SLOT(check_switchButton_click()));
//


}
void Form02::on_backMainButton_click() {
//    this->hide();
}
void Form02::check_switchButton_click(){
    this->hide();
}
void Form02::kp_Finished_slot(){
    sParam.pidParam[0] =edit_kp->text().toDouble();
    
}
void Form02::ki_Finished_slot(){
    sParam.pidParam[1] =edit_ki->text().toDouble();
}
void Form02::kd_Finished_slot(){
    sParam.pidParam[2] =edit_kd->text().toDouble();
}
void Form02::iGate_Finished_slot(){
    sParam.pidParam[3] =edit_iGate->text().toDouble();
}
void Form02::outGate_Finished_slot(){
    sParam.pidParam[4] =edit_outGate->text().toDouble();
}
void Form02::getDynParam(Params_s &param)
{
    for(int8_t i=0;i<5;i++){
        param.pidParam[i] = sParam.pidParam[i];
    }
    
}
void Form02::keyPressEvent(QKeyEvent *event){
    switch (event->key()){
        case Qt::Key_Control:

            break;

        case Qt::Key_Down:
            label->setText("key down");
            break;
        case Qt::Key_Left:
            label->setText("key left");
            break;
        case Qt::Key_Right:
            label->setText("key right");
            break;
        case Qt::Key_Up:
            label->setText("key up");
            break;
        default:
            break;
    }
}