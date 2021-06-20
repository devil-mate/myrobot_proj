//
// Created by jl on 2020/6/12.
//

#ifndef SRC_COMMUNICATION_STRUCT_H
#define SRC_COMMUNICATION_STRUCT_H
typedef unsigned char  INT8U;
typedef signed   char  INT8S;
typedef unsigned short INT16U;
typedef signed   short INT16S;
typedef unsigned int   INT32U;
typedef signed   int   INT32S;
typedef unsigned long long INT64U;
typedef long long      INT64S;
typedef float          FP32;
typedef double         FP64;

#include "stdint.h"
typedef struct
{
    INT16S X;
    INT16S Y;
    INT16S Z;
    INT16S B;
}JoyStick;

//typedef struct
//{
//    //6+7+2+4+2*4*2+2=39
//    INT8U Head[2];		//Head[0] = 0xBB, Head[1] = 0xB3
//    INT8U ControlCode;	//ControlCode = 0xC1
//    INT8U Button[7];	//按钮
//    INT16U ADC[2];			//ADC
//    INT8U Switch[4];	//开关量
//    JoyStick Js[2];		//摇杆
//    INT16S Encoder;		//编码器旋钮
//    INT8U CheckSum;
//    INT8U Tail[2];		// Tail[0] = 0xD3, Tail[1] = 0xDD
//}ControlBox_Send_S;
typedef struct{
     float brightnessValue;
     bool autoParamAdj;
     bool cameraTriggerMode;
     float gain;
     float exposure;

}CameraParam_s;
typedef struct
{
    int16_t controlDigit;
    float autoSpeed;
    float manualSpeed;
    float omega;
    bool forward;
    bool backward;
    int8_t mode;
    CameraParam_s  cameraParamS;

}ControlBox_Send_S;
#endif //SRC_COMMUNICATION_STRUCT_H
