模块组织与设计

[toc]
#设计
* gui与功能（分开）
    方法1：图像处理作为一个节点，提供服务，由GUI请求。那么需要提供很多服务吗？（多个服务）
    方法2：在对应串口类中创建 图像处理类实例，那么耗时的操作需要创建线程，作线程同步（多个线程）
* 
* 作为一个包，一个节点，调试用gui（发布控制命令和订阅，显示图片等）

#功能


#代码解释

# 临时
* 手提箱结构体
>typedef unsigned char  INT8U;                             
typedef signed   char  INT8S;                             
typedef unsigned short INT16U;                               
typedef signed   short INT16S;                            
typedef unsigned int   INT32U;                                
typedef signed   int   INT32S;     
typedef unsigned long long INT64U;                           
typedef long long      INT64S;                            
typedef float          FP32;                          
typedef double         FP64;       

>typedef struct
{
	INT16S X;
	INT16S Y;
	INT16S Z;
	INT16S B;
}JoyStick;

>typedef struct
{	
	//6+7+2+4+2*4*2+2=39
	INT8U Head[2];		//Head[0] = 0xBB, Head[1] = 0xB3
    INT8U ControlCode;	//ControlCode = 0xC1
	INT8U Button[7];	//按钮
	INT16U ADC[2];			//ADC
	INT8U Switch[4];	//开关量
	JoyStick Js[2];		//摇杆
	INT16S Encoder;		//编码器旋钮
    INT8U CheckSum;
	INT8U Tail[2];		// Tail[0] = 0xD3, Tail[1] = 0xDD
}SEND_STR;