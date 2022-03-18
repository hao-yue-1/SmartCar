/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。

//头文件引用
#include "headfile.h"       //逐飞的封装库
#include "Binarization.h"   //二值化处理
#include "Steer.h"          //舵机控制
#include "Motor.h"          //电机控制
#include "ImageBasic.h"     //图像的基础处理
#include "ImageSpecial.h"   //图像特殊元素处理
#include "ImageTack.h"      //循迹误差计算
#include "PID.h"            //PID
#include "BluetoothSend.h"  //蓝牙发送信息给手机APP上位机
#include "Filter.h"         //滤波头文件

#pragma section all "cpu0_dsram"    //将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

//定义变量
int LeftLine[MT9V03X_H]={0}, CentreLine[MT9V03X_H]={0}, RightLine[MT9V03X_H]={0};   //扫线处理左中右三线
SteerPID SteerK;    //舵机PID参数
MotorPID MotorK;    //电机PID参数
char power_switch=1;//电源总开关

int core0_main(void)
{
	get_clk();//获取时钟频率  务必保留
	//用户在此处调用各种初始化函数等
	//***************************变量定义**************************
	Point LeftDownPoint,RightDownPoint;     //左右下拐点
	LeftDownPoint.X=0;LeftDownPoint.Y=0;RightDownPoint.X=0;RightDownPoint.Y=0;
	Point ForkUpPoint;
	ForkUpPoint.X=0;ForkUpPoint.Y=0;
	Point CrossRoadUpLPoint,CrossRoadUpRPoint;
	CrossRoadUpLPoint.X=0;CrossRoadUpLPoint.Y=0;CrossRoadUpRPoint.X=0;CrossRoadUpRPoint.Y=0;
	float Bias=0;
	uint32 StreePWM=STEER_MID;
	int flag;//三岔识别的标志变量
	//*****************************************************************

	//***************************交互的初始化**************************
	uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);      //初始化串口0与电脑上位机通讯
	uart_init(BLUETOOTH_CH9141_UART, BLUETOOTH_CH9141_UART_BAUD, BLUETOOTH_CH9141_UART_TX, BLUETOOTH_CH9141_UART_RX);//初始化蓝牙模块所用的串口
	lcd_init();                                                     //初始化TFT屏幕
	gpio_init(P20_8, GPO, 1, PUSHPULL);                             //初始化LED：设置P20_8为输出
	gpio_init(P20_9, GPO, 1, PUSHPULL);
    gpio_init(P21_4, GPO, 1, PUSHPULL);
    gpio_init(P21_5, GPO, 1, PUSHPULL);
    //*****************************************************************

    //**************************传感器模块初始化**************************
	mt9v03x_init(); //初始化摄像头
	//********************************************************************

	//**************************驱动模块初始化**************************
	gtm_pwm_init(STEER_PIN, 50, STEER_MID);                         //初始化舵机
	gtm_pwm_init(LEFT_MOTOR_PIN1,17*1000,0);                        //初始化左电机
	gpio_init(P02_6, GPO, 1, PUSHPULL);
//	gtm_pwm_init(LEFT_MOTOR_PIN2,17*1000,0);
	gtm_pwm_init(RIGHT_MOTOR_PIN1,17*1000,0);                       //初始化右电机
	gpio_init(P02_7, GPO, 1, PUSHPULL);
//	gtm_pwm_init(RIGHT_MOTOR_PIN2,17*1000,0);

	gpt12_init(LEFT_ENCODER, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);    //初始化左编码器
	gpt12_init(RIGHT_ENCODER, GPT12_T6INA_P20_3, GPT12_T6EUDA_P20_0);   //初始化右编码器
	//********************************************************************

	/**********************PID初始化***********************************************/
	PID_init(&SteerK,&MotorK);
	/**********************定时器中断初始化**************************/
//	pit_interrupt_ms(CCU6_0,PIT_CH0,5);
	/**************************************************************/

    //等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();

	/*电机驱动测试*/
	MotorSetPWM(1500,1500);

	while (TRUE)
	{
	    /*初始化参数*/
	    //图像处理模块
	    if(mt9v03x_finish_flag)
	    {
	        ImageBinary();//图像二值化
	        //SPI发送图像到1.8TFT
	        lcd_displayimage032(BinaryImage[0],MT9V03X_W,MT9V03X_H);    //二值化后的图像

	        /*扫线函数测试*/
	        GetImagBasic(LeftLine,CentreLine,RightLine);

	        /*路径检测*/
	        GetDownInflection(100,40,LeftLine,RightLine,&LeftDownPoint,&RightDownPoint);
	        if(!CrossRoadsIdentify(LeftLine,RightLine,LeftDownPoint,RightDownPoint))    //十字
	        {
	            flag=ForkIdentify(100,40,LeftLine,RightLine,LeftDownPoint,RightDownPoint);  //三岔
	        }

	        //把中线画出来
            for(int i=MT9V03X_H;i>0;i--)
            {
//                lcd_showint32(0, 0, LeftLine[i], 3);
//                systick_delay_ms(STM0,100);
                lcd_drawpoint(LeftLine[i],i,GREEN);
                lcd_drawpoint(CentreLine[i],i,RED);
//                BluetooothSendBias(LeftLine[i]);
                lcd_drawpoint(RightLine[i],i,BLUE);
            }

	        /*斜率函数测试*/
//	        Bias=Regression_Slope(100,40,CentreLine);
//	        Bias=DifferentBias(100,40,CentreLine);
            if(flag==1)
            {
                Bias=DifferentBias(110,90,CentreLine);
                gpio_toggle(P21_4);
                flag=0;
            }
            else
            {
                Bias=DifferentBias(100,60,CentreLine);
            }
            lcd_showfloat(0, 0, Bias, 3, 3);

//	        BluetooothSendBias(Bias);//蓝牙发送


	        gpio_toggle(P20_8);//翻转IO：LED
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
	    }

	    /*开环转向环无元素测试*/
	    StreePWM=Steer_Position_PID(Bias,SteerK);
	    printf("Bias=%f     StreePWM=%d\r\n",Bias,StreePWM);
	    SteerCtrl(StreePWM);

	    /*电机速度环测试*/
//	    MotorEncoder(&encoder_l,&encoder_r);              //获取左右电机编码器
//	    BluetoothSendToApp(encoder_l,encoder_r);
//	    printf("encoder_l=%d      encoder_r=%d\r\n",encoder_l,encoder_r);
//	    systick_delay_ms(STM0,100);
//	    pwm_l=Speed_PI_Left(encoder_l,50,MotorK);    //左右电机PID
//	    pwm_r=Speed_PI_Right(encoder_r,50,MotorK);
//	    MotorSetPWM(pwm_l,pwm_r);                             //电机PWM赋值

	}
}

#pragma section all restore


