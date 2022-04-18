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
#include "Steer.h"          //舵机控制
#include "Motor.h"          //电机控制
#include "PID.h"            //PID
#include "protocol.h"       //野火上位机协议
#include "ImageProcess.h"   //图像处理
#include "Key.h"            //按键处理

#pragma section all "cpu0_dsram"    //将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

int16 base_speed=125;        //基础速度

int core0_main(void)
{
	get_clk();//获取时钟频率  务必保留
	/***************************交互的初始化**************************/
//	uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);      //初始化串口0与电脑上位机通讯
	uart_init(UART_2, 115200, UART2_TX_P10_5, UART2_RX_P10_6);      //初始化蓝牙模块所用的串口2
	lcd_init();     //初始化TFT屏幕
	gpio_init(P20_8, GPO, 1, PUSHPULL);     //初始化核心板的LED
	gpio_init(P20_9, GPO, 1, PUSHPULL);
    gpio_init(P21_4, GPO, 1, PUSHPULL);
    gpio_init(P21_5, GPO, 1, PUSHPULL);
    gpio_init(P33_10, GPI, 0, PULLDOWN);    //初始化按键
    gpio_init(P33_11, GPI, 0, PULLDOWN);
    gpio_init(P33_12, GPI, 0, PULLDOWN);
    gpio_init(P33_13, GPI, 0, PULLDOWN);
    gpio_init(P32_4, GPI, 0, PULLDOWN);
    gpio_init(P23_1, GPO, 1, PUSHPULL);     //初始化主控板的LED
    gpio_init(P22_1, GPO, 1, PUSHPULL);
    gpio_init(P22_2, GPO, 1, PUSHPULL);
    gpio_init(P22_3, GPO, 1, PUSHPULL);
    gpio_init(P21_2, GPO, 1, PUSHPULL);
    /**************************传感器模块初始化**********************/
	mt9v03x_init();     //初始化摄像头
	/***************************驱动模块初始化***********************/
	gtm_pwm_init(STEER_PIN, 50, STEER_MID);       //初始化舵机
	gtm_pwm_init(LEFT_MOTOR_PIN1,17*1000,0);      //初始化左电机
//	gpio_init(P02_6, GPO, 1, PUSHPULL);           //逐飞驱动：左电机
	gtm_pwm_init(LEFT_MOTOR_PIN2,17*1000,0);      //自制驱动：左电机
	gtm_pwm_init(RIGHT_MOTOR_PIN1,17*1000,0);     //初始化：右电机
//	gpio_init(P02_7, GPO, 1, PUSHPULL);           //逐飞驱动：右电机
	gtm_pwm_init(RIGHT_MOTOR_PIN2,17*1000,0);     //自制驱动：右电机
	gpt12_init(LEFT_ENCODER, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);    //初始化左编码器
	gpt12_init(RIGHT_ENCODER, GPT12_T6INA_P20_3, GPT12_T6EUDA_P20_0);   //初始化右编码器
	/**********************PID初始化***********************************************/
	PID_init(&SteerK,&MotorK);          //初始化PID参数

    //等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();

	while (TRUE)
	{

	}
}

#pragma section all restore


