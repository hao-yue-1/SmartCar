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

#include "headfile.h"
#pragma section all "cpu1_dsram"

#include "Binarization.h"   //二值化处理
#include "Steer.h"          //舵机控制
#include "ImageProcess.h"   //图像处理集合
#include "PID.h"            //PID
#include <string.h>
#include "Key.h"            //按键调参

//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

uint8 outgarage_flag=0;     //出库的flag
uint8 key_flag=0;           //按键调参的flag

void core1_main(void)
{
	disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //用户在此处调用各种初始化函数等

    /*各个模块的初始化在CPU0中完成*/

	//等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();

    //**********按键调参*************
    //*******************************
    pit_interrupt_ms(CCU6_0,PIT_CH0,6); //初始化电机定时器中断
    pit_interrupt_ms(CCU6_0,PIT_CH1,20);//初始化舵机定时器中断
//    //完成出库
//    if(outgarage_flag==0)
//    {
//        OutGarage();
//        outgarage_flag=1;
//    }

    while (TRUE)
    {
        //图像处理模块
        if(mt9v03x_finish_flag)
        {
            ImageBinary();      //图像二值化
//            lcd_displayimage032(BinaryImage[0],MT9V03X_W,MT9V03X_H);    //发送二值化后的图像到LCD
            //画110行在哪
//            for(int cloum=0;cloum<MT9V03X_W-1;cloum++)
//            {
//                lcd_drawpoint(cloum,110,PURPLE);
//            }
            ImageProcess();     //图像处理、元素识别
            //把三线画出来
//            for(int i=MT9V03X_H-1;i>0;i--)
//            {
//                lcd_drawpoint(LeftLine[i],i,GREEN);
//                lcd_drawpoint(CentreLine[i],i,RED);
//                lcd_drawpoint(RightLine[i],i,BLUE);
//            }
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
        }
        //调试
        gpio_set(P20_8,1);
    }
}



#pragma section all restore
