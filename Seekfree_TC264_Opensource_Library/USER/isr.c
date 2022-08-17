
 
/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
#include "motor.h"
#include "steer.h"
#include "PID.h"
#include "ImageTack.h"
#include "zf_gpio.h"
#include <math.h>
#include "Filter.h"
#include "ICM20602.h"
#include "LED.h"
#include  "zf_stm_systick.h"
#include "ImageProcess.h"

uint32 SteerPWM=STEER_MID;    //舵机PWM
float icm_target_angle_z=0;   //陀螺仪Z轴积分目标角度
uint8 icm_angle_z_flag=0;     //陀螺仪Z轴积分达到目标角度
//PIT中断函数  示例

//电机速度环控制中断
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套

	//阿克曼结构差速，减速版
	int diff_steerpwm=SteerPWM-STEER_MID;
	double radian;
	if(diff_steerpwm>0) //左转
	{
	    radian=0.0082794*diff_steerpwm;    //左轮与竖直线实际夹角 /   //若舵机阈值与实际最大打角改变，则需要修改此处
	    speed_l=(41/(41+15*tan(radian)))*base_speed;    //左转左轮减速    //此处由前轮轮距和前后轮轴距决定，一般不需要改动
	    speed_r=base_speed;
//	    speed_r=((41+30*tan(radian))/(41+15*tan(radian)))*base_speed;  //右轮加速
	}
	else                //右转
	{
	    diff_steerpwm=-diff_steerpwm;
	    radian=0.0082794*diff_steerpwm;     //右轮与竖直线实际夹角    //若舵机阈值与实际最大打角改变，则需要修改此处
	    speed_r=(41/(41+15*tan(radian)))*base_speed;    //右转右轮减速    //此处由前轮轮距和前后轮轴距决定，一般不需要改动
	    speed_l=base_speed;
//	    speed_l=((41+30*tan(radian))/(41+15*tan(radian)))*base_speed;  //左轮加速
	}
	MotorCtrl(speed_l,speed_r);         //PID控制电机速度
	//调试
	gpio_set(P20_8,0);
//	MotorCtrl(base_speed, base_speed);
//	printf("%d,%d\n",speed_l,speed_r);

	PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
}

//舵机转向环控制中断
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套

	//舵机PID控制
    SteerPWM=Steer_Position_PID(Bias,Slope,SteerK);
    SteerCtrl(SteerPWM);

//    printf("%f,%d\n",Bias,SteerPWM);  //调参

//	//舵机演示
//	static int16 pwm=STEER_LEFT,flag=0;
//	if(pwm>STEER_RIGHT&&flag==0)    //右摆
//	{
//	    pwm--;
//	}
//	else    //切换方向
//	{
//	    flag=1;
//	}
//	if(pwm<STEER_LEFT&&flag==1)     //左摆
//	{
//	    pwm++;
//	}
//	else    //切换方向
//	{
//	    flag=0;
//	}
//	SteerCtrl(pwm);
//	if(pwm==STEER_MID||pwm==STEER_LEFT||pwm==STEER_RIGHT)  //暂停查看中值
//	{
//	    systick_delay_ms(STM0,1000);
//	}

	PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);
}

//陀螺仪角度积分中断
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套

	float angle_z=GetICM20602Angle_Z(0);    //角度积分
	if(angle_z>icm_target_angle_z||angle_z<-icm_target_angle_z)  //判断积分角度是否大于目标角度
	{
	    icm_angle_z_flag=1;                     //积分到达目标flag=1
	    pit_disable_interrupt(CCU6_1, PIT_CH0); //关闭中断
	}

//	printf("%f\n",angle_z);
//	lcd_showfloat(0, 0, angle_z, 3, 3);

	PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}




IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//通道0中断
	{
		CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
	}

	if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//通道4中断
	{
		CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
	}
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//通道1中断
	{
		CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
	}

	if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//通道5中断
	{
		CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
	}
}

//由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//	enableInterrupts();//开启中断嵌套
//	if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//	}
//	if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//通道6中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//	}
//}



IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//通道3中断
	{
		CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
		if		(CAMERA_GRAYSCALE == camera_type)	mt9v03x_vsync();
		else if (CAMERA_BIN_UART  == camera_type)	ov7725_uart_vsync();
		else if	(CAMERA_BIN       == camera_type)	ov7725_vsync();

	}
	if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
	{
		CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

	}
}



IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套

	if		(CAMERA_GRAYSCALE == camera_type)	mt9v03x_dma();
	else if (CAMERA_BIN_UART  == camera_type)	ov7725_uart_dma();
	else if	(CAMERA_BIN       == camera_type)	ov7725_dma();
}


//串口中断函数  示例
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    if		(CAMERA_GRAYSCALE == camera_type)	mt9v03x_uart_callback();
    else if (CAMERA_BIN_UART  == camera_type)	ov7725_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}

/*
 ** 对于串口中断的一点注释
 ** IFX_INTERRUPT是用于声明中断服务函数的宏
 ** 三个参数：1.中断服务函数的名字    2.0指示CPU0提供中断服务     3.优先级
 **
 ** 下面三个宏函数都是配置串口2的中断，第一个是TX，第二个是RX，第三个是ER，这里我们只关注第二个RX接收的中断
 ** */

//串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart2_handle);

    //下面是逐飞用来选择串口2连接不同模块的
//    switch(wireless_type)
//    {
//    	case WIRELESS_SI24R1:
//    	{
//    		wireless_uart_callback();
//    	}break;
//
//    	case WIRELESS_CH9141:
//		{
//		    bluetooth_ch9141_uart_callback();
//		}break;
//    	default:break;
//    }

    bluetooth_ch9141_uart_callback();   //直接调用蓝牙模块的回调函数

}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
