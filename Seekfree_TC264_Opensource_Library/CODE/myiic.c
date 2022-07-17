//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//IIC驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2019/9/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#include "myiic.h"
#include "zf_stm_systick.h"

//IIC初始化
void IIC_Init(void)
{
    gpio_init (OLED_SCL, GPO, 1, PUSHPULL);
    gpio_init (OLED_SDA, GPO, 1, PUSHPULL);
    
    SDA_1();
    SCL_1();
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SDA_1();
	SCL_1();
	systick_delay_us(STM0,4);
	SDA_0();//START:when CLK is high,DATA change form high to low
 	systick_delay_us(STM0,4);
 	SCL_0();//钳住I2C总线，准备发送或接收数据
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	SCL_0();
	SDA_0();//STOP:when CLK is high DATA change form low to high
	systick_delay_us(STM0,4);
	SCL_1();
	SDA_1();//发送I2C总线结束信号
	systick_delay_us(STM0,4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char IIC_Wait_Ack(void)
{
    unsigned char ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	SDA_1();systick_delay_us(STM0,1);
	SCL_1();systick_delay_us(STM0,1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_0();//时钟输出0
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
    SCL_0();
	SDA_OUT();
	SDA_0();
	systick_delay_us(STM0,2);
	SCL_1();
	systick_delay_us(STM0,2);
	SCL_0();
}
//不产生ACK应答		    
void IIC_NAck(void)
{
    SCL_0();
	SDA_OUT();
	SDA_1();
	systick_delay_us(STM0,2);
	SCL_1();
	systick_delay_us(STM0,2);
	SCL_0();
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(unsigned char txd)
{                        
    unsigned char t;
	SDA_OUT(); 	    
	SCL_0();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>>7==1)    //SDA发送1
        {
            SDA_1();
        }
        else                    //SDA发送0
        {
            SDA_0();
        }
        txd<<=1; 	  
        systick_delay_us(STM0,2);   //对TEA5767这三个延时都是必须的
        SCL_1();
		systick_delay_us(STM0,2);
		SCL_0();
		systick_delay_us(STM0,2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        SCL_0();
        systick_delay_us(STM0,2);
        SCL_1();
        receive<<=1;
        if(READ_SDA)receive++;   
        systick_delay_us(STM0,1);
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



