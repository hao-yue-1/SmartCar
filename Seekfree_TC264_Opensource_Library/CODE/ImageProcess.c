/*
 * ImageProcess.c
 *
 * Created on: 2022年3月29日
 * Author: 30516
 * Effect: Image element processing logic
 */
#include "ImageProcess.h"
#include "zf_gpio.h"
#include "PID.h"
#include "Motor.h"
#include "LED.h"
#include <stdio.h>
#include "zf_ccu6_pit.h"
#include "ICM20602.h"
#include "Key.h"

#define STATE_LED_DEBUG 0

uint8 bias_startline=95,bias_endline=50;        //动态前瞻
uint8 Fork_flag=0;              //三岔识别的标志变量
uint8 Garage_flag=0;            //车库识别标志变量
uint8 Circle_flag=0;            //环内寻迹标志变量
uint8 speed_case_1=200,speed_case_2=220,speed_case_3=200,speed_case_4=200,speed_case_5=200,speed_case_6=200,speed_case_7=200;
uint32 SobelResult=0;
int LeftLine[MT9V03X_H]={0}, CentreLine[MT9V03X_H]={0}, RightLine[MT9V03X_H]={0};   //扫线处理左中右三线
uint8 process_flag=0;   //状态机跳转标志

/********************************************************************************************
 ** 函数功能: 对图像的各个元素之间的逻辑处理函数，最终目的是为了得出Bias给中断去控制
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: LJF
 ** 注    意：无
 *********************************************************************************************/
void ImageProcess()
{
    /***************************变量定义****************************/
    static uint8 encoder_flag,icm_flag;  //是否查询编码器的flag,是否查询陀螺仪的flag
    Point InflectionL,InflectionR;     //左右下拐点
    InflectionL.X=0;InflectionL.Y=0;InflectionR.X=0;InflectionR.Y=0;
    /*****************************扫线*****************************/
    if(process_flag==1)
    {
        GetImagBasic_Garage(LeftLine, CentreLine, RightLine, 'L');
    }
    else
    {
        GetImagBasic(LeftLine, CentreLine, RightLine, 'L');
    }
    /*************************搜寻左右下拐点***********************/
    GetDownInflection(110,45,LeftLine,RightLine,&InflectionL,&InflectionR);
    /*************************特殊元素判断*************************/
#if IMAGE_KEY_DEBUG
    ImageParameterHandle(key_num_1, key_num_2, InflectionL, InflectionR);   //按键显示图像Debug
#endif
    /****************************状态机***************************/
#if !IMAGE_KEY_DEBUG
    switch(process_flag)
    {
        case 0: //识别左十字回环
        {
#if STATE_LED_DEBUG
            gpio_set(LED_RED, 0);
#endif
            if(CrossLoopIdentify_L(LeftLine, RightLine, InflectionL, InflectionR)==1)
            {
#if STATE_LED_DEBUG
                gpio_set(LED_RED, 1);
#endif
                base_speed=speed_case_1;
                process_flag=1;
            }
            break;
        }
        case 1: //识别右车库，直行
        {
#if STATE_LED_DEBUG
            gpio_set(LED_YELLOW, 0);
#endif
            if(RNINGarageStatusIdentify(InflectionL, InflectionR, &Garage_flag)==1)
            {
#if STATE_LED_DEBUG
                gpio_set(LED_YELLOW, 1);
#endif
                EncoderDistance(1, 1.6, 0, 0);//此处为跑普通赛道，防止三岔误判
                encoder_flag=1;
                base_speed=speed_case_2;
                process_flag=2;
            }
            break;
        }
        case 2: //识别第一遍三岔
        {
            if(encoder_flag==1)//查询编码器，等编码器状态到了才跳转
            {
                if(encoder_dis_flag==1)//此处标定到三岔入口
                {
                    encoder_flag=0;
                }
                break;
            }
#if STATE_LED_DEBUG
                    gpio_set(LED_WHITE, 0);
#endif
            if(ForkFStatusIdentify(InflectionL, InflectionR, &Fork_flag)==1)
            {
#if STATE_LED_DEBUG
                gpio_set(LED_WHITE, 1);
#endif
                base_speed=speed_case_3;
                process_flag=3;
            }
            break;
        }
        case 3: //识别右环岛
        {
#if STATE_LED_DEBUG
            gpio_set(LED_BLUE, 0);
#endif
            if(CircleIslandIdentify_R(RightLine, InflectionR)==1)
            {
#if STATE_LED_DEBUG
                gpio_set(LED_BLUE, 1);
#endif
                base_speed=speed_case_4;
                process_flag=4;
            }
            break;
        }
        case 4: //识别右十字回环
        {
#if STATE_LED_DEBUG
                gpio_set(LED_GREEN, 0);
#endif
            if(CrossLoopIdentify_R(LeftLine, RightLine, InflectionL, InflectionR)==1)
            {
#if STATE_LED_DEBUG
                gpio_set(LED_GREEN, 1);
#endif
                base_speed=speed_case_5;
                process_flag=5;
            }
            break;
        }
        case 5: //识别左环岛
        {
#if STATE_LED_DEBUG
            gpio_set(LED_RED, 0);
#endif
            if(CircleIslandIdentify_L(LeftLine, InflectionL)==1)
            {
#if STATE_LED_DEBUG
                gpio_set(LED_RED, 1);
#endif
                StartIntegralAngle_Z(30);//出状态之后转了30度左右才开启三岔识别，避免状态机出错导致误判
                icm_flag=1;
                base_speed=speed_case_6;
                process_flag=6;
            }
            break;
        }
        case 6: //识别第二遍三岔
        {
            if(icm_flag==1)
            {
                if(icm_angle_z_flag==1)
                {
                    icm_flag=0;
                }
                break;
            }
#if STATE_LED_DEBUG
            gpio_set(LED_YELLOW, 0);
#endif
            if(ForkSStatusIdentify(InflectionL, InflectionR, &Fork_flag)==1)
            {
#if STATE_LED_DEBUG
            gpio_set(LED_YELLOW, 1);
#endif
                base_speed=speed_case_7;
                process_flag=7;
            }
            break;
        }
        case 7: //识别左车库，入库
        {
#if STATE_LED_DEBUG
            gpio_set(LED_WHITE, 0);
#endif
            if(GarageInIdentify()==1)
            {
#if STATE_LED_DEBUG
            gpio_set(LED_WHITE, 1);
#endif
                Stop();
            }
            break;
        }
    }
#endif
    /***************************偏差计算**************************/
    if(Fork_flag!=0||Garage_flag!=0||Circle_flag!=0)    //在识别函数里面已经计算了Bias
    {
        Garage_flag=0;Fork_flag=0;Circle_flag=0;      //重置flag
    }
    else
    {
        Bias=DifferentBias(bias_startline,bias_endline,CentreLine); //动态前瞻计算偏差
        bias_startline=95;bias_endline=50;                          //恢复默认前瞻
    }
    //LCD绘制图像
#if 0
    for(uint8 i=0;i<MT9V03X_W-1;i++)
    {
        lcd_drawpoint(i, bias_startline, YELLOW);
        lcd_drawpoint(i, bias_endline, YELLOW);
    }
    lcd_showfloat(0, 1, Bias, 1, 2);
#endif
}

/*
 *******************************************************************************************
 ** 函数功能: 停车，使用PID将电机速度降为0，关闭舵机中断
 ** 参    数: 无InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void Stop(void)
{
    base_speed=0;                   //设置目标速度为0
    MotorK_L.I=1.5;MotorK_R.I=1.5;  //修改PID参数快速停车
    pit_disable_interrupt(CCU6_0, PIT_CH1); //关闭舵机中断
    gpio_set(P21_4, 0);
}

/*
 *******************************************************************************************
 ** 函数功能: 以xy轴交点的方式，在LCD上定位一个坐标
 ** 参    数: Inflection：坐标
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawPoint(Point Inflection,uint16 color)
{
    for(uint8 column=0;column<MT9V03X_W-1;column++)
    {
        lcd_drawpoint(column,Inflection.Y,color);
    }
    for(uint8 row=0;row<MT9V03X_H-1;row++)
    {
        lcd_drawpoint(Inflection.X,row,color);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 以xy轴交点的方式，在LCD上定位一个坐标
 ** 参    数:
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawPoint_V2(uint8 row,uint8 column,uint16 color)
{
    for(uint8 cloum=0;cloum<MT9V03X_W-1;cloum++)
    {
        lcd_drawpoint(cloum,row,color);
    }
    for(uint8 row=0;row<MT9V03X_H-1;row++)
    {
        lcd_drawpoint(column,row,color);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 以一条垂直于Y轴直线的方式绘制出Y坐标
 ** 参    数:
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawRow(uint8 row,uint16 color)
{
    for(uint8 column=0;column<MT9V03X_W-1;column++)
    {
        lcd_drawpoint(column,row,color);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 以一条垂直于X轴直线的方式绘制出X坐标
 ** 参    数:
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawColumn(uint8 column,uint16 color)
{
    for(uint8 row=0;row<MT9V03X_H-1;row++)
    {
        lcd_drawpoint(column,row,color);
    }
}
