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

uint8 bias_startline=95,bias_endline=50;        //动态前瞻
uint8 Fork_flag=0;              //三岔识别的标志变量
uint8 Garage_flag=0;            //车库识别标志变量
uint8 speed_case_1=200,speed_case_2=170,speed_case_3=155,speed_case_4=165,speed_case_5=160,speed_case_6=160,speed_case_7=170;
uint32 SobelResult=0;
int LeftLine[MT9V03X_H]={0}, CentreLine[MT9V03X_H]={0}, RightLine[MT9V03X_H]={0};   //扫线处理左中右三线
uint8 stop_flag=0;

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
    static uint8 flag=4;  //状态机跳转标志位
    static uint8 case_5,case_0,case_2,case_1,case_4,case_6,case_3;  //数帧数
    Point InflectionL,InflectionR;     //左右下拐点
    InflectionL.X=0;InflectionL.Y=0;InflectionR.X=0;InflectionR.Y=0;
    Point ForkUpPoint;  //三岔
    ForkUpPoint.X=0;ForkUpPoint.Y=0;
    Point CrossRoadUpLPoint,CrossRoadUpRPoint;  //十字路口
    CrossRoadUpLPoint.X=0;CrossRoadUpLPoint.Y=0;CrossRoadUpRPoint.X=0;CrossRoadUpRPoint.Y=0;
    /*****************************扫线*****************************/
    GetImagBasic(LeftLine, CentreLine, RightLine, 'L');
    /*************************搜寻左右下拐点***********************/
    GetDownInflection(110,45,LeftLine,RightLine,&InflectionL,&InflectionR);
    /*************************特殊元素判断*************************/
//    CircleIslandIdentify_R(RightLine, InflectionR);
    /****************************状态机***************************/
#if 0
    switch(flag)
    {
        case 0: //识别左十字回环
        {

            break;
        }
        case 1: //识别右车库，直行
        {

            break;
        }
        case 2: //识别第一遍三岔
        {

            break;
        }
        case 3: //识别右环岛
        {
            if(CircleIslandIdentify_R(RightLine, InflectionR)==1)
            {
                flag=4;
            }
            break;
        }
        case 4: //识别右十字回环
        {
            if(CrossLoopIdentify_R(LeftLine, RightLine, InflectionL, InflectionR)==1)
            {
                flag=5;
                gpio_set(LED_GREEN, 0);
            }
            break;
        }
        case 5: //识别左环岛
        {
            if(CircleIslandIdentify_L(LeftLine, InflectionL)==1)
            {
                flag=6;
            }
            break;
        }
        case 6: //识别第二遍三岔
        {
            Stop();
            break;
        }
        case 7: //识别左车库，入库
        {

            break;
        }
    }
#endif
    /***************************偏差计算**************************/
    if(Fork_flag!=0||Garage_flag!=0)    //在识别函数里面已经计算了Bias
    {
        Garage_flag=0;Fork_flag=0;      //重置flag
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
    lcd_showfloat(0, 0, Bias, 1, 2);
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
    base_speed=-5;   //设置目标速度为0
    stop_flag=1;     //开启辅助停车防止静差导致车滑行
    pit_disable_interrupt(CCU6_0, PIT_CH1); //关闭舵机中断
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
