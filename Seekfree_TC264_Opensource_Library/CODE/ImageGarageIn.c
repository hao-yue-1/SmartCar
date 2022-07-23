/*
 * ImageGarageIn.c
 *
 *  Created on: 2022年7月22日
 *      Author: yue
 */

#include "ImageSpecial.h"
#include "ImageProcess.h"
#include <math.h>
#include "LED.h"
#include "zf_gpio.h"
#include "ICM20602.h"
#include "PID.h"

/********************************************************************************************
 ** 函数功能: 检测斑马线
 ** 参    数: start_line：检测起始行
 **           end_line：检测结束行
 ** 返 回 值: 1：检测到斑马线
 **           0：没有检测到斑马线
 ** 作    者: WBN
 *********************************************************************************************/
uint8 ZebraCrossingSearch(uint8 start_line,uint8 end_line)
{
    uint8 num=0;
//    uint8 last_num=0; //调试
    for(uint8 row=start_line;row-2>end_line;row-=2)    //向上扫
    {
        num=0;
        for(uint8 column=0;column+1<MT9V03X_W-1;column++)   //向右扫
        {
            if(BinaryImage[row][column]!=BinaryImage[row][column+1])    //跳变点
            {
                num++;
                if(num>10)
                {
                    return 1;
                }
//                //调试
//                if(num>last_num)
//                {
//                    last_num=num;
//                }
            }
        }
    }
    return 0;
}

/********************************************************************************************
 ** 函数功能: 补线入库，调用该函数必将补线
 ** 参    数: 无
 ** 返 回 值: 0：没有完成入库
 **          1：完成入库
 ** 作    者: WBN
 *********************************************************************************************/
void GarageInBegin(void)
{
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-2,column=1,flag_1=0,flag_2=0;
    //寻找补线起点
    if(BinaryImage[row][column]==IMAGE_BLACK)   //左下角为黑
    {
        flag_2=1;   //存在右拐点
    }
    else                                        //左下角为白
    {
        for(;row-1>2*(MT9V03X_H/3);row--)   //向上扫，左下角
        {
            //白-黑跳变点在图像下方三分之一处
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
            {
                flag_2=1;
                break;
            }
        }
    }
    if(flag_2==1)  //存在左拐点
    {
        //向右上方寻找拐点
        for(;column+1<MT9V03X_W-1;column++)   //将指针移动到底部最右端
        {
            if(BinaryImage[row][column+1]==IMAGE_WHITE)
            {
                break;
            }
        }
        while(column+1<MT9V03X_W-1&&row-1>0)  //右上
        {
            if(BinaryImage[row][column+1]==IMAGE_BLACK) //右黑
            {
                column++;   //右移
                flag_1=1;
            }
            if(BinaryImage[row-1][column]==IMAGE_BLACK) //上黑
            {
                row--;      //上移
                flag_1=1;
            }
            if(flag_1==1)
            {
                flag_1=0;
                continue;
            }
            break;
        }
        //判断是不是真正的拐点，排除已经看不到直道的情况
        if(row>MT9V03X_H-5) //基本是最低行
        {
            flag_2=0;   //假拐点
        }
        StarPoint.Y=row;            //起点：左拐点对应右边界
        StarPoint.X=RightLine[row];
    }
    if(flag_2==0)   //这里不用else是因为上面的if中还有对flag_2的进一步确认
    {
        StarPoint.Y=MT9V03X_H-2;    //起点：右下角
        StarPoint.X=MT9V03X_W-2;
    }
    uint8 start_row=(uint8)StarPoint.Y;
    //寻找补线终点
    for(;row-1>0;row--) //向上扫
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            for(;column+1<MT9V03X_W-1;column++)   //将指针移动到底部最右端
            {
                if(BinaryImage[row][column+1]==IMAGE_BLACK)
                {
                    break;
                }
            }
            while(column+1<MT9V03X_W-1&&row+1<MT9V03X_H-1)  //右下
            {
                if(BinaryImage[row][column+1]==IMAGE_BLACK) //右黑
                {
                    column++;   //右移
                    flag_1=1;
                }
                if(BinaryImage[row+1][column]==IMAGE_BLACK) //下黑
                {
                    row++;      //下移
                    flag_1=1;
                }
                if(flag_1==1)
                {
                    flag_1=0;
                    continue;
                }
                break;
            }
            break;
        }
    }
    EndPoint.Y=row;     //终点：谷底
    EndPoint.X=column;
    if(EndPoint.Y>=StarPoint.Y||EndPoint.X>=StarPoint.X)    //异常
    {
        return;
    }
    //补右线左拐
    FillingLine('R', StarPoint, EndPoint);
//    //补左线修正
    StarPoint.X=0;
    EndPoint.X=0;
    FillingLine('L', StarPoint, EndPoint);
    //特殊情况求Bias，防止其他元素干扰
//    if(row>2*(MT9V03X_H/3)) //补线终点过低
//    {
////        lcd_showuint8(0, 0, 0);
//        return;
//    }
//    if(row>bias_endline)        //补线终点低于前瞻终点
//    {
//        bias_endline=row;
//    }
//    if(start_row<bias_startline)//补线起点高于前瞻起点
//    {
//        bias_startline=start_row;
//    }
//    for(uint8 i=0;i<MT9V03X_W-1;i++)
//    {
//        lcd_drawpoint(i, bias_startline, BROWN);
//        lcd_drawpoint(i, bias_endline, YELLOW);
//    }
//    lcd_showfloat(0, 1, Bias, 1, 2);
    Bias=DifferentBias_Garage(bias_startline,bias_endline,CentreLine); //动态前瞻计算偏差
    bias_startline=95;bias_endline=50;                          //恢复默认前瞻
    Garage_flag=1;
}

/********************************************************************************************
 ** 函数功能: 入库状态机
 ** 参    数: 无
 ** 返 回 值: 0：没有完成入库
 **          1：完成入库
 ** 作    者: WBN
 *********************************************************************************************/
uint8 GarageInEnd(void)
{
    //最简单：Bias<1
    Bias=DifferentBias(bias_startline,bias_endline,CentreLine); //动态前瞻计算偏差
    if(fabs(Bias)<1)
    {
        return 1;
    }
    //中线在图像下三分之一处丢线
    for(uint8 row=MT9V03X_H-1;row>MT9V03X_H/3;row--)
    {
        if(BinaryImage[row][CentreLine[row]]==IMAGE_BLACK)
        {
            return 1;
        }
    }
    return 0;
}

/********************************************************************************************
 ** 函数功能: 入库状态机
 ** 参    数: 无
 ** 返 回 值: 0：没有完成入库
 **          1：完成入库
 ** 作    者: WBN
 *********************************************************************************************/
uint8 GarageInIdentify(void)
{
    static uint8 flag=0,flag_in=0;
    switch(flag)
    {
        case 0: //识别斑马线
        {
            if(ZebraCrossingSearch(MT9V03X_H/2+15, MT9V03X_H/2-15)==1)    //识别到斑马线
            {
                base_speed=150;
                flag=1;
                gpio_set(LED_YELLOW, 0);
            }
            break;
        }
        case 1: //补线过渡
        {
            GarageInBegin();    //优先补线
            if(flag_in==1&&icm_angle_z_flag==1) //积分完成
            {
                flag=2;
                gpio_set(LED_GREEN, 0);
            }
            else if(flag_in==0)
            {
                StartIntegralAngle_Z(20);       //开启积分
                flag_in=1;
            }
            break;
        }
        case 2: //打死入库
        {
            if(GarageInEnd()==1)    //识别到已经入库
            {
                Stop();
                gpio_set(LED_WHITE, 0);
            }
            else
            {
//                GarageInBegin();    //补线入库
                Bias=10;
                Garage_flag=1;
            }
        }
    }
    return 0;
}
