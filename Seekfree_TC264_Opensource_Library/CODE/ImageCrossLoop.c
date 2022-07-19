/*
 * ImageCrossLoop.c
 *
 *  Created on: 2022年5月25日
 *      Author: yue
 */

#include "ImageSpecial.h"
#include "PID.h"
#include "ICM20602.h"
#include <stdio.h>
#include "SEEKFREE_18TFT.h"
#include "ImageProcess.h"
#include "Motor.h"
#include "LED.h"

/*
 *******************************************************************************************
 ** 函数功能: 识别左十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **          1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopBegin_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0&&InflectionR.X==0&&InflectionR.Y==0)    //存在左拐点且不存在右拐点
    {
        uint8 row_up=0;
        for(uint8 row=InflectionL.Y+2,column=InflectionL.X-2;row-1>0;row--)  //左拐点往上扫
        {
            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
            {
                for(;row-1>0;row--)   //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                    {
                        row_up=row; //保存补线的终点Y坐标
                        for(;row-1>0;row--)   //继续向上扫
                        {
                            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
                            {
                                for(;row-1;row--)   //继续向上扫
                                {
                                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                                    {
                                        for(row_up-=5;column<MT9V03X_W;column++)
                                        {
                                            if(BinaryImage[row_up][column]==IMAGE_WHITE)
                                            {
                                                Point end;
                                                end.Y=row_up;
                                                end.X=column;
                                                FillingLine('L', InflectionL, end); //补线
                                                return 1;
                                            }
                                        }
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    if(LostNum_LeftLine>70&&LostNum_RightLine<35)   //无拐点但左右丢线符合
    {
        float right_bias=0;
        right_bias=Regression_Slope(110, 60, RightLine);    //求右边线斜率
        if(fabsf(right_bias)>0.6)   //防止进环后的误判
        {
            return 0;
        }
        for(uint8 row=0;row<MT9V03X_H-1;row++)  //向下扫
        {
            if(BinaryImage[row][20]==IMAGE_BLACK&&BinaryImage[row+1][20]==IMAGE_WHITE)  //黑-白
            {
                for(;row<MT9V03X_H-1;row++) //继续向下扫
                {
                    if(BinaryImage[row][20]==IMAGE_WHITE&&BinaryImage[row+1][20]==IMAGE_BLACK)  //白-黑
                    {
                        for(;row<MT9V03X_H-1;row++) //继续向下扫
                        {
                            if(BinaryImage[row][20]==IMAGE_BLACK&&BinaryImage[row+1][20]==IMAGE_WHITE)  //黑-白
                            {
                                //寻找补线点
                                for(uint8 row=100;row-1>0;row--)    //向上扫
                                {
                                    if(LeftLine[row]==0&&LeftLine[row-1]!=0)
                                    {
                                        if(row-5>0)
                                        {
                                            row-=5;
                                        }
                                        Point start,end;
                                        start.Y=119;
                                        start.X=LeftLine[row];
                                        end.Y=row;
                                        end.X=LeftLine[row]+1;  //不能补垂直的线，稍作偏移
                                        FillingLine('L', start, end);   //补线
                                        return 1;
                                    }
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 第二次识别左十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **          1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopOverBegin_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
    if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
    {
        for(uint8 row=MT9V03X_H-30,column=20;row-1>0;row--)    //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
            {
                uint8 row_f=row;
                for(;row-1>0;row--)
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                    {
                        uint8 row_s=row;
                        for(;row-1>0;row--)
                        {
                            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
                            {
                                row=(row_f+row_s)/2;
                                for(;column+1<MT9V03X_W;column++)   //向右扫
                                {
                                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column+1]==IMAGE_WHITE)
                                    {
                                        /*补线操作*/
                                        Point end,start;
                                        end.Y=row;
                                        end.X=column;
                                        start.Y=MT9V03X_H-2;
                                        start.X=1;
                                        FillingLine('L', start, end);   //补线
                                        return 1;
                                    }
                                }
                                return 0;
                            }
                        }
                        return 0;
                    }
                }
                return 0;
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别左十字回环出口
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环出口
 **          1：识别到十字回环出口
 ** 作    者: WBN
 ** 注    意：没有对车子在环中贴边行驶的情况作出分析，若是贴内环不会有太大问题（右拐不会压角）；
 **           若是车子贴外环行驶会有压角风险。预计处理方法是通过右拐点的位置来判断极端情况
 ********************************************************************************************
 */
uint8 CrossLoopEnd_L(void)
{
    static uint8 flag=0;    //连续补线flag，依赖第一次补线判断
    if((LostNum_LeftLine<90&&fabsf(Bias)<4)||flag==1) //符合约束条件或处于连续补线
    {
        Point StarPoint,EndPoint;
        uint8 row=MT9V03X_H-2,column=1,flag_1=0,flag_2=0;
        //寻找补线起点
        if(BinaryImage[row][column]==IMAGE_BLACK)   //左下角为黑
        {
            flag_2=1;
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
        if(flag_2==1)   //存在左拐点
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
        }
        else    //不存在左拐点
        {
            row=MT9V03X_H-2;
        }
        StarPoint.Y=row;    //起点：左拐点or左下角
        StarPoint.X=column;
        uint8 start_row=row;
        //寻找补线终点
        row=MT9V03X_H-2;column=MT9V03X_W-2;flag_1=0;
        if(BinaryImage[row][column]==IMAGE_BLACK)       //右下角为黑（存在右拐点）
        {
            for(;column-1>0;column--)   //将指针移动到底部最左端
            {
                if(BinaryImage[row][column-1]==IMAGE_WHITE)
                {
                    break;
                }
            }
            while(column-2>0&&row-2>0)  //向左上方寻找右拐点
            {
                if(BinaryImage[row][column-1]==IMAGE_BLACK||BinaryImage[row][column-2]==IMAGE_BLACK) //左黑
                {
                    column--;   //左移
                    flag_1=1;
                }
                if(BinaryImage[row-1][column]==IMAGE_BLACK||BinaryImage[row-2][column]==IMAGE_BLACK) //上黑
                {
                    row--;      //上移
                    flag_1=1;
                }
                if(flag_1==1)
                {
                    flag_1=0;
                    continue;
                }
                break;  //探针没有移动
            }
            //通过线性方程求出补线终点
            Point point_1,point_2;
            point_1.Y=MT9V03X_H-2;
            point_1.X=RightLine[MT9V03X_H-2];
            point_2.Y=row;
            point_2.X=column;
            for(;row-1>0;row--) //向上扫，右拐点
            {
                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)   //白-黑
                {
                    break;  //补线终点Y坐标
                }
            }
            column=SlopeUntie_X(point_1, point_2, row); //补线终点X坐标
            if(column<StarPoint.X)
            {
                column=(uint8)StarPoint.X;
            }
        }
        else    //不存在右拐点
        {
            row=2*(MT9V03X_H/3);//防止下方小黑洞的干扰
            for(;row-1>0;row--) //向上扫
            {
                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)   //白-黑
                {
                    break;
                }
            }
        }
        EndPoint.Y=row;     //终点：右拐点上方边界处or右边界上方
        EndPoint.X=column;
        //补左线右转出环
        FillingLine('L', StarPoint, EndPoint);
        //特殊情况求Bias，防止其他元素干扰
        if(row>bias_endline)        //补线终点低于前瞻终点
        {
            bias_endline=row;
        }
        if(start_row<bias_startline)//补线起点高于前瞻起点
        {
            bias_startline=start_row;
        }
        flag=1; //连续补线标志
        return 1;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 左十字回环状态机
 ** 参    数: 无
 ** 返 回 值: 0：没有完成十字回环
 **          1：完成十字回环
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopIdentify_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,flag_in,flag_end;
    switch(flag)
    {
        case 0: //小车识别十字回环的入口，进行补线直行
        {
            if(CrossLoopBegin_L(LeftLine, RightLine, InflectionL, InflectionR)==1&&flag_in==0)  //第一次识别到回环入口
            {
                StartIntegralAngle_Z(20);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启积分
            }
            if(flag_in==1)  //积分已开启，之前已识别到回环入口
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=1;   //跳转到状态1
                    break;
                }
                CrossLoopOverBegin_L(LeftLine, RightLine, InflectionL, InflectionR);    //未入环，补线直行
            }
            break;
        }
        case 1: //小车在环中自主寻迹
        {
            if(flag_in==1)
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=2;   //跳转到状态2
                    break;
                }
            }
            if(flag_in==0)
            {
                StartIntegralAngle_Z(200);  //开启陀螺仪辅助出环
                flag_in=1;                  //避免重复开启陀螺仪
            }
            break;
        }
        case 2: //小车识别十字回环的出口，右转出环
        {
            if(CrossLoopEnd_L()==1&&flag_end==0)  //第一次检测到回环出口
            {
                StartIntegralAngle_Z(30);   //开启积分
                flag_end=1;                 //避免重复开启积分
            }
            if(flag_end==1&&icm_angle_z_flag==1)  //积分已开启
            {
                flag_end=0;flag=3;  //跳转到未知状态，作废
                return 1;
            }
            break;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别右十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **          1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopBegin_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionR.X!=0&&InflectionR.Y!=0&&InflectionL.X==0&&InflectionL.Y==0)    //存在右拐点且不存在左拐点
    {
        //判断依据：黑-白-黑-白-黑
        uint8 row_up=0;
        for(uint8 row=InflectionR.Y+2,column=InflectionR.X+2;row-1>0;row--)  //右拐点往上扫
        {
            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
            {
                for(;row-1>0;row--)   //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                    {
                    row_up=row; //保存补线的终点Y坐标
                    for(;row-1>0;row--)   //继续向上扫
                    {
                        if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
                        {
                            for(;row-1;row--)   //继续向上扫
                            {
                                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                                {
                                    for(row_up-=5;column>0;column--) //向左扫
                                    {
                                        if(BinaryImage[row_up][column]==IMAGE_WHITE)
                                        {
                                            Point end;
                                            end.Y=row_up;
                                            end.X=column;
                                            FillingLine('R', InflectionR, end); //补线
                                            return 1;
                                        }
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    break;
                    }
                }
                break;
            }
        }
    }
    else if(LostNum_RightLine>70&&LostNum_LeftLine<35)   //无拐点但左右丢线符合
    {
        float left_bias=0;
        left_bias=Regression_Slope(110, 60, LeftLine);    //求左边线斜率
        if(fabsf(left_bias)>0.6)   //防止进环后的误判
        {
           return 0;
        }
        //判断依据：黑-白-黑-白
        for(uint8 row=0;row<MT9V03X_H-1;row++)  //向下扫
        {
           if(BinaryImage[row][140]==IMAGE_BLACK&&BinaryImage[row+1][140]==IMAGE_WHITE)  //黑-白
           {
               for(;row<MT9V03X_H-1;row++) //继续向下扫
               {
                   if(BinaryImage[row][140]==IMAGE_WHITE&&BinaryImage[row+1][140]==IMAGE_BLACK)  //白-黑
                   {
                       for(;row<MT9V03X_H-1;row++) //继续向下扫
                       {
                           if(BinaryImage[row][140]==IMAGE_BLACK&&BinaryImage[row+1][140]==IMAGE_WHITE)  //黑-白
                           {
                               //寻找补线点
                               for(uint8 row=100;row-1>0;row--)    //向上扫
                               {
                                   if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)   //丢线-不丢线
                                   {
                                       if(row-5>0)
                                       {
                                           row-=5;
                                       }
                                       Point start,end;
                                       start.Y=MT9V03X_H-1; //起点：终点对应列底部
                                       start.X=RightLine[row];
                                       end.Y=row;
                                       end.X=RightLine[row]-1;  //不能补垂直的线，稍作偏移
                                       FillingLine('R', start, end);   //补线
                                       return 1;
                                   }
                               }
                               break;
                           }
                       }
                       break;
                   }
               }
               break;
           }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 第二次识别右十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **          1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopOverBegin_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
    if(fabsf(bias_left)<G_LINEBIAS)    //左边界为直道
    {
        for(uint8 row=MT9V03X_H-30,column=140;row-1>0;row--)    //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
            {
                uint8 row_f=row;    //黑洞底部
                for(;row-1>0;row--)
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                    {
                        uint8 row_s=row;    //黑洞顶部
                        for(;row-1>0;row--)
                        {
                            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
                            {
                                row=(row_f+row_s)/2;    //补线终点Y坐标
                                for(;column-1>0;column--)   //向左扫
                                {
                                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column-1]==IMAGE_WHITE)
                                    {
                                        /*补线操作*/
                                        Point end,start;
                                        end.Y=row;
                                        end.X=column;
                                        start.Y=MT9V03X_H-2;    //起点：右下角
                                        start.X=MT9V03X_W-2;
                                        FillingLine('R', start, end);   //补线
                                        return 1;
                                    }
                                }
                                return 0;
                            }
                        }
                        return 0;
                    }
                }
                return 0;
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别右十字回环出口
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环出口
 **          1：识别到十字回环出口
 ** 作    者: WBN
 ** 注    意：没有对车子在环中贴边行驶的情况作出分析，若是贴内环不会有太大问题（左拐不会压角）；
 **           若是车子贴外环行驶会有压角风险。预计处理方法是通过左拐点的位置来判断极端情况
 ********************************************************************************************
 */
uint8 CrossLoopEnd_R(void)
{
    static uint8 flag=0;    //连续补线flag，依赖第一次补线判断
    if((LostNum_RightLine<90&&fabsf(Bias)<1.5)||flag==1) //符合约束条件或处于连续补线
    {
        Point StarPoint,EndPoint;
        uint8 row=MT9V03X_H-2,column=MT9V03X_W-2,flag_1=0,flag_2=0;
        //寻找补线起点
        if(BinaryImage[row][column]==IMAGE_BLACK)   //右下角为黑
        {
            flag_2=1;   //存在右拐点
        }
        else                                        //右下角为白
        {
            for(;row-1>2*(MT9V03X_H/3);row--)   //向上扫，右下角
            {
                //白-黑跳变点在图像下方三分之一处
                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                {
                    flag_2=1;
                    break;
                }
            }
        }
        if(flag_2==1)   //存在右拐点
        {
            //向左上方寻找谷底
            for(;column-1>0;column--)   //将指针移动到底部最左端
            {
                if(BinaryImage[row][column-1]==IMAGE_WHITE)
                {
                    break;
                }
            }
            while(column-1>0&&row-1>0)  //左上
            {
                if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                {
                    column--;   //左移
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
        }
        else    //不存在右拐点
        {
            row=MT9V03X_H-2;
        }
        StarPoint.Y=row;    //起点：右拐点or右下角
        StarPoint.X=column;
        uint8 start_row=row;
        //寻找补线终点
        row=MT9V03X_H-2;column=1;flag_1=0;
        if(BinaryImage[row][column]==IMAGE_BLACK)       //左下角为黑（存在左拐点）
        {
            for(;column+1<MT9V03X_W-1;column++)   //将指针移动到底部最右端
            {
                if(BinaryImage[row][column+1]==IMAGE_WHITE)
                {
                    break;
                }
            }
            while(column+2<MT9V03X_W-1&&row-2>0)  //向右上方寻找
            {
                if(BinaryImage[row][column+1]==IMAGE_BLACK||BinaryImage[row][column+2]==IMAGE_BLACK) //左黑
                {
                    column++;   //右移
                    flag_1=1;
                }
                if(BinaryImage[row-1][column]==IMAGE_BLACK||BinaryImage[row-2][column]==IMAGE_BLACK) //上黑
                {
                    row--;      //上移
                    flag_1=1;
                }
                if(flag_1==1)
                {
                    flag_1=0;
                    continue;
                }
                break;  //探针没有移动
            }
            //通过线性方程求出补线终点
            Point point_1,point_2;
            point_1.Y=MT9V03X_H-2;
            point_1.X=LeftLine[MT9V03X_H-2];
            point_2.Y=row;
            point_2.X=column;
            for(;row-1>0;row--) //向上扫，左拐点
            {
                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)   //白-黑
                {
                    break;  //补线终点Y坐标
                }
            }
            column=SlopeUntie_X(point_1, point_2, row); //补线终点X坐标
            if(column>StarPoint.X)
            {
                column=(uint8)StarPoint.X;
            }
        }
        else    //不存在左拐点
        {
            row=2*(MT9V03X_H/3);//防止下方小黑洞的干扰
            for(;row-1>0;row--) //向上扫
            {
                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)   //白-黑
                {
                    break;
                }
            }
        }
        EndPoint.Y=row;     //终点：左拐点上方边界处or左边界上方
        EndPoint.X=column;
        //补右线左转出环
        FillingLine('R', StarPoint, EndPoint);
        //特殊情况求Bias，防止其他元素干扰
        if(row>bias_endline)        //补线终点低于前瞻终点
        {
            bias_endline=row;
        }
        if(start_row<bias_startline)//补线起点高于前瞻起点
        {
            bias_startline=start_row;
        }
        flag=1; //连续补线标志
        return 1;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 右十字回环状态机
 ** 参    数: 无
 ** 返 回 值: 0：没有完成十字回环
 **          1：完成十字回环
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopIdentify_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,flag_in,flag_end;
    switch(flag)
    {
        case 0: //小车识别十字回环的入口，进行补线直行
        {
            if(CrossLoopBegin_R(LeftLine, RightLine, InflectionL, InflectionR)==1&&flag_in==0)  //第一次识别到回环入口
            {
                StartIntegralAngle_Z(20);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启积分
            }
            if(flag_in==1)  //积分已开启，之前已识别到回环入口
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=1;   //跳转到状态1
                    break;
                }
                CrossLoopOverBegin_R(LeftLine, RightLine, InflectionL, InflectionR);    //未入环，补线直行
            }
            break;
        }
        case 1: //小车在环中，自主寻迹
        {
            if(flag_in==1)
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=2;   //跳转到状态2
                    break;
                }
            }
            if(flag_in==0)
            {
                StartIntegralAngle_Z(200);  //开启陀螺仪辅助出环
                flag_in=1;                  //避免重复开启陀螺仪
            }
            break;
        }
        case 2: //小车识别十字回环的出口，右转出环
        {
            if(CrossLoopEnd_R()==1&&flag_end==0)  //第一次检测到回环出口
            {
                StartIntegralAngle_Z(30);   //开启积分
                flag_end=1;                 //避免重复开启积分
            }
            if(flag_end==1&&icm_angle_z_flag==1)  //积分已开启
            {
                flag_end=0;flag=3;  //跳转到未知状态，作废
                return 1;
            }
            break;
        }
    }
    return 0;
}
