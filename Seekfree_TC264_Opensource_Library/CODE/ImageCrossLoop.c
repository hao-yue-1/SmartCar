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
uint8 CrossLoopEnd_L(Point InflectionL,Point InflectionR)
{
    static uint8 flag=0;    //连续补线flag，依赖第一次补线判断

    if(flag==1) //已经补过线，避免不可预知干扰打断补线
    {
        for(uint8 row=MT9V03X_H-1;row-1>0;row--)  //中间列向上扫
        {
            if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
            {
                uint8 EndX=0;
                if(row>70)  //根据逼近边界程度来决定补线矫正程度
                {
                    EndX=MT9V03X_W-1;
                }
                else
                {
                    EndX=MT9V03X_W/2;
                }
                //补左线，右转
                Point StartPoint,EndPoint;
                StartPoint.Y=MT9V03X_H-1;   //起点：最底行的左边界点
                StartPoint.X=LeftLine[MT9V03X_H-1];
                EndPoint.Y=row;             //终点：分界线
                EndPoint.X=EndX;
                FillingLine('L', StartPoint, EndPoint);
                Bias=DifferentBias(StartPoint.Y,EndPoint.Y,CentreLine);
                return 1;
            }
        }
        //若在上面的for循环中没有发现，下面实施补救
    }
    else if(LostNum_LeftLine>55&&LostNum_RightLine>55&&fabsf(Bias)<1.5) //出口判断的约束条件
    {
        for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //中间列向上扫
        {
            if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
            {
                //补左线，右转
                Point StartPoint,EndPoint;
                StartPoint.Y=MT9V03X_H-1;   //起点：最底行的左边界点
                StartPoint.X=LeftLine[MT9V03X_H-1];
                EndPoint.Y=row;             //终点：分界行中点
                EndPoint.X=MT9V03X_W/2;
                FillingLine('L', StartPoint, EndPoint);
                flag=1; //连续补线标记
                Bias=DifferentBias(StartPoint.Y,EndPoint.Y,CentreLine);
                return 1;
            }
        }
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
    static uint8 flag,flag_end,num_1,num_2;

    switch(flag)
    {
        case 0: //小车识别十字回环的入口，进行补线直行
        {
            if(CrossLoopBegin_L(LeftLine, RightLine, InflectionL, InflectionR)==1)  //识别到回环入口，补线直行
            {
                if(num_1<100) //num限幅
                {
                    num_1++;
                }
            }
            else if(num_1>0) //没有识别到入口但之前有识别到，对应刚压过入口的情况
            {
                CrossLoopOverBegin_L(LeftLine, RightLine, InflectionL, InflectionR);    //第二次识别回环入口，补线直行
                num_2++;
            }
//            if(num_2>/*某个阈值*/)  //代表已经进入回环
//            {
//                flag=1; //跳转到状态1
//                break;
//            }
            break;
        }
        case 1: //小车识别十字回环的出口，右转出环
        {
            if(CrossLoopEnd_L(InflectionL, InflectionR)==1&&flag_end==0)  //第一次检测到回环出口
            {
                StartIntegralAngle_Z(70);   //开启积分
                flag_end=1;
            }
            if(flag_end==1)  //积分已经开启
            {
                if(icm_angle_z_flag==1) //检测积分状态
                {
                    flag_end=0;flag=2; //跳转到状态2
                    return 1;
                    break;
                }
            }
            break;
        }
    }
    return 0;
}
