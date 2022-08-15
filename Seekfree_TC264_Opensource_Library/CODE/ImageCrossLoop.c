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
uint8 CrossLoopBegin_L(Point InflectionL,uint8 status)
{
    static uint8 flag=0;    //连续补线flag
    //函数重启
    if(status==1)
    {
        flag=0;
        return 0;
    }

    if((InflectionL.X!=0&&InflectionL.Y>50)||flag==1)  //符合约束条件or处于连续补线状态
    {
        uint8 row=MT9V03X_H-5,column=4,flag_1=0;
        Point StarPoint,EndPoint;
        //寻找补线起点
        if(BinaryImage[row][column]==IMAGE_BLACK)   //左下角为黑（存在左拐点）
        {
            for(;column+1<MT9V03X_W-1;column++)   //将指针移动到底部最右端
            {
                if(BinaryImage[row][column+1]==IMAGE_WHITE)
                {
                    break;
                }
            }
            while(column+1<MT9V03X_W-1&&row-1>0)  //向右上方寻找
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
            StarPoint.Y=row;    //起点：左拐点
            StarPoint.X=column;
        }
        else
        {
            StarPoint.Y=row;    //起点：左下角
            StarPoint.X=column;
            column=MT9V03X_W/4; //为寻找上方黑洞，重置X坐标为左四分之一处
        }
        //寻找补线终点
        for(;row-1>0;row--)    //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
            {
                uint8 row_f=row;
                for(;row-1>0;row--) //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
                    {
                        uint8 row_s=row;
                        for(;row-1>0;row--) //继续向上扫
                        {
                            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                            {
                                row=(row_f+row_s)/2;
                                for(;column+1<MT9V03X_W;column++)   //向右扫
                                {
                                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column+1]==IMAGE_WHITE)  //黑-白
                                    {
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
        //限制补线终点：车子已经接近入环不需要补线的情况
        if(row<MT9V03X_H/6)
        {
            return 0;
        }
        //补左线直行
        EndPoint.Y=row; //终点：黑洞右边界点
        EndPoint.X=column;
        FillingLine('L', StarPoint, EndPoint);
        flag=1;
        return 1;
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
uint8 CrossLoopEnd_L(uint8 status)
{
    static uint8 flag=0;    //连续补线flag，依赖第一次补线判断
    //函数重启
    if(status==1)
    {
        flag=0;
        return 0;
    }

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
uint8 CrossLoopIdentify_L(Point InflectionL)
{
    static uint8 flag,flag_in,flag_end;
    switch(flag)
    {
        case 0: //小车识别十字回环的入口，进行补线直行
        {
            if(CrossLoopBegin_L(InflectionL,0)==1&&flag_in==0)  //第一次识别到回环入口
            {
                StartIntegralAngle_Z(20);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启积分
            }
            if(flag_in==1&&icm_angle_z_flag==1)  //陀螺仪识别到已经入环
            {
                flag_in=0;flag=1;   //跳转到状态1
                break;
            }
            break;
        }
        case 1: //小车在环中自主寻迹
        {
            if(flag_in==1&&icm_angle_z_flag==1) //陀螺仪识别到已经接近出口
            {
                flag_in=0;flag=2;   //跳转到状态2
                break;
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
            if(CrossLoopEnd_L(0)==1&&flag_end==0)  //第一次检测到回环出口
            {
                StartIntegralAngle_Z(30);   //开启积分
                flag_end=1;                 //避免重复开启积分
            }
            if(flag_end==1&&icm_angle_z_flag==1)  //陀螺仪识别到已经出环
            {
                flag_end=0;flag=0;  //跳转到状态0，等待二次启用
                //重启函数
                CrossLoopBegin_L(InflectionL,1);
                CrossLoopEnd_L(1);
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
 ** 注    意：该函数和环岛Exit函数是一样的，唯一的区别是这里对补线的终点作出了限制，适应十字回环入环
 **           的情况，避免在已经不需要补线的回环入口处补线导致入环失败
 **           8.15
 ********************************************************************************************
 */
uint8 CrossLoopBegin_R(Point InflectionR,uint8 status)
{
    static uint8 flag=0;    //连续补线flag
    //函数重启
    if(status==1)
    {
        flag=0;
        return 0;
    }

    if((InflectionR.X!=0&&InflectionR.Y>50)||flag==1)  //符合约束条件or处于连续补线状态
    {
        uint8 row=MT9V03X_H-5,column=MT9V03X_W-5,flag_1=0;
        Point StarPoint,EndPoint;
        //寻找补线起点
        if(BinaryImage[row][column]==IMAGE_BLACK)   //右下角为黑（存在右拐点）
        {
            for(;column-1>0;column--)   //将指针移动到底部最左端
            {
                if(BinaryImage[row][column-1]==IMAGE_WHITE)
                {
                    break;
                }
            }
            while(column-1>0&&row-1>0)  //向左上方寻找
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
            StarPoint.Y=row;    //起点：右拐点
            StarPoint.X=column;
        }
        else
        {
            StarPoint.Y=row;    //起点：右下角
            StarPoint.X=column;
            column=3*(MT9V03X_W/4); //为寻找上方黑洞，重置X坐标为右四分之一处
        }
        //寻找补线终点
        for(;row-1>0;row--)    //向上扫，寻找黑洞下边界
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
            {
                uint8 row_f=row;    //黑洞下边界
                for(;row-1>0;row--) //继续向上扫，寻找黑洞上边界
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
                    {
                        uint8 row_s=row;    //黑洞上边界
                        for(;row-1>0;row--) //继续向上扫，对黑洞进行验证
                        {
                            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                            {
                                row=(row_f+row_s)/2;    //黑洞中点Y坐标
                                for(;column-1>0;column--)   //向左扫，寻找黑洞左边界
                                {
                                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column-1]==IMAGE_WHITE)  //黑-白
                                    {
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
        //限制补线终点：车子已经接近入环不需要补线的情况
        if(row<MT9V03X_H/6)
        {
            return 0;
        }
        //补右线直行
        EndPoint.Y=row; //终点：黑洞左边界点
        EndPoint.X=column;
        FillingLine('R', StarPoint, EndPoint);
        flag=1;
        return 1;
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
uint8 CrossLoopEnd_R(uint8 status)
{
    static uint8 flag=0;    //连续补线flag，依赖第一次补线判断
    //函数重启
    if(status==1)
    {
        flag=0;
        return 0;
    }

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
uint8 CrossLoopIdentify_R(Point InflectionR)
{
    static uint8 flag,flag_in,flag_end;
    switch(flag)
    {
        case 0: //小车识别十字回环的入口，进行补线直行
        {
            if(CrossLoopBegin_R(InflectionR,0)==1&&flag_in==0)  //第一次识别到回环入口
            {
                StartIntegralAngle_Z(20);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启积分
            }
            if(flag_in==1&&icm_angle_z_flag==1)  //陀螺仪识别到已经入环
            {
                flag_in=0;flag=1;   //跳转到状态1
                break;
            }
            break;
        }
        case 1: //小车在环中，自主寻迹
        {
            if(flag_in==1&&icm_angle_z_flag==1) //陀螺仪识别到已经接近出口
            {
                flag_in=0;flag=2;   //跳转到状态2
                break;
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
            if(CrossLoopEnd_R(0)==1&&flag_end==0)  //第一次检测到回环出口
            {
                StartIntegralAngle_Z(30);   //开启积分
                flag_end=1;                 //避免重复开启积分
            }
            if(flag_end==1&&icm_angle_z_flag==1)  //陀螺仪识别到已经出环
            {
                flag_end=0;flag=0;  //跳转到状态0，等待二次启用
                //重启函数
                CrossLoopBegin_R(InflectionR,1);
                CrossLoopEnd_R(1);
                return 1;
            }
            break;
        }
    }
    return 0;
}
