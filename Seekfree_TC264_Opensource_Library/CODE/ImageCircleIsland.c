/*
 * ImageCircleIsland.c
 *
 *  Created on: 2022年5月25日
 *      Author: yue
 */

#include "ImageSpecial.h"
#include "PID.h"
#include "common.h"
#include "ICM20602.h"
#include "LED.h"
#include "zf_gpio.h"
#include "ImageProcess.h"
#include "zf_stm_systick.h"
#include <stdio.h>
#include "ImageTack.h"

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，左侧
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **          1：识别到环岛且在车身左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandBegin_L(void)
{
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-1,column=0;
    //第一段补线：
    //寻找补线起点
    if(BinaryImage[MT9V03X_H-10][1]==IMAGE_BLACK&&BinaryImage[MT9V03X_H-20][9]==IMAGE_BLACK)    //左下角存在黑洞
    {
        for(;row>0;row--)   //向上扫，右边界
        {
            if(RightLine[row]!=MT9V03X_W-1) //不丢线
            {
                break;
            }
        }
    }
    StarPoint.Y=row;    //起点：右边界不丢线处or右下角
    StarPoint.X=RightLine[row];
    uint8 start_row=row;//记录补线起点的Y坐标（用于求Bias）
    //寻找补线终点
    for(row=MT9V03X_H-1,column=MT9V03X_W/3;row-1>0;row--)    //向上扫，靠左三分之一
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            uint8 flag_down=0;  //边界点的位置（1：左边界；2：右边界）
            if(BinaryImage[row][column+1]==IMAGE_BLACK)      //右边为黑
            {
                flag_down=1;    //左边界
            }
            else if(BinaryImage[row][column-1]==IMAGE_BLACK) //左边为黑
            {
                flag_down=2;    //右边界
            }
            else    //X坐标没有落在跳变点上，默认是左边界，手动迫近
            {
                for(;column<MT9V03X_H-1;column++)   //向右扫
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK)
                    {
                        flag_down=1;    //左边界
                        break;
                    }
                }
             }
            switch(flag_down)
            {
                case 1: //左边界，向右寻找谷底
                {
                    uint8 flag_1=0;
                    while(column+1<MT9V03X_W-1&&row+1<MT9V03X_H-1)
                    {
                        if(BinaryImage[row][column+1]==IMAGE_BLACK) //右黑
                        {
                            column++;   //右移
                            flag_1=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK)    //下黑
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
                case 2: //右边界，向左寻找谷底
                {
                    uint8 flag_2=0;
                    while(column-1>0&&row+1<MT9V03X_H-1)
                    {
                        if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                        {
                            column--;   //左移
                            flag_2=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK) //下黑
                        {
                            row++;      //下移
                            flag_2=1;
                        }
                        if(flag_2==1)
                        {
                            flag_2=0;
                            continue;
                        }
                        break;
                    }
                    break;
                }
                default:    //意外情况：无法判别边界点位置
                {
                    EndPoint.Y=MT9V03X_H/2;     //终点：定为图像的中心点
                    EndPoint.X=MT9V03X_W/2;
                }
            }
            EndPoint.Y=row;     //终点：谷底
            EndPoint.X=column;
            break;
        }
    }
    //补右线左拐入环
    FillingLine('R', StarPoint, EndPoint);
    //第二段补线：
    //寻找补线起点
    StarPoint.Y=row;    //起点：第一段补线的终点
    StarPoint.X=column;
    //寻找补线终点
    for(column=1;row-1>0;row--)
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            break;
        }
    }
    EndPoint.Y=row;     //终点：左边谷顶
    EndPoint.X=column;
    //补右线左拐入环
    FillingLine('R', StarPoint, EndPoint);
    //修正左边界
    StarPoint.X=column;
    FillingLine('L', StarPoint, EndPoint);
//    //特殊情况求Bias
//    if(row>bias_endline)    //防止其他元素干扰，当补线终点低于默认前瞻时，视为特殊情况
//    {
//        Bias=DifferentBias(start_row,row,CentreLine);//特殊情况求Bias
//        CircleIsland_flag=1;
//    }
    return 1;
}

/*
 *******************************************************************************************
 ** 函数功能: 第二次识别环岛入口，左侧，补线直行忽略
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **          1：识别到环岛且在车身左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandOverBegin_L(int *LeftLine)
{
    //判断是否完全出环：丢线数符合且左下方不存在黑洞
    if(LostNum_LeftLine<50) //丢线数符合
    {
        uint8 flag=0;
        if(BinaryImage[MT9V03X_H-5][4]==IMAGE_BLACK)    //左下角为黑
        {
            for(uint8 row=MT9V03X_H-5,column=4;row-2>0;row--)   //向上扫
            {
                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE&&BinaryImage[row-2][column]==IMAGE_WHITE)    //黑-白-白（避免偶然出现的白点误判）
                {
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)    //白-黑
                        {
                            flag=1; //存在黑洞
                            break;
                        }
                    }
                    break;
                }
            }
        }
        if(flag==0)
        {
            return 0;
        }
    }
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-1,column=0;
    //寻找补线起点
    StarPoint.Y=row;    //起点：左边界不丢线处or左下角
    StarPoint.X=LeftLine[row];
    //寻找补线终点
    for(row=MT9V03X_H-1,column=MT9V03X_W/3;row-1>0;row--)    //向上扫，靠左三分之一
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            uint8 flag_down=0;  //边界点的位置（1：左边界；2：右边界）
            if(BinaryImage[row][column+1]==IMAGE_BLACK)      //右边为黑
            {
                flag_down=1;    //左边界
            }
            else if(BinaryImage[row][column-1]==IMAGE_BLACK) //左边为黑
            {
                flag_down=2;    //右边界
            }
            else    //X坐标没有落在跳变点上，默认是左边界，手动迫近
            {
                flag_down=3;    //出环的时候车身右斜太厉害，若和入环一样找谷底出错概率变大，不如简单处理
            }
            switch(flag_down)
            {
                case 1: //左边界，向右寻找谷底
                {
                    uint8 flag_1=0;
                    while(column+1<MT9V03X_W-1&&row+1<MT9V03X_H-1)
                    {
                        if(BinaryImage[row][column+1]==IMAGE_BLACK) //右黑
                        {
                            column++;   //右移
                            flag_1=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK)    //下黑
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
                case 2: //右边界，向左寻找谷底
                {
                    uint8 flag_2=0;
                    while(column-1>0&&row+1<MT9V03X_H-1)
                    {
                        if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                        {
                            column--;   //左移
                            flag_2=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK) //下黑
                        {
                            row++;      //下移
                            flag_2=1;
                        }
                        if(flag_2==1)
                        {
                            flag_2=0;
                            continue;
                        }
                        break;
                    }
                    break;
                }
                default:    //意外情况：无法判别边界点位置
                {
                    EndPoint.Y=MT9V03X_H/2;     //终点：定为图像的中心点
                    EndPoint.X=MT9V03X_W/2;
                }
            }
            EndPoint.Y=row;     //终点：谷底
            EndPoint.X=column;
            break;
        }
    }
    //补左线直行
    FillingLine('L', StarPoint, EndPoint);
    return 1;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛结束标志出口，左侧
 ** 参    数: InflectionL：左下拐点
 **          InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **          1：识别到环岛出口且在车身左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandEnd_L(void)
{
    static uint8 flag=0;    //保证补线的连续性，依赖于第一次判断
    if((LostNum_LeftLine<90&&fabsf(Bias)<4)||flag==1)  //符合约束条件或处于连续补线
    {
        Point StarPoint,EndPoint;
        //寻找补线起点
        uint8 row=MT9V03X_H-2,column=MT9V03X_W-2,flag_1;
        if(BinaryImage[row][column]==IMAGE_BLACK)   //右下角为黑（可能存在右拐点）
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
            flag_1=1;   //标记找到了右拐点，为下面找终点提供分类依据
        }
        StarPoint.Y=row;    //起点：右拐点or右下角
        StarPoint.X=column;
        //寻找补线终点
        column=1;           //终点X坐标取左边界（由于环岛出环没有像十字回环一样的直角压角问题，所以这里可以简单处理）
        for(;row-1>0;row--) //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
            {
                EndPoint.Y=row;     //终点：上边界点
                EndPoint.X=column;
                break;
            }
        }
        //补右线左转出环
        FillingLine('R', StarPoint, EndPoint);
        flag=1; //连续补线标志
        return 1;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛出口，左侧，补线直行
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 **          InflectionL：左下拐点
 **          InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到目标
 **          1：识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandExit_L(Point InflectionL)
{
    static uint8 flag=0;    //连续补线flag
    if((InflectionL.X!=0&&InflectionL.Y>85)||flag==1)  //符合约束条件or处于连续补线状态
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
            column=MT9V03X_W/3; //为寻找上方黑洞，重置X坐标为右三分之一处
        }

        //寻找补线终点
        for(;row-1>0;row--)    //向上扫
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
 ** 函数功能: 识别环岛中部，左侧
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：识别到目标
 **          1：没有识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandMid_L(void)
{
    uint8 flag=0;  //环岛中部约束条件
    if(BinaryImage[MT9V03X_H-5][4]==IMAGE_BLACK||BinaryImage[MT9V03X_H-3][2]==IMAGE_BLACK)    //正常情况，左下为黑
    {
        flag=1; //符合约束条件
        //下面的程序防止在Exit处误判：误判了圆环与直角交界处的黑块，上面有一个黑洞
        uint8 column=2,row=MT9V03X_H-3,flag_1=0;    //寻找黑洞所在X坐标
        for(;column+1>MT9V03X_W-1;column++)   //将指针移动到底部最右端
        {
            if(BinaryImage[row][column+1]==IMAGE_WHITE)
            {
                break;
            }
        }
        while(column+1<MT9V03X_W-1&&row-1>0)
        {
            if(BinaryImage[row][column+1]==IMAGE_BLACK)    //右黑
            {
                column++;   //右移
                flag_1=1;
            }
            if(BinaryImage[row-1][column]==IMAGE_BLACK)    //上黑
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
        for(;row-1>0;row--)    //向上扫，左边界
        {
            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)    //黑-白
            {
                for(;row-1>0;row--) //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)    //白-黑
                    {
                        for(;row-1>0;row--) //继续向上扫
                        {
                            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)    //黑-白
                            {
                                flag=0; //不符合约束条件
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    else    //判断是否满足车子靠右的情况（左边有一个接近正中间的，极小的黑洞）
    {
        uint8 row=MT9V03X_H-1;
        for(;row-1>0;row--)  //向上扫，左边界
        {
            if(BinaryImage[row][1]==IMAGE_WHITE&&BinaryImage[row-1][1]==IMAGE_BLACK)    //白-黑（黑洞下边界）
            {
                uint8 row_low=row;  //记录黑洞下边界
                for(;row-1>0;row--)   //继续向上扫
                {
                    if(BinaryImage[row][1]==IMAGE_BLACK&&BinaryImage[row-1][1]==IMAGE_WHITE)    //黑-白（黑洞上边界）
                    {
                        row=(row+row_low)/2;    //计算出黑洞中点Y坐标
                        if(row>50&&row<70)      //黑洞位于中间位置
                        {
                            for(uint8 column=0;column+1<MT9V03X_W-1;column++)   //向右扫，黑洞中点
                            {
                                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column+1]==IMAGE_WHITE)  //黑-白（黑洞右边界）
                                {
                                    if(column<MT9V03X_W/4)  //黑洞右边界位于图像左部
                                    {
                                        flag=1; //符合约束条件
                                    }
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
        //下面的程序防止在Exit处误判：将圆环与直道交界处误判为黑洞，上面还有一个真正的黑洞
        if(flag==1) //上面条件成立的情况下作以下约束
        {
            for(;row-1>0;row--) //黑洞中点处向上扫，左边界
            {
                if(BinaryImage[row][1]==IMAGE_BLACK&&BinaryImage[row-1][1]==IMAGE_WHITE)    //黑-白
                {
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][1]==IMAGE_WHITE&&BinaryImage[row-1][1]==IMAGE_BLACK)    //白-黑
                        {
                            for(;row-1>0;row--) //继续向上扫
                            {
                                if(BinaryImage[row][1]==IMAGE_BLACK&&BinaryImage[row-1][1]==IMAGE_WHITE)    //黑-白
                                {
                                    for(;row-1>0;row--) //继续向上扫
                                    {
                                        if(BinaryImage[row][1]==IMAGE_WHITE&&BinaryImage[row-1][1]==IMAGE_BLACK)    //白-黑
                                        {
                                            flag=0; //不符合约束条件
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
    }
    //满足约束条件下的进一步判断
    if(flag==1)
    {
        return 1;
    }
    return 0;
}
/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag3是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag3不成立
 **           1：Flag3成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandInside_L(void)
{
    if(LostNum_LeftLine>70)    //左边接近全丢线
    {
        //下面采用经验值随机抽样法
        if(BinaryImage[90][40]==IMAGE_WHITE&&BinaryImage[30][120]==IMAGE_BLACK)
        {
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别左环岛
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 **          InflectionL：左下拐点
 **          InflectionR：右下拐点
 ** 返 回 值: 0：还未完成环岛
 **          1：完成环岛
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandIdentify_L(int *LeftLine,Point InflectionL)
{
    static uint8 flag,flag_exit,flag_in,flag_begin,flag_last_begin,flag_last2_begin,flag_end;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(flag_exit==1)        //之前已识别到环岛出口
            {
                if(CircleIslandMid_L()==1)   //识别到环岛中部
                {
                    flag_exit=0;flag=1; //跳转到状态1
                    break;
                }
            }
            if(CircleIslandExit_L(InflectionL)==1)  //识别到环岛Exit作为开启环岛中部检测的条件
            {
                flag_exit=1;
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_L()==1&&flag_in==0) //识别到环岛入口
            {
                StartIntegralAngle_Z(45);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启陀螺仪
            }
            if(flag_in==1)  //之前已经识别到环岛入口
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=2;   //跳转到状态2
                    break;
                }
            }
            break;
        }
        case 2: //此时小车在环中，自主寻迹
        {
            if(flag_in==1)
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=3;   //跳转到状态3
                    break;
                }
            }
            if(flag_in==0)
            {
                StartIntegralAngle_Z(180);  //开启陀螺仪辅助出环
                flag_in=1;                  //避免重复开启陀螺仪
            }
            break;
        }
        case 3: //此时小车已经接近环岛出口，开始判断环岛出口
        {
            if(CircleIslandEnd_L()==1&&flag_end==0)  //第一次检测到环岛出口
            {
                StartIntegralAngle_Z(70);   //开启积分
                flag_end=1;
            }
            if(flag_end==1)  //积分已经开启
            {
                if(icm_angle_z_flag==1) //检测积分状态
                {
                    flag_end=0;flag=4; //跳转到状态4
                    break;
                }
            }
            break;
        }
        case 4: //此时小车已经出环，但是会再次进过环岛入口，需要补线直行
        {
            flag_begin=CircleIslandOverBegin_L(LeftLine);     //识别环岛入口补线忽略
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=5;     //跳转到未知状态，状态机作废
                return 1;   //退出状态机
            }
            flag_last2_begin=flag_last_begin;   //保存上上次的状态
            flag_last_begin=flag_begin;         //保存上一次的状态
            break;
        }
        default:break;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandBegin_R(void)
{
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-1,column=MT9V03X_W-1;
    //第一段补线：
    //寻找补线起点
    if(BinaryImage[MT9V03X_H-10][MT9V03X_W-2]==IMAGE_BLACK&&BinaryImage[MT9V03X_H-20][MT9V03X_W-10]==IMAGE_BLACK)    //右下角存在黑洞
    {
        for(;row>0;row--)   //向上扫，左边界
        {
            if(LeftLine[row]!=0) //不丢线
            {
                break;
            }
        }
    }
    StarPoint.Y=row;    //起点：
    StarPoint.X=LeftLine[row];
    uint8 start_row=row;//记录补线起点的Y坐标（用于求Bias）
    //寻找补线终点
    for(row=MT9V03X_H-1,column=3*(MT9V03X_W/4);row-1>0;row--)    //向上扫，靠右四分之三处
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            uint8 flag_down=0;  //边界点的位置（1：左边界；2：右边界）
            if(BinaryImage[row][column+1]==IMAGE_BLACK)      //右边为黑
            {
                flag_down=1;    //左边界
            }
            else if(BinaryImage[row][column-1]==IMAGE_BLACK) //左边为黑
            {
                flag_down=2;    //右边界
            }
            else    //X坐标没有落在跳变点上，默认为右边界，手动迫近
            {
                for(;column>0;column--)   //向左扫
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK)
                    {
                        flag_down=2;    //右边界
                        break;
                    }
                }
            }
            switch(flag_down)
            {
                case 1: //左边界，向右寻找谷底
                {
                    uint8 flag_1=0;
                    while(column+1<MT9V03X_W-1&&row+1<MT9V03X_H-1)
                    {
                        if(BinaryImage[row][column+1]==IMAGE_BLACK) //右黑
                        {
                            column++;   //右移
                            flag_1=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK)    //下黑
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
                case 2: //右边界，向左寻找谷底
                {
                    uint8 flag_2=0;
                    while(column-1>0&&row+1<MT9V03X_H-1)    //左下
                    {
                        if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                        {
                            column--;   //左移
                            flag_2=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK) //下黑
                        {
                            row++;      //下移
                            flag_2=1;
                        }
                        if(flag_2==1)
                        {
                            flag_2=0;
                            continue;
                        }
                        break;
                    }
                    break;
                }
                default:    //意外情况：无法判别边界点位置
                {
                    EndPoint.Y=MT9V03X_H/2;     //终点：定为图像的中心点
                    EndPoint.X=MT9V03X_W/2;
                }
            }
            EndPoint.Y=row;     //终点：谷底
            EndPoint.X=column;
            break;
        }
    }
    //补左线右拐入环
    FillingLine('L', StarPoint, EndPoint);
    //第二段补线：
    //寻找补线起点
    StarPoint.Y=row;    //起点：将第一段补线的终点作为起点
    StarPoint.X=column;
    //寻找补线终点
    for(column=MT9V03X_W-2;row-1>0;row--)   //向上扫，补线起点
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            break;
        }
    }
    EndPoint.Y=row;     //终点：右谷顶
    EndPoint.X=column;
    //补左线右拐入环
    FillingLine('L', StarPoint, EndPoint);
    //修正右边界
    StarPoint.X=column;
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

    return 1;
}

/*
 *******************************************************************************************
 ** 函数功能: 第二次识别环岛入口，右侧，补线直行通过
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandOverBegin_R(int *RightLine)
{
    //判断是否完全出环：丢线数符合且右下方不存在黑洞
    if(LostNum_RightLine<50) //丢线数符合
    {
        uint8 flag=0;
        if(BinaryImage[MT9V03X_H-5][MT9V03X_W-5]==IMAGE_BLACK)    //右下角为黑
        {
            for(uint8 row=MT9V03X_H-5,column=MT9V03X_W-5;row-2>0;row--)   //向上扫
            {
                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE&&BinaryImage[row-2][column]==IMAGE_WHITE)    //黑-白-白（避免偶然出现的白点误判）
                {
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)    //白-黑
                        {
                            flag=1; //存在黑洞
                            break;
                        }
                    }
                    break;
                }
            }
        }
        if(flag==0)
        {
            return 0;
        }
    }
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-1,column=MT9V03X_W-1;
    //寻找补线起点
    StarPoint.Y=row;    //起点：右边界不丢线处or右下角
    StarPoint.X=RightLine[row];
    //寻找补线终点
    for(row=MT9V03X_H-1,column=3*(MT9V03X_W/4);row-1>0;row--)    //向上扫，靠右四分之三处
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            uint8 flag_down=0;  //边界点的位置（1：左边界；2：右边界）
            if(BinaryImage[row][column+1]==IMAGE_BLACK)      //右边为黑
            {
                flag_down=1;    //左边界
            }
            else if(BinaryImage[row][column-1]==IMAGE_BLACK) //左边为黑
            {
                flag_down=2;    //右边界
            }
            else    //X坐标没有落在跳变点上，默认为右边界，手动迫近
            {
                flag_down=3;    //出环的时候车身左斜太厉害，若和入环一样找谷底出错概率变大，不如简单处理
            }
            switch(flag_down)
            {
                case 1: //左边界，向右寻找谷底
                {
                    uint8 flag_1=0;
                    while(column+1<MT9V03X_W-1&&row+1<MT9V03X_H-1)
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
                case 2: //右边界，向左寻找谷底
                {
                    uint8 flag_2=0;
                    while(column-1>0&&row+1<MT9V03X_H-1)    //左下
                    {
                        if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                        {
                            column--;   //左移
                            flag_2=1;
                        }
                        if(BinaryImage[row+1][column]==IMAGE_BLACK) //下黑
                        {
                            row++;      //下移
                            flag_2=1;
                        }
                        if(flag_2==1)
                        {
                            flag_2=0;
                            continue;
                        }
                        break;
                    }
                    break;
                }
                default:    //意外情况：无法判别边界点位置
                {
                    EndPoint.Y=MT9V03X_H/2;     //终点：定为图像的中心点
                    EndPoint.X=MT9V03X_W/2;
                }
            }
            EndPoint.Y=row;     //终点：谷底
            EndPoint.X=column;
            break;
        }
    }
    //补右线直行
    FillingLine('R', StarPoint, EndPoint);
    return 1;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛出口，右侧
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛出口且在车身右侧
 ** 作    者: WBN
 ** 注    意：该函数调用时应确保小车已在环岛中
 ********************************************************************************************
 */
uint8 CircleIslandEnd_R(void)
{
    static uint8 flag=0;    //保证补线的连续性，依赖于第一次判断
    if((LostNum_RightLine<90&&fabsf(Bias)<4)||flag==1)  //符合约束条件或处于连续补线
    {
        Point StarPoint,EndPoint;
        //寻找补线起点
        uint8 row=MT9V03X_H-2,column=1,flag_1;
        if(BinaryImage[row][column]==IMAGE_BLACK)   //左下角为黑（可能存在左拐点）
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
            flag_1=1;   //标记找到了右拐点，为下面找终点提供分类依据
        }
        StarPoint.Y=row;    //起点：左拐点or左下角
        StarPoint.X=column;
        //寻找补线终点
        column=MT9V03X_W-2;           //终点X坐标取右边界（由于环岛出环没有像十字回环一样的直角压角问题，所以这里可以简单处理）
        for(;row-1>0;row--) //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
            {
                EndPoint.Y=row;     //终点：上边界点
                EndPoint.X=column;
                break;
            }
        }
        //补左线右转出环
        FillingLine('L', StarPoint, EndPoint);
        flag=1; //连续补线标志
        return 1;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛出口，补线直行，右侧
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 **          InflectionL：左下拐点
 **          InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到目标
 **          1：识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandExit_R(Point InflectionR)
{
    static uint8 flag=0;    //连续补线flag
    if((InflectionR.X!=0&&InflectionR.Y>85)||flag==1)  //符合约束条件or处于连续补线状态
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
            column=2*(MT9V03X_W/3); //为寻找上方黑洞，重置X坐标为右三分之一处
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
 ** 函数功能: 是被环岛中部，右侧
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：没有识别到目标
 **          1：识别到目标
 ** 作    者: WBN
 ** 注    意：上坡路段仍会误判
 ********************************************************************************************
 */
uint8 CircleIslandMid_R(void)
{
    uint8 flag=0;  //环岛中部约束条件
    if(BinaryImage[MT9V03X_H-5][MT9V03X_W-5]==IMAGE_BLACK||BinaryImage[MT9V03X_H-3][MT9V03X_W-3]==IMAGE_BLACK)    //正常情况，右下为黑
    {
        flag=1; //符合约束条件
        //下面的程序防止在Exit处误判：误判了圆环与直角交界处的黑块，上面有一个黑洞，寻找黑洞
        uint8 column=MT9V03X_W-3,row=MT9V03X_H-3,flag_1=0;
        for(;column-1<0;column--)   //将指针移动到底部最左端
        {
            if(BinaryImage[row][column-1]==IMAGE_WHITE)
            {
                break;
            }
        }
        while(column-1>0&&row-1>0)
        {
            if(BinaryImage[row][column-1]==IMAGE_BLACK)    //左黑
            {
                column--;   //左移
                flag_1=1;
            }
            if(BinaryImage[row-1][column]==IMAGE_BLACK)    //上黑
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
        //这里为了防止远处十字环的干扰，和左环岛不同，采用另外一种判断方法
        if(row-5>0)     //手动上移切点Y坐标，方便下面的判断
        {
            row-=5;
        }
        for(;column+1<MT9V03X_W-1;column++) //向右扫，黑洞左切点
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row][column+1]==IMAGE_WHITE)    //谷底右侧为白：误判了Exit处圆环和直道的夹角为黑洞
            {
                flag=0;
                break;
            }
        }
    }
    else    //判断是否满足车子靠左的情况（右边有一个接近正中间的，极小的黑洞）
    {
        uint8 row=MT9V03X_H-1,column=MT9V03X_W-1;
        for(;row-1>0;row--)  //向上扫，右边界
        {
            if(BinaryImage[row][MT9V03X_W-2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W-2]==IMAGE_BLACK)    //白-黑（黑洞下边界）
            {
                uint8 row_low=row;  //记录黑洞下边界
                for(;row-1>0;row--)   //继续向上扫
                {
                    if(BinaryImage[row][MT9V03X_W-2]==IMAGE_BLACK&&BinaryImage[row-1][MT9V03X_W-2]==IMAGE_WHITE)    //黑-白（黑洞上边界）
                    {
                        row=(row+row_low)/2;    //计算出黑洞中点Y坐标
                        if(row>50&&row<70)      //黑洞位于中间位置
                        {
                            for(;column-1>0;column--)   //向左扫，黑洞中点
                            {
                                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column-1]==IMAGE_WHITE)  //黑-白（黑洞左边界）
                                {
                                    if(column>3*(MT9V03X_W/4))  //黑洞左边界位于图像右部
                                    {
                                        flag=1; //符合约束条件
                                    }
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
//        //这里为了防止远处十字环的干扰，和左环岛不同，采用另外一种判断方法
//        for(;column+1<MT9V03X_W-1;column++) //向右扫，黑洞左切点
//        {
//            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row][column+1]==IMAGE_WHITE)    //谷底右侧为白：误判了Exit处圆环和直道的夹角为黑洞
//            {
//                flag=0;
//                break;
//            }
//        }
    }
    //满足约束条件下的进一步判断
    if(flag==1)
    {
        return 1;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别已经进入环岛
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：没有识别到目标
 **          1：识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandInside_R(void)
{
    if(LostNum_RightLine>70)    //右边接近全丢线
    {
        //下面采用经验值随机抽样法
        if(BinaryImage[90][119]==IMAGE_WHITE&&BinaryImage[60][20]==IMAGE_BLACK)
        {
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别右环岛
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：还未完成环岛
 **           1：完成环岛
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandIdentify_R(int *RightLine,Point InflectionR)
{
    static uint8 flag,flag_exit,flag_in,flag_begin,flag_last_begin,flag_last2_begin,flag_end;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(flag_exit==1)        //之前已识别到环岛出口
            {
                if(CircleIslandMid_R()==1)   //识别到环岛中部
                {
                    flag_exit=0;flag=1; //跳转到状态1
                    break;
                }
            }
            if(CircleIslandExit_R(InflectionR)==1)  //识别到环岛Exit作为开启环岛中部检测的条件
            {
                flag_exit=1;
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_R()==1&&flag_in==0) //识别到环岛入口
            {
                StartIntegralAngle_Z(45);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启陀螺仪
            }
            if(flag_in==1)  //之前已经识别到环岛入口
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=2;   //跳转到状态2
                    break;
                }
            }
            break;
        }
        case 2: //此时小车在环中，自主寻迹
        {
            if(flag_in==1)
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=3;   //跳转到状态3
                    break;
                }
            }
            if(flag_in==0)
            {
                StartIntegralAngle_Z(180);  //开启陀螺仪辅助出环
                flag_in=1;                  //避免重复开启陀螺仪
            }
            break;
        }
        case 3: //此时小车已经接近环岛出口，开始判断环岛出口
        {
            if(CircleIslandEnd_R()==1&&flag_end==0)  //第一次检测到环岛出口
            {
                StartIntegralAngle_Z(70);   //开启积分
                flag_end=1;
            }
            if(flag_end==1)  //积分已经开启
            {
                if(icm_angle_z_flag==1) //检测积分状态
                {
                    flag_end=0;flag=4; //跳转到状态4
                    break;
                }
            }
            break;
        }
        case 4: //此时小车已经出环，但是会再次进过环岛入口，需要补线直行
        {
            flag_begin=CircleIslandOverBegin_R(RightLine);     //识别环岛入口补线忽略
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=5;     //跳转到未知状态，状态机作废
                return 1;   //退出状态机
            }
            flag_last2_begin=flag_last_begin;   //保存上上次的状态
            flag_last_begin=flag_begin;         //保存上一次的状态
            break;
        }
        default:break;
    }
    return 0;
}
