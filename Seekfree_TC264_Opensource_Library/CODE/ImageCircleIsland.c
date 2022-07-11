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
uint8 CircleIslandBegin_L(int *LeftLine,int *RightLine)
{
    static uint8 flag=0;  //连续补线flag，依赖于第一次补线判断

    if(flag==1) //已经补过线，避免不可预知干扰打断补线
    {
        for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查左边界线
        {
            if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //该行丢线而下一行不丢线
            {
                Point StarPoint,EndPoint;   //定义补线的起点和终点
                EndPoint.Y=row;             //终点赋值
                EndPoint.X=LeftLine[row];
                StarPoint.Y=MT9V03X_H-1;    //起点赋值
                StarPoint.X=MT9V03X_W-1;
                FillingLine('R',StarPoint,EndPoint);    //补线
                return 1;
            }
        }
        //若在上面的for循环中没有发现，下面实施补救
    }
    else if(BinaryImage[115][5]!=IMAGE_BLACK)   //补线约束条件
    {
        if(LostNum_LeftLine>C_LOSTLINE)   //左边丢线：环岛入口在左边
        {
            for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查左边界线
            {
                if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //该行丢线而下一行不丢线
                {
                    //下面这个防止进入环岛后误判
                    for(int column=0;column+1<MT9V03X_W-1;column++) //向右扫
                    {
                        if(BinaryImage[10][column]!=BinaryImage[10][column+1])
                        {
                            Point StarPoint,EndPoint;   //定义补线的起点和终点
                            EndPoint.Y=row;             //终点赋值
                            EndPoint.X=LeftLine[row];
                            StarPoint.Y=MT9V03X_H-1;    //起点赋值
                            StarPoint.X=MT9V03X_W-1;
                            FillingLine('R',StarPoint,EndPoint);    //补线
                            flag=1; //连续补线标记
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
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
uint8 CircleIslandOverBegin_L(int *LeftLine,int *RightLine)
{
    //环岛入口在左边
    for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查左边界线
    {
        if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //该行丢线而下一行不丢线
        {
            //下面这个防止进入环岛后误判
            for(int column=0;column+1<MT9V03X_W-1;column++) //向右扫
            {
                if(BinaryImage[10][column]!=BinaryImage[10][column+1])
                {
                    if(row-10>0)
                    {
                        row-=10;
                    }
                    Point StarPoint,EndPoint;   //定义补线的起点和终点
                    EndPoint.Y=row;             //终点赋值
                    EndPoint.X=LeftLine[row];
                    StarPoint.Y=MT9V03X_H-1;    //起点赋值
                    StarPoint.X=0;
                    FillingLine('L',StarPoint,EndPoint);    //补线
                    return 1;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛结束标志出口，左侧
 ** 参    数: InflectionL：左下拐点
 **          InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **          1：识别到环岛出口且在车身左侧
 ** 作    者: WBN
 ** 注    意：该函数调用时应确保小车已在环岛中
 ********************************************************************************************
 */
uint8 CircleIslandEnd_L(Point InflectionL,Point InflectionR)
{
    static uint8 flag=0;
    if(flag==1) //已经识别到环岛出口，跳过各种判断，保证补线的连续性
    {
        if(InflectionR.X>MT9V03X_W/2&&InflectionR.Y>MT9V03X_H/2)    //右拐点真实存在
        {
            //沿着右边界线补线
            for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint;
                    StartPoint.X=RightLine[0];    //起点为最底行的右边界点
                    StartPoint.Y=0;
                    FillinLine_V2('R', InflectionR.Y, row, StartPoint, InflectionR);
                    break;
                }
            }
        }
        else    //不存在右拐点
        {
            //图像右下角补线到赛道中间顶部
            for(uint8 row=MT9V03X_H-20;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint,EndPoint;
                    StartPoint.Y=MT9V03X_H-1;
                    StartPoint.X=MT9V03X_W-1;
                    EndPoint.Y=row;
                    EndPoint.X=MT9V03X_W/2;
                    FillingLine('R',StartPoint,EndPoint);
                    break;
                }
            }
        }
        return 1;
    }
    if(LostNum_LeftLine>55&&LostNum_RightLine>55&&fabsf(Bias)<1.5)  //约束条件，识别到环岛出口
    {
        /*校赛过后使用补线配合陀螺仪的方法出环*/
        if(InflectionR.X>MT9V03X_W/2&&InflectionR.Y>MT9V03X_H/2)    //右拐点真实存在
        {
            //沿着右边界线补线
            for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint;
                    StartPoint.X=RightLine[0];    //起点为最底行的右边界点
                    StartPoint.Y=0;
                    FillinLine_V2('R', InflectionR.Y, row, StartPoint, InflectionR);
                    break;
                }
            }
        }
        else    //不存在右拐点
        {
            //图像右下角补线到赛道中间顶部
            for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint,EndPoint;
                    StartPoint.Y=MT9V03X_H-1;
                    StartPoint.X=MT9V03X_W-1;
                    EndPoint.Y=row;
                    EndPoint.X=MT9V03X_W/2;
                    FillingLine('R',StartPoint,EndPoint);
                    break;
                }
            }
        }
        flag=1; //已经识别到环岛出口flag
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
uint8 CircleIslandExit_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0&&InflectionL.Y>85)  //判断条件一：是否存在左拐点与右侧直道，且车子接近环岛
    {
        float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
        if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
        {
            for(uint8 row=InflectionL.Y;row-1>0;row--)    //从左拐点开始向上扫
            {
                if(BinaryImage[row][InflectionL.X]==IMAGE_WHITE&&BinaryImage[row-1][InflectionL.X]==IMAGE_BLACK)
                {
                    uint8 row_f=row;
                    for(;row-1>0;row--)
                    {
                        if(BinaryImage[row][InflectionL.X]==IMAGE_BLACK&&BinaryImage[row-1][InflectionL.X]==IMAGE_WHITE)
                        {
                            row=(row+row_f)/2;
                            for(uint8 column=InflectionL.X;column<MT9V03X_W-1;column++)   //向右扫
                            {
                                if(BinaryImage[row-1][column]==IMAGE_WHITE)
                                {
                                    /*补线操作*/
                                    Point end;
                                    end.Y=row-1;
                                    end.X=column;
                                    FillingLine('L', InflectionL, end);   //补线
                                    return 1;
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
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 第二次识别环岛出口，左侧，补线直行
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：没有识别到目标
 **          1：识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandOverExit_L(int *LeftLine,int *RightLine)
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
 ** 函数功能: 识别环岛中部，左侧
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：识别到目标
 **          1：没有识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandMid_L(int *LeftLine,int *RightLine)
{
    /*两种情况：1.正常的左下角是黑色，而在进入中部前绝对不会有，所以这个单独作为一个条件可行
     *          2.当车子在很靠左的地方时，左下角是白色的，而且和未进入中部的图像很相似；这里的唯一的区别在于右边黑洞部分的高度不同
     *            通过识别黑白跳变点来确定黑洞高度，从而确定是哪种情况*/

    if(BinaryImage[MT9V03X_H-1][0]==IMAGE_BLACK)    //经验位置为黑
    {
        //下面这个for防止在环岛出口时误判为环岛中部
        for(uint8 row=MT9V03X_H/2;row+1<MT9V03X_H-1;row++) //向下扫
        {
            if(LeftLine[row]==0&&LeftLine[row+1]!=0)    //丢线-不丢线
            {
                return 0;
            }
        }
        //判断环岛中部的依据
        for(uint8 row=60;row-1>0;row--)  //向上扫
        {
            if(LeftLine[row]!=0&&LeftLine[row-1]==0)    //不丢线-丢线
            {
                for(;row-1>0;row--)    //继续向上扫
                {
                    if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //丢线-不丢线
                    {
                        return 1;
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
uint8 CircleIslandIdentify_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,flag_exit,flag_in,flag_begin,flag_last_begin,flag_last2_begin,flag_end;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(flag_exit==0)    //还未识别到环岛出口
            {
                if(CircleIslandExit_L(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别到环岛出口部分路段
                {
                    flag_exit=1;
                }
            }
            if(flag_exit==1)   //之前已识别到环岛出口
            {
                if(CircleIslandMid_L(LeftLine, RightLine)==1)   //识别到环岛中部
                {
                    flag_exit=0;flag=1; //跳转到状态1
                    gpio_set(LED_GREEN, 0);
                    break;
                }
                CircleIslandOverExit_L(LeftLine, RightLine);    //第二次识别环岛出口，补线直行
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_L(LeftLine, RightLine)==1&&flag_in==0) //识别到环岛入口
            {
                StartIntegralAngle_Z(20);   //开启陀螺仪辅助入环
                flag_in=1;                  //避免重复开启陀螺仪
                gpio_set(LED_WHITE, 0);
            }
            if(flag_in==1)  //之前已经识别到环岛入口
            {
                if(icm_angle_z_flag==1) //陀螺仪识别到已经入环
                {
                    flag_in=0;flag=2;   //跳转到状态2
                    gpio_set(LED_YELLOW,0);
                    break;
                }
            }
            break;
        }
        case 2: //此时小车已经在环岛中，开始判断环岛出口
        {
            if(CircleIslandEnd_L(InflectionL, InflectionR)==1&&flag_end==0)  //第一次检测到环岛出口
            {
                StartIntegralAngle_Z(70);   //开启积分
                flag_end=1;
            }
            if(flag_end==1)  //积分已经开启
            {
                if(icm_angle_z_flag==1) //检测积分状态
                {
                    flag_end=0;flag=3; //跳转到状态3
                    break;
                }
            }
            break;
        }
        case 3: //此时小车已经出环，但是会再次进过环岛入口，需要补线直行
        {
            flag_begin=CircleIslandOverBegin_L(LeftLine, RightLine);     //识别环岛入口补线忽略
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=4;     //跳转到未知状态，状态机作废
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
uint8 CircleIslandBegin_R(int *LeftLine,int *RightLine)
{
    if(BinaryImage[115][154]==IMAGE_BLACK)    //防止提前拐入环岛
    {
        return 0;
    }
    //环岛入口在右边
    if(LostNum_RightLine>C_LOSTLINE)   //右边丢线：环岛入口在右边
    {
        for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查右边界线
        {
            if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)    //该行丢线而下一行不丢线
            {
                //下面这个防止进入环岛后误判
                for(int column=MT9V03X_W-1;column-1>0;column--) //向左扫
                {
                    if(BinaryImage[30][column]!=BinaryImage[30][column-1])
                    {
                        Point StarPoint,EndPoint;   //定义补线的起点和终点
                        EndPoint.Y=row;             //终点赋值
                        EndPoint.X=RightLine[row];
                        StarPoint.Y=MT9V03X_H-1;    //起点赋值
                        StarPoint.X=0;
                        FillingLine('L',StarPoint,EndPoint);    //补线
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
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
uint8 CircleIslandOverBegin_R(int *LeftLine,int *RightLine)
{
    //环岛入口在右边
    for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查右边界线
    {
        if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)    //该行丢线而下一行不丢线
        {
            //下面这个防止进入环岛后误判
            for(int column=MT9V03X_W-1;column-1>0;column--) //向左扫
            {
                if(BinaryImage[30][column]!=BinaryImage[30][column-1])
                {
                    if(row-10>0)
                    {
                        row-=10;
                    }
                    Point StarPoint,EndPoint;   //定义补线的起点和终点
                    EndPoint.Y=row;             //终点赋值
                    EndPoint.X=RightLine[row];
                    StarPoint.Y=MT9V03X_H-1;    //起点赋值
                    StarPoint.X=MT9V03X_W-1;
                    FillingLine('R',StarPoint,EndPoint);    //补线
                    return 1;
                }
            }
        }
    }
    return 0;
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
uint8 CircleIslandEnd_R(Point InflectionL,Point InflectionR)
{
    static uint8 flag=0;
    if(flag==1) //已经识别到环岛出口，跳过各种判断，保证补线的连续性
    {
        if(InflectionL.X>MT9V03X_W/2&&InflectionL.Y<MT9V03X_H/2)    //左拐点真实存在
        {
            //沿着左边界线补线
            for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint;
                    StartPoint.X=LeftLine[0];    //起点为最底行的左边界点
                    StartPoint.Y=0;
                    FillinLine_V2('L', InflectionL.Y, row, StartPoint, InflectionL);
                    break;
                }
            }
        }
        else    //不存在左拐点
        {
            //图像右下角补线到赛道中间顶部
            for(uint8 row=MT9V03X_H-20;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint,EndPoint;
                    StartPoint.Y=MT9V03X_H-1;
                    StartPoint.X=0;
                    EndPoint.Y=row;
                    EndPoint.X=MT9V03X_W/2;
                    FillingLine('L',StartPoint,EndPoint);
                    break;
                }
            }
        }
        return 1;
    }
    if(LostNum_LeftLine>55&&LostNum_RightLine>55&&fabsf(Bias)<1.5)  //约束条件，识别到环岛出口
    {
        /*校赛过后使用补线配合陀螺仪的方法出环*/
        if(InflectionL.X>MT9V03X_W/2&&InflectionL.Y>MT9V03X_H/2)    //左拐点真实存在
        {
            //沿着左边界线补线
            for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint;
                    StartPoint.X=LeftLine[0];    //起点为最底行的左边界点
                    StartPoint.Y=0;
                    FillinLine_V2('L', InflectionL.Y, row, StartPoint, InflectionL);
                    break;
                }
            }
        }
        else    //不存在左拐点
        {
            //图像左下角补线到赛道中间顶部
            for(uint8 row=MT9V03X_H/2;row-1>0;row--)  //向上扫
            {
                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
                {
                    //补线
                    Point StartPoint,EndPoint;
                    StartPoint.Y=MT9V03X_H-1;
                    StartPoint.X=0;
                    EndPoint.Y=row;
                    EndPoint.X=MT9V03X_W/2;
                    FillingLine('L',StartPoint,EndPoint);
                    break;
                }
            }
        }
        flag=1; //已经识别到环岛出口flag
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
uint8 CircleIslandExit_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionR.X!=0&&InflectionR.Y!=0&&InflectionR.Y>85)  //判断条件一：是否存在右拐点与左侧直道，，且车子接近环岛
    {
        float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
        if(fabsf(bias_left)<G_LINEBIAS)    //左边界为直道
        {
            for(int row=InflectionR.Y;row-1>0;row--)    //从右拐点开始向上扫
            {
                if(BinaryImage[row][InflectionR.X]==IMAGE_WHITE&&BinaryImage[row-1][InflectionR.X]==IMAGE_BLACK)
                {
                    uint8 row_f=row;
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][InflectionR.X]==IMAGE_BLACK&&BinaryImage[row-1][InflectionR.X]==IMAGE_WHITE)
                        {
                            row=(row_f+row)/2;
                            for(uint8 column=InflectionR.X;column>0;column--)   //向左扫
                            {
                                if(BinaryImage[row][column]==IMAGE_WHITE)
                                {
                                    /*补线操作*/
                                    Point end;
                                    end.Y=row;
                                    end.X=column;
                                    FillingLine('R', InflectionR, end);   //补线
                                    return 1;
                                }
                            }
                        }
                    }
                    return 0;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 第二次识别环岛出口，补线直行，右侧
 ** 参    数: LeftLine：左线数组
 **          RightLine：右线数组
 ** 返 回 值: 0：没有识别到目标
 **          1：识别到目标
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandOverExit_R(int *LeftLine,int *RightLine)
{
    float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
    if(fabsf(bias_left)<G_LINEBIAS)    //左边界为直道
    {
        for(uint8 row=MT9V03X_H-20,column=MT9V03X_W-20;row-1>0;row--)    //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
            {
                uint8 row_f=row;
                for(;row-1>0;row--) //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                    {
                        row=(row+row_f)/2;
                        for(;column>0;column--)   //向左扫
                        {
                            if(BinaryImage[row][column]==IMAGE_WHITE)
                            {
                                /*补线操作*/
                                Point start,end;
                                end.Y=row-1;
                                end.X=column;
                                start.Y=MT9V03X_H-1;    //右下
                                start.X=MT9V03X_W-1;
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
uint8 CircleIslandMid_R(int *LeftLine,int *RightLine)
{
    if(LostNum_RightLine<35)    //左边界为直道且右边丢线小于
    {
        if(BinaryImage[100][149]==IMAGE_BLACK)    //经验位置为黑
        {
//            for(int cloum=0;cloum<MT9V03X_W-1;cloum++)
//            {
//                lcd_drawpoint(cloum,60,PURPLE);
//            }
            //下面这个for防止在环岛出口时误判为环岛中部
            for(int row=60;row+1<MT9V03X_H-1;row++) //向下扫
            {
                if(RightLine[row]==MT9V03X_W-1&&RightLine[row+1]!=MT9V03X_W-1)    //丢线-不丢线
                {
                    return 0;
                }
            }
            for(int row=80;row-1>0;row--)  //向上扫
            {
                if(RightLine[row]!=MT9V03X_W-1&&RightLine[row-1]==MT9V03X_W-1)    //不丢线-丢线
                {
                    for(;row-1>0;row--)    //继续向上扫
                    {
                        if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)    //丢线-不丢线
                        {
                            return 1;
                        }
                    }
                    break;
                }
            }
        }
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
uint8 CircleIslandIdentify_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,num_1,num_2,flag_begin,flag_last_begin,flag_last2_begin,flag_end,flag_in;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(CircleIslandExit_R(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别环岛出口，进行补线
            {
                if(num_1<100)   //限幅
                {
                    num_1++;    //识别到flag1的帧数++
                }
            }
            if(CircleIslandMid_R(LeftLine, RightLine)==1)    //识别到环岛中部
            {
                if(num_1>0) //在此之前有识别到环岛出口
                {
                    num_1=0;flag=1; //跳转到状态1
                }
            }
            if(num_1>0)   //没有识别到出口，应对刚压过出口的情况，也需要补线，优先级最低
            {
                CircleIslandOverExit_R(LeftLine, RightLine);    //第二次识别环岛出口，补线直行
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_R(LeftLine, RightLine)==1) //识别到环岛入口
            {
                if(num_1<100)   //限幅
                {
                    num_1++;    //识别到环岛入口的帧数++
                }
                if(flag_in==0)  //避免重复开启陀螺仪
                {
                    StartIntegralAngle_Z(20);   //开启陀螺仪辅助入环
                    flag_in=1;
                }
            }
            if(icm_angle_z_flag==1) //陀螺仪判断入环
            {
                num_1=0;num_2=0;flag=2; //跳转到状态2
                break;
            }
            break;
        }
        case 2: //此时小车已经在环岛中，开始判断环岛出口
        {
            if(CircleIslandEnd_R(InflectionL, InflectionR)==1&&flag_end==0)  //第一次检测到环岛出口
            {
                StartIntegralAngle_Z(70);   //开启积分
                flag_end=1;
            }
            if(flag_end==1)  //积分已经开启
            {
                if(icm_angle_z_flag==1) //检测积分状态
                {
                   flag_end=0;flag=3; //跳转到状态3
                   break;
                }
            }
            break;
        }
        case 3: //此时小车已经出环，但是会再次进过环岛入口，需要补线直行
        {
            flag_begin=CircleIslandOverBegin_R(LeftLine, RightLine);
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=4;
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

