/*
 * ImageSpecial.c
  *  赛道上各种特殊元素的识别
  *  该文件的函数只实现识别是否有该元素的出现以及出现的位置，至于该元素出现了多少次以及对应措施，不在该文件范围内
 *  Created on: 2022年1月17日
 *      Author: yue
 */

#include "ImageSpecial.h"
#include <math.h>
#include "zf_gpio.h"            //调试用的LED
#include "Binarization.h"       //二值化之后的图像数组
#include "SEEKFREE_18TFT.h"
#include "Steer.h"
#include <stdlib.h>             //abs函数，fabs在math.h
#include "SEEKFREE_MT9V03X.h"
#include "Motor.h"              //停止电机
#include "PID.h"
#include "LED.h"                //LED宏定义
#include "ICM20602.h"           //陀螺仪积分完成标志变量以及开启积分函数



/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandBegin_L(int *LeftLine,int *RightLine)
{
    if(BinaryImage[115][5]==IMAGE_BLACK)    //防止提前拐入环岛
    {
        return 0;
    }
    //环岛入口在左边
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
 ** 函数功能: 识别环岛入口，左侧，补线直行忽略
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
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
 ** 函数功能: 识别环岛出口，左侧
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛出口且在车身左侧
 ** 作    者: WBN
 ** 注    意：该函数调用时应确保小车已在环岛中
 ********************************************************************************************
 */
uint8 CircleIslandEnd_L(void)
{
    if(LostNum_LeftLine>100)    //防止还未出环岛的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>55&&LostNum_RightLine>55)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            /*在这里将舵机打死，考虑要不要加延时*/
//            systick_delay_ms(STM0,30);  //加一点延时防止撞到内环
            Bias=10;
            systick_delay_ms(STM0,150);
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag1是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：Flag1不成立
 **           1：Flag1成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0)  //判断条件一：是否存在左拐点与右侧直道
    {
        float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
        if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
        {
            for(int row=InflectionL.Y;row-1>0;row--)    //从左拐点开始向上扫
            {
                if(BinaryImage[row][InflectionL.X]==IMAGE_WHITE&&BinaryImage[row-1][InflectionL.X]==IMAGE_BLACK)
                {
                    uint8 row_f=row;
                    for(;row-1>0;row--)
                    {
                        if(BinaryImage[row][InflectionL.X]==IMAGE_BLACK&&BinaryImage[row-1][InflectionL.X]==IMAGE_WHITE)
                        {
                            row=(row+row_f)/2;
                            for(int column=InflectionL.X;column<MT9V03X_W-1;column++)   //向右扫
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
 ** 函数功能: 判断环岛Flag1_1是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：Flag1_1不成立
 **           1：Flag1_1成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_1_L(int *LeftLine,int *RightLine)
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
                                    lcd_showuint8(0, 3, 3);
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
 ** 函数功能: 判断环岛Flag2是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag2不成立
 **           1：Flag2成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_2_L(int *LeftLine,int *RightLine)
{
    if(LostNum_RightLine>55)   //防止误判十字入口
    {
        return 0;
    }
    float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
    if(fabsf(bias_right)<G_LINEBIAS&&LostNum_LeftLine<35)    //右边界为直道且左边丢线小于
    {
        if(BinaryImage[100][10]==IMAGE_BLACK)    //经验位置为黑
        {
            //下面这个for防止在环岛出口时误判为环岛中部
            for(int row=80;row+1<MT9V03X_H-1;row++) //向下扫
            {
                if(LeftLine[row]==0&&LeftLine[row+1]!=0)    //丢线-不丢线
                {
                    return 0;
                }
            }
            for(int row=80;row-1>0;row--)  //向上扫
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
uint8 CircleIsFlag_3_L(void)
{
    if(LostNum_LeftLine>100)    //左边接近全丢线
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
 ** 函数功能: 识别环岛
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：还未完成环岛
 **           1：完成环岛
 ** 作    者: WBN
 ** 注    意：这里只有环岛在小车左边的情况
 ********************************************************************************************
 */
uint8 CircleIslandIdentify_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,num_1,num_2,flag_begin,flag_last_begin,flag_last2_begin;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(CircleIsFlag_1_L(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别环岛出口，进行补线
            {
                if(num_1<100)
                {
                    num_1++;    //识别到flag1的帧数++
                }
            }
            else
            {
                CircleIsFlag_1_1_L(LeftLine, RightLine);
            }
//            else    //没有识别到环岛出口
//            {
//                if(num_2<100)
//                {
//                    num_2++;    //没有识别到flag1的帧数++
//                }
//                else    //超过100帧没有识别到环岛flag1
//                {
//                    num_1=0;
//                    num_2=0;
//                }
//            }
            if(CircleIsFlag_2_L(LeftLine, RightLine)==1)    //识别到环岛中部
            {
                if(num_1>2) //在此之前有识别到环岛flag1
                {
                    num_1=0;
                    num_2=0;
                    flag=1; //跳转到状态1
                    base_speed=150; //降速准备入环
                    break;
                }
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_L(LeftLine, RightLine)==1)
            {
                if(num_1<100)
                {
                    num_1++;    //识别到环岛入口的帧数++
                }
            }
            else
            {
                if(num_2<30)
                {
                    num_2++;    //识别不到环岛入口的帧数++
                }
                else    //超过30帧识别不到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=0; //跳转回到状态0
                    break;
                }
            }
            if(CircleIsFlag_3_L()==1)    //识别已经进入环岛
            {
                if(num_2>2) //在此之前有识别到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=2;
                    break;
                }
            }
            break;
        }
        case 2: //此时小车已经在环岛中，开始判断环岛出口
        {
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            while(CircleIslandEnd_L()==0)  //识别到环岛出口跳出循环
            {
                if(mt9v03x_finish_flag)
                {
                    ImageBinary();                                  //图像二值化
                    GetImagBasic(LeftLine,CentreLine,RightLine);    //基本扫线
                    Bias=DifferentBias(110,60,CentreLine);          //计算偏差，此时在环岛中取特殊前瞻
                    mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
                }
            }
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            flag=3; //跳转到状态3
            break;
        }
        case 3:
        {
            flag_begin=CircleIslandOverBegin_L(LeftLine, RightLine);     //识别环岛入口补线忽略
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=4;
                return 1;   //退出状态机
            }
            flag_last2_begin=flag_last_begin;   //保存上上次的状态
            flag_last_begin=flag_begin; //保存上一次的状态
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
    for(uint8 row=60;row-1>0;row--)          //防止在Flag1处误判
    {
        if(BinaryImage[row][150]==IMAGE_BLACK&&BinaryImage[row-1][150]==IMAGE_WHITE) //黑跳白
        {
            return 0;
        }
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
 ** 函数功能: 识别环岛入口，右侧，补线直行通过
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
uint8 CircleIslandEnd_R(void)
{
    if(LostNum_RightLine>110)    //防止还未出环岛的误判
    {
        return 0;
    }
    if(BinaryImage[115][155]==IMAGE_BLACK)  //防止刚进环岛的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>C_LOSTNUM&&LostNum_RightLine>C_LOSTNUM)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            /*在这里将舵机打死，考虑要不要加延时*/
            systick_delay_ms(STM0,50);   //防止切内环
            Bias=-10;
            systick_delay_ms(STM0,200);
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag1是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：Flag1不成立
 **           1：Flag1成立且在右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionR.Y<60)    //防止还在下坡时的误判
    {
        return 0;
    }
    if(InflectionR.X!=0&&InflectionR.Y!=0)  //判断条件一：是否存在右拐点与左侧直道
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
 ** 函数功能: 判断环岛Flag1_1是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag1_1不成立
 **           1：Flag1_1成立且在右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_1_R(int *LeftLine,int *RightLine)
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
 ** 函数功能: 判断环岛Flag2是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag2不成立
 **           1：Flag2成立且在右侧
 ** 作    者: WBN
 ** 注    意：上坡路段仍会误判
 ********************************************************************************************
 */
uint8 CircleIsFlag_2_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
    if(fabsf(bias_left)<G_LINEBIAS&&LostNum_RightLine<50)    //左边界为直道且右边丢线小于
    {
        if(BinaryImage[100][149]==IMAGE_BLACK)    //经验位置为黑
        {
            //下面这个for防止在环岛出口时误判为环岛中部
            for(int row=80;row+1<MT9V03X_H-1;row++) //向下扫
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
 ** 函数功能: 判断环岛Flag3是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag3不成立
 **           1：Flag3成立且在右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_3_R(void)
{
    if(LostNum_RightLine>100)    //右边接近全丢线
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
 ** 函数功能: 识别环岛
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：还未完成环岛
 **           1：完成环岛
 ** 作    者: WBN
 ** 注    意：这里只有环岛在小车右边的情况
 ********************************************************************************************
 */
uint8 CircleIslandIdentify_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,num_1,num_2,flag_begin,flag_last_begin,flag_last2_begin;
    //使用switch实现简单的状态机机制
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(CircleIsFlag_1_R(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别环岛出口，进行补线
            {
                if(num_1<100)
                {
                    num_1++;    //识别到flag1的帧数++
                }
            }
            else
            {
                CircleIsFlag_1_1_R(LeftLine, RightLine);
            }
            // else    //没有识别到环岛出口
            // {
            //     if(num_2<100)
            //     {
            //         num_2++;    //没有识别到flag1的帧数++
            //     }
            //     else    //超过100帧没有识别到环岛flag1
            //     {
            //         num_1=0;
            //         num_2=0;
            //     }
            // }
            if(CircleIsFlag_2_R(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别到环岛中部
            {
                if(num_1>0) //在此之前有识别到环岛flag1
                {
                    num_1=0;
                    num_2=0;
                    flag=1; //跳转到状态1
                }
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_R(LeftLine, RightLine)==1)
            {
                if(num_1<100)
                {
                    num_1++;    //识别到环岛入口的帧数++
                }
            }
            else
            {
                if(num_2<100)
                {
                    num_2++;    //识别不到环岛入口的帧数++
                }
                else    //超过100帧识别不到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=0; //跳转回到状态0
                }
            }
            if(CircleIsFlag_3_R()==1)    //识别已经进入环岛
            {
                if(num_2>2) //在此之前有识别到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=2;
                }
            }
            break;
        }
        case 2: //此时小车已经在环岛中，开始判断环岛出口
        {
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            while(CircleIslandEnd_R()==0)  //识别到环岛出口跳出循环
            {
                if(mt9v03x_finish_flag)
                {
                    ImageBinary();                                  //图像二值化
                    GetImagBasic(LeftLine,CentreLine,RightLine);    //基本扫线
                    Bias=DifferentBias(110,60,CentreLine);          //计算偏差，此时在环岛中取特殊前瞻
                    mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
                }
            }
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            flag=3;
            break;
        }
        case 3:
        {
            flag_begin=CircleIslandOverBegin_R(LeftLine, RightLine);
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=4;
                return 1;   //退出状态机
            }
            flag_last2_begin=flag_last_begin;   //保存上上次的状态
            flag_last_begin=flag_begin; //保存上一次的状态
            break;
        }
        default:break;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第一个十字回环出口
 ** 参    数: 无
 ** 返 回 值: 0：没有识别到十字回环出口
 **           1：识别到十字回环出口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopEnd_F(void)
{
    for(uint8 row=10;row+1<40;row++)  //向下扫
    {
        if(BinaryImage[row][30]==IMAGE_BLACK&&BinaryImage[row+1][30]==IMAGE_WHITE)
        {
            return 0;
        }
    }
    if(LostNum_LeftLine>110)    //防止还未出环的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>L_LOSTNUM&&LostNum_RightLine>L_LOSTNUM)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            //舵机向右打死并加上一定的延时实现出弯
            Bias=-10;
            systick_delay_ms(STM0,300);
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第二个十字回环出口
 ** 参    数: 无
 ** 返 回 值: 0：没有识别到十字回环出口
 **           1：识别到十字回环出口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopEnd_S(void)
{
    //防止三岔误判，判断是十字回环出口，这个条件的成立是建立在十字回环用路肩挡起来
//    uint8 row_1=0,flag=0;
//    for(uint8 row=65;row-1>0;row--)    //中间向上扫
//    {
//        if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
//        {
//            for(;row-1>0;row--) //继续向上扫
//            {
//                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_BLACK&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_WHITE)
//                {
//                    if(row_1-row<10)    //约束两个黑白跳变点之间的距离
//                    {
//                        flag=1;
//                    }
//                }
//            }
//            break;  //这里的break可以滤去远处的干扰
//        }
//    }
//    if(flag==0)
//    {
//        return 0;
//    }
    //防三岔误判，判断是三岔，这个条件的成立是建立在十字回环用蓝布挡起来
//    uint8 flag_l=0,flag_r=0;
//    for(uint8 row=100,column=MT9V03X_W/2;row-1>20;row--)    //向上扫
//    {
//        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
//        {
//            for(row-=3;column>0;column--)                       //向左扫
//            {
//                if(BinaryImage[row][column]==IMAGE_WHITE)   //向左找到白
//                {
//                    flag_l=1;
//                    break;
//                }
//            }
//            for(column=MT9V03X_W/2;column<MT9V03X_W-1;row++)    //向右扫
//            {
//                if(BinaryImage[row][column]==IMAGE_WHITE)   //向右找到白
//                {
//                    flag_r=1;
//                    break;
//                }
//            }
//            if(flag_l==1&&flag_r==1)
//            {
//                return 0;
//            }
//            break;
//        }
//    }
    for(uint8 row=MT9V03X_H/2;row+1<MT9V03X_H-1;row++)  //防止出三岔急弯误判
    {
        if(BinaryImage[row][155]==IMAGE_BLACK&&BinaryImage[row][155]==IMAGE_WHITE)  //黑-白
        {
            return 0;
        }
    }
    if(LostNum_LeftLine>110)    //防止还未出环的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>L_LOSTNUM&&LostNum_RightLine>L_LOSTNUM)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            //舵机向右打死并加上一定的延时实现出弯
            gpio_set(LED_GREEN, 0);
            base_speed=140;     //降速出环
            Bias=-10;
            systick_delay_ms(STM0,300);
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第一个十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **           1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopBegin_F(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
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
 ** 函数功能: 识别第二个十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **           1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopBegin_S(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0)    //存在左拐点
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
                                        return 0;
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
    float right_bias=0;
    right_bias=Regression_Slope(110, 60, RightLine);    //求右边线斜率
    if(fabsf(right_bias)>0.6)   //防止进环后的误判
    {
        return 0;
    }
    for(uint8 row=MT9V03X_H-20,column=20;row-1>0;row--) //向上扫
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
        {
            uint8 row_f=row;
            for(;row-1>0;row--) //继续向上扫
            {
                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                {
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
                        {
                            for(row=row_f;column+1<MT9V03X_W-1;column++)
                            {
                                if(BinaryImage[row-5][column]==IMAGE_WHITE)
                                {
                                    Point end,start;
                                    end.Y=row_f;
                                    end.X=column;
                                    start.Y=119;
                                    start.X=0;
                                    FillingLine('L', start, end); //补线
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
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断是否已经进入第二个回环
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：还未进入回环
 **           1：已经进入回环
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopIn_S(void)
{
    float left_slope=Regression_Slope(119, 40, RightLine);
    //下面采用经验值随机抽样法
    if(left_slope>1)
    {
        return 1;
    }
    return 0;
}
