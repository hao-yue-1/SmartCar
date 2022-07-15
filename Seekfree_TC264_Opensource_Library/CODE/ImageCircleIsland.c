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
 ********************************************************************************************
 */
uint8 CircleIslandEnd_L(Point InflectionL,Point InflectionR)
{
    static uint8 flag=0;    //保证补线的连续性，依赖于第一次判断
    if((LostNum_LeftLine<90&&fabsf(Bias)<4)||flag==1)  //符合约束条件或处于连续补线
    {
        Point StarPoint,EndPoint;
        //寻找补线起点
        uint8 row=MT9V03X_H-2,column=MT9V03X_W-2,flag_1;
        if(BinaryImage[row][column]==IMAGE_BLACK)   //右下角为黑（可能存在右拐点）
        {
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
    uint8 flag=0;  //环岛中部约束条件
    if(BinaryImage[MT9V03X_H-5][4]==IMAGE_BLACK||BinaryImage[MT9V03X_H-3][2]==IMAGE_BLACK)    //正常情况，左下为黑
    {
        flag=1; //符合约束条件
        //下面的程序防止在Exit处误判：误判了圆环与直角交界处的黑块，上面有一个黑洞
        uint8 column=2,row=MT9V03X_H-3,flag_1=0;    //寻找黑洞所在X坐标
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
uint8 CircleIslandIdentify_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,flag_exit,flag_in,flag_begin,flag_last_begin,flag_last2_begin,flag_end;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(flag_exit==1)        //之前已识别到环岛出口
            {
                if(CircleIslandMid_L(LeftLine, RightLine)==1)   //识别到环岛中部
                {
                    flag_exit=0;flag=1; //跳转到状态1
                    break;
                }
                CircleIslandOverExit_L(LeftLine, RightLine);    //第二次识别环岛出口，补线直行
            }
            else if(flag_exit==0)    //还未识别到环岛出口
            {
                if(CircleIslandExit_L(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别到环岛出口部分路段
                {
                    flag_exit=1;
                }
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
            if(CircleIslandEnd_L(InflectionL, InflectionR)==1&&flag_end==0)  //第一次检测到环岛出口
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
            flag_begin=CircleIslandOverBegin_L(LeftLine, RightLine);     //识别环岛入口补线忽略
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
 ** 注    意：这里和左环岛的区别在于我们使用的扫线方式是默认向左寻找中线，所以左环岛即使不做处理也会
 ********************************************************************************************
 */
uint8 CircleIslandBegin_R()
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
    for(row=MT9V03X_H-1,column=3*(MT9V03X_W/4);row-1>0;row--)    //向上扫，靠右三分之一处
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
    //特殊情况求Bias
    Bias=DifferentBias(start_row,row,CentreLine);//无特殊处理时的偏差计算
    CircleIsland_flag=1;
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
    static uint8 flag=1,num_1,num_2,flag_begin,flag_last_begin,flag_last2_begin,flag_end,flag_in;
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
            if(CircleIslandBegin_R()==1) //识别到环岛入口
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
                gpio_set(LED_WHITE, 0);
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

