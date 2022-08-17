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
#include "Motor.h"

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
    uint8 row=MT9V03X_H-1,column=0,flag_1=0;;
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
    for(row=MT9V03X_H/2,column=MT9V03X_W/5;row-1>0;row--)    //向上扫，靠左五分之一
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            //向右下方寻找谷底
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
    //开始补线
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-1,column=0,flag_1=0;
    //寻找补线起点
    StarPoint.Y=row;    //起点：左边界不丢线处or左下角
    StarPoint.X=LeftLine[row];
    //寻找补线终点
    for(row=MT9V03X_H/2,column=MT9V03X_W/5;row-1>0;row--)    //向上扫，靠左五分之一
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            //向右下方寻找谷底
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
    //限制补线终点
    if(row<MT9V03X_H/4||row>MT9V03X_H/2)
    {
        return 0;
    }
    else if(column>MT9V03X_W/2)
    {
        return 0;
    }
    EndPoint.Y=row;     //终点：谷底
    EndPoint.X=column;
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
uint8 CircleIslandEnd_L(uint8 status)
{
    static uint8 flag=0;    //保证补线的连续性，依赖于第一次判断
    //函数重启
    if(status==1)
    {
        flag=0;
        return 0;
    }

    if(LostNum_LeftLine<90||flag==1)  //符合约束条件或处于连续补线
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
            //向左上方寻找拐点
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
                if(BinaryImage[row-1][column]==IMAGE_BLACK&&flag_1==0) //上黑（由于拐点的左边界线较缓，限制先左移后上移）
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
        else            //不存在右拐点
        {
            row=MT9V03X_H-2;
        }
        StarPoint.Y=row;    //起点：右拐点or右下角
        StarPoint.X=column;
        uint8 start_row=row;
//        //寻找补线终点
//        column=1;           //终点X坐标取左边界（由于环岛出环没有像十字回环一样的直角压角问题，所以这里可以简单处理）
//        for(;row-1>0;row--) //向上扫
//        {
//            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
//            {
//                EndPoint.Y=row;     //终点：上边界点
//                EndPoint.X=column;
//                break;
//            }
//        }
        //寻找补线终点V2
        column=1;
        if(BinaryImage[MT9V03X_H-2][1]==IMAGE_BLACK)   //左下存在黑洞
        {
            row=MT9V03X_H-2;   //坐标重置为左下角
            for(;column+1<MT9V03X_W-1;column++) //向右扫
            {
                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column+1]==IMAGE_WHITE)  //黑-白
                {
                    break;  //修正补线终点的X坐标
                }
            }
        }
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
uint8 CircleIslandExit_L(Point InflectionL,uint8 status)
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
        //防止普通弯道误判
        if(flag==0) //非连续补线状态
        {
            uint8 flag_exit=0;
            //判断拐点上方是否有黑洞
            for(uint8 row=InflectionL.Y,column=InflectionL.X;row-1>0;row--)  //拐点处，向上扫
            {
                if(BinaryImage[row-1][column]==IMAGE_WHITE&&BinaryImage[row][column]==IMAGE_BLACK)  //白-黑（黑洞下边界）
                {
                    for(;row-1>0;row--)
                    {
                        if(BinaryImage[row-1][column]==IMAGE_BLACK&&BinaryImage[row][column]==IMAGE_WHITE)  //黑-白（黑洞上边界）
                        {
                            for(;row-1>0;row--)
                            {
                                if(BinaryImage[row-1][column]==IMAGE_WHITE&&BinaryImage[row][column]==IMAGE_BLACK)  //白-黑（黑洞外）
                                {
                                    //判断黑洞右侧是否是直道
                                    float slope_right=Regression_Slope(InflectionL.Y, row, RightLine);
                                    lcd_showfloat(0, 0, slope_right, 1, 2);
                                    if(fabs(slope_right)<1)
                                    {
                                        flag_exit=1;    //确认环岛
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
            if(flag_exit==0)    //没有确认
            {
                return 0;
            }
        }

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
        //下面的方法只适用于左环岛：前方没有其他圆环干扰，但是在右环岛情况并非如此
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
//        //下面是参考右环岛的方法写的另一种可以避免远处干扰的方法，若华师赛场有其他干扰可以启动这一种，但是要配合测距，因为在Exit处很可能误判
//        for(;column-1>0;column--) //向左扫，黑洞右切点
//        {
//            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row][column-1]==IMAGE_WHITE)    //谷底右侧为白：误判了Exit处圆环和直道的夹角为黑洞
//            {
//                flag=0;
//                break;
//            }
//        }
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
//        //下面的程序防止在Exit处误判：将圆环与直道交界处误判为黑洞，上面还有一个真正的黑洞（其实并没有很必要，因为上面的约束已经很严）
//        if(flag==1) //上面条件成立的情况下作以下约束
//        {
//            for(;row-1>0;row--) //黑洞中点处向上扫，左边界
//            {
//                if(BinaryImage[row][1]==IMAGE_BLACK&&BinaryImage[row-1][1]==IMAGE_WHITE)    //黑-白
//                {
//                    for(;row-1>0;row--) //继续向上扫
//                    {
//                        if(BinaryImage[row][1]==IMAGE_WHITE&&BinaryImage[row-1][1]==IMAGE_BLACK)    //白-黑
//                        {
//                            for(;row-1>0;row--) //继续向上扫
//                            {
//                                if(BinaryImage[row][1]==IMAGE_BLACK&&BinaryImage[row-1][1]==IMAGE_WHITE)    //黑-白
//                                {
//                                    for(;row-1>0;row--) //继续向上扫
//                                    {
//                                        if(BinaryImage[row][1]==IMAGE_WHITE&&BinaryImage[row-1][1]==IMAGE_BLACK)    //白-黑
//                                        {
//                                            flag=0; //不符合约束条件
//                                            break;
//                                        }
//                                    }
//                                    break;
//                                }
//                            }
//                            break;
//                        }
//                    }
//                    break;
//                }
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
            if(flag_exit==1&&encoder_dis_flag==1)        //之前已识别到环岛出口
            {
                if(CircleIslandMid_L()==1)   //识别到环岛中部
                {
                    flag_exit=0;flag=1; //跳转到状态1
                    break;
                }
            }
            if(CircleIslandExit_L(InflectionL,0)==1&&flag_exit==0)  //识别到环岛Exit作为开启环岛中部检测的条件
            {
                EncoderDistance(1, 0.3, 0, 0);  //开启测距：0.3m
                flag_exit=1;
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_L()==1&&flag_in==0) //识别到环岛入口
            {
                StartIntegralAngle_Z(30);   //开启陀螺仪辅助入环
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
            if(flag_in==1&&icm_angle_z_flag==1) //陀螺仪识别到已经入环
            {
                flag_in=0;flag=3;   //跳转到状态3
                break;
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
            if(CircleIslandEnd_L(0)==1&&flag_end==0)  //第一次检测到环岛出口
            {
                StartIntegralAngle_Z(60);   //开启积分
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
                flag_begin=0;flag_last_begin=0;flag_last2_begin=0;flag=0;     //跳转到状态0，等待二次启用
                //重启函数
                CircleIslandExit_L(InflectionL, 1);
                CircleIslandEnd_L(1);
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
    uint8 row=MT9V03X_H-1,column=MT9V03X_W-1,flag_1=0;
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
    for(row=3*(MT9V03X_H/4),column=4*(MT9V03X_W/5);row-1>0;row--)    //向上扫，靠右五分之一处
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            //向左下方寻找谷底
            for(;column-1>0;column--)   //将指针移动到底部最左端
            {
                if(BinaryImage[row][column-1]==IMAGE_BLACK)
                {
                    break;
                }
            }
            while(column-1>0&&row+1<MT9V03X_H-1)  //左下
            {
                if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                {
                    column--;   //左移
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
    //开始补线
    Point StarPoint,EndPoint;
    uint8 row=MT9V03X_H-1,column=MT9V03X_W-1,flag_1=0;
    //寻找补线起点
    StarPoint.Y=row;    //起点：右边界不丢线处or右下角
    StarPoint.X=RightLine[row];
    //寻找补线终点
    for(row=3*(MT9V03X_H/4),column=4*(MT9V03X_W/5);row-1>0;row--)    //向上扫，靠右五分之一处
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
        {
            //向左下方寻找谷底
            for(;column-1>0;column--)   //将指针移动到底部最左端
            {
                if(BinaryImage[row][column-1]==IMAGE_BLACK)
                {
                    break;
                }
            }
            while(column-1>0&&row+1<MT9V03X_H-1)  //左下
            {
                if(BinaryImage[row][column-1]==IMAGE_BLACK) //左黑
                {
                    column--;   //左移
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
    //限制补线终点
    if(row<MT9V03X_H/4||row>MT9V03X_H/2)
    {
        return 0;
    }
    else if(column<MT9V03X_W/2)
    {
        return 0;
    }
    EndPoint.Y=row;     //终点：谷底
    EndPoint.X=column;
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
uint8 CircleIslandEnd_R(uint8 status)
{
    static uint8 flag=0;    //保证补线的连续性，依赖于第一次判断
    //函数重启
    if(status==1)
    {
        flag=0;
        return 0;
    }

    if(LostNum_RightLine<90||flag==1)  //符合约束条件或处于连续补线
    {
        Point StarPoint,EndPoint;
        uint8 row=MT9V03X_H-2,column=1,flag_1=0,flag_2=0;
        //寻找补线起点
        if(BinaryImage[row][column]==IMAGE_BLACK)   //左下角为黑（可能存在左拐点）
        {
            flag_2=1;
        }
        else
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
                if(BinaryImage[row-1][column]==IMAGE_BLACK&&flag_1==0) //上黑（由于拐点的右边界线较缓，限制先右移后上移）
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
        else            //不存在左拐点
        {
            row=MT9V03X_H-2;
        }
        StarPoint.Y=row;    //起点：左拐点or左下角
        StarPoint.X=column;
        uint8 start_row=row;
//        //寻找补线终点
//        column=MT9V03X_W-2;           //终点X坐标取右边界（由于环岛出环没有像十字回环一样的直角压角问题，所以这里可以简单处理）
//        for(;row-1>0;row--) //向上扫
//        {
//            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
//            {
//                EndPoint.Y=row;     //终点：上边界点
//                EndPoint.X=column;
//                break;
//            }
//        }
        //寻找补线终点V2
        column=MT9V03X_W-2;
        if(BinaryImage[MT9V03X_H-2][MT9V03X_W-2]==IMAGE_BLACK)   //右下存在黑洞
        {
            row=MT9V03X_H-2;   //坐标重置为右下角
            for(;column-1>0;column--) //向左扫
            {
                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column-1]==IMAGE_WHITE)  //黑-白
                {
                    break;  //修正补线终点的X坐标
                }
            }
        }
        for(;row-1>0;row--) //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
            {
                break;
            }
        }
        EndPoint.Y=row;     //终点：上边界点
        EndPoint.X=column;
        //补左线右转出环
        FillingLine('L', StarPoint, EndPoint);
        flag=1; //连续补线标志
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
uint8 CircleIslandExit_R(Point InflectionR,uint8 status)
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
        //防止普通弯道误判
        if(flag==0) //非连续补线状态
        {
            uint8 flag_exit=0;
            //判断拐点上方是否有黑洞
            for(uint8 row=InflectionR.Y,column=InflectionR.X;row-1>0;row--)  //拐点处，向上扫
            {
                if(BinaryImage[row-1][column]==IMAGE_WHITE&&BinaryImage[row][column]==IMAGE_BLACK)  //白-黑（黑洞下边界）
                {
                    for(;row-1>0;row--)
                    {
                        if(BinaryImage[row-1][column]==IMAGE_BLACK&&BinaryImage[row][column]==IMAGE_WHITE)  //黑-白（黑洞上边界）
                        {
                            for(;row-1>0;row--)
                            {
                                if(BinaryImage[row-1][column]==IMAGE_WHITE&&BinaryImage[row][column]==IMAGE_BLACK)  //白-黑（黑洞外）
                                {
                                    //判断黑洞右侧是否是直道
                                    float slope_left=Regression_Slope(InflectionR.Y, row, LeftLine);
                                    if(fabs(slope_left)<1)
                                    {
                                        flag_exit=1;    //确认环岛
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
            if(flag_exit==0)    //没有确认
            {
                return 0;
            }
        }
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
            if(flag_exit==1&&encoder_dis_flag==1)   //已识别到环岛出口且测距完成
            {
                if(CircleIslandMid_R()==1)   //识别到环岛中部
                {
                    flag_exit=0;flag=1; //跳转到状态1
                    break;
                }
            }
            if(CircleIslandExit_R(InflectionR,0)==1&&flag_exit==0)  //识别到环岛Exit作为开启环岛中部检测的条件
            {
                EncoderDistance(1, 0.3, 0, 0);  //开启测距：0.3m
                flag_exit=1;
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_R()==1&&flag_in==0) //识别到环岛入口
            {
                StartIntegralAngle_Z(30);   //开启陀螺仪辅助入环
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
            if(flag_in==1&&icm_angle_z_flag==1) //陀螺仪识别到已经入环
            {
                flag_in=0;flag=3;   //跳转到状态3
                break;
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
            if(CircleIslandEnd_R(0)==1&&flag_end==0)  //第一次检测到环岛出口
            {
                StartIntegralAngle_Z(60);   //开启积分
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
                flag_begin=0;flag_last_begin=0;flag_last2_begin=0;flag=0;     //跳转到状态0，等待二次启用
                //重启函数
                CircleIslandExit_R(InflectionR, 1);
                CircleIslandEnd_R(1);
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
