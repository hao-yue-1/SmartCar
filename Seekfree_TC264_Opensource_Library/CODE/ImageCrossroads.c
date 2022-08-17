/*
 * Crossroads.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放十字路口相关的源代码
 */

#include "ImageSpecial.h"
#include "ICM20602.h"           //陀螺仪积分完成标志变量以及开启积分函数
#include "LED.h"                //debug
#include "headfile.h"

#define CROSSROADSIDENTIFYMODE 0    //那种模式找上拐点

/********************************************************************************************
 ** 函数功能: 识别十字路口
 ** 参    数: 左线数组：int *LeftLine 右线数组：int *RightLine//全局变量
 **           左下拐点：Point DownInflectionL
 **           右下拐点：Point DownInflectionR
 ** 返 回 值: 0：不是十字路口
 **           1：正入十字
 **           2：右斜入十字
 **           3：左斜入十字
 ** 作    者: LJF
 ** 注    意：无
 *********************************************************************************************/
uint8 CrossRoadsIdentify(Point DownInflectionL,Point DownInflectionR)
{
    int row=0,cloum=0;//起始行
    Point UpInflectionL,UpInflectionR;//左右上拐点
    UpInflectionL.X=0;UpInflectionL.Y=0;//左上拐点置零
    UpInflectionR.X=0;UpInflectionR.Y=0;//右上拐点置零

    /**********************debug************************/
//    lcd_showint32(0, 0, LostNum_LeftLine, 3);
//    lcd_showint32(0, 1, LostNum_RightLine, 3);
//    lcd_showint32(TFT_X_MAX-50, 0, DownInflectionL.X, 3);
//    lcd_showint32(TFT_X_MAX-50, 1, DownInflectionR.X, 3);
    /***************************************************/

    //左右两边大量丢线，并且左右下拐点都存在,并且中上是白点
    if(LostNum_LeftLine>30 && LostNum_RightLine>30 && DownInflectionR.X!=0 && DownInflectionL.X!=0 && BinaryImage[50][MT9V03X_W/2]==IMAGE_WHITE)
    {
        DownInflectionL.Y-=10;DownInflectionR.Y-=10;//避免拐点噪点
        //搜寻十字上拐点
        GetRightangleUPInflection('L', DownInflectionL, &UpInflectionL, 10, MT9V03X_W-20);
        GetRightangleUPInflection('R', DownInflectionR, &UpInflectionR, 10, 20);
        if(UpInflectionL.Y!=0 && UpInflectionR.Y!=0)
        {
            DownInflectionL.Y+=10;DownInflectionR.Y+=10;//补线恢复点
            FillingLine('L', DownInflectionL, UpInflectionL);
            FillingLine('R', DownInflectionR, UpInflectionR);
            return 1;//正入十字
        }
    }
    //最下面一行丢线，并且左右拐点找不到，入十字中的状态
    else if(LeftLine[MT9V03X_H-10]==0 && RightLine[MT9V03X_H-10]==MT9V03X_W-1 && DownInflectionL.X==0 && DownInflectionR.X==0)
    {
        Point PointL,PointR;//临时的左右下拐点
        PointL.X=10;PointL.Y=MT9V03X_H;//给定一个左下角的点
        PointR.X=MT9V03X_W-10;PointR.Y=MT9V03X_H;//给定一个右下角的点
        //丢失左右下拐点的时候根据边沿去找上拐点
        GetRightangleUPInflection('L', PointL, &UpInflectionL, 10, MT9V03X_W-20);
        GetRightangleUPInflection('R', PointR, &UpInflectionR, 10, 20);
        if(UpInflectionL.Y!=0 && UpInflectionR.Y!=0)
        {
            PointL.X=LeftLine[UpInflectionL.Y-7];PointL.Y=UpInflectionL.Y-7;//寻找正确边线上跟左上拐点一起的点来补线
            PointR.X=RightLine[UpInflectionR.Y-7];PointR.Y=UpInflectionR.Y-7;//寻找正确边线上跟右上拐点一起的点来补线
            FillinLine_V2('L', MT9V03X_H-1, UpInflectionL.Y, UpInflectionL, PointL);
            FillinLine_V2('R', MT9V03X_H-1, UpInflectionR.Y, UpInflectionR, PointR);
            return 1;//正入十字
        }
    }
    //左边丢线超过一半[60,无穷]，右边也存在丢线[10,60]，右拐点存在，并且右拐点上面一段对应的左边丢线，并且右拐点不能在最左边附近
    else if(LostNum_LeftLine>60 && LostNum_RightLine>10 && LostNum_RightLine<60 && DownInflectionR.X!=0 && LeftLine[DownInflectionR.Y-5]==0)
    {
        DownInflectionR.Y-=10;//避免拐点噪点
        //搜寻十字上拐点
        GetRightangleUPInflection('R', DownInflectionR, &UpInflectionR, 10, 20);
        if(UpInflectionR.Y!=0)
        {
            DownInflectionR.Y+=10;//恢复拐点行数用于补线
            FillingLine('R', DownInflectionR, UpInflectionR);
            //右上拐点的赛道另外一个点
            for(uint8 column=UpInflectionR.X-3;column>3;column--)
            {
                if(BinaryImage[UpInflectionR.Y][column]==IMAGE_WHITE && BinaryImage[UpInflectionR.Y][column-1]==IMAGE_BLACK)
                {
                    UpInflectionL.Y=UpInflectionR.Y;UpInflectionL.X=column-1;
                    break;
                }
            }
            DownInflectionL.Y=MT9V03X_H-5;DownInflectionL.X=5;
//            FillingLine('L', DownInflectionL, UpInflectionL);
            return 2;//向右斜入十字
        }
    }
    //右边丢线超过一半，左边也存在丢线，左拐点存在，并且右拐点上面一段对应的左边丢线
    else if(LostNum_RightLine>60 && LostNum_LeftLine>10 && LostNum_LeftLine<60 && DownInflectionL.X!=0 && RightLine[DownInflectionL.Y-5]==MT9V03X_W-1)
    {
        DownInflectionL.Y-=10;//避免拐点噪点
        //搜寻十字上拐点
        GetRightangleUPInflection('L', DownInflectionL, &UpInflectionL, 10, MT9V03X_W-20);
        if(UpInflectionL.Y!=0)
        {
            DownInflectionL.Y+=10;//恢复拐点行数用于补线
            FillingLine('L', DownInflectionL, UpInflectionL);
            //右上拐点的赛道另外一个点
            for(uint8 column=UpInflectionL.X+3;column<MT9V03X_W-3;column++)
            {
                if(BinaryImage[UpInflectionL.Y][column]==IMAGE_WHITE && BinaryImage[UpInflectionL.Y][column+1]==IMAGE_BLACK)
                {
                    UpInflectionR.Y=UpInflectionL.Y;UpInflectionR.X=column-1;
                    break;
                }
            }
            DownInflectionR.Y=MT9V03X_H-5;DownInflectionR.X=MT9V03X_W-5;
//            FillingLine('R', DownInflectionR, UpInflectionR);
            return 3;//向左斜入十字
        }
    }
    return 0;
}
/********************************************************************************************
 ** 函数功能: 十字的状态机转移
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值: 0：十字还未结束
 **           1：十字已结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 CrossRoadsStatusIdentify(Point DownInflectionL,Point DownInflectionR)
{
    //十字状态变量，用来看状态是否跳转
    static uint8 StatusChange;
    uint8 NowFlag=0;//十字识别的暂存标志变量

    NowFlag=CrossRoadsIdentify(DownInflectionL, DownInflectionR);
    switch(StatusChange)
    {
        //根据ICM为主判断是否完成十字元素
        case 0:
        {
//            gpio_set(LED_BLUE, 0);
            //如果是正入状态
            if(NowFlag==1)
            {
//                gpio_set(LED_BLUE, 1);
                StartIntegralAngle_Z(270);//开启陀螺仪作为出状态标志
                StatusChange=1;//进入陀螺仪积分出环状态
            }
            break;
        }
        //结束状态
        case 1:
        {
//            gpio_set(LED_GREEN, 0);
            //陀螺仪积分达到出十字环状态,证明已经直着进了十字中间，可以不用再补线也能出去了
            if(icm_angle_z_flag==1)
            {
//                gpio_set(LED_GREEN, 1);
                return 1;
            }
            break;
        }
    }
    return 0;
}
