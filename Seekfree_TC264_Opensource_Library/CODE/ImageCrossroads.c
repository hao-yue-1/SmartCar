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

/*********************************************************************************
 ** 函数功能: 根据左右下拐点搜寻出十字路口的左右上拐点
 ** 参    数: 左右线数组已经是全局变量
 **           Point InflectionL: 左边拐点
 **           Point InflectionR: 右边拐点
 **           Point *UpInflectionC: 左边上拐点
 **           Point *UpInflectionC: 右边上拐点
 ** 返 回 值: 无
 ** 说    明: 此函数仅仅是正入十字时的一个操作函数，不是识别函数
 ** 作    者: LJF
 **********************************************************************************/
void GetCrossRoadsUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR)
{
    int row=0,cloum=0;//起始行,列
    UpInflectionL->X=0;UpInflectionL->Y=0;//左上拐点置零
    UpInflectionR->X=0;UpInflectionR->Y=0;//右上拐点置零

    //从左下拐点往上找，找到白跳黑
    for(row=DownInflectionL.Y-15;row>1;row--)
    {
        /******************debug:把从下往上的找点轨迹画出来******************/
//        lcd_drawpoint(DownInflectionL.X-3, row, YELLOW);
        /******************************************************************/
        //从下面往上面找白的时候列数主动往左右黑色区域偏一点,防止因为找拐点函数找的拐点很怪使得补线补错
        if(BinaryImage[row][DownInflectionL.X-3]==IMAGE_WHITE && BinaryImage[row-1][DownInflectionL.X-3]==IMAGE_BLACK)  //白点->黑点
        {
            //从白黑跳变点上几行再找黑跳白，得到很好的拐点去补线
            //多往右边扫三十列，别勉稍微有一点斜但是左右拐点又存在
            for(cloum=DownInflectionL.X;cloum<MT9V03X_W/2+30;cloum++)
            {
                /******************debug:把从左往右的找点轨迹画出来******************/
//                lcd_drawpoint(cloum, row, YELLOW);
                /******************************************************************/
                if(BinaryImage[row-3][cloum]==IMAGE_BLACK && BinaryImage[row-3][cloum+1]==IMAGE_WHITE)  //黑点->白点
                {
                    //记录上拐点
                    UpInflectionL->Y=row-1;UpInflectionL->X=cloum;
                    break;
                }
            }
            break;//记录完之后就退出循环
        }
    }

    for(row=DownInflectionR.Y-15;row>1;row--)
    {
        /******************debug:把从下往上的找点轨迹画出来******************/
//        lcd_drawpoint(DownInflectionR.X+3, row, YELLOW);
        /******************************************************************/
        //从下面往上面找白的时候列数主动往左右黑色区域偏一点,防止因为找拐点函数找的拐点很怪使得补线补错
        if(BinaryImage[row][DownInflectionR.X+3]==IMAGE_WHITE && BinaryImage[row-1][DownInflectionR.X+3]==IMAGE_BLACK)  //由白到黑跳变
        {
            //从白黑跳变点上几行再找黑跳白，得到很好的拐点去补线
            //多往左边扫三十列，别勉稍微有一点斜但是左右拐点又存在
            for(cloum=DownInflectionR.X;cloum>MT9V03X_W/2-30;cloum--)
            {
                /******************debug:把从左往右的找点轨迹画出来******************/
//                lcd_drawpoint(cloum, row, YELLOW);
                /******************************************************************/
                if(BinaryImage[row-3][cloum]==IMAGE_BLACK && BinaryImage[row-3][cloum-1]==IMAGE_WHITE)  //黑点->白点
                {
                    //记录上拐点
                    UpInflectionR->Y=row-1;UpInflectionR->X=cloum;
                    break;
                }
            }
            break;//记录完之后就退出循环
        }
    }
}
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
        GetCrossRoadsUpInflection(DownInflectionL, DownInflectionR, &UpInflectionL, &UpInflectionR);
        if(UpInflectionL.Y!=0 && UpInflectionR.Y!=0)
        {
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
        GetCrossRoadsUpInflection(PointL, PointR, &UpInflectionL, &UpInflectionR);
        if(UpInflectionL.Y!=0 && UpInflectionR.Y!=0)
        {
            PointL.X=LeftLine[UpInflectionL.Y-7];PointL.Y=UpInflectionL.Y-7;//寻找正确边线上跟左上拐点一起的点来补线
            PointR.X=RightLine[UpInflectionR.Y-7];PointR.Y=UpInflectionR.Y-7;//寻找正确边线上跟右上拐点一起的点来补线
            FillinLine_V2('L', MT9V03X_H, UpInflectionL.Y, UpInflectionL, PointL);
            FillinLine_V2('R', MT9V03X_H, UpInflectionR.Y, UpInflectionR, PointR);
            return 1;//正入十字
        }
    }
    //左边丢线超过一半[60,无穷]，右边也存在丢线[10,60]，右拐点存在，并且右拐点上面一段对应的左边丢线，并且右拐点不能在最左边附近
    else if(LostNum_LeftLine>60 && LostNum_RightLine>10 && LostNum_RightLine<60 && DownInflectionR.X!=0 && LeftLine[DownInflectionR.Y-5]==0)
    {
        //直接右下拐点往上冲找到黑色边缘
        for(row=DownInflectionR.Y-5;row>1;row--)
        {
            /******************debug:把从下往上的找点轨迹画出来******************/
//            lcd_drawpoint(DownInflectionR.X, row, YELLOW);
            /******************************************************************/
            if(BinaryImage[row][DownInflectionR.X]==IMAGE_WHITE && BinaryImage[row-1][DownInflectionR.X]==IMAGE_BLACK)  //由白到黑跳变
            {
                //注意此处不能说遍历到一半就停下来了，因为斜入的时候上拐点本来就比较中间
                for(cloum=DownInflectionR.X;cloum>30;cloum--)
                {
                    /******************debug:把从右往左的找点轨迹画出来******************/
//                    lcd_drawpoint(cloum, row, YELLOW);
                    /******************************************************************/
                    if(BinaryImage[row-3][cloum]==IMAGE_BLACK && BinaryImage[row-3][cloum-1]==IMAGE_WHITE)  //黑点->白点
                    {
                        //记录上拐点
                        UpInflectionR.Y=row-1;UpInflectionR.X=cloum;
                        FillingLine('R', DownInflectionR, UpInflectionR);
                        return 2;//向右斜入十字
                    }
                }
            }
        }
    }
    //右边丢线超过一半，左边也存在丢线，左拐点存在，并且右拐点上面一段对应的左边丢线
    else if(LostNum_RightLine>60 && LostNum_LeftLine>10 && LostNum_LeftLine<60 && DownInflectionL.X!=0 && RightLine[DownInflectionL.Y-5]==MT9V03X_W-1)
    {
        for(row=DownInflectionL.Y-5;row>1;row--)//初始行就在之前的基础上减个5，毕竟拐点找的太严格又容易找不到，不严格很容易这个拐点的下一行就是白跳黑
        {
            /******************debug:把从右往左的找点轨迹画出来******************/
//            lcd_drawpoint(DownInflectionL.X, row, YELLOW);
            /******************************************************************/
            if(BinaryImage[row][DownInflectionL.X]==IMAGE_WHITE && BinaryImage[row-1][DownInflectionL.X]==IMAGE_BLACK)  //由白到黑跳变
            {
                //从白黑跳变点上几行再找黑跳白，得到很好的拐点去补线
                //注意此处不能说遍历到一半就停下来了，因为斜入的时候上拐点本来就比较中间
                for(cloum=DownInflectionL.X;cloum<MT9V03X_W-30;cloum++)
                {
                    /******************debug:把从右往左的找点轨迹画出来******************/
//                    lcd_drawpoint(cloum, row, YELLOW);
                    /******************************************************************/
                    if(BinaryImage[row-3][cloum]==IMAGE_BLACK && BinaryImage[row-3][cloum+1]==IMAGE_WHITE)  //黑点->白点
                    {
                        //记录上拐点
                        UpInflectionL.Y=row-1;UpInflectionL.X=cloum;
                        FillingLine('L', DownInflectionL, UpInflectionL);
                        return 3;//向左斜入十字
                    }
                }
            }
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
