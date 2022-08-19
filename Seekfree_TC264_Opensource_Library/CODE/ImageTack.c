/*
 * ImageTack.c
 *  Created on: 2021年12月22日
 *  Author: 30516
 *  Effect: 用于存放从图像中提取循迹元素的函数
 */

#include "ImageTack.h"
#include "SEEKFREE_18TFT.h"
#include "PID.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

//变量定义
float Bias=0;       //偏差
float Slope=0;      //斜率

/*
 *******************************************************************************************
 ** 函数功能: 根据中线数组所在离散的点拟合指定的一段的回归方程，并根据线性回归方程求偏差
 ** 参    数: int starline:    离散点的起始行
 **           int endline:     离散点的结束行
 **           int *CentreLine：中线数组
 ** 返 回 值: 拟合出来的回归方程的斜率的倒数
 ** 作    者: LJF
 ** 注    意： - 因为偏差是int类型的所以return的时候如果是偏差小于45度也就是斜率小于1的时候可能会因为精度确实造成是0
 **           - 图像原点在左上方，所以我们进行公式运算的时候把图像模拟到第四象限，所以进行Y的运算的时候加个负号
 **           - startline>endline
 **           - 需要向又拐的时候是正的，左边是负的
 ********************************************************************************************
 */
float Regression_Slope(int startline,int endline,int *CentreLine)
{
    //Y=BX+A
    static float Last_Bias;//上一次的偏差用于前瞻在黑色区域的时候继承上一次的偏差
    int i=0,SumX=0,SumY=0,SumLines=0;
    float SumUp=0,SumDown=0,avrX=0,avrY=0,Bias=0;

    for(i=startline;i>endline;i--)
    {
        if(BinaryImage[i][CentreLine[i]]==IMAGE_BLACK)  //中线在赛道外的情况，跳出累积
        {
            endline=i;
            break;
        }
        else if(abs(CentreLine[i]-CentreLine[i+1])>MT9V03X_W/3)  //中线发生突变，跳出累积
        {
            endline=i;
            break;
        }
        else
        {
            SumX += i;
            SumY += CentreLine[i];
        }
    }
    SumLines=startline-endline;   // startline 为开始行， //endline 结束行 //SumLines
    //特殊判断：判断一下是否大部分前瞻是在赛道外了，如果是的话那么就集成为上次的偏差
    if(SumLines<=5)  return Last_Bias;

    avrX=(float)(SumX/SumLines);     //X的平均值
    avrY=(float)(SumY/SumLines);     //Y的平均值

    for(i=startline;i>endline;i--)
    {
        SumUp+=(CentreLine[i]-avrY)*(i-avrX);//分子
        SumDown+=(i-avrX)*(i-avrX);//分母
    }
    if(SumDown==0)
        Bias=0;
    else
        Bias=SumUp/SumDown;
    if(Bias==Bias)  //bias是真值
    {
        Last_Bias=Bias;
        return Bias;
    }
    else
    {
        return Last_Bias;   //计算错误，忽略此次计算，返回上一次的值
    }
}
float Regression_Slope_Garage(int startline,int endline,int *CentreLine)
{
    //Y=BX+A
    static float Last_Bias;//上一次的偏差用于前瞻在黑色区域的时候继承上一次的偏差
    int i=0,SumX=0,SumY=0,SumLines=0;
    float SumUp=0,SumDown=0,avrX=0,avrY=0,Bias=0;

    for(i=startline;i>endline;i--)
    {
            SumX += i;
            SumY += CentreLine[i];
    }
    SumLines=startline-endline;   // startline 为开始行， //endline 结束行 //SumLines
    //特殊判断：判断一下是否大部分前瞻是在赛道外了，如果是的话那么就集成为上次的偏差
    if(SumLines<=5)  return Last_Bias;

    avrX=(float)(SumX/SumLines);     //X的平均值
    avrY=(float)(SumY/SumLines);     //Y的平均值

    for(i=startline;i>endline;i--)
    {
        SumUp+=(CentreLine[i]-avrY)*(i-avrX);//分子
        SumDown+=(i-avrX)*(i-avrX);//分母
    }
    if(SumDown==0)
        Bias=0;
    else
        Bias=SumUp/SumDown;
    if(Bias==Bias)  //bias是真值
    {
        Last_Bias=Bias;
        return Bias;
    }
    else
    {
        return Last_Bias;   //计算错误，忽略此次计算，返回上一次的值
    }
}

/********************************************************************************************
 ** 函数功能: 根据两点进行补线(直线)
 ** 参    数: int *LeftLine: 左线，int *CentreLine: 中线，int *RightLine: 右线 （变为全局变量了）
 **           char Choose: 选择补左线还是右线
 **           Point StarPoint:    起点
 **           Point EndPoint:     终点
 ** 返 回 值: 无
 ** 作    者: LJF
 ** 注    意：- StarPoint.Y>EndPoint.Y
 **           - 把图像映射到第四象限进行y=kx+b的操作，y先全取负运算之后，描黑的时候再负运算
 **           - 2022/2/27 17:40 DeBuglog: K应该为浮点型，否则精度损失为0
 *********************************************************************************************/
void FillingLine(char Choose, Point StarPoint,Point EndPoint)
{
    float K;//斜率为浮点型，否则K<1时，K=0
    int B,Y,X;

    /*特殊情况：当要补的线是一条垂线的时候*/
    if(EndPoint.X==StarPoint.X)
    {
        for(Y=StarPoint.Y;Y>EndPoint.Y;Y--)
        {
            switch(Choose)
            {
                case 'L':
                    LeftLine[Y]=StarPoint.X;
                    CentreLine[Y]=(StarPoint.X+RightLine[Y])/2;
                    break;
                case 'R':
                    RightLine[Y]=StarPoint.X;
                    CentreLine[Y]=(LeftLine[Y]+StarPoint.X)/2;//在里面进行中线的修改，因为不会出现补两边的情况，正入十字就直接冲，斜入就补一边而已
                    break;
                default:break;
            }
        }
        return;
    }

    K=(float)(-EndPoint.Y+StarPoint.Y)/(EndPoint.X-StarPoint.X);//k=(y2-y1)/(x2-x1)，强制类型转化否则会损失精度仍然为0
    B=-StarPoint.Y-K*StarPoint.X;//b=y-kx

    for(Y=StarPoint.Y;Y>EndPoint.Y;Y--)
    {
        X=(int)((-Y-B)/K);          //强制类型转化：指针索引的时候只能是整数

        //判断X会不会越界
        if(X<0)                X=0;
        else if(X>MT9V03X_W-1) X=MT9V03X_W-1;

        switch(Choose)
        {
            case 'L':
                LeftLine[Y]=X;
                CentreLine[Y]=(X+RightLine[Y])/2;
                break;
            case 'R':
                RightLine[Y]=X;
                CentreLine[Y]=(LeftLine[Y]+X)/2;//在里面进行中线的修改，因为不会出现补两边的情况，正入十字就直接冲，斜入就补一边而已
                break;
            default:break;
        }
    }
}
void FillinLine_V2(char Choose,int startline,int endline,Point Point1,Point Point2)
{
    float K,B;//斜率为浮点型，否则K<1时，K=0
    int Y,X;

    /*特殊情况：当要补的线是一条垂线的时候*/
    if(Point1.X==Point2.X)
    {
        for(Y=Point1.Y;Y>Point2.Y;Y--)
        {
            switch(Choose)
            {
                case 'L':
                    LeftLine[Y]=Point1.X;
                    CentreLine[Y]=(Point1.X+RightLine[Y])/2;
                    break;
                case 'R':
                    RightLine[Y]=Point1.X;
                    CentreLine[Y]=(LeftLine[Y]+Point1.X)/2;//在里面进行中线的修改，因为不会出现补两边的情况，正入十字就直接冲，斜入就补一边而已
                    break;
                default:break;
            }
        }
        return;
    }
    /***********************************************/

    K=(float)(-Point2.Y+Point1.Y)/(Point2.X-Point1.X);//k=(y2-y1)/(x2-x1)，强制类型转化否则会损失精度仍然为0
    B=(float)-Point1.Y-K*Point1.X;//b=y-kx

    for(Y=startline;Y>endline;Y--)
    {
        X=(int)((-Y-B)/K);          //强制类型转化：指针索引的时候只能是整数

        //判断X会不会越界
        if(X<0)                X=0;
        else if(X>MT9V03X_W-1) X=MT9V03X_W-1;

        switch(Choose)
        {
            case 'L':
                LeftLine[Y]=X;
                CentreLine[Y]=(X+RightLine[Y])/2;
                break;
            case 'R':
                RightLine[Y]=X;
                CentreLine[Y]=(LeftLine[Y]+X)/2;//在里面进行中线的修改，因为不会出现补两边的情况，正入十字就直接冲，斜入就补一边而已
                break;
            default:break;
        }
    }
}
/********************************************************************************************
 ** 函数功能: 根据中线数组所在离散的点计算出离中线的偏差Bias
 ** 参    数: int starline:    离散点的起始行
 **           int endline:     离散点的结束行
 **           int *CentreLine： 中线数组
 ** 返 回 值: 偏差Bias
 ** 作    者: WBN
 *********************************************************************************************/
float DifferentBias(int startline,int endline,int *CentreLine)
{
    static float last_bias;
    float bias=0;
    uint8 rownum=0;//用于计数求了多少行的偏差

    for(uint8 i=startline;i>endline;i--)
    {
        if(BinaryImage[i][CentreLine[i]]==IMAGE_BLACK)  //中线在赛道外的情况，跳出累积
        {
            break;
        }
        else
        {
            bias+=(float)(MT9V03X_W/2-CentreLine[i]);  //累积偏差，Mid-Centre，左正右负（中线在车头的左/右，应该往左/右）
            rownum++;
        }
    }
    bias=bias/rownum/10;   //求偏差均值

    if(bias<0.5&&bias>-0.5) //分段加权
    {
        bias=bias*0.1;
    }
//    else if(bias<-3||bias>3)
//    {
//        bias=bias*1.5;
//    }

    if(bias==bias)  //bias是真值
    {
        last_bias=bias;
        return bias;
    }
    else
    {
        return last_bias;   //计算错误，忽略此次计算，返回上一次的值
    }
}

/********************************************************************************************
 ** 函数功能: 环内行驶专属求Bias，可避免远处干扰
 ** 参    数: starline:    离散点的起始行
 **           endline:     离散点的结束行
 **           *CentreLine：中线数组
 ** 返 回 值: 偏差Bias
 ** 作    者: WBN
 *********************************************************************************************/
float DifferentBias_Circle(uint8 startline,uint8 endline,int *CentreLine)
{
    static float last_bias;
    float bias=0;
    uint8 rownum=0;//用于计数求了多少行的偏差

    for(uint8 i=startline;i>endline;i--)
    {
        if(BinaryImage[i][CentreLine[i]]==IMAGE_BLACK)  //中线在赛道外的情况，跳出累积
        {
            break;
        }
        else if(abs(CentreLine[i]-CentreLine[i+1])>MT9V03X_W/3)  //中线发生突变，跳出累积
        {
            break;
        }
        else
        {
            bias+=(float)(MT9V03X_W/2-CentreLine[i]);  //累积偏差，Mid-Centre，左正右负（中线在车头的左/右，应该往左/右）
            rownum++;
        }
    }
    bias=bias/rownum/10;   //求偏差均值

//    if(bias<0.5&&bias>-0.5) //分段加权
//    {
//        bias=bias*0.1;
//    }
//    else if(bias<-3||bias>3)
//    {
//        bias=bias*1.5;
//    }

    if(bias==bias)  //bias是真值
    {
        last_bias=bias;
        return bias;
    }
    else
    {
        return last_bias;   //计算错误，忽略此次计算，返回上一次的值
    }
}

/*********************************************
 * 函数功能:车库专属循迹偏差
 * 注   意：此函数，不会管屏幕循迹是否是黑色
 *********************************************/
float DifferentBias_Garage(int startline,int endline,int *CentreLine)
{
    static float last_bias;
    float bias=0;
    uint8 rownum=0;//用于计数求了多少行的偏差

    for(uint8 i=startline;i>endline;i--)
    {

        bias+=(float)(MT9V03X_W/2-CentreLine[i]);  //累积偏差，Mid-Centre，左正右负（中线在车头的左/右，应该往左/右）
        rownum++;
    }
    bias=bias/rownum/10;   //求偏差均值

    if(bias<0.5&&bias>-0.5) //分段加权
    {
        bias=bias*0.1;
    }
//    else if(bias<-3||bias>3)
//    {
//        bias=bias*1.5;
//    }

    if(bias==bias)  //bias是真值
    {
        last_bias=bias;
        return bias;
    }
    else
    {
        return last_bias;   //计算错误，忽略此次计算，返回上一次的值
    }
}
/********************************************************************************************
 ** 函数功能: 根据赛道宽度单边巡线消除中线失真
 ** 参    数: char ManualorAuto:自动或者手动判断补线
 **           char LorR：手动的话选择是根据左线来的还是右线
 **           int starline:    离散点的起始行
 **           int endline:     离散点的结束行
 ** 返 回 值: 偏差Bias
 ** 作    者: LJF
 *********************************************************************************************/
void Unilaterally_Plan_CenterLine(char ManualorAuto ,char LorR,int startline,int endline)
{
    int row=0,test=0;
//    lcd_showint32(TFT_X_MAX-50, 0, LeftLine[60], 3);
//    lcd_showint32(TFT_X_MAX-50, 1, RightLine[60], 3);
    for(row=startline;row>endline;row--)
    {
        switch(ManualorAuto)
        {
            case 'M':
            {
                switch(LorR)
                {
                    case 'L':
                    {
                        if(LeftLine[row]!=0)//左边丢线就不单边巡线
                        {
                            CentreLine[row]=LeftLine[row]+(137-(119-row)*1.1)/2;
                            if(CentreLine[row]<0) CentreLine[row]=0;
                            else if(CentreLine[row]>MT9V03X_W-1) CentreLine[row]=MT9V03X_W-1;
//                            test=LeftLine[row]+(137-(119-row)*1.1)/2;
//                            if(test<0) test=0;
//                            else if(test>MT9V03X_W-1) test=MT9V03X_W-1;
//                            lcd_drawpoint(test, row, PURPLE);
                        }
                        break;
                    }
                    case 'R':
                    {
                        CentreLine[row]=RightLine[row]-(137-(119-row)*1.1)/2;
                        if(CentreLine[row]<0) CentreLine[row]=0;
                        else if(CentreLine[row]>MT9V03X_W-1) CentreLine[row]=MT9V03X_W-1;
//                        test=RightLine[row]-(137-(119-row)*1.1)/2;
//                        if(test<0) test=0;
//                        else if(test>MT9V03X_W-1) test=MT9V03X_W-1;
//                        lcd_drawpoint(test, row, PURPLE);
                        break;
                    }
                    default:break;
                }
                break;
            }
            case 'A':
            {
                //左边丢线右边不丢
                if(LeftLine[row]==0 && RightLine[row]!=MT9V03X_W-1)
                {
//                    CentreLine[row]=RightLine[row]-(137-(119-row)*1.1)/2;
//                    if(CentreLine[row]<0) CentreLine[row]=0;
//                    else if(CentreLine[row]>MT9V03X_W-1) CentreLine[row]=MT9V03X_W-1;
                    test=RightLine[row]-(137-(119-row)*1.1)/2;
                    if(test<0) test=0;
                    else if(test>MT9V03X_W-1) test=MT9V03X_W-1;
                    lcd_drawpoint(test, row, PURPLE);
                }
                //右边丢线左边不丢
                else if(LeftLine[row]!=0 && RightLine[row]==MT9V03X_W-1)
                {
//                    CentreLine[row]=LeftLine[row]+(137-(119-row)*1.1)/2;
//                    if(CentreLine[row]<0) CentreLine[row]=0;
//                    else if(CentreLine[row]>MT9V03X_W-1) CentreLine[row]=MT9V03X_W-1;
                      test=LeftLine[row]+(137-(119-row)*1.1)/2;
                      if(test<0) test=0;
                      else if(test>MT9V03X_W-1) test=MT9V03X_W-1;
                      lcd_drawpoint(test, row, PURPLE);
                }
                break;
            }
            default:break;
        }

    }
}

/********************************************************************************************
 ** 函数功能: 给出两个点的坐标并给出一个未知点的Y坐标，
 ** 参    数:
 ** 返 回 值: 偏差Bias
 ** 作    者: WBN
 *********************************************************************************************/
uint8 SlopeUntie_X(Point point_1,Point point_2,uint8 y)
{
    float k;//斜率为浮点型，否则K<1时，K=0
    float b;
    int x;

    /*特殊情况：当要补的线是一条垂线的时候*/
    if(point_1.X==point_2.X)
    {
        x=point_1.X;
    }
    else
    {
        k=(float)(-point_2.Y+point_1.Y)/(point_2.X-point_1.X);//k=(y2-y1)/(x2-x1)，强制类型转化否则会损失精度仍然为0
        b=-point_1.Y-k*point_1.X;//b=y-kx
        x=(int)((-y-b)/k);
    }

    if(x>MT9V03X_W-1)
    {
        x=MT9V03X_W-1;
    }
    else if(x<0)
    {
        x=0;
    }

    return (uint8)x;
}
