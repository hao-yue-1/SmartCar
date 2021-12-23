/*
 * ImageTack.c
 *  Created on: 2021年12月22日
 *  Author: 30516
 *  Effect: 用于存放从图像中提取循迹元素的函数
 */

#include "ImageTack.h"

/*
 *******************************************************************************************
 ** 函数功能: 根据中线数组所在离散的点拟合指定的一段的回归方程，并根据线性回归方程求偏差
 ** 参    数: int starline:    离散点的起始行
 **           int endline:     离散点的结束行
 **           int *CentreLine：中线数组
 ** 返 回 值: 拟合出来的回归方程的斜率的倒数
 ** 作    者: LJF
 ** 注    意：因为偏差是int类型的所以return的时候如果是偏差小于45度也就是斜率小于1的时候可能会因为精度确实造成是0
 ********************************************************************************************
 */
float Regression_Slope(int startline,int endline,int *CentreLine)
{
    //Y=BX+A
    int i=0,SumX=0,SumY=0,SumLines=0;
    float SumUp=0,SumDown=0,avrX=0,avrY=0,Bias=0;
    SumLines=endline-startline;   // startline 为开始行， //endline 结束行 //SumLines

    for(i=startline;i<endline;i++)
    {
        SumY+=i;                      //Y行数进行求和
        SumX+=CentreLine[i];         //X列数进行求和
    }
    avrX=(float)(SumX/SumLines);     //X的平均值
    avrY=(float)(SumY/SumLines);     //Y的平均值

    for(i=startline;i<endline;i++)
    {
        SumUp+=(CentreLine[i]-avrY)*(i-avrX);
        SumDown+=(i-avrX)*(i-avrX);
    }
    if(SumDown==0)//斜率不存在的时候所以偏差是0
        Bias=0;
    else
        //B=(int)(SumUp/SumDown);斜率
        Bias=SumDown/SumUp;//我们要的是与Y轴的夹角所以是斜率的倒数正负代表方向
    //A=(SumY-B*SumX)/SumLines;  //截距
    return Bias;
}


