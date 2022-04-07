/*
 * ImageTack.c
 *  Created on: 2021年12月22日
 *  Author: 30516
 *  Effect: 用于存放从图像中提取循迹元素的函数
 */

#include "ImageTack.h"
#include "SEEKFREE_18TFT.h"

//变量定义
float Bias=0;       //偏差

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
    int i=0,SumX=0,SumY=0,SumLines=0;
    float SumUp=0,SumDown=0,avrX=0,avrY=0,Bias=0;
    SumLines=startline-endline;   // startline 为开始行， //endline 结束行 //SumLines

    for(i=startline;i>endline;i--)
    {
        SumX += i;
        SumY += CentreLine[i];
//        SumY-=i;                      //Y行数进行求和
//        SumX+=CentreLine[i];         //X列数进行求和
    }
    avrX=(float)(SumX/SumLines);     //X的平均值
    avrY=(float)(SumY/SumLines);     //Y的平均值

    for(i=startline;i>endline;i--)
    {
        SumUp+=(CentreLine[i]-avrY)*(i-avrY);//分子
        SumDown+=(i-avrX)*(i-avrX);//分母
//        SumUp+=(CentreLine[i]-avrX)*(-i-avrY);//分子
//        SumDown+=(CentreLine[i]-avrX)*(CentreLine[i]-avrX);//分母
    }
    if(SumDown==0)
        Bias=0;
    else
        Bias=SumUp/SumDown;
//    if(SumUp==0)//分子为0时即直线与x轴平行，所以此时Bias的分母为0需要做处理
//        Bias=57.3;//tan89°为57.2899
//    else
//        //B=(int)(SumUp/SumDown);斜率
//        Bias=SumDown/SumUp;//我们要的是与Y轴的夹角所以是斜率的倒数正负代表方向
//    //A=(SumY-B*SumX)/SumLines;  //截距
    return Bias;
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
        //BinaryImage[Y][X]=IMAGE_BLACK;//BinaryImage[H][W],x=(y-b)/k//对二值化图像进行补线
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 根据中线数组所在离散的点计算出离中线的偏差Bias
 ** 参    数: int starline:    离散点的起始行
 **           int endline:     离散点的结束行
 **           int *CentreLine： 中线数组
 ** 返 回 值: 偏差Bias
 ** 作    者: WBN
 ********************************************************************************************
 */
float DifferentBias(int startline,int endline,int *CentreLine)
{
    static float last_bias;
    float bias=0;
    int rownum=0;//用于计数求了多少行的偏差

    for(int i=startline;i>endline;i--)
    {
        if(BinaryImage[i][CentreLine[i]]==IMAGE_WHITE)
        {
            bias+=(float)(MT9V03X_W/2-CentreLine[i]);  //累积偏差
            rownum++;
        }
    }

    bias=bias/rownum/10;   //求偏差均值
    if(bias<0.5&&bias>-0.5) //分段加权
    {
        bias=bias*0.1;
    }

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
