/*
 * ImageTack.c
 *  Created on: 2021年12月22日
 *  Author: 30516
 *  Effect: 用于存放从图像中提取循迹元素的函数
 */

/*
 *******************************************************************************************
 ** 函数功能: 根据中线数组所在离散的点拟合指定的一段的回归方程
 ** 参    数: int starline:    离散点的起始行
 **           int endline:     离散点的结束行
 **           int *CentreLine：中线数组
 ** 返 回 值: 拟合出来的回归方程的斜率
 ** 作    者: LJF
 ** 注    意：在求X的平均值和Y平均值以及斜率的时候可能涉及到数据类型的问题，暂未得到合理的验证C语言运行起来倒是没错
 ********************************************************************************************
 */
int Regression_Slope(int startline,int endline,int *CentreLine)
{
    //Y=BX+A
    int i=0,SumX=0,SumY=0,SumLines=0,B,A;//在原来那份代码中B和A是float类型，但是这样返回的又是int，所以这里改成int
    float SumUp=0,SumDown=0,avrX=0,avrY=0;
    SumLines=endline-startline;   // startline 为开始行， //endline 结束行 //SumLines

    for(i=startline;i<endline;i++)
    {
        SumX+=i;
        SumY+=CentreLine[i];    //这里Middle_black为存放中线的数组
    }
    avrX=(float)(SumX/SumLines);     //X的平均值
    avrY=(float)(SumY/SumLines);     //Y的平均值
    SumUp=0;
    SumDown=0;
    for(i=startline;i<endline;i++)
    {
        SumUp+=(CentreLine[i]-avrY)*(i-avrX);
        SumDown+=(i-avrX)*(i-avrX);
    }
    if(SumDown==0)
        B=0;
    else
        B=(int)(SumUp/SumDown);
    A=(SumY-B*SumX)/SumLines;  //截距
    return B;  //返回斜率
}


