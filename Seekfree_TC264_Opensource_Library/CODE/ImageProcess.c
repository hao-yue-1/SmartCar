/*
 * ImageProcess.c
 *
 * Created on: 2022年3月29日
 * Author: 30516
 * Effect: Image element processing logic
 */
#include "ImageProcess.h"
#include "zf_gpio.h"

/********************************************************************************************
 ** 函数功能: 对图像的各个元素之间的逻辑处理函数，最终目的是为了得出Bias给中断去控制
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: LJF
 ** 注    意：无
 *********************************************************************************************/
void ImageProcess()
{
    /***************************变量定义****************************/
    Point LeftDownPoint,RightDownPoint;     //左右下拐点
    LeftDownPoint.X=0;LeftDownPoint.Y=0;RightDownPoint.X=0;RightDownPoint.Y=0;
    Point ForkUpPoint;
    ForkUpPoint.X=0;ForkUpPoint.Y=0;
    Point CrossRoadUpLPoint,CrossRoadUpRPoint;
    CrossRoadUpLPoint.X=0;CrossRoadUpLPoint.Y=0;CrossRoadUpRPoint.X=0;CrossRoadUpRPoint.Y=0;
    uint8 CrossRoads_flag=0;        //十字标志变量
    uint8 Fork_flag=0;              //三岔识别的标志变量
    uint8 CircleIsland_flag=0;      //环岛标志变量
    /******************************扫线***************************/
    GetImagBasic(LeftLine,CentreLine,RightLine);
    /*************************搜寻左右下拐点***********************/
    GetDownInflection(110,10,LeftLine,RightLine,&LeftDownPoint,&RightDownPoint);
    /*****************************特殊元素判断********************************/
    if(!CrossRoadsIdentify(LeftLine,RightLine,LeftDownPoint,RightDownPoint))        //十字
    {
        Fork_flag=ForkIdentify(100,40,LeftLine,RightLine,LeftDownPoint,RightDownPoint);  //三岔
    }
    CircleIslandIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint); //环岛

//    CircleIsFlag_2(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
//    CircleIslandBegin(LeftLine, RightLine);

//    CircleIsFlag_3(LeftLine, RightLine);
//    CircleIsFlag_2(LeftLine, RightLine, LeftDownPoint, RightDownPoint);

    /***************************偏差计算**************************/
    if(Fork_flag!=0)  //在识别函数里面已经计算了Bias
    {
        return;
    }
    else
    {
        Bias=DifferentBias(100,60,CentreLine);//无特殊处理时的偏差计算
    }
}

