/*
 * ImageProcess.c
 *
 * Created on: 2022年3月29日
 * Author: 30516
 * Effect: Image element processing logic
 */
#include "ImageProcess.h"
#include "zf_gpio.h"

uint8 CrossRoads_flag=0;        //十字标志变量
uint8 Fork_flag=0;              //三岔识别的标志变量
uint8 CircleIsland_flag=0;      //环岛标志变量

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
    /*****************************扫线*****************************/
    GetImagBasic(LeftLine,CentreLine,RightLine);
    /*************************搜寻左右下拐点***********************/
    GetDownInflection(110,45,LeftLine,RightLine,&LeftDownPoint,&RightDownPoint);
    /*************************特殊元素判断*************************/
    Fork_flag=ForkIdentify(LeftLine,RightLine,LeftDownPoint,RightDownPoint);  //三岔
    if(Fork_flag==0)
    {
        CrossRoads_flag=CrossRoadsIdentify(LeftLine,RightLine,LeftDownPoint,RightDownPoint);//十字
        if(CrossRoads_flag==0||CircleIsland_flag!=0)    //识别不到十字或环岛不处于状态0
        {
            gpio_set(P20_9,0);
            CircleIsland_flag=CircleIslandIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint); //环岛
        }
        else
        {
            gpio_set(P20_9,1);
        }
    }
//    gpio_set(P21_5, 1);
//    int64 Sobel=SobelTest();
//    if(Sobel>3900)  //识别到车库
//    {
//        //这里将舵机右打死完成入库
////        Bias=-100;
////        systick_delay_ms(STM0,500);
//        gpio_set(P21_5, 0);
//    }

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

