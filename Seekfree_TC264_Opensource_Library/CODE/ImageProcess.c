/*
 * ImageProcess.c
 *
 * Created on: 2022年3月29日
 * Author: 30516
 * Effect: Image element processing logic
 */
#include "ImageProcess.h"
#include "zf_gpio.h"
#include "PID.h"

uint8 CrossRoads_flag=0;        //十字标志变量
uint8 Fork_flag=0;              //三岔识别的标志变量
uint8 CircleIsland_flag=0;      //环岛标志变量
uint8 Garage_flag=0;            //车库识别标志变量
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
    static uint8 flag;
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
//    CrossLoopEnd_S();
    /****************************状态机***************************/
#if 1
    switch(flag)
    {
        case 0: //识别左环岛
        {
//            flag=2; //调试用，跳转到指定状态
            gpio_set(LED_WHITE, 0);
            if(CircleIslandIdentify_L(LeftLine, RightLine, LeftDownPoint, RightDownPoint)==9)
            {
                gpio_set(LED_WHITE, 1);
                base_speed=100; //提速上坡进入第一个十字回
                flag=1; //跳转到状态1
            }
            break;
        }
        case 1: //识别第一个十字回环
        {
            gpio_set(LED_GREEN, 0);
            if(CrossLoopEnd_F()==1)
            {
                gpio_set(LED_GREEN, 1);
                base_speed=90;  //减速进行右环岛
                flag=2; //跳转到状态2
            }
            else
            {
                CrossLoopBegin_F(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
            }
            break;
        }
        case 2: //识别右环岛
        {
            gpio_set(LED_BLUE, 0);
            if(CircleIslandIdentify_R(LeftLine, RightLine, LeftDownPoint, RightDownPoint)==9)
            {
                gpio_set(LED_BLUE, 1);
                base_speed=95;  //提速进入左车库
                flag=3; //跳转到状态3
            }
            break;
        }
        case 3: //识别左车库
        {
            gpio_set(LED_RED, 0);
            if(SobelLCount<SobelLTestStop)//车库识别到两帧即结束，这里不知道稳不稳
            {
                if(LostNum_LeftLine>40 && LostNum_RightLine<30)
                {
                    Garage_flag=GarageIdentify('L', LeftDownPoint, RightDownPoint);//识别车库
                }
            }
            else
            {
                gpio_set(LED_RED, 1);
                base_speed=90;      //降速进入三岔
                flag=4;
            }
            break;
        }
        case 4: //识别三岔
        {
            gpio_set(LED_YELLOW, 0);
            Fork_flag=ForkIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
            if(ForkStatusIdentify(LeftDownPoint, RightDownPoint,Fork_flag)==1)
            {
                gpio_set(LED_YELLOW, 1);
                base_speed=95;      //提速进入十字回环
                flag=5; //三岔从车库这边进入不太稳定
            }
            break;
        }
        case 5: //识别第二个十字回环
        {
            gpio_set(P21_4, 0);
            if(CrossLoopEnd_S()==1)
            {
                gpio_set(P21_4, 1);
                base_speed=95;  //提速进入三岔和入库
                flag=6;
            }
            else
            {
                CrossLoopBegin_S(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
            }
            break;
        }
        case 6: //识别三岔和右车库
        {
            gpio_set(P21_5, 0);
            if(LostNum_RightLine>40 && LostNum_RightLine<90 && LostNum_LeftLine<10 && LostNum_LeftLine>0)
            {
                Garage_flag=GarageIdentify('R', LeftDownPoint, RightDownPoint);//识别车库
                if(Garage_flag==0)//如果没识别到车库，再继续识别三岔，怕误判,补救
                {
                    Fork_flag=ForkIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
                }
            }
            else
            {
                Fork_flag=ForkIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
            }
            break;
        }
    }
#endif
    /***************************偏差计算**************************/
    if(Fork_flag!=0 || Garage_flag!=0)  //在识别函数里面已经计算了Bias
    {
        Garage_flag=0;Fork_flag=0;
        return;
    }
    else
    {
        Bias=DifferentBias(100,60,CentreLine);//无特殊处理时的偏差计算
    }
}
