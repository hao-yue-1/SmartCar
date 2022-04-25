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
#include "Motor.h"

uint8 CrossRoads_flag=0;        //十字标志变量
uint8 Fork_flag=0;              //三岔识别的标志变量
uint8 CircleIsland_flag=0;      //环岛标志变量
uint8 Garage_flag=0;            //车库识别标志变量
uint8 speed_case_1=200,speed_case_2=150,speed_case_3=130,speed_case_4=155,speed_case_5=145,speed_case_6=160,speed_case_7=135;

uint32 SobelResult=0;

void Stop(void)
{
    while(1)
    {
        base_speed=0;
        diff_speed_kp=0;
    }
}

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
    static uint8 case_5,case_0,case_2,case_1,case_4,case_6,case_3;
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
//    CircleIsFlag_1_L(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
//    CircleIsFlag_1_1_L(LeftLine, RightLine);
    /****************************状态机***************************/
#if 1
    switch(flag)
    {
        case 0: //识别左环岛
        {
//            flag=2; //调试用，跳转到指定状态
            if(case_0<200)  //出库后延时一会再开启下一个元素的识别，防止误判
            {
                case_0++;
                break;
            }
            gpio_set(LED_WHITE, 0);
            if(CircleIslandIdentify_L(LeftLine, RightLine, LeftDownPoint, RightDownPoint)==1)
            {
                gpio_set(LED_WHITE, 1);
                base_speed=speed_case_1; //提速上坡进入第一个十字回环
                flag=1;         //跳转到状态1
            }
            break;
        }
        case 1: //识别第一个十字回环
        {
            if(case_1<100)   //延时一会再进入十字判断
            {
                case_1++;
                break;
            }
            gpio_set(LED_GREEN, 0);
            if(CrossLoopEnd_F()==1)
            {
                gpio_set(LED_GREEN, 1);
                base_speed=speed_case_2; //提速上坡进行右环岛
                flag=2;         //跳转到状态2
            }
            else
            {
                if(CrossLoopBegin_F(LeftLine, RightLine, LeftDownPoint, RightDownPoint)==1)
                {
                    if(case_1==30)  //只进行一次
                    {
                        case_1++;
                        base_speed=150; //分段减速
                    }
                }
                if(CircleIsFlag_3_L()==1)
                {
                    base_speed=120;//降速入环，为出环做准备
                }
            }
            break;
        }
        case 2: //识别右环岛
        {
            if(case_2<100)   //延时开启识别
            {
                case_2++;
                break;
            }
            gpio_set(LED_BLUE, 0);
            if(CircleIslandIdentify_R(LeftLine, RightLine, LeftDownPoint, RightDownPoint)==1)
            {
                gpio_set(LED_BLUE, 1);
                base_speed=speed_case_2;    //借用case2调试
                flag=3;          //跳转到状态3
            }
            break;
        }
        case 3: //识别左车库
        {
            if(case_3<160)//帧率从50变成100，数的帧数也要翻倍，这里是大S
            {
                case_3++;
                break;
            }
            base_speed=speed_case_3;  //减速进入左车库
            gpio_set(LED_RED, 0);
            if(LostNum_LeftLine>40 && LostNum_RightLine<30)
            {
                Garage_flag=GarageIdentify('L', LeftDownPoint, RightDownPoint);//识别车库
            }
            if(GarageLStatusIdentify(LeftDownPoint, RightDownPoint,Garage_flag)==1)
            {
                gpio_set(LED_RED, 1);
                flag=4;          //跳转到状态4
            }
            break;
        }
        case 4: //识别三岔
        {
            if(case_4<50)    //延迟防止误判
            {
                case_4++;
                break;
            }
            base_speed=speed_case_4;  //提速进入三岔
            gpio_set(LED_YELLOW, 0);
            Fork_flag=ForkIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint);   //获取三岔状态
            if(ForkFStatusIdentify(LeftDownPoint, RightDownPoint,Fork_flag)==1)
            {
                gpio_set(LED_YELLOW, 1);
                diff_speed_kp=0.1;  //增大差速过U字弯
                base_speed=speed_case_5; //提速进入第二个十字回环
                flag=5;         //跳转到状态5
            }
            break;
        }
        case 5: //识别第二个十字回环
        {
            if(case_5<110)  //结束三岔后延时一会再开启下一个元素的识别，防止误判
            {
                case_5++;
                break;
            }
            if(case_5==110)
            {
                diff_speed_kp=0.05; //差速改回去
                case_5++;
            }
            gpio_set(P21_4, 0);
            if(CrossLoopEnd_S()==1)
            {
                gpio_set(P21_4, 1);
                base_speed=speed_case_6; //提速进入三岔
                flag=6;         //跳转到状态6
            }
            else
            {
               CrossLoopBegin_S(LeftLine, RightLine, LeftDownPoint, RightDownPoint);
               if(CircleIsFlag_3_L()==1)
               {
                   base_speed=130;//降速入环，为出环做准备
               }
            }
            break;
        }
        case 6: //识别第二遍三岔
        {
            if(case_6<90)  //结束十字回环后延时一会再开启下一个元素的识别，防止S弯误判成三岔入口
            {
                case_6++;
                break;
            }
            gpio_set(P21_5, 0);
            Fork_flag=ForkIdentify(LeftLine, RightLine, LeftDownPoint, RightDownPoint);   //获取三岔状态
            if(ForkSStatusIdentify(LeftDownPoint, RightDownPoint,Fork_flag)==1)
            {
                gpio_set(P21_5, 1);
                base_speed=speed_case_7; //降速准备入库
                MotorK.P=15;    //提高响应速度
                MotorK.I=1.2;
                flag=7;         //跳转到状态7
            }
            break;
        }
        case 7: //识别右车库，入库
        {
            gpio_set(P20_9, 0);
            Garage_flag=GarageIdentify('R', LeftDownPoint, RightDownPoint);//识别车库
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
        Bias=DifferentBias(95,50,CentreLine);//无特殊处理时的偏差计算
//        lcd_showfloat(0, 7, Bias, 2, 3);
    }
}
