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
#include "LED.h"
#include <stdio.h>
#include "zf_ccu6_pit.h"
#include "ICM20602.h"
#include "Key.h"

#define STATE_LED_DEBUG 0   //使用主控板LED进行Debug

uint8 bias_startline=70,bias_endline=50;        //动态前瞻
uint8 Fork_flag=0;              //三岔识别的标志变量
uint8 Garage_flag=0;            //车库识别标志变量
uint8 Circle_flag=0;            //环内寻迹标志变量
int16 speed_case_1=220,speed_case_2=240,speed_case_3=220,speed_case_4=220,speed_case_5=220,speed_case_6=220,speed_case_7=200;
int LeftLine[MT9V03X_H]={0}, CentreLine[MT9V03X_H]={0}, RightLine[MT9V03X_H]={0};   //扫线处理左中右三线
uint8 process_flag=3;   //状态机跳转标志

/*1:左十字回环 2：右边十字回环 3：左环岛 4：右环岛 5：三岔里面直道 6：三岔里面有坡道 7：右边车库不入库 8：入库 9：十字路口 'E':编码器 'M':陀螺仪 'S':停车*/
uint8 process_status[20]={1,  7,  5,  4,  2,  3,  5,  8};//总状态机元素执行顺序数组
uint16 process_speed[20]={230,240,250,230,230,230,230,220};//上面数组对应的元素路段的速度
uint8 process_encoder[5]={3,3,3,3};//编码器计距离的数组 **注意右车库不入库的编码器距离不在此处**
uint8 process_icm[5];//陀螺仪积距离的数组
uint8 process_status_cnt=0;//元素状态数组的计数器
uint8 process_encoder_cnt=0;//编码器测距的距离数组计数器
uint8 process_icm_cnt=0;//陀螺仪积分角度的角度数组计数器

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
    static uint8 encoder_flag,icm_flag;  //是否查询编码器的flag,是否查询陀螺仪的flag
    Point InflectionL,InflectionR;     //左右下拐点
    InflectionL.X=0;InflectionL.Y=0;InflectionR.X=0;InflectionR.Y=0;
    /*****************************扫线*****************************/
    if(process_status[process_status_cnt]==7 || process_status[process_status_cnt]==1)    //采用车库专属扫线方案，忽视斑马线影响
    {
        GetImagBasic_Garage(LeftLine, CentreLine, RightLine, 'L');
    }
    else    //正常扫线
    {
        GetImagBasic(LeftLine, CentreLine, RightLine, 'L');
    }
    /*************************搜寻左右下拐点***********************/
    GetDownInflection(110,45,LeftLine,RightLine,&InflectionL,&InflectionR);
    /*************************特殊元素判断*************************/
    /****************************状态机***************************/
#if 1
    switch(process_status[process_status_cnt])
    {
        case 1: //识别左十字回环
        {
            if(CrossLoopIdentify_L(InflectionL)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 2: //识别右十字回环
        {
            if(CrossLoopIdentify_R(InflectionR)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 3: //识别左环岛
        {
            if(CircleIslandIdentify_L(LeftLine, InflectionL)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 4: //识别右环岛
        {
            if(CircleIslandIdentify_R(RightLine, InflectionR)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 5: //识别没坡道的三岔
        {
            if(ForkFStatusIdentify(InflectionL, InflectionR, &Fork_flag)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 6: //识别有坡道的三岔
        {
            if(ForkSStatusIdentify(InflectionL, InflectionR, &Fork_flag)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 7: //识别右车库，直行
        {
            //不处理
            if(encoder_flag==0)
            {
                EncoderDistance(1, 0.7, 0, 0);
                encoder_flag=1;
            }
            if(encoder_dis_flag==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 8: //识别左车库，入库
        {
            if(GarageInIdentify()==1)
            {
                Stop();
            }
            break;
        }
        case 9: //识别十字路口
        {
            if(CrossRoadsStatusIdentify(InflectionL, InflectionR)==1)
            {
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 'E'://编码器计数无元素循迹
        {
            if(encoder_flag==0)//判断是否未开启编码器
            {
                if(process_encoder[process_encoder_cnt]>0)
                {
                    EncoderDistance(1, process_encoder[process_encoder_cnt], 0, 0);//开启编码器
                    process_encoder_cnt++;
                    encoder_flag=1;
                }
                else//如果检测到编码器不需要测距距离=0，那么马上换下一个状态
                {
                    process_status_cnt++;
                    base_speed=process_speed[process_status_cnt];
                    break;
                }
            }
            if(encoder_dis_flag==1 && encoder_flag==1)//判断编码器是否计数完
            {
                encoder_flag=0;
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
        case 'M'://陀螺仪积角度无元素循迹
        {
            if(icm_flag==0)//判断是否未开启编码器
            {
                if(process_icm[process_icm_cnt]>0)
                {
                    StartIntegralAngle_Z(process_icm[process_icm_cnt]);//开启编码器
                    process_icm_cnt++;
                    icm_flag=1;
                }
                else//如果检测到编码器不需要测距距离=0，那么马上换下一个状态
                {
                    process_status_cnt++;
                    base_speed=process_speed[process_status_cnt];
                    break;
                }
            }
            if(icm_angle_z_flag==1 && icm_flag==1)
            {
                icm_flag=0;
                process_status_cnt++;
                base_speed=process_speed[process_status_cnt];
            }
            break;
        }
    }
#endif
    /***************************偏差计算**************************/
    if(Garage_flag!=0)  //在识别函数里面已经计算了Bias
    {
        Garage_flag=0;  //重置flag
    }
    else if (process_status[process_status_cnt]==7)//车库直行偏差
    {
        Bias=DifferentBias_Garage(bias_startline,bias_endline,CentreLine);
    }
    else
    {
        Bias=DifferentBias_Circle(bias_startline,bias_endline,CentreLine); //动态前瞻计算偏差
        Slope=Regression_Slope(bias_startline,bias_endline,CentreLine);    //动态前瞻计算斜率
        bias_startline=70;bias_endline=50;                                 //恢复默认前瞻
    }
    //LCD绘制图像
#if 0
    for(uint8 i=0;i<MT9V03X_W-1;i++)
    {
        lcd_drawpoint(i, bias_startline, YELLOW);
        lcd_drawpoint(i, bias_endline, YELLOW);
    }
#endif
}

/*
 *******************************************************************************************
 ** 函数功能: 停车，使用PID将电机速度降为0，关闭舵机中断
 ** 参    数: 无InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void Stop(void)
{
    base_speed=0;                   //设置目标速度为0
    MotorK_L.I=1.5;MotorK_R.I=1.5;  //修改PID参数快速停车
    pit_disable_interrupt(CCU6_0, PIT_CH1); //关闭舵机中断
    gpio_set(P21_4, 0);
}

/*
 *******************************************************************************************
 ** 函数功能: 以xy轴交点的方式，在LCD上定位一个坐标
 ** 参    数: Inflection：坐标
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawPoint(Point Inflection,uint16 color)
{
    for(uint8 column=0;column<MT9V03X_W-1;column++)
    {
        lcd_drawpoint(column,Inflection.Y,color);
    }
    for(uint8 row=0;row<MT9V03X_H-1;row++)
    {
        lcd_drawpoint(Inflection.X,row,color);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 以xy轴交点的方式，在LCD上定位一个坐标
 ** 参    数:
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawPoint_V2(uint8 row,uint8 column,uint16 color)
{
    for(uint8 cloum=0;cloum<MT9V03X_W-1;cloum++)
    {
        lcd_drawpoint(cloum,row,color);
    }
    for(uint8 row=0;row<MT9V03X_H-1;row++)
    {
        lcd_drawpoint(column,row,color);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 以一条垂直于Y轴直线的方式绘制出Y坐标
 ** 参    数:
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawRow(uint8 row,uint16 color)
{
    for(uint8 column=0;column<MT9V03X_W-1;column++)
    {
        lcd_drawpoint(column,row,color);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 以一条垂直于X轴直线的方式绘制出X坐标
 ** 参    数:
 **           color：颜色
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void LcdDrawColumn(uint8 column,uint16 color)
{
    for(uint8 row=0;row<MT9V03X_H-1;row++)
    {
        lcd_drawpoint(column,row,color);
    }
}
