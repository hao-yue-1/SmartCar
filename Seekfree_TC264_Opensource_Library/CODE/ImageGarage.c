/*
 * Garage.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放车库相关的源代码
 * *右边不入库：240 入库：200(角度不能看不到车库)
 * 左边不入库：230（从三岔的直角弯发车）250：从过了直角弯发车 入库：180但是感觉停不下来
 */

#include "ImageSpecial.h"
#include "PID.h"
#include "ICM20602.h"           //陀螺仪积分完成标志变量以及开启积分函数
#include "ImageProcess.h"

#include "LED.h" //debug
#include "SEEKFREE_18TFT.h"
#include "zf_gpio.h"
#include "Motor.h"

extern uint8 bias_startline,bias_endline;        //动态前瞻
uint8   Garage_LastRightangleRow=20;            //车库的全局变量上一次从上往下遍历边线数组寻找直角拐点的行数
uint8   zebre_row=0;                             //识别到斑马线的行
uint8   zebre_column=0;

#define GARAGE_IDENTIFY_MODE 1    //哪种模式找上拐点
//Sobel算子检测
#define FastABS(x) (x > 0 ? x : x * -1.0f)
#define BinaryImage(i, j)    BinaryImage[i][j]
//找拐点函数的debug
#define SEEDGROWFINDVALLEY_DEBUG 0         //种子生长找谷底是否开启DEBUG
#define SEEDGROWFINDPEAK_DEBUG  0
//左车库
#define ZebraTresholeL 1200  //索贝尔测试的阈值
#define IN_GARAGE_ZEBRA_THR_L   250//左边入库的索贝尔阈值,隔行隔列抽取加速sobel运算速度
#define IN_L_GARAGE_ANGLE   40  //入左库开启陀螺仪积分的目标角度
#define L_GARAGE_LOSTLLINE_MIN_THR 25   //左边车库开启索贝尔的左边丢线最小阈值
#define L_GARAGE_LOSTLLINE_MAX_THR 35   //左边车库开启索贝尔的左边丢线最大阈值
#define L_GARAGE_LOSTRLINE_MAX_THR 50   //左边车库开启索贝尔的右边丢线最大阈值
#define LIN_GARAGE_LOSTLLINE_MIN_THR 30 //左边车库入库左丢线数的最小阈值
#define LIN_GARAGE_LOSTRLINE_MAX_THR 30 //左边车库入库左丢线数的最大阈值
#define LINGARAGEENTRANCE_SEEDGROW_THR 100        //左边入库入口种子生长列坐标的阈值
//右车库
#define ZebraTresholeR 300  //索贝尔测试车库在右边的阈值
#define IN_R_GARAGE_ANGLE   60  //入右库开启陀螺仪积分的目标角度
#define R_GARAGE_LOSTRLINE_THR 35   //右边车库开启索贝尔的右边丢线阈值
#define RINGARAGEENTRANCE_SEEDGROW_THR 5        //右边入库入口种子生长列坐标的阈值
//DEBUG开启的宏定义
#define LINGARAGEENTRANCE_DEBUG    0    //左边车库入库是否开启debug
#define RINGARAGEENTRANCE_DEBUG 0                //右边入库入口处是否开启DEBUG
#define RNOINGARAGE_DEBUG   0   //右车库不入库的DEBUG
#define GARAGE_DEBUG    0       //是否需要开启车库的DEBUG
#define GARAGE_LED_DEBUG 0
#define L_IN_GARAGE_LED_DEBUG 1

/********************************************************************************************
 ** 函数功能: Sobel算子检测起跑线
 ** 参    数: uint8 starline,uint8 endline,uint8 starcloumn,uint8 endcloumn
 ** 返 回 值: Sobel阈值
 ** 作    者: LJF
 ** 注    意：之前的行是MT9V03X_H-1-20~50
 **                列是40~MT9V03X_W-1-40
 *********************************************************************************************/
int64 SobelTest(uint8 starline,uint8 endline,uint8 starcloumn,uint8 endcloumn)
{
    int64 Sobel = 0;
    int64 temp = 0;

    for (uint8 i = starline; i > endline ; i-=2)
    {
        for (uint8 j = starcloumn; j < endcloumn; j+=2)
        {
            int64 Gx = 0, Gy = 0;
            Gx = -BinaryImage(i-1, j-1)+BinaryImage(i-1, j+1)
                 -2*BinaryImage(i, j-1)+2*BinaryImage(i, j+1)
                 -BinaryImage(i+1, j-1)+BinaryImage(i+1, j+1);
            Gy = BinaryImage(i-1, j-1)+2*BinaryImage(i-1, j)+BinaryImage(i-1, j+1)
                 -BinaryImage(i+1, j+1)-2*BinaryImage(i+1, j)-BinaryImage(i+1, j+1);
            temp += FastABS(Gx) + FastABS(Gy);
            Sobel += temp / 255;
            temp = 0;
        }
    }
    return Sobel;
}
/********************************************************************************************
 ** 函数功能: 检测斑马线
 ** 参    数: start_line：检测起始行
 **           end_line：检测结束行
 ** 返 回 值: 1：检测到斑马线
 **           0：没有检测到斑马线
 ** 作    者: WBN
 *********************************************************************************************/
uint8 ZebraIndentify(uint8 start_line,uint8 end_line,uint8* black_width)
{
    uint8 num=0,black_finish_flag=0;//黑白跳变的次数，黑色区域消失的标志变量
    for(uint8 row=start_line;row-2>end_line;row-=2)    //向上扫
    {
        num=0;*black_width=0,black_finish_flag=0;
        for(uint8 column=MT9V03X_W-1;column>5;column--)   //向右扫
        {
//            lcd_drawpoint(column, row, PURPLE);
            if(BinaryImage[row][column]!=BinaryImage[row][column-1])    //跳变点
            {
                num++;
            }
            if(num>7)
            {
                zebre_row=row;
                zebre_column=column;
                return 1;
            }
        }
    }
    return 0;
}
/*********************************************************************************
 ** 函数功能: 车库种子生长生长至谷底寻找Y上拐点
 ** 参    数:char Choose：选择是在谷的左边还是右边
 **          Point Seed
 **          int endline
 **          Point *UpInflectionC
 **          char TRANSVERSE_THR:横向列坐标移动的阈值
 ** 返 回 值: 无
 ** 作    者: LJF
 **********************************************************************************/
void SeedGrowFindValley_Garage(char Choose,Point Seed,int endline,Point *UpInflectionC,char TRANSVERSE_THR)
{
    char transversenum=0;//记录种子是否一直横向移动,种子横向生长的次数
    Point tempSeed;//临时的种子
    for(;Seed.Y<endline && Seed.X<MT9V03X_W-1 && Seed.X>0;)
    {
#if SEEDGROWFINDVALLEY_DEBUG
        lcd_drawpoint(Seed.X, Seed.Y, GREEN);
#endif
        switch(Choose)
        {
            case 'L':
                if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.Y++,Seed.X++;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                    Seed.Y++;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.X++;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                        UpInflectionC->Y=Seed.Y,UpInflectionC->X=Seed.X;
                    return;
                }
                break;
            case 'R':
                if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_BLACK)
                {
                    Seed.Y++,Seed.X--;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_WHITE)
                {
                    Seed.Y++;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_BLACK)
                {
                    Seed.X--;
                    if(transversenum!=0)//判断是否是第一次往右走
                    {
                        tempSeed=Seed;
                    }
                    transversenum++;;//说明在往左边走
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_WHITE)
                {
                    if(transversenum!=0)//说明之前一直都是往右走找到了谷底
                    {
                        UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
                    }
                    else
                    {
                        UpInflectionC->Y=Seed.Y,UpInflectionC->X=Seed.X;
                    }
                    return;
                }
                break;
            default:break;
        }
        //当种子横向生长的次数大于了阈值
        if(transversenum>TRANSVERSE_THR)
        {
            UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
            break;
        }
        if(Seed.X>MT9V03X_W-1 || Seed.X<0 || Seed.Y<0 || Seed.Y>MT9V03X_H-1)
        {
            break;//防止找不到死循环
        }
    }
}
//车库种子生长找到山顶，跟上面函数相反
void SeedGrowFindPeak_Garage(char Choose,Point Seed,int endline,Point *PeakInflection,char TRANSVERSE_THR)
{
    for(;Seed.Y>endline && Seed.X<MT9V03X_W-1 && Seed.X>0;)
    {
#if SEEDGROWFINDPEAK_DEBUG
        lcd_drawpoint(Seed.X, Seed.Y, GREEN);
#endif
        switch(Choose)
        {
            //往右上爬山
            case 'R':
            {
                if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                    Seed.X++;Seed.Y--;
                }
                else if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                    Seed.X++;
                }
                else if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.Y--;
                }
                else if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    PeakInflection->Y=Seed.Y,PeakInflection->X=Seed.X;
                    return;
                }
                break;
            }
            //往左边爬山
            case 'L':
            {
                if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_WHITE)
                {
                    Seed.X--;Seed.Y--;
                }
                else if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_WHITE)
                {
                    Seed.X--;
                }
                else if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_BLACK)
                {
                    Seed.Y--;
                }
                else if(BinaryImage[Seed.Y-1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_BLACK)
                {
                    PeakInflection->Y=Seed.Y,PeakInflection->X=Seed.X;
                    return;
                }
                break;
            }
            default:break;
        }
        if(Seed.X>MT9V03X_W-1 || Seed.X<0 || Seed.Y<0 || Seed.Y>MT9V03X_H-1)
        {
            break;//防止找不到死循环
        }
    }
}
//左边入库入口状态识别函数
uint8 LINGarageEntrance(Point InflectionL,Point InflectionR)
{
#if LINGARAGEENTRANCE_DEBUG
    lcd_showint32(TFT_X_MAX-50, 0, InflectionL.X, 3);
    lcd_showint32(TFT_X_MAX-50, 1, InflectionR.X, 3);
#endif
    //上拐点,爬行到谷底的种子点,补右线的起点,补左线的起点,补左线的终点,开始搜寻直角拐点的基准点
    Point UpInflection,PeakSeed,ValleySeed,RDownPoint,LDownPoint,LUpPoint,StarPoint;
    UpInflection.X=0,UpInflection.Y=0;//初始化为0
    char NoInflectionLFlag=0;//1：不存在右拐点 0：存在右拐点
    static float LastBias;
    //判断左下拐点是否为不存在
    if(InflectionL.X==0 || (InflectionL.Y-InflectionR.Y)<10 || InflectionL.X>MT9V03X_W/2 || BinaryImage[InflectionL.Y+5][5]!=IMAGE_BLACK)
    {
        StarPoint.Y=MT9V03X_H/2+15;StarPoint.X=5;//屏幕左中边边点
        //判断左下点是否是我想得到的去寻找种子的基准点
        if(BinaryImage[StarPoint.Y][StarPoint.X]==IMAGE_WHITE)
        {
            InflectionL=StarPoint;
            NoInflectionLFlag=1;//把拐点不存在的flag置1
        }
    }
    else //左拐点存在
    {
        StarPoint.Y=InflectionL.Y;StarPoint.X=5;//屏幕左中边边点
        NoInflectionLFlag=0;
    }
    //找点
    //找山顶点和谷底的种子点
    for(int row=StarPoint.Y-5;row>10;row--)//由于左车库比较显而易见，那个拐点的左边斜率是正常的，比较适合用种子生长
    {
#if LINGARAGEENTRANCE_DEBUG
        lcd_drawpoint(StarPoint.X, row, PURPLE);
#endif
        if(BinaryImage[row][StarPoint.X]==IMAGE_WHITE&&BinaryImage[row-1][StarPoint.X]==IMAGE_BLACK)
        {
            if(BinaryImage[row-1][StarPoint.X+5]==IMAGE_WHITE)//如果找到的点的右边是白色说明是在山边要爬到山顶
            {
                PeakSeed.X=StarPoint.X;PeakSeed.Y=row;//初始点也是白色的否则怕爬不到
                SeedGrowFindPeak_Garage('R', PeakSeed, 10, &ValleySeed, 100);
            }
            else
            {
                ValleySeed.X=StarPoint.X;ValleySeed.Y=row-1;
            }
            break;
        }
    }
    if(ValleySeed.X!=0 && ValleySeed.Y!=0)
    {
        ValleySeed.X+=5;//防止因为山顶差点意思而使得上拐点找错
        SeedGrowFindValley_Garage('L', ValleySeed, MT9V03X_H-1, &UpInflection,LINGARAGEENTRANCE_SEEDGROW_THR);//种子生长法攀爬至谷底
        ValleySeed.X-=5;//找完即还原
#if LINGARAGEENTRANCE_DEBUG
        LcdDrawPoint_V2(PeakSeed.Y, PeakSeed.X, PINK);
        LcdDrawPoint_V2(ValleySeed.Y, ValleySeed.X, YELLOW);
        LcdDrawPoint_V2(UpInflection.Y, UpInflection.X, GREEN);
#endif
        //判断是否找到了上拐点
        if(UpInflection.Y!=0 && UpInflection.Y>30)
        {
            //这里分是否丢失拐点来确定补线的起点
            if(NoInflectionLFlag==1)
            {
                //判断最下面的那一行是否有正常的左线，如果有则补线起点为左线上的点，否则自定义
                if(RightLine[MT9V03X_H-2]>130)
                {
                    RDownPoint.Y=MT9V03X_H-2;RDownPoint.X=RightLine[MT9V03X_H-2];
                }
                else
                {
                    RDownPoint.Y=MT9V03X_H-2;RDownPoint.X=MT9V03X_W-2;
                }
                LDownPoint.Y=MT9V03X_H-2;LDownPoint.X=1;
                LUpPoint.X=1;LUpPoint.Y=ValleySeed.Y;
            }
            else
            {
                RDownPoint.Y=InflectionL.Y;RDownPoint.X=RightLine[InflectionL.Y-5];
                LDownPoint.Y=InflectionL.Y;LDownPoint.X=InflectionL.X;
                LUpPoint.X=1;LUpPoint.Y=ValleySeed.Y;
            }
            //补线
            FillingLine('R', RDownPoint, UpInflection);
            FillingLine('R', UpInflection, ValleySeed);
            FillingLine('L', LDownPoint, LUpPoint);
            //求偏差的处理
            if(LUpPoint.Y<bias_endline)
            {
                Bias=DifferentBias_Garage(bias_startline,bias_endline, CentreLine);
            }
            else if(LUpPoint.Y<bias_startline && bias_endline<LUpPoint.Y)
            {
                Bias=DifferentBias_Garage(bias_startline,LUpPoint.Y, CentreLine);
            }
            else if(bias_startline<LUpPoint.Y)
            {
                Bias=DifferentBias_Garage(RDownPoint.Y,LUpPoint.Y, CentreLine);
            }
//            if(Bias<1) Bias=LastBias;//如果偏差是往右边的
//            else LastBias=Bias;//给上一次偏差赋值
#if LINGARAGEENTRANCE_DEBUG
            lcd_showint32(TFT_X_MAX-50, 2, UpInflection.X, 3);
            lcd_showint32(TFT_X_MAX-50, 3, UpInflection.Y, 3);
#endif
            lcd_showint32(TFT_X_MAX-50, 2, UpInflection.X, 3);
            lcd_showint32(TFT_X_MAX-50, 3, UpInflection.Y, 3);
            if(UpInflection.Y>100)
                return 2;//如果上拐点已经到了100行以下了，可以正常巡线去入库，并且合理停车即可
            return 1;
        }
    }
    return 0;
}
//左侧车库入库状态机
uint8 LINGarageStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag)
{
    static uint8 StatusChange;//状态变量
    uint8 NowFlag=0;//这次的识别结果
    uint8 black_width=0;//识别到斑马线那行的从右往左的黑色宽度
    switch(StatusChange)
    {
        //识别斑马线
        case 0:
        {
            if(ZebraIndentify(80, 40, &black_width)==1)
            {
                NowFlag=LINGarageEntrance(InflectionL, InflectionR);
                *GarageLFlag=NowFlag;
                StatusChange=1;
            }
            break;
        }
        case 1:
        {
            NowFlag=LINGarageEntrance(InflectionL, InflectionR);
            *GarageLFlag=NowFlag;
            if(NowFlag==2)//flag=2的时候是车库门口的上拐点已经到了右下角
            {
                return 1;
            }
            if(NowFlag==0)
            {
                Bias=4;
            }
            break;
        }
    }
    return 0;
}
/********************************************************************************************
 ** 函数功能: 右车库不入库识别补线函数
 ** 参    数: Point InflectionL:左拐点
 **           Point InflectionR:右拐点
 ** 返 回 值: 0：已经过了车库了
 **           1：还未过完车库
 ** 作    者: LJF
 ** 注    意: 这个函数更多的是在于补线，找到上拐点，而不是在于识别，识别由Sobel做，并且Sobel放入状态机的第一层判断
 *********************************************************************************************/
uint8 RNINGarageIdentify(Point InflectionL,Point InflectionR)
{
    //判断左线的宽度是否超过阈值，超过就不处理，否则补线
//    lcd_showuint8(0, 0, LeftLine[100]);
//    if(LeftLine[100]>45)
//    {
//        return 0;//不需要处理
//    }
    //确定好找山边种子的遍历点
    if(InflectionR.X==0)//拐点不存在
    {
        if(BinaryImage[MT9V03X_H-5][MT9V03X_W-5]==IMAGE_WHITE)
        {
            InflectionR.X=MT9V03X_W-5;InflectionR.Y=bias_startline;//随机给点
        }
        else
        {
            InflectionR.X=RightLine[MT9V03X_H-5];InflectionR.Y=MT9V03X_H-5;
        }
    }
    //左边遍历一下再次验证
    uint8 black_width=0;
    for(uint8 column=InflectionR.X;column>InflectionR.X-30;column--)
    {
        //30列以内有白跳黑说明他不是拐点
        if(BinaryImage[InflectionR.Y][column]==IMAGE_WHITE && BinaryImage[InflectionR.Y][column-1]==IMAGE_BLACK)
        {
            if(BinaryImage[MT9V03X_H-5][MT9V03X_W-5]==IMAGE_WHITE)
            {
                InflectionR.X=MT9V03X_W-5;InflectionR.Y=bias_startline;//随机给点
            }
            else
            {
                InflectionR.X=RightLine[MT9V03X_H-5];InflectionR.Y=MT9V03X_H-5;
            }
            break;
        }
    }
//    LcdDrawPoint_V2(InflectionR.Y, InflectionR.X, GREEN);
    //从遍历点往上找白跳黑,找山边点or谷边点
    Point ValleySeed;
    ValleySeed.X=0;ValleySeed.Y=0;
    if(InflectionR.X!=0 && InflectionR.Y!=0)
    {
        for(uint8 row=InflectionR.Y-15;row>5;row--)
        {
//            gpio_toggle(LED_BLUE);
//            lcd_drawpoint(InflectionR.X, row, PURPLE);
            if(BinaryImage[row][InflectionR.X]==IMAGE_WHITE && BinaryImage[row-1][InflectionR.X]==IMAGE_BLACK)
            {
                ValleySeed.X=InflectionR.X;ValleySeed.Y=row-1;
                break;
            }
        }
    }
//    LcdDrawPoint_V2(ValleySeed.Y, ValleySeed.X, YELLOW);
    Point UpInflection;
    UpInflection.X=0;UpInflection.Y=0;
    //上面找到了谷边点，开始爬谷
    if(ValleySeed.X!=0 && ValleySeed.Y!=0)
    {
        SeedGrowFindValley_Garage('R', ValleySeed, 100, &UpInflection, 100);
    }
//    LcdDrawPoint_V2(UpInflection.Y, UpInflection.X, PINK);
    //找到拐点就补线
    if(UpInflection.Y!=0)
    {
       FillingLine('R', InflectionR, UpInflection);
       //偏差处理
       if(UpInflection.Y<bias_endline)//starline<endline<Up.y，则正常循迹
       {
           Bias=DifferentBias_Garage(bias_startline,bias_endline,CentreLine);
       }
       else if(UpInflection.Y<bias_startline && bias_endline<UpInflection.Y)//starline<UP.y<endline,则按照起始行到上拐点
       {
           Bias=DifferentBias_Garage(bias_startline,UpInflection.Y,CentreLine);
       }
       else if(bias_startline<UpInflection.Y)//UP.y<starline<endline
       {
           Bias=DifferentBias_Garage(InflectionR.Y,UpInflection.Y,CentreLine);
       }
       return 1;
    }
    return 0;
}
/********************************************************************************************
 ** 函数功能: 右边车库不入库的状态机转移
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR: 右下拐点
 **           uint8* GarageLFlag：把状态机内的识别函数结果返回出去
 ** 返 回 值: 0：元素状态没结束
 **           1：元素状态结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 RNINGarageStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag)
{
    static uint8 StatusChange;//上一次识别的结果和状态变量
    uint8 no_effect;
    //因为右车库不入库不需要做处理也行所以此处的状态机条件很宽松
    switch(StatusChange)
    {
        case 0:
        {
            //检测到斑马线之后开启编码器计数0.7M
            if(ZebraIndentify(65, 55,&no_effect)==1)
            {
                EncoderDistance(1, 0.7, 0, 0);
                StatusChange=1;
            }
            break;
        }
        case 1:
        {
            if(encoder_dis_flag==1)
            {
                StatusChange=0;//重置状态机
                return 1;
            }
            break;
        }
        default:break;
    }
    return 0;
}


/********************************************************************************************
 ** 函数功能: 完成出库（开环）
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 *********************************************************************************************/
void OutGarage(void)
{
    Bias=10;    //向左打死
    StartIntegralAngle_Z(45);
    while(!icm_angle_z_flag);   //左转45°进入正常寻迹
}
