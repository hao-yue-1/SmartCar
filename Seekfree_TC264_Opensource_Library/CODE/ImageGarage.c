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
            if (BinaryImage[row][column]==IMAGE_BLACK && black_finish_flag==0)
            {
                (*black_width)++;
            }
            if(BinaryImage[row][column]==IMAGE_WHITE)
            {
                black_finish_flag=1;
            }
            if(num>7)
            {
                zebre_row=row;
                zebre_column=column;
                lcd_showuint8(TFT_X_MAX-50, 0, zebre_row);
                lcd_showuint8(TFT_X_MAX-50, 1, zebre_column);
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
        lcd_drawpoint(Seed.X, Seed.Y, YELLOW);
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
        lcd_drawpoint(Seed.X, Seed.Y, YELLOW);
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
            default:break;
        }
        if(Seed.X>MT9V03X_W-1 || Seed.X<0 || Seed.Y<0 || Seed.Y>MT9V03X_H-1)
        {
            break;//防止找不到死循环
        }
    }
}
/********************************************************************************************
 ** 函数功能: 左车库识别补线函数
 ** 参    数: char Choose : 选择是否要入库，'Y'or'N'
 **           Point InflectionL:左拐点
 **           Point InflectionR:右拐点
 ** 返 回 值: 0：已经过了车库了
 **           1：还未过完车库
 ** 作    者: LJF
 ** 注    意: 这个函数更多的是在于补线，找到上拐点，而不是在于识别，识别由Sobel做，并且Sobel放入状态机的第一层判断
 *********************************************************************************************/
uint8 GarageLIdentify(char Choose,Point InflectionL,Point InflectionR)
{
    Point UpInflection;//上拐点的变量
    UpInflection.X=0,UpInflection.Y=0;//初始化为0
    uint8 NoInflectionLFlag=0;//左库左拐点不存在的标志变量，用于选取哪种补线方式
    float LastBias=Bias;//记录上一次的Bias

#if GARAGE_DEBUG
    lcd_showint32(TFT_X_MAX-50, 0, InflectionL.X, 3);
    lcd_showint32(TFT_X_MAX-50, 1, InflectionR.X, 3);
#endif

#if GARAGE_IDENTIFY_MODE
    //此处使用遍历数组找上拐点的方法
    if(InflectionL.X==0 || (InflectionL.Y-InflectionR.Y)<10 || InflectionL.X>MT9V03X_H/2 || BinaryImage[InflectionL.Y+5][5]!=IMAGE_BLACK)
    {
        if(BinaryImage[MT9V03X_H/2+10][3]==IMAGE_WHITE)
        {
            //从上往下遍历数组找到上拐点
            GetUpInflection('L', 20, MT9V03X_H/2+15, &UpInflection);
            NoInflectionLFlag=1;
        }
        else return 0;//否则说明左边都是黑的了直接返回已经过了左库
    }
    else//下拐点存在
        GetUpInflection('L', 20, InflectionL.Y+15, &UpInflection);
#else
    //此处使用直角黑白跳变找上拐点法
    //如果是在斑马线路段了并且左拐点不存在,或者左右拐点之间的横坐标差太多，因为扫线的混乱拐点可能出现在斑马线中间
    //并且左拐点那行的图像最左边要是黑点
    if(InflectionL.X==0 || (InflectionL.Y-InflectionR.Y)<10 || InflectionL.X>MT9V03X_W/2 || BinaryImage[InflectionL.Y+5][5]!=IMAGE_BLACK)
    {
        //中间左边的的点去找上拐点
        if(BinaryImage[MT9V03X_H/2+15][3]==IMAGE_WHITE)
        {
            InflectionL.X=3;InflectionL.Y=MT9V03X_H/2+15;
            NoInflectionLFlag=1;
        }
        else return 0;//否则说明左边都是黑的了直接返回已经过了左库
    }
    if(NoInflectionLFlag==1 && Choose=='Y')//入库的上拐点不一定是直角，当进去的时候就是里面那个直角了
    {
        for(int row=InflectionL.Y-10;row>10;row--)
        {
#if GARAGE_DEBUG
            lcd_drawpoint(InflectionL.X, row, YELLOW);
#endif
            //从下往上白调黑
            if(BinaryImage[row][InflectionL.X]==IMAGE_WHITE && BinaryImage[row-1][InflectionL.X]==IMAGE_BLACK)
            {
                UpInflection.X=InflectionL.X;
                UpInflection.Y=row-3;//现在这个if是找到行所以要给行赋值才能补到线
                break;
            }
        }
    }
    else
    {
        GetRightangleUPInflection('L',InflectionL,&UpInflection,10,MT9V03X_W-10);
    }
#endif
    //判断是否找到上拐点，满足才补线
    if(UpInflection.X!=0 && UpInflection.Y!=0)
    {
        switch(Choose)
        {
            //入左库
            case 'Y':
            {
                Point RightDownPoint;
                RightDownPoint.X=MT9V03X_W-5;RightDownPoint.Y=MT9V03X_H-5;
                FillingLine('R', RightDownPoint, UpInflection);
                //给偏差给舵机
                Bias=Regression_Slope(RightDownPoint.Y, UpInflection.Y, RightLine);
                //对斜率求出来的偏差进行个缩放
                if(Bias<=1)
                    Bias=Bias*1.75;//这里因为是入库，感觉跟左上拐点补线的时候太大了使得打到库边，所以手动减小打角，毕竟这个缩放是盲调的
                                   //修改之后感觉良好，如果车头是正着开启ICM的话是可以的，但是问题就在于车头不正不能用ICM去判断
                else
                    Bias=Bias*1.5;
                return 1;
            }
            //不入左库
            case 'N':
            {
#if GARAGE_DEBUG
                for(int i=0;i<MT9V03X_W-1;i++)
                {
                    lcd_drawpoint(i, UpInflection.Y-8, PURPLE);
                    lcd_drawpoint(i, UpInflection.Y-5, PURPLE);
                    lcd_drawpoint(i, InflectionL.Y+5, YELLOW);
                    lcd_drawpoint(i, InflectionL.Y+3, YELLOW);
                }
#endif
//                Bias=DifferentBias(UpInflection.Y-5, UpInflection.Y-8, CentreLine);
                if(NoInflectionLFlag==0)//如果没丢失下拐点则用下拐点下面巡线
                {
                    Bias=DifferentBias(InflectionL.Y+5, InflectionL.Y+3, CentreLine);
                }
                else
                {
                    Bias=DifferentBias(UpInflection.Y-5, UpInflection.Y-8, CentreLine);//直接以上拐点的上面正常的线去循迹
                }
                if(Bias>1.5) Bias=LastBias;//如果偏差往左太大则否认这一次的控制
                return 1;
            }
            default :break;
        }
    }
    return 0;
}
/********************************************************************************************
 ** 函数功能: 左车库的状态机转移
 ** 参    数: char Choose: 选择是入库状态机还是不入库状态机
 **           LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：元素状态没结束
 **           1：元素状态结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 GarageLStatusIdentify(char Choose,Point InflectionL,Point InflectionR,uint8* GarageLFlag)
{
    static uint8 StatusChange;//上一次识别的结果和状态变量
    uint8 NowFlag=0;//这次的识别结果
    int64 SobelResult=0;//索贝尔计算的结果
    switch(StatusChange)
    {
        //第一个状态Sobel检测，通过了再开启识别函数
        case 0:
        {
#if GARAGE_DEBUG
            lcd_showuint8(0, 0, LostNum_LeftLine);
            lcd_showuint8(0, 1, LostNum_RightLine);
#endif
            if(LostNum_LeftLine>L_GARAGE_LOSTLLINE_MIN_THR && LostNum_LeftLine<L_GARAGE_LOSTLLINE_MAX_THR && LostNum_RightLine<L_GARAGE_LOSTRLINE_MAX_THR)
            {
                SobelResult=SobelTest(80,50,40,MT9V03X_W-1-40);
#if GARAGE_DEBUG
                lcd_showint32(0, 2, SobelResult, 5);
#endif
            }
            if(SobelResult>ZebraTresholeL)
            {
                StatusChange=1;
                *GarageLFlag=1;//不让它正常循迹
                Bias=DifferentBias(95, 80, CentreLine);//手动缩短前瞻
                break;
            }
            break;
        }
        case 1:
        {
            switch(Choose)
            {
                case 'Y':
                {
                    //开启陀螺仪积分判定是否入库成功
                    StartIntegralAngle_Z(IN_L_GARAGE_ANGLE);
                    StatusChange=2;
                    break;
                }
                case 'N':
                {
                    NowFlag=GarageLIdentify('N', InflectionL, InflectionR);
                    *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
                    if(NowFlag==0)
                    {
                        StatusChange=3;//跳转到结束状态
                        break;
                    }
                    break;
                }
            }
            break;
        }
        case 2:
        {
#if 0   //0:直接打死入库 1:补线打死入库
            NowFlag=GarageLIdentify('Y', InflectionL, InflectionR);
            *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
#else
            SteerK.P=20;SteerK.D=0;//把微分项的阻尼去掉，防止弯道接反方向车库进不去
            Bias=5;//打死入库
            *GarageLFlag=1;//穿出去不让外面的循迹函数循迹
#endif
            //检测是否入库成功，入库成功停车
            if(icm_angle_z_flag==1)
            {
                return 1;
            }
            break;
        }
        case 3:
        {
            SobelResult=SobelTest(80,50,40,MT9V03X_W-1-40);
            if(SobelResult<ZebraTresholeL)
            {
                return 1;//再次sobel一次确保状态机没出错
            }
            else
            {
                StatusChange=1;//否则回到状态1继续判断
                break;
            }
        }
    }
    return 0;
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
                gpio_set(LED_GREEN, 0);
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
#if RNOINGARAGE_DEBUG
    lcd_showint32(TFT_X_MAX-50, 0, InflectionL.X, 3);
    lcd_showint32(TFT_X_MAX-50, 1, InflectionR.X, 3);
#endif
    Point UpInflection;//上拐点的变量
    UpInflection.X=0,UpInflection.Y=0;//初始化为0
    uint8 NoInflectionRFlag=0,black_width=0;//左库左拐点不存在的标志变量，用于选取哪种补线方式，拐点左边的黑色宽度
    //判断右拐点存不存在
#if RNOINGARAGE_DEBUG
    LcdDrawPoint_V2(InflectionR.Y+5, MT9V03X_W-5, YELLOW);
#endif
    if(InflectionR.X==0 || (InflectionR.Y-InflectionL.Y)<10 || InflectionR.X<MT9V03X_H/2 || BinaryImage[InflectionR.Y+5][MT9V03X_W-5]!=IMAGE_BLACK)
    {
        NoInflectionRFlag=1;
    }
#if GARAGE_IDENTIFY_MODE
    //此处使用遍历数组找上拐点的方法
    GetUpInflection('R', Garage_LastRightangleRow, bias_startline, &UpInflection);
    if(UpInflection.Y<20)   Garage_LastRightangleRow=20;
    else    Garage_LastRightangleRow=UpInflection.Y-10;
#if RNOINGARAGE_DEBUG
    LcdDrawPoint_V2(UpInflection.Y, UpInflection.X, GREEN);
#endif
#else
    //此处使用直角黑白跳变找上拐点法
    //如果是在斑马线路段了并且右拐点不存在,或者左右拐点之间的横坐标差太多，因为扫线的混乱拐点可能出现在斑马线中间
    //并且右拐点那行的图像最右边-5要是黑点
    if(InflectionR.X==0 || InflectionR.X<MT9V03X_W/2 || BinaryImage[InflectionR.Y+5][MT9V03X_W-5]==IMAGE_WHITE)
    {
        //中间右边的的点去找上拐点,此处不随机播种，而是让它一定要找到白点作为找上拐点的种子
        for(int row=MT9V03X_H/2+15;row<MT9V03X_H-1;row++)
        {
            if(BinaryImage[row][MT9V03X_W-3]==IMAGE_WHITE)
            {
                InflectionR.X=MT9V03X_W-3;InflectionR.Y=row+10;//+10是因为下面的循环是-10了，防止因为-10直接到黑色区域所以找不到拐点
                NoInflectionRFlag=1;
                break;
            }
        }
        if(NoInflectionRFlag!=1) return 0;//否则说明左边都是黑的了直接返回已经过了右库
    }
    GetRightangleUPInflection('R',InflectionR,&UpInflection,10,10);
#endif
    //判断是否找到上拐点，满足才补线
    if(UpInflection.X!=0 && UpInflection.Y!=0 && UpInflection.X>40 && UpInflection.Y>30)
    {
//        //特殊检测一下别让上拐点走到斑马线上这里进行一次判断,判断是否在斑马线上
        for(int cloumn=UpInflection.X;cloumn>UpInflection.X-30;cloumn--)
        {
            if(BinaryImage[UpInflection.Y][cloumn]==IMAGE_BLACK)
            {
                black_width++;
            }
//            lcd_showint8(6, 2, black_width);
            if(black_width>10)
                return 0;
        }
#if RNOINGARAGE_DEBUG
        lcd_showint8(6, 0, NoInflectionRFlag);
#endif
       if(NoInflectionRFlag==0)//如果没丢失下拐点则用下拐点下面巡线
       {
#if RNOINGARAGE_DEBUG
            for(int i=0;i<MT9V03X_W-1;i++)
            {
                lcd_drawpoint(i, InflectionR.Y+5, PURPLE);
                lcd_drawpoint(i, InflectionR.Y+3, PURPLE);
            }
#endif
//           Bias=DifferentBias(InflectionR.Y+5, InflectionR.Y+3, CentreLine);
            Bias=DifferentBias(UpInflection.Y-3, UpInflection.Y-5, CentreLine);
       }
       else
       {
#if RNOINGARAGE_DEBUG
          for(int i=0;i<MT9V03X_W-1;i++)
          {
              lcd_drawpoint(i, UpInflection.Y-3, PURPLE);
              lcd_drawpoint(i, UpInflection.Y-5, PURPLE);
          }
#endif
           Bias=DifferentBias(UpInflection.Y-3, UpInflection.Y-5, CentreLine);//直接以上拐点的上面正常的线去循迹
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
    uint8 NowFlag=0;//这次的识别结果
    int64 SobelResult=0;//索贝尔计算的结果
    //因为右车库不入库不需要做处理也行所以此处的状态机条件很宽松
    switch(StatusChange)
    {
        //第一个状态Sobel检测，通过了再开启识别函数
        case 0:
        {
            EncoderDistance(1, 0.7, 0, 0);
            SobelResult=SobelTest(80,50,50,MT9V03X_W-1-50);
            if(SobelResult>ZebraTresholeR)
            {
//                gpio_set(LED_WHITE, 0);
                NowFlag=RNINGarageIdentify(InflectionL, InflectionR);
                *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
                StatusChange=2;
                break;
            }
            StatusChange=1;//状态0为开启编码器状态
            break;
        }
        case 1:
        {
            SobelResult=SobelTest(80,50,50,MT9V03X_W-1-50);
            if(SobelResult>ZebraTresholeR)
            {
//                gpio_set(LED_WHITE, 0);
                NowFlag=RNINGarageIdentify(InflectionL, InflectionR);
                *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
                StatusChange=2;
                break;
            }
            if(encoder_dis_flag==1)
            {
                return 1;//莽撞一点这里直接不再次sobel因为下一个是弯道，sobel可能会使得控制滞后
            }
            break;
        }
        case 2:
        {
            NowFlag=RNINGarageIdentify(InflectionL, InflectionR);
            *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
            if(encoder_dis_flag==1)
            {
                return 1;//莽撞一点这里直接不再次sobel因为下一个是弯道，sobel可能会使得控制滞后
            }
//            SobelResult=SobelTest(65,50,50,MT9V03X_W-1-50);
//            if(SobelResult<ZebraTresholeR)
//            {
//                return 1;//再次sobel一次确保状态机没出错
//            }
//            else
//            {
//                StatusChange=1;//否则回到状态1继续判断
//                break;
//            }
            break;
        }
        default:break;
    }
    return 0;
}
//右边入库入口状态识别函数
uint8 RINGarageEntrance(Point InflectionL,Point InflectionR)
{
#if RINGARAGEENTRANCE_DEBUG
    lcd_showint32(TFT_X_MAX-50, 0, InflectionL.X, 3);lcd_showint32(TFT_X_MAX-50, 1, InflectionL.Y, 3);
    lcd_showint32(TFT_X_MAX-50, 2, InflectionR.X, 3);lcd_showint32(TFT_X_MAX-50, 3, InflectionR.Y, 3);
    lcd_showuint8(TFT_X_MAX-50, 4, BinaryImage[InflectionR.Y+5][MT9V03X_W-5]);
#endif
    //上拐点,爬行到谷底的种子点,补左线的起点,补右线的起点,补右线的终点,开始搜寻直角拐点的基准点
    Point UpInflection,Seed,LDownPoint,RDownPoint,RUpPoint,RightangleStarPoint;
    UpInflection.X=0,UpInflection.Y=0;//初始化为0
    Seed.X=0,Seed.Y=0;//初始化为0
    char NoInflectionRFlag=0;//1：不存在右拐点 0：存在右拐点
    static char GarageLUpInflection;//记录车库里面的左上直角点是否出现
    static float LastBias;
    //判断右下拐点是否为不存在
    if(InflectionR.X==0 || InflectionR.X<MT9V03X_W/2 || BinaryImage[InflectionR.Y+5][MT9V03X_W-5]==IMAGE_WHITE)
    {
        //判断是否已经开始入库了一定角度
        if(GarageLUpInflection==1)
        {
            RightangleStarPoint.Y=MT9V03X_H-10;RightangleStarPoint.X=MT9V03X_W/2+40;//屏幕最下一行中间点
        }
        else
        {
            RightangleStarPoint.Y=MT9V03X_H/2+15;RightangleStarPoint.X=MT9V03X_W-3;//屏幕右下点
        }
        //判断右下点是否是我想得到的去寻找种子的基准点
        if(BinaryImage[RightangleStarPoint.Y][RightangleStarPoint.X]==IMAGE_WHITE)
        {
            InflectionR=RightangleStarPoint;
            NoInflectionRFlag=1;
        }
    }
    else //右拐点存在的话
    {
        NoInflectionRFlag=0;
    }
    //开始找上拐点
    if(GarageLUpInflection==1)//根据是否以及入了一定的角度来决定寻找上拐点的方式
    {
        GetRightangleUPInflection('R', InflectionR, &Seed, 1, 1);//黑白跳变找到的是种子点
    }
    else
    {
        GetUpInflection('R', Garage_LastRightangleRow, InflectionR.Y+15, &Seed);
#if RINGARAGEENTRANCE_DEBUG
        LcdDrawPoint_V2(Seed.Y, Seed.X, GREEN);
#endif
        //动态的寻找上拐点的起始行
        if(Seed.Y<20)   Garage_LastRightangleRow=20;
        else if(Seed.Y>60)  ;
        else    Garage_LastRightangleRow=Seed.Y-10;
    }
    SeedGrowFindValley_Garage('L', Seed, MT9V03X_H-1, &UpInflection,RINGARAGEENTRANCE_SEEDGROW_THR);//种子生长法攀爬至谷底

    //判断是否找到了上拐点
    if(UpInflection.Y!=0)
    {
        //这里分是否丢失拐点来确定补线的起点
        if(NoInflectionRFlag==1)
        {
            //判断最下面的那一行是否有正常的左线，如果有则补线起点为左线上的点，否则自定义
            if(LeftLine[MT9V03X_H-2]<30)
            {
                LDownPoint.Y=MT9V03X_H-2;LDownPoint.X=LeftLine[MT9V03X_H-2];
            }
            else
            {
                LDownPoint.Y=MT9V03X_H-2;LDownPoint.X=2;
            }
            RDownPoint.Y=MT9V03X_H-2;RDownPoint.X=MT9V03X_W-1;
            RUpPoint.Y=2;RUpPoint.X=MT9V03X_W-1;
        }
        else
        {
            for(int row=InflectionR.Y-5;row>10;row--)
            {
#if RINGARAGEENTRANCE_DEBUG
                lcd_drawpoint(MT9V03X_W-2, row, PURPLE);
#endif
                if (BinaryImage[row][MT9V03X_W-2]==IMAGE_WHITE && BinaryImage[row-1][MT9V03X_W-2]==IMAGE_BLACK)
                {
                    RUpPoint.Y=row;RUpPoint.X=MT9V03X_W-1;
                    break;
                }
                else
                {
                    RUpPoint.Y=2;RUpPoint.X=MT9V03X_W-1;
                }
            }
            LDownPoint.Y=InflectionR.Y;LDownPoint.X=LeftLine[InflectionR.Y-5];
            RDownPoint.Y=InflectionR.Y;RDownPoint.X=InflectionR.X;
        }
        FillinLine_V2('L', LDownPoint.Y, 2, LDownPoint, UpInflection);
#if RINGARAGEENTRANCE_DEBUG
        LcdDrawPoint_V2(RUpPoint.Y, RUpPoint.X, GREEN);
#endif
        FillinLine_V2('R', RDownPoint.Y, 2, RDownPoint, RUpPoint);
        //求偏差的处理

        Bias=DifferentBias_Garage(bias_startline,bias_endline, CentreLine);
        if(Bias>0) Bias=LastBias;//如果偏差是往左边的
        else LastBias=Bias;//给上一次偏差赋值
        //结束的最后给静态局部变量判断是否需要变为1
        if(UpInflection.X<MT9V03X_W/2)
        {
            GarageLUpInflection=1;
        }
        return 1;
    }
    return 0;
}

/*******************************************************
 ** 函数功能: 右边车库入库的状态机转移
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR: 右下拐点
 **           uint8* GarageLFlag：把状态机内的识别函数结果返回出去
 ** 返 回 值: 0：元素状态没结束
 **           1：元素状态结束
 ** 作    者: LJF
 *********************************************************/
uint8 RINGarageStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag)
{
    static uint8 StatusChange;//状态变量
    uint8 NowFlag=0;//这次的识别结果
    int64 SobelResult=0;//索贝尔计算的结果
    switch(StatusChange)
    {
        //识别斑马线
        case 0:
        {
            if(LostNum_RightLine>R_GARAGE_LOSTRLINE_THR)
            {
                SobelResult=SobelTest(65,50,50,MT9V03X_W-1-50);
            }
            if(SobelResult>ZebraTresholeR)
            {
                StartIntegralAngle_Z(IN_R_GARAGE_ANGLE);//开启陀螺仪积分入库
                StatusChange=1;
                break;
            }
            break;
        }
        case 1:
        {
            NowFlag=RINGarageEntrance(InflectionL, InflectionR);
            *GarageLFlag=NowFlag;
            if(NowFlag==0)
            {
                StatusChange=2;//跳转到结束状态
                break;
            }
            break;
        }
        case 2:
        {
            //检测是否入库成功，入库成功停车
            NowFlag=RINGarageEntrance(InflectionL, InflectionR);
            *GarageLFlag=NowFlag;
            if(icm_angle_z_flag==1)
            {
//                gpio_set(LED_WHITE, 0);
                Stop();
                return 1;
            }
            break;
        }
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
//    systick_delay_ms(STM0,50);
    //舵机向右打死并加上一定的延时实现出库
    Bias=-10;
    systick_delay_ms(STM0,300);
}
