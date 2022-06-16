/*
 * Garage.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放车库相关的源代码
 * *右边不入库：180 入库：170
 * 左边不入库：200 入库：180但是感觉停不下来
 */

#include "ImageSpecial.h"
#include "PID.h"
#include "LED.h" //debug
#include "headfile.h"
#include "ICM20602.h"           //陀螺仪积分完成标志变量以及开启积分函数

#define GARAGE_IDENTIFY_MODE 0    //哪种模式找上拐点
#define IN_L_GARAGE_ANGLE   50  //入左库开启陀螺仪积分的目标角度
#define IN_R_GARAGE_ANGLE   60  //入右库开启陀螺仪积分的目标角度

/********************************************************************************************
 ** 函数功能: Sobel算子检测起跑线
 ** 参    数: 无
 ** 返 回 值: Sobel阈值
 ** 作    者: 师兄
 *********************************************************************************************/
int64 SobelTest(void)
{
    int64 Sobel = 0;
    int64 temp = 0;

    for (uint8 i = MT9V03X_H-1-20; i > 50 ; i--)
    {
        for (uint8 j = 40; j < MT9V03X_W-1-40; j++)
        {
            int64 Gx = 0, Gy = 0;
            Gx = (-1*BinaryImage(i-1, j-1) + BinaryImage(i-1, j+1) - 2*BinaryImage(i, j-1)
                  + 2*BinaryImage(i, j+1) - BinaryImage(i+1, j-1) + BinaryImage(i+1, j+1));
            Gy = (-1 * BinaryImage(i-1, j-1) - 2 * BinaryImage(i-1, j) - BinaryImage(i-1, j+1)
                  + BinaryImage(i+1, j+1) + 2 * BinaryImage(i+1, j) + BinaryImage(i+1, j+1));
            temp += FastABS(Gx) + FastABS(Gy);
            Sobel += temp / 255;
            temp = 0;
        }
    }
    return Sobel;
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

    /**********************debug************************/
//    lcd_showint32(TFT_X_MAX-50, 0, InflectionL.X, 3);
//    lcd_showint32(TFT_X_MAX-50, 1, InflectionR.X, 3);
    /***************************************************/

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
        /**********************debug************************/
//        lcd_showint32(0, 0, BinaryImage[MT9V03X_H/2+10][3], 3);
        /***************************************************/

        //中间左边的的点去找上拐点
        if(BinaryImage[MT9V03X_H/2+15][3]==IMAGE_WHITE)
        {
            InflectionL.X=3;InflectionL.Y=MT9V03X_H/2+15;
            NoInflectionLFlag=1;
        }
        else return 0;//否则说明左边都是黑的了直接返回已经过了左库
    }
    //开始遍历去寻找上拐点
    for(int row=InflectionL.Y-10;row>10;row--)
    {
        /******************debug:把从下往上的找点轨迹画出来******************/
//        lcd_drawpoint(InflectionL.X, row, YELLOW);
        /******************************************************************/
        //从下往上白调黑
        if(BinaryImage[row][InflectionL.X]==IMAGE_WHITE && BinaryImage[row-1][InflectionL.X]==IMAGE_BLACK)
        {
            //在这里分是否入库的情况，如果入库就去判断是否丢失拐点，丢失了的话补线要补右下角和左上角，为了打角大点
            if(NoInflectionLFlag==1 && Choose=='Y')
            {
                UpInflection.X=InflectionL.X;
                UpInflection.Y=row-3;//现在这个if是找到行所以要给行赋值才能补到线
            }
            else
            {
                for(int column=InflectionL.X;column<MT9V03X_W-10;column++)
                {
                    /******************debug:把从左往右的找点轨迹画出来******************/
//                    lcd_drawpoint(column, row, YELLOW);
                    /******************************************************************/
                    //从左往右黑跳白
                    if(BinaryImage[row-3][column]==IMAGE_BLACK && BinaryImage[row-3][column+1]==IMAGE_WHITE)
                    {
                        UpInflection.X=column;UpInflection.Y=row-3;
                        break;
                    }
                }
            }
            break;//进去之后就退出循环，避免没必要的图像遍历，就算失败也退出说明确实是失败了
        }
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
#if 1   //1:上拐点取上面那几行作为循迹 0：补线斜率作为循迹偏差
                if(NoInflectionLFlag==0)//如果没丢失下拐点则用下拐点下面巡线
                {
                    Bias=DifferentBias(InflectionL.Y+5, InflectionL.Y, CentreLine);
                }
                else
                {
                    Bias=DifferentBias(UpInflection.Y, UpInflection.Y-5, CentreLine);//直接以上拐点的上面正常的线去循迹
                }
                return 1;
#else
                //判断是否丢失下拐点
               if(NoInflectionLFlag==1)
               {
                   Point LeftPoint;
                   LeftPoint.X=LeftLine[UpInflection.Y-5];LeftPoint.Y=UpInflection.Y-5;
                   FillinLine_V2('L', MT9V03X_H, UpInflection.Y, UpInflection, LeftPoint);
                   //给偏差给舵机
                   Bias=Regression_Slope(MT9V03X_H, UpInflection.Y, LeftLine);
                   //对斜率求出来的偏差进行个缩放
                   if(Bias<=1) Bias=Bias*2.25;
                   else Bias=Bias*1.75;
                   /******************debug*******************************************/
//                   gpio_toggle(LED_BLUE);
                   /******************************************************************/
                   return 1;
               }
               else
               {
                   FillingLine('L', InflectionL, UpInflection);
                   //给偏差给舵机
                   Bias=Regression_Slope(InflectionL.Y, UpInflection.Y, LeftLine);
                   //对斜率求出来的偏差进行个缩放
                   if(Bias<=1) Bias=Bias*2.25;
                   else Bias=Bias*1.75;
                   /******************debug*******************************************/
//                  gpio_toggle(LED_GREEN);
                  /******************************************************************/
                   return 1;
               }
#endif
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
            gpio_set(LED_BLUE, 0);
//            lcd_showuint8(0, 1, LostNum_LeftLine);
            if(LostNum_LeftLine>30)
            {
                SobelResult=SobelTest();
//            lcd_showint32(0, 0, SobelResult, 5);
            }
            if(SobelResult>ZebraTresholeL)
            {
                gpio_set(LED_BLUE, 1);
                StatusChange=1;
                break;
            }
            break;
        }
        case 1:
        {
            gpio_set(LED_GREEN, 0);
            switch(Choose)
            {
                case 'Y':
                {
                    gpio_set(LED_GREEN, 1);
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
                        gpio_set(LED_GREEN, 1);
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
            gpio_set(LED_WHITE, 0);
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
            gpio_set(LED_WHITE, 0);
            SobelResult=SobelTest();
            if(SobelResult<ZebraTresholeL)
            {
                gpio_set(LED_WHITE, 1);
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
/********************************************************************************************
 ** 函数功能: 右车库识别补线函数
 ** 参    数: char Choose : 选择是否要入库，'Y'or'N'
 **           Point InflectionL:左拐点
 **           Point InflectionR:右拐点
 ** 返 回 值: 0：已经过了车库了
 **           1：还未过完车库
 ** 作    者: LJF
 ** 注    意: 这个函数更多的是在于补线，找到上拐点，而不是在于识别，识别由Sobel做，并且Sobel放入状态机的第一层判断
 *********************************************************************************************/
uint8 GarageRIdentify(char Choose,Point InflectionL,Point InflectionR)
{
    Point UpInflection;//上拐点的变量
    UpInflection.X=0,UpInflection.Y=0;//初始化为0
    uint8 NoInflectionLFlag=0;//左库左拐点不存在的标志变量，用于选取哪种补线方式

    /**********************debug************************/
//    lcd_showint32(TFT_X_MAX-50, 0, InflectionL.X, 3);
//    lcd_showint32(TFT_X_MAX-50, 1, InflectionR.X, 3);
    /***************************************************/

#if GARAGEIDENTIFYMODE
    //此处使用遍历数组找上拐点的方法
    if(InflectionL.X==0 || (InflectionL.Y-InflectionR.Y)<10 || InflectionL.X>MT9V03X_H/2 || BinaryImage[InflectionL.Y+5][5]!=IMAGE_BLACK)
    {
        if(BinaryImage[MT9V03X_H/2+10][3]==IMAGE_WHITE)
        {
            //从上往下遍历数组找到上拐点
            GetUpInflection('R', 20, MT9V03X_H/2+15, &UpInflection);
            NoInflectionLFlag=1;
        }
        else return 0;//否则说明左边都是黑的了直接返回已经过了左库
    }
    else//下拐点存在
        GetUpInflection('R', 20, InflectionL.Y+15, &UpInflection);
#else
    //此处使用直角黑白跳变找上拐点法
    //如果是在斑马线路段了并且右拐点不存在,或者左右拐点之间的横坐标差太多，因为扫线的混乱拐点可能出现在斑马线中间
    //并且右拐点那行的图像最右边要是黑点
    /**********************debug************************/
//    lcd_showuint8(30, 1, BinaryImage[InflectionR.Y+5][MT9V03X_W-1]);
    /***************************************************/
    if(InflectionR.X==0 || InflectionR.X<MT9V03X_W/2 || BinaryImage[InflectionR.Y+5][MT9V03X_W-1]==IMAGE_WHITE)
    {
        /**********************debug************************/
//        lcd_showint32(0, 0, BinaryImage[MT9V03X_H/2+10][3], 3);
//        gpio_toggle(LED_BLUE);
        /***************************************************/
        //中间右边的的点去找上拐点,此处不随机播种，而是让它一定要找到白点作为找上拐点的种子
        for(int row=MT9V03X_H/2+15;row<MT9V03X_H;row++)
        {
            if(BinaryImage[row][MT9V03X_W-3]==IMAGE_WHITE)
            {
                InflectionR.X=MT9V03X_W-3;InflectionR.Y=row+10;//+10是因为下面的循环是-10了，防止因为-10直接到黑色区域所以找不到拐点
                NoInflectionLFlag=1;
                break;
            }
        }
        if(NoInflectionLFlag!=1) return 0;//否则说明左边都是黑的了直接返回已经过了左库
    }
    //开始遍历去寻找上拐点
    for(int row=InflectionR.Y-10;row>10;row--)
    {
        /******************debug:把从下往上的找点轨迹画出来******************/
//        lcd_drawpoint(InflectionL.X, row, YELLOW);
        /******************************************************************/
        //从下往上白调黑
        if(BinaryImage[row][InflectionR.X]==IMAGE_WHITE && BinaryImage[row-1][InflectionR.X]==IMAGE_BLACK)
        {
            //在这里分是否入库的情况，如果入库就去判断是否丢失拐点，丢失了的话补线要补右下角和左上角，为了打角大点
            if(NoInflectionLFlag==1 && Choose=='Y')
            {
                UpInflection.X=InflectionR.X;
                UpInflection.Y=row-4;//现在这个if是找到行所以要给行赋值才能补到线
            }
            else
            {
                for(int column=InflectionR.X;column>10;column--)
                {
                    /******************debug:把从左往右的找点轨迹画出来******************/
//                    lcd_drawpoint(column, row, YELLOW);
                    /******************************************************************/
                    //从左往右黑跳白
                    if(BinaryImage[row-4][column]==IMAGE_BLACK && BinaryImage[row-4][column-1]==IMAGE_WHITE)
                    {
                        UpInflection.X=column;UpInflection.Y=row-4;
                        break;
                    }
                }
            }
            break;//进去之后就退出循环，避免没必要的图像遍历，就算失败也退出说明确实是失败了
        }
    }
#endif
    //判断是否找到上拐点，满足才补线
    if(UpInflection.X!=0 && UpInflection.Y!=0)
    {
        switch(Choose)
        {
            //入右库
            case 'Y':
            {
                Point LeftDownPoint;
                LeftDownPoint.X=5;LeftDownPoint.Y=MT9V03X_H-5;
                FillingLine('L', LeftDownPoint, UpInflection);//入右库补左线
                //给偏差给舵机
                Bias=Regression_Slope(LeftDownPoint.Y, UpInflection.Y, LeftLine);
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
#if 1   //1:上拐点取上面那几行作为循迹 0：补线斜率作为循迹偏差
               if(NoInflectionLFlag==0)//如果没丢失下拐点则用下拐点下面巡线
               {
                    /******************debug*******************************************/
//                    for(int i=0;i<MT9V03X_W-1;i++)
//                    {
//                        lcd_drawpoint(i, InflectionR.Y+5, PURPLE);
//                        lcd_drawpoint(i, InflectionR.Y, PURPLE);
//                    }
                    /******************************************************************/
                    Bias=DifferentBias(InflectionR.Y+5, InflectionR.Y, CentreLine);
               }
               else
               {
                  /******************debug*******************************************/
//                  for(int i=0;i<MT9V03X_W-1;i++)
//                  {
//                      lcd_drawpoint(i, UpInflection.Y, PURPLE);
//                      lcd_drawpoint(i, UpInflection.Y-5, PURPLE);
//                  }
                  /******************************************************************/
                   //莫名其妙的BUG就是补线看到的是斑马线中的点和最上面的点进行了补线，暂未查明原因
                   Bias=DifferentBias(UpInflection.Y, UpInflection.Y-5, CentreLine);//直接以上拐点的上面正常的线去循迹
               }
               return 1;
#else
               //判断是否丢失下拐点
               if(NoInflectionLFlag==1)
               {
                   Point RightPoint;
                   RightPoint.X=RightLine[UpInflection.Y-5];RightPoint.Y=UpInflection.Y-5;
                   /*****DEBUG******/
//                   lcd_drawpoint(RightPoint.X, RightPoint.Y, PURPLE);
//                   systick_delay_ms(STM0,800);
                   /************/
                   FillinLine_V2('R', MT9V03X_H, UpInflection.Y, UpInflection, RightPoint);
                   //给偏差给舵机
                   Bias=Regression_Slope(MT9V03X_H, UpInflection.Y, RightLine);
                   //对斜率求出来的偏差进行个缩放
                   if(Bias<=1) Bias=Bias*2.25;
                   else Bias=Bias*1.75;
                   /******************debug*******************************************/
//                   gpio_toggle(LED_BLUE);
                   /******************************************************************/
                   return 1;
               }
               else
               {
                   FillingLine('R', InflectionR, UpInflection);
                   //给偏差给舵机
                   Bias=Regression_Slope(InflectionR.Y, UpInflection.Y, RightLine);
                   //对斜率求出来的偏差进行个缩放
                   if(Bias<=1) Bias=Bias*2.25;
                   else Bias=Bias*1.75;
                   /******************debug*******************************************/
//                  gpio_toggle(LED_GREEN);
                  /******************************************************************/
                   return 1;
               }
#endif
            }
            default :break;
        }
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
uint8 GarageROStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag)
{
    static uint8 StatusChange;//上一次识别的结果和状态变量
    uint8 NowFlag=0;//这次的识别结果
    int64 SobelResult=0;//索贝尔计算的结果
    switch(StatusChange)
    {
        //第一个状态Sobel检测，通过了再开启识别函数
        case 0:
        {
            gpio_set(LED_BLUE, 0);
            if(LostNum_RightLine>25)
            {
                SobelResult=SobelTest();
            }
            if(SobelResult>ZebraTresholeL)
            {
                gpio_set(LED_BLUE, 1);
                StatusChange=1;
                break;
            }
            break;
        }
        case 1:
        {
            NowFlag=GarageRIdentify('N', InflectionL, InflectionR);
            *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
            if(NowFlag==0)
            {
                gpio_set(LED_GREEN, 1);
                StatusChange=2;//跳转到结束状态
                break;
            }
            break;
        }
        case 2:
        {
            gpio_set(LED_WHITE, 0);
            SobelResult=SobelTest();
            if(SobelResult<ZebraTresholeL)
            {
                gpio_set(LED_WHITE, 1);
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
/********************************************************************************************
 ** 函数功能: 右边车库入库的状态机转移
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR: 右下拐点
 **           uint8* GarageLFlag：把状态机内的识别函数结果返回出去
 ** 返 回 值: 0：元素状态没结束
 **           1：元素状态结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 GarageRIStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag)
{
    static uint8 StatusChange;//上一次识别的结果和状态变量
    uint8 NowFlag=0;//这次的识别结果
    int64 SobelResult=0;//索贝尔计算的结果
    switch(StatusChange)
    {
        //第一个状态Sobel检测，通过了再开启识别函数
        case 0:
        {
//            gpio_set(LED_BLUE, 0);
            if(LostNum_RightLine>25)
            {
                SobelResult=SobelTest();
            }
            if(SobelResult>ZebraTresholeR)
            {
                StatusChange=1;
                break;
            }
            break;
        }
        case 1:
        {
//            gpio_set(LED_GREEN, 0);
            //开启陀螺仪积分判定是否入库成功
            StartIntegralAngle_Z(IN_R_GARAGE_ANGLE);
            StatusChange=2;
            break;
        }
        case 2:
        {
//            gpio_set(LED_WHITE, 0);
#if 0   //0:直接打死入库 1:补线打死入库
            NowFlag=GarageRIdentify('Y', InflectionL, InflectionR);
            *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
#else
            Bias=-5;//打死
            *GarageLFlag=1;
#endif
            //检测是否入库成功，入库成功停车
            if(icm_angle_z_flag==1)
            {
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
