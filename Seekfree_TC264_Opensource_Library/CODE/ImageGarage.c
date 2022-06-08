/*
 * Garage.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放车库相关的源代码
 */

#include "ImageSpecial.h"
#include "PID.h"
#include "LED.h" //debug
#include "headfile.h"
#include "ICM20602.h"           //陀螺仪积分完成标志变量以及开启积分函数

//索贝尔计算超过阈值的次数当大于某个值就不再进行sobel测试
uint8 SobelLCount=0;             //左边索贝尔
uint8 SobelRCount=0;             //右边索贝尔
#define GARAGEIDENTIFYMODE 0    //哪种模式找上拐点

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
 ** 函数功能: 左车库特殊情况处理，左斜情况
 ** 参    数: char Direction: 选择检测左边车库还是检测右边车库L或者R
 **           Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值:  0：没有识别到起跑线
 **           1: 判断成立
 ** 作    者: LJF
 *********************************************************************************************/
uint8 DealGarageLSpecial(void)
{
    int row=0,cloumn=0,flag=0,BiasRow=0;
    Point LeftDownPoint,LeftUpPoint;
    LeftDownPoint.X=5,LeftDownPoint.Y=MT9V03X_H-5,LeftUpPoint.X=0,LeftUpPoint.Y=0;
    //这里相当于是这种情况的判断条件，即：这个点为白点
    if(BinaryImage[MT9V03X_H/2+10][5]==IMAGE_WHITE)
    {
        row=MT9V03X_H/2+10;cloumn=5;
        for(;row>0;row--)
        {
            //从左下角白点往上找到白跳黑
            if(flag==0 && BinaryImage[row][cloumn]==IMAGE_WHITE && BinaryImage[row-1][cloumn]==IMAGE_BLACK)
            {
                BiasRow=row;//计算偏差的行
                flag++;
            }
            //再继续找到黑跳白
            if(flag==1 && BinaryImage[row][cloumn]==IMAGE_BLACK && BinaryImage[row-1][cloumn]==IMAGE_WHITE)
            {
                LeftUpPoint.Y=row+5;
                break;
            }
        }
        if(row==0)//遍历完了没找到说明不是
            return 0;
        //从左往右找到黑跳白
        for(;cloumn<MT9V03X_W-30;cloumn++)
        {
            if(BinaryImage[LeftUpPoint.Y][cloumn]==IMAGE_BLACK && BinaryImage[LeftUpPoint.Y][cloumn+1]==IMAGE_WHITE)
            {
                LeftUpPoint.X=cloumn;
                break;
            }
        }
        FillingLine('L', LeftDownPoint, LeftUpPoint);
        //用补的线去求斜率偏差
        Bias=Regression_Slope(LeftDownPoint.Y, LeftUpPoint.Y, LeftLine);
        //对斜率求出来的偏差进行个缩放
        if(Bias<=1)
            Bias=Bias*2.5;
        else
            Bias=Bias*2;
        return 1;
    }
    else
        return 0;
}
/********************************************************************************************
 ** 函数功能: 识别起跑线
 ** 参    数: char Direction: 选择检测左边车库还是检测右边车库L或者R
 **           Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值:  0：没有识别到起跑线
 **           1：识别到起跑线且车库在车右侧
 **           2：识别到起跑线且车库在车左侧
 ** 作    者: LJF
 *********************************************************************************************/
uint8 GarageIdentify(char Direction,Point InflectionL,Point InflectionR)
{
    int64 SobelResult=0;//索贝尔计算的结果
    Point UpInflection;//上拐点的变量
    UpInflection.X=0,UpInflection.Y=0;//初始化为0
    uint8 NoInflectionLFlag=0;//左库左拐点不存在的标志变量，用于给补线的左下拐点赋值
    switch(Direction)
    {
        case 'L':
            //左边丢线(40,95)、右边丢线(0,20)（上面那个转弯的地方）只有一小段，进入索贝尔计算
            SobelResult=SobelTest();//进行索贝尔计算
            if(SobelResult>ZebraTresholeL)
            {
                SobelLCount++;
                //对入库的特殊情况进行单帧判断
                if(DealGarageLSpecial()==1)
                {
                    return 2;
                }

                //如果左拐点不是我想得到的那个点说明那个点已经不在图像中
                if(InflectionL.X==0)
                {
                    InflectionL.X=3;
                    InflectionL.Y=MT9V03X_H/2+10;
                    NoInflectionLFlag=1;

                }
                //从下往上找到白跳到黑，确定上拐点的行数
                for(int row=InflectionL.Y;row-1>0;row--)
                {
                    if(BinaryImage[row][InflectionL.X]==IMAGE_WHITE && BinaryImage[row-1][InflectionL.X]==IMAGE_BLACK)
                    {
                        UpInflection.Y=row-5;//得到这个行数之后把行给上拐点
                        break;//跳出循环
                    }
                }
                //从左往右边找到黑跳到白，确定上拐点的列数
                for(int column=InflectionL.X;column<MT9V03X_W;column++)
                {
                    if(BinaryImage[UpInflection.Y][column]==IMAGE_BLACK && BinaryImage[UpInflection.Y][column+1]==IMAGE_WHITE)
                    {
                        UpInflection.X=column+1;//得到这个列数之后把列给上拐点
                        break;//跳出循环
                    }
                }
                if(NoInflectionLFlag==1)
                {
                    InflectionL.Y=MT9V03X_H;InflectionL.X=UpInflection.X-10;
                }
                FillingLine('L', InflectionL, UpInflection);
                Bias=Regression_Slope(InflectionL.Y, UpInflection.Y, LeftLine);
                //对斜率求出来的偏差进行个缩放
                if(Bias<=1)
                    Bias=Bias*2.25;
                else
                    Bias=Bias*1.75;
                return 2;
            }
            break;
        case 'R':
            //这里如果为了防止跟三岔误判可以加上右边丢线大于40小于90左边丢线小于10即可
            //右边丢线、左边不丢线、存在右拐点进入索贝尔计算
            SobelResult=SobelTest();//进行索贝尔计算
            if(SobelResult>ZebraTresholeR)
            {
                MotorK_L.P=15;
                MotorK_L.I=1.5;
                MotorK_R.P=15;
                MotorK_R.I=1.5;
                /*方案一：右边打死入库，写个while循环把速度停掉打死入库*/
                base_speed=130;
                systick_delay_ms(STM0,15);//加10ms防止卡车库，如果速度提上去可以取消
                Bias=-15;//右边打死
                systick_delay_ms(STM0,150);
                while(1)
                {
//                    Bias=0;
                    base_speed=0;
                }
                return 1;
            }
            break;
        default:break;
    }
    return 0;
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

#if GARAGEIDENTIFYMODE
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
    if(InflectionL.X==0 || (InflectionL.Y-InflectionR.Y)<10 || InflectionL.X>MT9V03X_H/2 || BinaryImage[InflectionL.Y+5][5]!=IMAGE_BLACK)
    {
        /**********************debug************************/
//        lcd_showint32(0, 0, BinaryImage[MT9V03X_H/2+10][3], 3);
        /***************************************************/

        //中间左边的的点去找上拐点
        if(BinaryImage[MT9V03X_H/2+10][3]==IMAGE_WHITE)
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
            SobelResult=SobelTest();
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
                    StartIntegralAngle_Z(80);
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
            NowFlag=GarageLIdentify('Y', InflectionL, InflectionR);
            *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
            //检测是否入库成功，入库成功停车
            if(icm_angle_z_flag==1)
            {
                while(1);
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
    if(InflectionR.X==0 || (InflectionL.Y-InflectionR.Y)<10 || InflectionR.X>MT9V03X_H/2 || BinaryImage[InflectionR.Y+5][MT9V03X_W]!=IMAGE_BLACK)
    {
        /**********************debug************************/
//        lcd_showint32(0, 0, BinaryImage[MT9V03X_H/2+10][3], 3);
        /***************************************************/

        //中间右边的的点去找上拐点
        if(BinaryImage[MT9V03X_H/2+10][MT9V03X_W-3]==IMAGE_WHITE)
        {
            InflectionR.X=MT9V03X_W-3;InflectionR.Y=MT9V03X_H/2+15;
            NoInflectionLFlag=1;
        }
        else return 0;//否则说明左边都是黑的了直接返回已经过了左库
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
                UpInflection.Y=row-3;//现在这个if是找到行所以要给行赋值才能补到线
            }
            else
            {
                for(int column=InflectionR.X;column>10;column--)
                {
                    /******************debug:把从左往右的找点轨迹画出来******************/
//                    lcd_drawpoint(column, row, YELLOW);
                    /******************************************************************/
                    //从左往右黑跳白
                    if(BinaryImage[row-3][column]==IMAGE_BLACK && BinaryImage[row-3][column-1]==IMAGE_WHITE)
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
            //入右库
            case 'Y':
            {
                Point LeftDownPoint;
                LeftDownPoint.X=5;LeftDownPoint.Y=5;
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
                //判断是否丢失下拐点
               if(NoInflectionLFlag==1)
               {
                   Point RightPoint;
                   RightPoint.X=RightLine[UpInflection.Y-5];RightPoint.Y=UpInflection.Y-5;
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
                   FillingLine('R', InflectionL, UpInflection);
                   //给偏差给舵机
                   Bias=Regression_Slope(InflectionL.Y, UpInflection.Y, RightLine);
                   //对斜率求出来的偏差进行个缩放
                   if(Bias<=1) Bias=Bias*2.25;
                   else Bias=Bias*1.75;
                   /******************debug*******************************************/
//                  gpio_toggle(LED_GREEN);
                  /******************************************************************/
                   return 1;
               }
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
            SobelResult=SobelTest();
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
            gpio_set(LED_BLUE, 0);
            SobelResult=SobelTest();
            if(SobelResult>ZebraTresholeL)
            {
                StatusChange=1;
                break;
            }
            break;
        }
        case 1:
        {
            gpio_set(LED_GREEN, 0);
            //开启陀螺仪积分判定是否入库成功
            StartIntegralAngle_Z(80);
            StatusChange=2;
            break;
        }
        case 2:
        {
            gpio_set(LED_WHITE, 0);
            NowFlag=GarageRIdentify('Y', InflectionL, InflectionR);
            *GarageLFlag=NowFlag;//把识别结果带出去，告诉外面还需不需要正常巡线求的偏差
            //检测是否入库成功，入库成功停车
            if(icm_angle_z_flag==1)
            {
                while(1);
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
