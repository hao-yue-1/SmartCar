/*
 * ImageSpecial.c
  *  赛道上各种特殊元素的识别
  *  该文件的函数只实现识别是否有该元素的出现以及出现的位置，至于该元素出现了多少次以及对应措施，不在该文件范围内
 *  Created on: 2022年1月17日
 *      Author: yue
 */

#include "ImageSpecial.h"
#include <math.h>
#include "zf_gpio.h"            //调试用的LED
#include "Binarization.h"       //二值化之后的图像数组
#include "SEEKFREE_18TFT.h"
#include "Steer.h"
#include <stdlib.h>             //abs函数，fabs在math.h
#include "SEEKFREE_MT9V03X.h"
#include "Motor.h"              //停止电机
#include "PID.h"
#include "ImageProcess.h"       //LED宏定义

//索贝尔计算超过阈值的次数当大于某个值就不再进行sobel测试
uint8 SobelLCount=0;             //左边索贝尔
uint8 SobelRCount=0;             //右边索贝尔

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
                MotorK.P=15;
                MotorK.I=1.5;
                /*方案一：右边打死入库，写个while循环把速度停掉打死入库*/
                base_speed=130;
                systick_delay_ms(STM0,15);//加10ms防止卡车库，如果速度提上去可以取消
                Bias=-15;//右边打死
                systick_delay_ms(STM0,150);
                while(1)
                {
//                    Bias=0;
                    diff_speed_kp=0;
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
 ** 函数功能: 左车库的状态机转移
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
 ** 作    者: LJF
 *********************************************************************************************/
uint8 GarageLStatusIdentify(Point InflectionL,Point InflectionR,uint8 GarageLFlag)
{
    static uint8 LastFlag,StatusChange,Statusnums;//上一次识别的结果和状态变量
    uint8 NowFlag=0;//这次的识别结果
    NowFlag=GarageLFlag;
    switch(StatusChange)
    {
        case 0:
        {
            //从没有识别到左车库到识别到左车库
            if(LastFlag==0 && NowFlag==2)
            {
                StatusChange=1;
            }
            break;
        }
        case 1:
        {
            //从识别到左车库到识别不到左车库
            if(LastFlag==2 && NowFlag==0)
            {
                StatusChange=2;
            }
            break;
        }
        case 2:
        {
            //从识别不到左车库到识别不到左车库
            if(LastFlag==0 && NowFlag==0 && Statusnums>1)
            {
                StatusChange=3;
            }
            Statusnums++;//延迟一帧，防止误判
            break;
        }
    }
    LastFlag=NowFlag;
    if(StatusChange==3)
    {
        StatusChange=0;
        return 1;
    }
    else
        return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandBegin_L(int *LeftLine,int *RightLine)
{
    if(BinaryImage[115][5]==IMAGE_BLACK)    //防止提前拐入环岛
    {
        return 0;
    }
    //环岛入口在左边
    if(LostNum_LeftLine>C_LOSTLINE)   //左边丢线：环岛入口在左边
    {
        for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查左边界线
        {
            if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //该行丢线而下一行不丢线
            {
                //下面这个防止进入环岛后误判
                for(int column=0;column+1<MT9V03X_W-1;column++) //向右扫
                {
                    if(BinaryImage[10][column]!=BinaryImage[10][column+1])
                    {
                        Point StarPoint,EndPoint;   //定义补线的起点和终点
                        EndPoint.Y=row;             //终点赋值
                        EndPoint.X=LeftLine[row];
                        StarPoint.Y=MT9V03X_H-1;    //起点赋值
                        StarPoint.X=MT9V03X_W-1;
                        FillingLine('R',StarPoint,EndPoint);    //补线
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，左侧，补线直行忽略
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandOverBegin_L(int *LeftLine,int *RightLine)
{
    //环岛入口在左边
    for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查左边界线
    {
        if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //该行丢线而下一行不丢线
        {
            //下面这个防止进入环岛后误判
            for(int column=0;column+1<MT9V03X_W-1;column++) //向右扫
            {
                if(BinaryImage[10][column]!=BinaryImage[10][column+1])
                {
                    if(row-10>0)
                    {
                        row-=10;
                    }
                    Point StarPoint,EndPoint;   //定义补线的起点和终点
                    EndPoint.Y=row;             //终点赋值
                    EndPoint.X=LeftLine[row];
                    StarPoint.Y=MT9V03X_H-1;    //起点赋值
                    StarPoint.X=0;
                    FillingLine('L',StarPoint,EndPoint);    //补线
                    return 1;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛出口，左侧
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛出口且在车身左侧
 ** 作    者: WBN
 ** 注    意：该函数调用时应确保小车已在环岛中
 ********************************************************************************************
 */
uint8 CircleIslandEnd_L(void)
{
    if(LostNum_LeftLine>100)    //防止还未出环岛的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>55&&LostNum_RightLine>55)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            /*在这里将舵机打死，考虑要不要加延时*/
//            systick_delay_ms(STM0,30);  //加一点延时防止撞到内环
            Bias=10;
            systick_delay_ms(STM0,150);
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag1是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：Flag1不成立
 **           1：Flag1成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0)  //判断条件一：是否存在左拐点与右侧直道
    {
        float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
        if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
        {
            for(int row=InflectionL.Y;row-1>0;row--)    //从左拐点开始向上扫
            {
                if(BinaryImage[row][InflectionL.X]==IMAGE_WHITE&&BinaryImage[row-1][InflectionL.X]==IMAGE_BLACK)
                {
                    uint8 row_f=row;
                    for(;row-1>0;row--)
                    {
                        if(BinaryImage[row][InflectionL.X]==IMAGE_BLACK&&BinaryImage[row-1][InflectionL.X]==IMAGE_WHITE)
                        {
                            row=(row+row_f)/2;
                            for(int column=InflectionL.X;column<MT9V03X_W-1;column++)   //向右扫
                            {
                                if(BinaryImage[row-1][column]==IMAGE_WHITE)
                                {
                                    /*补线操作*/
                                    Point end;
                                    end.Y=row-1;
                                    end.X=column;
                                    FillingLine('L', InflectionL, end);   //补线
                                    return 1;
                                }
                            }
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag1_1是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：Flag1_1不成立
 **           1：Flag1_1成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_1_L(int *LeftLine,int *RightLine)
{
        float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
        if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
        {
            for(uint8 row=MT9V03X_H-30,column=20;row-1>0;row--)    //向上扫
            {
                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
                {
                    uint8 row_f=row;
                    for(;row-1>0;row--)
                    {
                        if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                        {
                            uint8 row_s=row;
                            for(;row-1>0;row--)
                            {
                                if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
                                {
                                    lcd_showuint8(0, 3, 3);
                                    row=(row_f+row_s)/2;
                                    for(;column+1<MT9V03X_W;column++)   //向右扫
                                    {
                                        if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row][column+1]==IMAGE_WHITE)
                                        {
                                            /*补线操作*/
                                            Point end,start;
                                            end.Y=row;
                                            end.X=column;
                                            start.Y=MT9V03X_H-2;
                                            start.X=1;
                                            FillingLine('L', start, end);   //补线
                                            return 1;
                                        }
                                    }
                                    return 0;
                                }
                            }
                            return 0;
                        }
                    }
                    return 0;
                }
            }
        }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag2是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag2不成立
 **           1：Flag2成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_2_L(int *LeftLine,int *RightLine)
{
    if(LostNum_RightLine>55)   //防止误判十字入口
    {
        return 0;
    }
    float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
    if(fabsf(bias_right)<G_LINEBIAS&&LostNum_LeftLine<35)    //右边界为直道且左边丢线小于
    {
        if(BinaryImage[100][10]==IMAGE_BLACK)    //经验位置为黑
        {
            //下面这个for防止在环岛出口时误判为环岛中部
            for(int row=80;row+1<MT9V03X_H-1;row++) //向下扫
            {
                if(LeftLine[row]==0&&LeftLine[row+1]!=0)    //丢线-不丢线
                {
                    return 0;
                }
            }
            for(int row=80;row-1>0;row--)  //向上扫
            {
                if(LeftLine[row]!=0&&LeftLine[row-1]==0)    //不丢线-丢线
                {
                    for(;row-1>0;row--)    //继续向上扫
                    {
                        if(LeftLine[row]==0&&LeftLine[row-1]!=0)    //丢线-不丢线
                        {
                            return 1;
                        }
                    }
                    break;
                }
            }
        }
    }
    return 0;
}
/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag3是否成立，左侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag3不成立
 **           1：Flag3成立且在左侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_3_L(void)
{
    if(LostNum_LeftLine>100)    //左边接近全丢线
    {
        //下面采用经验值随机抽样法
        if(BinaryImage[90][40]==IMAGE_WHITE&&BinaryImage[30][120]==IMAGE_BLACK)
        {
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：还未完成环岛
 **           1：完成环岛
 ** 作    者: WBN
 ** 注    意：这里只有环岛在小车左边的情况
 ********************************************************************************************
 */
uint8 CircleIslandIdentify_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,num_1,num_2,flag_begin,flag_last_begin,flag_last2_begin;
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(CircleIsFlag_1_L(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别环岛出口，进行补线
            {
                if(num_1<100)
                {
                    num_1++;    //识别到flag1的帧数++
                }
            }
            else
            {
                CircleIsFlag_1_1_L(LeftLine, RightLine);
            }
//            else    //没有识别到环岛出口
//            {
//                if(num_2<100)
//                {
//                    num_2++;    //没有识别到flag1的帧数++
//                }
//                else    //超过100帧没有识别到环岛flag1
//                {
//                    num_1=0;
//                    num_2=0;
//                }
//            }
            if(CircleIsFlag_2_L(LeftLine, RightLine)==1)    //识别到环岛中部
            {
                if(num_1>2) //在此之前有识别到环岛flag1
                {
                    num_1=0;
                    num_2=0;
                    flag=1; //跳转到状态1
                    base_speed=150; //降速准备入环
                    break;
                }
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_L(LeftLine, RightLine)==1)
            {
                if(num_1<100)
                {
                    num_1++;    //识别到环岛入口的帧数++
                }
            }
            else
            {
                if(num_2<30)
                {
                    num_2++;    //识别不到环岛入口的帧数++
                }
                else    //超过30帧识别不到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=0; //跳转回到状态0
                    break;
                }
            }
            if(CircleIsFlag_3_L()==1)    //识别已经进入环岛
            {
                if(num_2>2) //在此之前有识别到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=2;
                    break;
                }
            }
            break;
        }
        case 2: //此时小车已经在环岛中，开始判断环岛出口
        {
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            while(CircleIslandEnd_L()==0)  //识别到环岛出口跳出循环
            {
                if(mt9v03x_finish_flag)
                {
                    ImageBinary();                                  //图像二值化
                    GetImagBasic(LeftLine,CentreLine,RightLine);    //基本扫线
                    Bias=DifferentBias(110,60,CentreLine);          //计算偏差，此时在环岛中取特殊前瞻
                    mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
                }
            }
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            flag=3; //跳转到状态3
            break;
        }
        case 3:
        {
            flag_begin=CircleIslandOverBegin_L(LeftLine, RightLine);     //识别环岛入口补线忽略
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=4;
                return 1;   //退出状态机
            }
            flag_last2_begin=flag_last_begin;   //保存上上次的状态
            flag_last_begin=flag_begin; //保存上一次的状态
            break;
        }
        default:break;
    }
    return 0;
}

/*********************************************************************************
 ** 函数功能: 根据左右下拐点搜寻出三岔上拐点
 ** 参    数: Point InflectionL: 左边拐点
 **           Point InflectionR: 右边拐点
 **           Point *UpInflectionC: 中间上拐点
 ** 返 回 值: 无
 ** 说    明: 这个函数最后面会修改，写入在找拐点里面，或者还要加一个元素类型的参数，根据类型来找不同的拐点，这里只针对三岔
 ** 作    者: LJF
 **********************************************************************************/
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC)
{
    int starline,i,cloumnL,cloumnR;
    UpInflectionC->X=0;UpInflectionC->Y=0;//上拐点置零
    UpInflectionC->X=(DownInflectionL.X+DownInflectionR.X)/2;//V型上拐点的列坐标为左右拐点均值，需要修改，不一定是正入三岔
    starline=(DownInflectionL.Y+DownInflectionR.Y)/2;//起始行为左右拐点行的均值
    //从下往上找到那个跳变的点即为上拐点
    for(i=starline;i>1;i--)
    {
        //图像数组是[高][宽]
        if(BinaryImage[i][UpInflectionC->X]==IMAGE_WHITE && BinaryImage[i-1][UpInflectionC->X]==IMAGE_BLACK)
        {
            for(cloumnL=UpInflectionC->X;cloumnL>10;cloumnL--)
            {
                if(BinaryImage[i-1][cloumnL]==IMAGE_WHITE)
                    break;
                if(cloumnL==11)//如果起始的列就小于了11，那么则不会return，会直接到后面的赋值
                    return;//遍历完了都没有找到白的即不是三岔，退出判断
            }
            for(cloumnR=UpInflectionC->X;cloumnR<MT9V03X_W-10;cloumnR++)
            {
                if(BinaryImage[i-1][cloumnR]==IMAGE_WHITE)
                    break;
                if(cloumnR==MT9V03X_W-11)
                    return;//遍历完了都没有找到白的即不是三岔，退出判断
            }
            UpInflectionC->Y=i;//Y坐标是行数
            return;
        }
    }
}

/********************************************************************************************
 ** 函数功能: 识别三岔
 ** 参    数: int startline:用户决定的起始行
 **           int endline:用户决定的结束行（表示对前几段的识别，根据速度不同进行调整）
 **           int *LeftLine：左线
 **           int *RightLine:右线
 **           Point *InflectionL:左边拐点
 **           Point *InflectionR:右边拐点
 **           Point *InflectionC:中间拐点
 ** 返 回 值:  0：没有识别到环岛
 **           1：正入三岔
 ** 作    者: LJF
 ** 注    意：1 . 目前仅仅是正入三岔的时候的函数，因为三岔前面都会有个弯道所以会出现车身斜的情况，此时的左右拐点并不一定都存在
 **           2.这个是进三岔的函数，出三岔时候应该重写一个，并在进入三岔后再开启出三岔的判断
 *********************************************************************************************/
uint8 ForkIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR)
{
    Point UpInflectionC;
    if(DownInflectionL.X!=0 && DownInflectionL.Y>60 && DownInflectionR.X!=0 && DownInflectionR.Y>60)//当左右拐点存在,且左右拐点不会太快出现丢线情况
    {
          //取消这个左右拐点行数的判断，增加运算速率
        if(abs((DownInflectionL.Y-DownInflectionR.Y))<40)//左右两个拐点的行数小于30，才进行判断
        {
            GetForkUpInflection(DownInflectionL, DownInflectionR, &UpInflectionC);//去搜索上拐点
            if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
            {
                FillingLine('R',DownInflectionR,UpInflectionC);//三岔成立了就在返回之前补线
                Bias=DifferentBias(DownInflectionR.Y,UpInflectionC.Y,CentreLine);//因为这里距离进入三岔还有一段距离，我怕打角太多，所以还是按照原来的方法
                diff_speed_kp=0.05;
                return 1;//三个拐点存在三岔成立：正入三岔
            }
        }
        else
            return 0;
    }
    else if(DownInflectionL.X==0 && DownInflectionR.X==0)//如果左右下拐点不存在并且下面一段出现就丢线的话的话,我们就去看存不存在正上的拐点
    {
        Point ImageDownPointL,ImageDownPointR;//以画面的左下角和右下角作为左右补线的点
        ImageDownPointL.X=0,ImageDownPointL.Y=MT9V03X_H,ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.Y=MT9V03X_H;
        GetForkUpInflection(ImageDownPointL, ImageDownPointR, &UpInflectionC);
        if(UpInflectionC.Y!=0 && UpInflectionC.Y>40)//直接访问Y即可，加快速度，因为X默认就会赋值了
        {
            FillingLine('R',ImageDownPointR,UpInflectionC);//三岔成立了就在返回之前补线
            //在此处就对偏差进行计算，就可以避免仅有一部分中线被补线到的问题，同时外部使用一个标志变量识别到了之后这一次则不进行外面自定义的前瞻偏差计算
            //这一次是越过了三岔很接近冲出三岔的拐角，我们手动把补到的线计算出来的bias扩大
            Bias=DifferentBias(ImageDownPointR.Y,UpInflectionC.Y,CentreLine)*1.5;
            diff_speed_kp=0.05;
            return 1;//三岔正入丢失左右拐点那一帧
        }
    }
    //左拐点x[0,70),加上列数不能太右边，避免极端情况影响了
    else if(LostNum_RightLine>=60 && DownInflectionL.X!=0)
    {
        Point ImageDownPointR;//以左拐点对称的点去补线和找拐点
        ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.Y=DownInflectionL.Y;
        GetForkUpInflection(DownInflectionL, ImageDownPointR, &UpInflectionC);
        if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
        {
            FillingLine('R',ImageDownPointR,UpInflectionC);//三岔成立了就在返回之前补线
            Bias=DifferentBias(ImageDownPointR.Y,UpInflectionC.Y,CentreLine);//在此处就对偏差进行计算，就可以避免仅有一部分中线被补线到的问题，同时外部使用一个标志变量识别到了之后这一次则不进行外面自定义的前瞻偏差计算
            diff_speed_kp=0.05;
            return 1;//三岔左斜入三岔
        }
    }
    //右拐点x[0,70)
    else if(LostNum_LeftLine>=60 && DownInflectionR.X!=0)
    {
        Point ImageDownPointL;//以左拐点对称的点去补线和找拐点
        ImageDownPointL.X=5,ImageDownPointL.Y=DownInflectionR.Y;
        GetForkUpInflection(ImageDownPointL, DownInflectionR, &UpInflectionC);
        if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
        {
            FillingLine('R',DownInflectionR,UpInflectionC);//三岔成立了就在返回之前补线
            Bias=DifferentBias(DownInflectionR.Y,UpInflectionC.Y,CentreLine);//在此处就对偏差进行计算，就可以避免仅有一部分中线被补线到的问题，同时外部使用一个标志变量识别到了之后这一次则不进行外面自定义的前瞻偏差计算
            diff_speed_kp=0.05;
            return 1;//三岔右斜入三岔
        }
    }
    return 0;
}

/********************************************************************************************
 ** 函数功能: 三岔状态跳转判断函数
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值:  0：三岔还未结束
 **           1：三岔已结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 ForkFStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 NowFlag)
{
    static uint8 StatusChange,num1,num3,numspecial;//三岔识别函数的零食状态变量，用来看状态是否跳转

    if(numspecial<200)//防止很久都没有出现进入入口的状态，及时去判断出口
    {
        numspecial++;
    }
    else if(StatusChange<1)//判断状态有没有度过入口状态，若没有则强制跳过
    {
        StatusChange=2;
    }

    switch(StatusChange)
    {
        //入口状态
        case 0:
        {
            if(NowFlag==1)
            {
                StatusChange=1;//只要开始识别到了三岔就说明已经是入口阶段了
            }
            break;
        }
        //中途状态
        case 1:
        {
            if(num1<120)  //给足够长的时间让车走到三岔运行中
            {
                num1++;
                break;
            }
            else if(NowFlag==0)
            {
                StatusChange=2;//过了中间过度态之后跳转至检测出口
            }
            break;
        }
        //出口状态
        case 2:
        {
            if(NowFlag==1)
            {
                StatusChange=3;
            }
            break;
        }
        //确保已经出三岔了，否则三岔口就出三岔了，使得出三岔其实是扫线出的
        case 3:
        {
            if(num3<35)  //给足够长的时间让车走出三岔中
            {
                num3++;
                break;
            }
            else
            {
                return 1;
            }
        }
        default:break;
    }
    return 0;
}

/********************************************************************************************
 ** 函数功能: 三岔状态跳转判断函数
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值:  0：三岔还未结束
 **           1：三岔已结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 ForkSStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 NowFlag)
{
    static uint8 StatusChange,num1,num3,numspecial;//三岔识别函数的零食状态变量，用来看状态是否跳转

    if(numspecial<255)//防止很久都没有出现进入入口的状态，及时去判断出口
    {
        numspecial++;
    }
    else if(StatusChange<1)//判断状态有没有度过入口状态，若没有则强制跳过
    {
        StatusChange=2;
    }

    switch(StatusChange)
    {
        //入口状态
        case 0:
        {
            if(NowFlag==1)
            {
                SteerK.P=18.25;//减小KP进入三岔
                StatusChange=1; //只要开始识别到了三岔就说明已经是入口阶段了
            }
            break;
        }
        //中途状态
        case 1:
        {
            if(num1<100)  //给足够长的时间让车走到三岔运行中
            {
                if(num1==50)
                {
                    SteerK.D=3;//进入直到之后D减小防止振荡
                    base_speed+=15; //进入三岔提速，确保是正常进入的三岔才会触发
                }
                num1++;
                break;
            }
            if(NowFlag==0)
            {
                StatusChange=2; //过了中间过度态之后跳转至检测出口
                base_speed-=5; //检测出口减速
            }
            break;
        }
        //出口状态
        case 2:
        {
            if(NowFlag==1)
            {
                StatusChange=3;
            }
            break;
        }
        //确保已经出三岔了，否则三岔口就出三岔了，使得出三岔其实是扫线出的
        case 3:
        {
            if(num3<50)
            {
                num3++;
                break;
            }
            else
            {
                SteerK.P=19.25;//还原KP
                return 1;
            }
        }
        default:break;
    }
    return 0;
}
/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandBegin_R(int *LeftLine,int *RightLine)
{
    if(BinaryImage[115][154]==IMAGE_BLACK)    //防止提前拐入环岛
    {
        return 0;
    }
    for(uint8 row=60;row-1>0;row--)          //防止在Flag1处误判
    {
        if(BinaryImage[row][150]==IMAGE_BLACK&&BinaryImage[row-1][150]==IMAGE_WHITE) //黑跳白
        {
            return 0;
        }
    }
    //环岛入口在右边
    if(LostNum_RightLine>C_LOSTLINE)   //右边丢线：环岛入口在右边
    {
        for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查右边界线
        {
            if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)    //该行丢线而下一行不丢线
            {
                //下面这个防止进入环岛后误判
                for(int column=MT9V03X_W-1;column-1>0;column--) //向左扫
                {
                    if(BinaryImage[30][column]!=BinaryImage[30][column-1])
                    {
                        Point StarPoint,EndPoint;   //定义补线的起点和终点
                        EndPoint.Y=row;             //终点赋值
                        EndPoint.X=RightLine[row];
                        StarPoint.Y=MT9V03X_H-1;    //起点赋值
                        StarPoint.X=0;
                        FillingLine('L',StarPoint,EndPoint);    //补线
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口，右侧，补线直行通过
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandOverBegin_R(int *LeftLine,int *RightLine)
{
    //环岛入口在右边
    for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查右边界线
    {
        if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)    //该行丢线而下一行不丢线
        {
            //下面这个防止进入环岛后误判
            for(int column=MT9V03X_W-1;column-1>0;column--) //向左扫
            {
                if(BinaryImage[30][column]!=BinaryImage[30][column-1])
                {
                    if(row-10>0)
                    {
                        row-=10;
                    }
                    Point StarPoint,EndPoint;   //定义补线的起点和终点
                    EndPoint.Y=row;             //终点赋值
                    EndPoint.X=RightLine[row];
                    StarPoint.Y=MT9V03X_H-1;    //起点赋值
                    StarPoint.X=MT9V03X_W-1;
                    FillingLine('R',StarPoint,EndPoint);    //补线
                    return 1;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛出口，右侧
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛出口且在车身右侧
 ** 作    者: WBN
 ** 注    意：该函数调用时应确保小车已在环岛中
 ********************************************************************************************
 */
uint8 CircleIslandEnd_R(void)
{
    if(LostNum_RightLine>110)    //防止还未出环岛的误判
    {
        return 0;
    }
    if(BinaryImage[115][155]==IMAGE_BLACK)  //防止刚进环岛的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>C_LOSTNUM&&LostNum_RightLine>C_LOSTNUM)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            /*在这里将舵机打死，考虑要不要加延时*/
            systick_delay_ms(STM0,50);   //防止切内环
            Bias=-10;
            systick_delay_ms(STM0,200);
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag1是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：Flag1不成立
 **           1：Flag1成立且在右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionR.Y<60)    //防止还在下坡时的误判
    {
        return 0;
    }
    if(InflectionR.X!=0&&InflectionR.Y!=0)  //判断条件一：是否存在右拐点与左侧直道
    {
        float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
        if(fabsf(bias_left)<G_LINEBIAS)    //左边界为直道
        {
            for(int row=InflectionR.Y;row-1>0;row--)    //从右拐点开始向上扫
            {
                if(BinaryImage[row][InflectionR.X]==IMAGE_WHITE&&BinaryImage[row-1][InflectionR.X]==IMAGE_BLACK)
                {
                    uint8 row_f=row;
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][InflectionR.X]==IMAGE_BLACK&&BinaryImage[row-1][InflectionR.X]==IMAGE_WHITE)
                        {
                            row=(row_f+row)/2;
                            for(uint8 column=InflectionR.X;column>0;column--)   //向左扫
                            {
                                if(BinaryImage[row][column]==IMAGE_WHITE)
                                {
                                    /*补线操作*/
                                    Point end;
                                    end.Y=row;
                                    end.X=column;
                                    FillingLine('R', InflectionR, end);   //补线
                                    return 1;
                                }
                            }
                        }
                    }
                    return 0;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag1_1是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag1_1不成立
 **           1：Flag1_1成立且在右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_1_1_R(int *LeftLine,int *RightLine)
{
    float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
    if(fabsf(bias_left)<G_LINEBIAS)    //左边界为直道
    {
        for(uint8 row=MT9V03X_H-20,column=MT9V03X_W-20;row-1>0;row--)    //向上扫
        {
            if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
            {
                uint8 row_f=row;
                for(;row-1>0;row--) //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                    {
                        row=(row+row_f)/2;
                        for(;column>0;column--)   //向左扫
                        {
                            if(BinaryImage[row][column]==IMAGE_WHITE)
                            {
                                /*补线操作*/
                                Point start,end;
                                end.Y=row-1;
                                end.X=column;
                                start.Y=MT9V03X_H-1;    //右下
                                start.X=MT9V03X_W-1;
                                FillingLine('R', start, end);   //补线
                                return 1;
                            }
                        }

                    }
                }
                return 0;
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag2是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag2不成立
 **           1：Flag2成立且在右侧
 ** 作    者: WBN
 ** 注    意：上坡路段仍会误判
 ********************************************************************************************
 */
uint8 CircleIsFlag_2_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    float bias_left=Regression_Slope(119,0,LeftLine);   //求出左边界线斜率
    if(fabsf(bias_left)<G_LINEBIAS&&LostNum_RightLine<50)    //左边界为直道且右边丢线小于
    {
        if(BinaryImage[100][149]==IMAGE_BLACK)    //经验位置为黑
        {
            //下面这个for防止在环岛出口时误判为环岛中部
            for(int row=80;row+1<MT9V03X_H-1;row++) //向下扫
            {
                if(RightLine[row]==MT9V03X_W-1&&RightLine[row+1]!=MT9V03X_W-1)    //丢线-不丢线
                {
                    return 0;
                }
            }
            for(int row=80;row-1>0;row--)  //向上扫
            {
                if(RightLine[row]!=MT9V03X_W-1&&RightLine[row-1]==MT9V03X_W-1)    //不丢线-丢线
                {
                    for(;row-1>0;row--)    //继续向上扫
                    {
                        if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)    //丢线-不丢线
                        {
                            return 1;
                        }
                    }
                    break;
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断环岛Flag3是否成立，右侧
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：Flag3不成立
 **           1：Flag3成立且在右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIsFlag_3_R(void)
{
    if(LostNum_RightLine>100)    //右边接近全丢线
    {
        //下面采用经验值随机抽样法
        if(BinaryImage[90][119]==IMAGE_WHITE&&BinaryImage[60][20]==IMAGE_BLACK)
        {
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：还未完成环岛
 **           1：完成环岛
 ** 作    者: WBN
 ** 注    意：这里只有环岛在小车右边的情况
 ********************************************************************************************
 */
uint8 CircleIslandIdentify_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    static uint8 flag,num_1,num_2,flag_begin,flag_last_begin,flag_last2_begin;
    //使用switch实现简单的状态机机制
    switch(flag)
    {
        case 0: //此时小车未到达环岛，开始判断环岛出口部分路段，这里需要补线
        {
            if(CircleIsFlag_1_R(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别环岛出口，进行补线
            {
                if(num_1<100)
                {
                    num_1++;    //识别到flag1的帧数++
                }
            }
            else
            {
                CircleIsFlag_1_1_R(LeftLine, RightLine);
            }
            // else    //没有识别到环岛出口
            // {
            //     if(num_2<100)
            //     {
            //         num_2++;    //没有识别到flag1的帧数++
            //     }
            //     else    //超过100帧没有识别到环岛flag1
            //     {
            //         num_1=0;
            //         num_2=0;
            //     }
            // }
            if(CircleIsFlag_2_R(LeftLine, RightLine, InflectionL, InflectionR)==1)    //识别到环岛中部
            {
                if(num_1>0) //在此之前有识别到环岛flag1
                {
                    num_1=0;
                    num_2=0;
                    flag=1; //跳转到状态1
                }
            }
            break;
        }
        case 1: //此时小车到达环岛中部，开始判断环岛入口并完成入环，这里需要补线
        {
            if(CircleIslandBegin_R(LeftLine, RightLine)==1)
            {
                if(num_1<100)
                {
                    num_1++;    //识别到环岛入口的帧数++
                }
            }
            else
            {
                if(num_2<100)
                {
                    num_2++;    //识别不到环岛入口的帧数++
                }
                else    //超过100帧识别不到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=0; //跳转回到状态0
                }
            }
            if(CircleIsFlag_3_R()==1)    //识别已经进入环岛
            {
                if(num_2>2) //在此之前有识别到环岛入口
                {
                    num_1=0;
                    num_2=0;
                    flag=2;
                }
            }
            break;
        }
        case 2: //此时小车已经在环岛中，开始判断环岛出口
        {
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            while(CircleIslandEnd_R()==0)  //识别到环岛出口跳出循环
            {
                if(mt9v03x_finish_flag)
                {
                    ImageBinary();                                  //图像二值化
                    GetImagBasic(LeftLine,CentreLine,RightLine);    //基本扫线
                    Bias=DifferentBias(110,60,CentreLine);          //计算偏差，此时在环岛中取特殊前瞻
                    mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
                }
            }
            mt9v03x_finish_flag = 0;//在图像使用完毕后务必清除标志位，否则不会开始采集下一幅图像
            flag=3;
            break;
        }
        case 3:
        {
            flag_begin=CircleIslandOverBegin_R(LeftLine, RightLine);
            if(flag_begin==0&&flag_last_begin==0&&flag_last2_begin==1)   //上上次识别到环岛入口而这两次都没有识别到环岛入口
            {
                flag=4;
                return 1;   //退出状态机
            }
            flag_last2_begin=flag_last_begin;   //保存上上次的状态
            flag_last_begin=flag_begin; //保存上一次的状态
            break;
        }
        default:break;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第一个十字回环出口
 ** 参    数: 无
 ** 返 回 值: 0：没有识别到十字回环出口
 **           1：识别到十字回环出口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopEnd_F(void)
{
    for(uint8 row=10;row+1<40;row++)  //向下扫
    {
        if(BinaryImage[row][30]==IMAGE_BLACK&&BinaryImage[row+1][30]==IMAGE_WHITE)
        {
            return 0;
        }
    }
    if(LostNum_LeftLine>110)    //防止还未出环的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>L_LOSTNUM&&LostNum_RightLine>L_LOSTNUM)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            //舵机向右打死并加上一定的延时实现出弯
            Bias=-10;
            diff_speed_kp+=0.2; //增大差速
            systick_delay_ms(STM0,300);
            diff_speed_kp-=0.2; //恢复差速
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第二个十字回环出口
 ** 参    数: 无
 ** 返 回 值: 0：没有识别到十字回环出口
 **           1：识别到十字回环出口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopEnd_S(void)
{
    //防止三岔误判，判断是十字回环出口，这个条件的成立是建立在十字回环用路肩挡起来
//    uint8 row_1=0,flag=0;
//    for(uint8 row=65;row-1>0;row--)    //中间向上扫
//    {
//        if(BinaryImage[row][MT9V03X_W/2]==IMAGE_WHITE&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_BLACK)
//        {
//            for(;row-1>0;row--) //继续向上扫
//            {
//                if(BinaryImage[row][MT9V03X_W/2]==IMAGE_BLACK&&BinaryImage[row-1][MT9V03X_W/2]==IMAGE_WHITE)
//                {
//                    if(row_1-row<10)    //约束两个黑白跳变点之间的距离
//                    {
//                        flag=1;
//                    }
//                }
//            }
//            break;  //这里的break可以滤去远处的干扰
//        }
//    }
//    if(flag==0)
//    {
//        return 0;
//    }
    //防三岔误判，判断是三岔，这个条件的成立是建立在十字回环用蓝布挡起来
//    uint8 flag_l=0,flag_r=0;
//    for(uint8 row=100,column=MT9V03X_W/2;row-1>20;row--)    //向上扫
//    {
//        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
//        {
//            for(row-=3;column>0;column--)                       //向左扫
//            {
//                if(BinaryImage[row][column]==IMAGE_WHITE)   //向左找到白
//                {
//                    flag_l=1;
//                    break;
//                }
//            }
//            for(column=MT9V03X_W/2;column<MT9V03X_W-1;row++)    //向右扫
//            {
//                if(BinaryImage[row][column]==IMAGE_WHITE)   //向右找到白
//                {
//                    flag_r=1;
//                    break;
//                }
//            }
//            if(flag_l==1&&flag_r==1)
//            {
//                return 0;
//            }
//            break;
//        }
//    }
    for(uint8 row=MT9V03X_H/2;row+1<MT9V03X_H-1;row++)  //防止出三岔急弯误判
    {
        if(BinaryImage[row][155]==IMAGE_BLACK&&BinaryImage[row][155]==IMAGE_WHITE)  //黑-白
        {
            return 0;
        }
    }
    if(LostNum_LeftLine>110)    //防止还未出环的误判
    {
        return 0;
    }
    if(LostNum_LeftLine>L_LOSTNUM&&LostNum_RightLine>L_LOSTNUM)  //左右边界均丢线
    {
        if(fabsf(Bias)<1.5)
        {
            //舵机向右打死并加上一定的延时实现出弯
            gpio_set(LED_GREEN, 0);
            base_speed=140;     //降速出环
            Bias=-10;
            diff_speed_kp+=0.1; //增大差速
            systick_delay_ms(STM0,300);
            diff_speed_kp-=0.1; //恢复差速
            return 1;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第一个十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **           1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopBegin_F(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0&&InflectionR.X==0&&InflectionR.Y==0)    //存在左拐点且不存在右拐点
    {
        uint8 row_up=0;
        for(uint8 row=InflectionL.Y+2,column=InflectionL.X-2;row-1>0;row--)  //左拐点往上扫
        {
            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
            {
                for(;row-1>0;row--)   //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                    {
                        row_up=row; //保存补线的终点Y坐标
                        for(;row-1>0;row--)   //继续向上扫
                        {
                            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
                            {
                                for(;row-1;row--)   //继续向上扫
                                {
                                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                                    {
                                        for(row_up-=5;column<MT9V03X_W;column++)
                                        {
                                            if(BinaryImage[row_up][column]==IMAGE_WHITE)
                                            {
                                                Point end;
                                                end.Y=row_up;
                                                end.X=column;
                                                FillingLine('L', InflectionL, end); //补线
                                                return 1;
                                            }
                                        }
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    if(LostNum_LeftLine>70&&LostNum_RightLine<35)   //无拐点但左右丢线符合
    {
        float right_bias=0;
        right_bias=Regression_Slope(110, 60, RightLine);    //求右边线斜率
        if(fabsf(right_bias)>0.6)   //防止进环后的误判
        {
            return 0;
        }
        for(uint8 row=0;row<MT9V03X_H-1;row++)  //向下扫
        {
            if(BinaryImage[row][20]==IMAGE_BLACK&&BinaryImage[row+1][20]==IMAGE_WHITE)  //黑-白
            {
                for(;row<MT9V03X_H-1;row++) //继续向下扫
                {
                    if(BinaryImage[row][20]==IMAGE_WHITE&&BinaryImage[row+1][20]==IMAGE_BLACK)  //白-黑
                    {
                        for(;row<MT9V03X_H-1;row++) //继续向下扫
                        {
                            if(BinaryImage[row][20]==IMAGE_BLACK&&BinaryImage[row+1][20]==IMAGE_WHITE)  //黑-白
                            {
                                //寻找补线点
                                for(uint8 row=100;row-1>0;row--)    //向上扫
                                {
                                    if(LeftLine[row]==0&&LeftLine[row-1]!=0)
                                    {
                                        if(row-5>0)
                                        {
                                            row-=5;
                                        }
                                        Point start,end;
                                        start.Y=119;
                                        start.X=LeftLine[row];
                                        end.Y=row;
                                        end.X=LeftLine[row]+1;  //不能补垂直的线，稍作偏移
                                        FillingLine('L', start, end);   //补线
                                        return 1;
                                    }
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别第二个十字回环入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到十字回环入口
 **           1：识别到十字回环入口
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopBegin_S(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0)    //存在左拐点
    {
        uint8 row_up=0;
        for(uint8 row=InflectionL.Y+2,column=InflectionL.X-2;row-1>0;row--)  //左拐点往上扫
        {
            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
            {
                for(;row-1>0;row--)   //继续向上扫
                {
                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                    {
                        row_up=row; //保存补线的终点Y坐标
                        for(;row-1>0;row--)   //继续向上扫
                        {
                            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)  //黑-白
                            {
                                for(;row-1;row--)   //继续向上扫
                                {
                                    if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)  //白-黑
                                    {
                                        for(row_up-=5;column<MT9V03X_W;column++)
                                        {
                                            if(BinaryImage[row_up][column]==IMAGE_WHITE)
                                            {
                                                Point end;
                                                end.Y=row_up;
                                                end.X=column;
                                                FillingLine('L', InflectionL, end); //补线
                                                return 1;
                                            }
                                        }
                                        return 0;
                                    }
                                }
                                return 0;
                            }
                        }
                        return 0;
                    }
                }
                return 0;
            }
        }
    }
    float right_bias=0;
    right_bias=Regression_Slope(110, 60, RightLine);    //求右边线斜率
    if(fabsf(right_bias)>0.6)   //防止进环后的误判
    {
        return 0;
    }
    for(uint8 row=MT9V03X_H-20,column=20;row-1>0;row--) //向上扫
    {
        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
        {
            uint8 row_f=row;
            for(;row-1>0;row--) //继续向上扫
            {
                if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column]==IMAGE_WHITE)
                {
                    for(;row-1>0;row--) //继续向上扫
                    {
                        if(BinaryImage[row][column]==IMAGE_WHITE&&BinaryImage[row-1][column]==IMAGE_BLACK)
                        {
                            for(row=row_f;column+1<MT9V03X_W-1;column++)
                            {
                                if(BinaryImage[row-5][column]==IMAGE_WHITE)
                                {
                                    Point end,start;
                                    end.Y=row_f;
                                    end.X=column;
                                    start.Y=119;
                                    start.X=0;
                                    FillingLine('L', start, end); //补线
                                    return 1;
                                }
                            }
                            return 0;
                        }
                    }
                    return 0;
                }
            }
            return 0;
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 判断是否已经进入第二个回环
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：还未进入回环
 **           1：已经进入回环
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CrossLoopIn_S(void)
{
    float left_slope=Regression_Slope(119, 40, RightLine);
    //下面采用经验值随机抽样法
    if(left_slope>1)
    {
        return 1;
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 完成出库（开环）
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void OutGarage(void)
{
//    systick_delay_ms(STM0,50);
    //舵机向右打死并加上一定的延时实现出库
    Bias=-10;
    diff_speed_kp=0.1;
    systick_delay_ms(STM0,300);
    diff_speed_kp=0.05;
}
