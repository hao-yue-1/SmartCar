/*
 * Garage.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放车库相关的源代码
 */

#include "ImageSpecial.h"
#include "PID.h"

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
    systick_delay_ms(STM0,300);
}
