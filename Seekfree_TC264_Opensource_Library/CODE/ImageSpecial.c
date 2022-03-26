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

uint8 circle_island_flag=0;   //环岛判断条件标志变量    //由于环岛的判断不是一帧图片，所以需要全局变量以保留上一帧图片的信息
uint8 circle_island_num=0;    //环岛判断条件的计数值    //防止距离过远的几帧图片联合形成误判
/*
 *******************************************************************************************
 ** 函数功能: 识别起跑线
 ** 参    数: *LeftLine：  左线数组
 **           *RightLine：右线数组
 **           InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到起跑线
 **           1：识别到起跑线且车库在车左侧
 **           2：识别到起跑线且车库在车右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 GarageIdentify(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    //车库在小车左侧
    if(InflectionL.X!=0&&InflectionL.Y!=0)  //左拐点存在
    {
        float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
        if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
        {
            int row=0;          //固定列扫描的行数
            int zebra_num=0;    //斑马线标志的数量
            if(InflectionL.X-G_HIGH<0)  //防止行数越界
            {
                return 0;   //如果存在越界那只能是某种不知名错误
            }
            row=InflectionL.X-G_HIGH;
            for(int column=RightLine[row];column-1>0;column--)    //固定行，从右到左列扫描
            {
                if(BinaryImage[row][column]!=BinaryImage[row][column-1])    //该点与下一个点不同颜色 //存在黑白跳变点
                {
                    zebra_num++;    //斑马线标志+1
                }
                if(zebra_num>G_ZEBRA_NUM)   //斑马线标志的数量高于阈值
                {
                    return 1;       //返回车库在左边
                }
            }
        }
    }
    //车库在小车右侧
    if(InflectionR.X!=0&&InflectionR.Y!=0)  //右拐点存在（车库在右边）
    {
        float bias_left=Regression_Slope(119,0,LeftLine);   //求出右边界线斜率
        if(fabsf(bias_left)<G_LINEBIAS)    //右边界为直道
        {
            int row=0;          //固定列扫描的行数
            int zebra_num=0;    //斑马线标志的数量
            if(InflectionR.X-G_HIGH<0)  //防止行数越界
            {
                return 0;   //如果存在越界那只能是某种不知名错误
            }
            row=InflectionR.X-G_HIGH;
            for(int column=LeftLine[row];column-1>0;column--)    //固定行，从右到左列扫描
            {
                if(BinaryImage[row][column]!=BinaryImage[row][column-1])    //该点与下一个点不同颜色 //存在黑白跳变点
                {
                    zebra_num++;    //斑马线标志+1
                }
                if(zebra_num>G_ZEBRA_NUM)   //斑马线标志的数量高于阈值
                {
                    return 2;       //返回车库在右边
                }
            }
        }
    }
    return 0;
}

/*
 *******************************************************************************************
 ** 函数功能: 识别环岛入口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
 **           2：识别到环岛且在车身右侧
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 CircleIslandBegin(int *LeftLine,int *RightLine)
{
    //环岛入口在左边
    if(LostNum_LeftLine>C_LOSTLINE)   //左边丢线：环岛入口在左边
    {
        for(int row=MT9V03X_H;row-1>0;row--)  //从下往上检查左边界线
        {
            if(LeftLine[row]==0&&LeftLine[row-1]!=0&&row>C_INROW)    //该行丢线而下一行不丢线且该行不会太远   //C_INROW用于防止误判急拐弯的情况
            {
                float bias_rightline=Regression_Slope(row,20,RightLine);   //求出右边界线的斜率
                if(fabsf(bias_rightline)<C_LINEBIAS)    //右边界为直道
                {
                    Point StarPoint,EndPoint;   //定义补线的起点和终点
                    EndPoint.Y=row;             //终点赋值
                    EndPoint.X=LeftLine[row];
                    StarPoint.Y=120;            //起点赋值
                    StarPoint.X=MT9V03X_W-1;
                    //下面这部分代码防止误判环岛的入口为出口
                    int c_left_flag=0;
                    for(;row-1>0;row--)
                    {
                        if(LeftLine[row]==0&&LeftLine[row-1]==0)    //又出现丢线情况
                        {
                            c_left_flag=1;
                        }
                    }
                    if(c_left_flag==0)
                    {
                        FillingLine(LeftLine, CentreLine, RightLine,StarPoint,EndPoint);    //补线
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
 ** 函数功能: 识别环岛出口
 ** 参    数: InflectionL：左下拐点
 **           InflectionR：右下拐点
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛出口且在车身左侧
 **           2：识别到环岛出口且在车身右侧
 ** 作    者: WBN
 ** 注    意：该函数调用时应确保小车已在环岛中
 ********************************************************************************************
 */
uint8 CircleIslandEnd(Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0&&InflectionR.X!=0&&InflectionR.Y!=0)  //左右拐点均存在
    {
        /*在这里将舵机打死，考虑要不要加延时*/

        return 1;
    }
    return 0;
}

/*下面三个函数是判断环岛整个过程的三个状态转移函数*/
//判断条件一是否成立，成立返回1，不成立返回0
uint8 CircleIsFlag_1(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    if(InflectionL.X!=0&&InflectionL.Y!=0)  //判断条件一：是否存在左拐点与右侧直道
    {
        float bias_right=Regression_Slope(119,0,RightLine);   //求出右边界线斜率
        if(fabsf(bias_right)<G_LINEBIAS)    //右边界为直道
        {
            return 1;   //左拐点与直道并存，条件一成立
        }
    }
    return 0;
}
//判断条件二是否成立，成立返回1，不成立返回0
uint8 CircleIsFlag_2(int *LeftLine,int *RightLine)
{
    /*寻找圆环的存在，即存在上下上下不丢线中间丢线的情况*/
    for(int row=MT9V03X_H/2;row-1>0;row--)               //从屏幕中间往上扫
    {
        if(LeftLine[row]==0&&LeftLine[row-1]!=0)         //左边界线上这一行丢线下一行不丢线
        {
            for(row=MT9V03X_H/2;row+1<MT9V03X_H-1;row++) //从屏幕中间往下扫
            {
                if(LeftLine[row]==0&&LeftLine[row+1]!=0) //左边界线上这一行丢线下一行不丢线
                {
                    return 1;   //找到被圆环包裹的切点，条件二成立
                }
            }
        }
    }
    return 0;
}
//判断条件三是否成立，成立返回1，不成立返回0
uint8 CircleIsFlag_3(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    /*首先找到左拐点，然后判断左拐点上面是否有黑点*/
    if(InflectionL.X!=0&&InflectionL.Y!=0)  //存在左拐点
    {
        for(int row=InflectionL.Y,column=LeftLine[InflectionL.X];row-1>0;row--)  //从拐点向上扫线
        {
            if(BinaryImage[row][column]==IMAGE_BLACK&&BinaryImage[row-1][column])
            {
                return 1;
            }
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
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到环岛且在车身左侧
 **           2：识别到环岛且在车身右侧
 ** 作    者: WBN
 ** 注    意：这里只有环岛在小车左边的情况
 **           该环岛的判断方法使用一个循环来解决，即若发现环岛入口则会卡死在该函数里直到小车出环岛，
 **           在这一段时间中，理论上只有定时器中断中的函数会执行，但是这样又有一个问题：进入环岛后
 **           主循环中的扫线更新Bias部分就不会执行，也就是Bias不会更新，所以需要我们在这个函数中加
 **           入扫线更新Bias的部分直到出环岛return出这个函数
 **           存在一个问题：三个flag之间的连续性判断没有做处理，可能出现三个flag在不同地方验证的情况
 ********************************************************************************************
 */
uint8 CircleIslandIdentify(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    //使用switch实现简单的状态机机制
    switch(circle_island_flag)
    {
        case 0:
        {
            if(CircleIsFlag_1(LeftLine, RightLine, InflectionL, InflectionR)==1)
            {
                circle_island_flag=1;   //跳转到下个状态
            }
            break;
        }
        case 1:
        {
            if(CircleIsFlag_2(LeftLine, RightLine)==1)
            {
                circle_island_flag=2;   //跳转到下个状态
                circle_island_num=0;    //重置计数值
            }
            else
            {
                circle_island_num++;
                if(circle_island_num>C_NUM) //计数值超过阈值
                {
                    circle_island_flag=0;   //重置状态
                    circle_island_num=0;    //重置计数值
                }
            }
            break;
        }
        case 2:
        {
            if(CircleIsFlag_3(LeftLine, RightLine, InflectionL, InflectionR)==1)
            {
                circle_island_flag=3;   //跳转到下个转态
                circle_island_num=0;    //重置计数值
            }
            else
            {
                circle_island_num++;
                if(circle_island_num>C_NUM)
                {
                    circle_island_flag=0;   //重置状态
                    circle_island_num=0;    //重置计数值

                }
            }
            break;
        }
        case 3:
        {
            while(CircleIslandEnd(InflectionL,InflectionR)==0)  //识别到环岛出口跳出循环
            {
                ImageBinary();                                  //图像二值化
                GetImagBasic(LeftLine,CentreLine,RightLine);    //基本扫线
                Bias=DifferentBias(100,60,CentreLine);          //计算偏差
            }
            circle_island_flag=0;   //重置状态
            circle_island_num=0;    //重置计数值
            break;
        }
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 识别十字回环出口
 ** 参    数: LeftLine：左线数组
 **           RightLine：右线数组
 **           InflectionL：左拐点
 **           InflectionR：右拐点
 ** 返 回 值: 0：没有识别十字回环出口
 **           1：识别到十字回环出口
 ** 作    者: WBN
 ** 注    意：传入的拐点需确保：若该图不存在拐点则拐点的数据均为0
 ********************************************************************************************
 */
uint8 CrossLoopEnd(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR)
{
    int row;  //行

    if(InflectionL.X!=0&&InflectionL.Y!=0)  //拐点在左边
    {
        for(row=InflectionL.Y;row>0;row--)
        {
            if(LeftLine[row]==0&&LeftLine[row-1]!=0)  //该行丢线而下一行不丢线
            {
                return 1;
            }
        }
    }
    if(InflectionR.X!=0&&InflectionR.Y!=0)  //拐点在右边
    {
        for(row=InflectionR.Y;row>0;row--)
        {
            if(RightLine[row]==MT9V03X_W-1&&RightLine[row-1]!=MT9V03X_W-1)  //该行丢线而下一行不丢线
            {
                return 1;
            }
        }
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
                if(cloumnL==11)
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
 ** 返 回 值: 0：没有识别到环岛
 **           1：识别到三岔
 ** 作    者: LJF
 ** 注    意：1 . 目前仅仅是正入三岔的时候的函数，因为三岔前面都会有个弯道所以会出现车身斜的情况，此时的左右拐点并不一定都存在
 **           2.这个是进三岔的函数，出三岔时候应该重写一个，并在进入三岔后再开启出三岔的判断
 *********************************************************************************************/
uint8 ForkIdentify(int startline,int endline,int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR)
{
    Point UpInflectionC;
    if(DownInflectionL.X!=0 && DownInflectionR.X!=0 && LeftLine[DownInflectionL.Y-5]!=0 && RightLine[DownInflectionR.Y-5]!=MT9V03X_W-1)//当左右拐点存在,且左右拐点不会太快出现丢线情况
    {
        GetForkUpInflection(DownInflectionL, DownInflectionR, &UpInflectionC);//去搜索上拐点
        if(UpInflectionC.X!=0 && UpInflectionC.Y!=0)
        {
            FillingLine(LeftLine, CentreLine, RightLine, DownInflectionL,UpInflectionC);//三岔成立了就在返回之前补线
            return 1;//三个拐点存在三岔成立
        }
    }
    else if(DownInflectionL.X==0 && DownInflectionR.X==0 && LeftLine[MT9V03X_H-20]==0 && RightLine[MT9V03X_H-20]==MT9V03X_W-1)//如果左右下拐点不存在并且下面一段出现就丢线的话的话,我们就去看存不存在正上的拐点
    {
        Point ImageDownPointL,ImageDownPointR;//以画面的左下角和右下角作为左右补线的点
        ImageDownPointL.X=0,ImageDownPointL.Y=MT9V03X_H-20,ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.Y=MT9V03X_H-20;
        GetForkUpInflection(ImageDownPointL, ImageDownPointR, &UpInflectionC);
        if(UpInflectionC.X!=0 && UpInflectionC.Y!=0)
        {
            FillingLine(LeftLine, CentreLine, RightLine, ImageDownPointL,UpInflectionC);//三岔成立了就在返回之前补线
            return 1;//三个拐点存在三岔成立
        }
    }
    return 0;
}

/*********************************************************************************
 ** 函数功能: 根据左右下拐点搜寻出十字路口的左右上拐点
 ** 参    数: Point InflectionL: 左边拐点
 **           Point InflectionR: 右边拐点
 **           Point *UpInflectionC: 左边上拐点
 **           Point *UpInflectionC: 右边上拐点
 ** 返 回 值: 无
 ** 说    明: 此函数仅仅是正入十字时的一个操作函数，不是识别函数
 ** 作    者: LJF
 **********************************************************************************/
void GetCrossRoadsUpInflection(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR)
{
    int row=0;//起始行
    UpInflectionL->X=DownInflectionL.X+10;UpInflectionL->Y=0;//左上拐点置零
    UpInflectionR->X=DownInflectionR.X-10;UpInflectionR->Y=0;//右上拐点置零

    for(row=DownInflectionL.Y;row>0;row--)
    {
        //对图像数组进行检测
        if(BinaryImage[row][UpInflectionL->X]==IMAGE_WHITE && BinaryImage[row-1][UpInflectionL->X]==IMAGE_BLACK)  //由白到黑跳变
        {
            //记录上拐点
            UpInflectionL->Y=row-1;
            break;//记录完之后就退出循环
        }
    }

    for(row=DownInflectionR.Y;row>0;row--)
    {
        if(BinaryImage[row][UpInflectionR->X]==IMAGE_WHITE && BinaryImage[row-1][UpInflectionR->X]==IMAGE_BLACK)  //由白到黑跳变
        {
            //记录上拐点
            UpInflectionR->Y=row-1;
            break;//记录完之后就退出循环
        }
    }
}

/********************************************************************************************
 ** 函数功能: 识别十字路口
 ** 参    数: 左线数组：int *LeftLine
 **           右线数组：int *RightLine
 **           左下拐点：Point DownInflectionL
 **           右下拐点：Point DownInflectionR
 ** 返 回 值: 0：不是十字路口
 **           1：正入十字
 **           2：右斜入十字
 **           3：左斜入十字
 ** 作    者: LJF
 ** 注    意：无
 *********************************************************************************************/
uint8 CrossRoadsIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR)
{
    int row=0;//起始行
    Point UpInflectionL,UpInflectionR;//左右上拐点
    UpInflectionL.X=DownInflectionL.X+10;UpInflectionL.Y=0;//左上拐点置零
    UpInflectionR.X=DownInflectionR.X-10;UpInflectionR.Y=0;//右上拐点置零
    if(LostNum_LeftLine>40 && LostNum_RightLine>40 && DownInflectionR.X!=0 && DownInflectionL.X!=0 && LeftLine[DownInflectionL.X-5]==0 && RightLine[DownInflectionR.X-5]==MT9V03X_W-1)//左右两边大量丢线，并且左右下拐点都存在
    {
        GetCrossRoadsUpInflection(LeftLine, RightLine, DownInflectionL, DownInflectionR, &UpInflectionL, &UpInflectionR);
        FillingLine(LeftLine, CentreLine, RightLine, DownInflectionL, UpInflectionL);
        FillingLine(LeftLine, CentreLine, RightLine, DownInflectionR, UpInflectionR);
        return 1;//正入十字
    }
    else if(LostNum_LeftLine>60 && DownInflectionR.X!=0 && LeftLine[DownInflectionR.Y-5]==0)//左边丢线超过一半，并且右拐点上面一段对应的左边丢线
    {
        for(row=DownInflectionR.Y;row>1;row--)//直接右下拐点往上冲找到上拐点
        {
            if(BinaryImage[row][UpInflectionR.X]==IMAGE_WHITE && BinaryImage[row-1][UpInflectionR.X]==IMAGE_BLACK)  //由白到黑跳变
            {
                UpInflectionR.Y=row-1;//记录上拐点
                FillingLine(LeftLine, CentreLine, RightLine, DownInflectionR, UpInflectionR);
                break;//记录完之后就退出循环
            }
        }
        return 2;//向右斜入十字
    }
    else if(LostNum_RightLine>60 && DownInflectionL.X!=0 && RightLine[DownInflectionL.Y-5]==MT9V03X_W-1)//右边丢线超过一半，并且左拐点上面一段对应的左边丢线
    {
        for(row=DownInflectionL.Y;row>1;row--)
        {
            if(BinaryImage[row][UpInflectionL.X]==IMAGE_WHITE && BinaryImage[row-1][UpInflectionL.X]==IMAGE_BLACK)  //由白到黑跳变
            {
                UpInflectionL.Y=row-1;//记录上拐点
                FillingLine(LeftLine, CentreLine, RightLine, DownInflectionL, UpInflectionL);
                break;//记录完之后就退出循环
            }
        }
        return 3;//向左斜入十字
    }
    else return 0;
}
