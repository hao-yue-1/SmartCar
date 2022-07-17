/*
 * Forkroads.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放三岔路口相关的源代码
 * 三岔测试最高速度为220，从环岛那边能入的比较好，从车库这边不行出界了，没跑状态机，是单独元素识别
 */

#include "ImageSpecial.h"
#include "PID.h"
#include <stdlib.h> //abs函数，fabs在math.h
#include "LED.h"
#include "zf_gpio.h"

#define L_FINDWHIDE_THRE  10 //Y拐点中间找左边白色区域停止的阈值
#define R_FINDWHIDE_THRE  150//Y拐点中间找右边白色区域停止的阈值
#define ROW_FINDWHIDE_THRE 110//Y拐点行的下限的阈值
#define CLOUMN_FINDWHIDE_THRE   10//列左右寻找白色区域的宽度，如果找不到则进入种子生长
#define SEED_TRANSVERSE_GROW_THRE    8//三岔种子生长横向生长的阈值范围，超出则默认横向生长的第一个为拐点
#define FORK_INFLECTION_WIDTH  120//打开三岔debug,当拐点在60行附近左右拐点的差值，补全的时候，依据没有丢失的拐点的行数做一个简单的比例关系到单边循迹思路上
#define FORK_DEBUG  0

extern uint8 bias_startline,bias_endline;        //动态前瞻

/*********************************************************************************
 ** 函数功能: 寻找三岔左边上顶点，防止因为扫线扫到三岔另外一边使得Bias太小
 ** 参    数: uint8 row,uint8 cloumn:去找寻虚拟补线折现上点的基准点坐标
 **           Point *LeftUpPoint:左边的特殊点
 ** 返 回 值: 无
 ** 说    明: 用完这个函数之后应该再补一条左边的垂直线和右边从上拐点补到左上角拐点
 ** 作    者: LJF
 **********************************************************************************/
void ForkFindSpecialPoint(int row,int cloumn,Point *LeftUpPoint)
{
    if(BinaryImage[row][cloumn]==IMAGE_WHITE)
    {
        for(;row>5;row--)
        {
#if FORK_DEBUG
            lcd_drawpoint(cloumn, row, PURPLE);
#endif
            if(BinaryImage[row][cloumn]==IMAGE_WHITE && BinaryImage[row-1][cloumn]==IMAGE_BLACK)
            {
                LeftUpPoint->X=cloumn,LeftUpPoint->Y=row;
                break;
            }
        }
    }
}
/*********************************************************************************
 ** 函数功能: 三岔种子生长生长至谷底寻找Y上拐点
 ** 参    数:char Choose：选择是在谷的左边还是右边
 **          Point Seed
 **          int endline
 **          Point *UpInflectionC
 ** 返 回 值: 无
 ** 说    明:
 ** 作    者: LJF
 **********************************************************************************/
void SeedGrowFindUpInflection(char Choose,Point Seed,int endline,Point *UpInflectionC)
{
    char transversenum=0;//记录种子是否一直横向移动,种子横向生长的次数
    Point tempSeed;//临时的种子
    for(;Seed.Y<endline && Seed.X<MT9V03X_W-1 && Seed.X>0;)
    {
#if FORK_DEBUG
        lcd_drawpoint(Seed.X, Seed.Y, GREEN);
#endif
        switch(Choose)
        {
            case 'L':
                if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.Y++,Seed.X++;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                    Seed.Y++;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.X++;
                    if(transversenum==0)//判断是否是第一次往右走
                    {
                        tempSeed=Seed;
                    }
                    transversenum++;//说明在往右边走
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
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
        if(transversenum>SEED_TRANSVERSE_GROW_THRE)
        {
            UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
        }
    }
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
    int row=0, cloumnL=0, cloumnR=0, cloumnnum=0;//cloumnnum记录从基准点偏移了多少列黑色区域
    Point Seed;
    char Choose=0,flagL=0,flagR=0;//判断是在谷的左边还是右边的函数,以及判断左右两边有没有白色区域的FLAG
    UpInflectionC->X = 0; UpInflectionC->Y = 0;//上拐点置零
    UpInflectionC->X = (DownInflectionL.X + DownInflectionR.X) / 2;//V型上拐点的列坐标为左右拐点均值
    //防止因为单边寻拐点使得溢出
    if(UpInflectionC->X<0) UpInflectionC->X=0;
    if(UpInflectionC->X>MT9V03X_W-1) UpInflectionC->X=MT9V03X_W-1;
    row = (DownInflectionL.Y + DownInflectionR.Y) / 2;//起始行为左右拐点行的均值
    for (; row > 20; row--)
    {
#if FORK_DEBUG
        lcd_drawpoint(UpInflectionC->X, row, PURPLE);
#endif
        if (BinaryImage[row][UpInflectionC->X] == IMAGE_WHITE && BinaryImage[row - 1][UpInflectionC->X] == IMAGE_BLACK)
        {
            //往左往右找白色区域
            for (cloumnL = UpInflectionC->X; cloumnL > L_FINDWHIDE_THRE; cloumnL--)
            {
#if FORK_DEBUG
                lcd_drawpoint(cloumnL, row-1, YELLOW);
#endif
                if(BinaryImage[row - 1][cloumnL] == IMAGE_BLACK)
                {
                    cloumnnum++;
                }
                if (BinaryImage[row - 1][cloumnL] == IMAGE_BLACK && BinaryImage[row - 1][cloumnL - 1] == IMAGE_WHITE && BinaryImage[row - 1][cloumnL - 3] == IMAGE_WHITE)
                {
                    cloumnnum=0;
                    flagL = 1;
                    break;
                }
                //条件1：循环结束了还没找到白色区域，或，条件2：黑色区域太多了大于了阈值，就算能找到白色区域补的线也不好
                if (cloumnL == L_FINDWHIDE_THRE + 1 || cloumnnum>CLOUMN_FINDWHIDE_THRE)
                {
                    cloumnnum=0;
                    Choose = 'R';//左边找不到说明在谷的右边
                    break;
                }
            }
            for (cloumnR = UpInflectionC->X; cloumnR < R_FINDWHIDE_THRE && Choose!='R'; cloumnR++)
            {
#if FORK_DEBUG
                lcd_drawpoint(cloumnR, row-1, YELLOW);
#endif
                if(BinaryImage[row - 1][cloumnR] == IMAGE_BLACK)
                {
                    cloumnnum++;
                }
                if (BinaryImage[row - 1][cloumnR] == IMAGE_BLACK && BinaryImage[row - 1][cloumnR + 1] == IMAGE_WHITE && BinaryImage[row - 1][cloumnL + 3] == IMAGE_WHITE)
                {
                    cloumnnum=0;
                    flagR = 1;
                    break;
                }
                if (cloumnR == R_FINDWHIDE_THRE - 1 || cloumnnum>CLOUMN_FINDWHIDE_THRE)
                {
                    cloumnnum=0;
                    Choose = 'L';//右边找不到说明在谷的左边
                    break;
                }
            }
            break;
        }
    }
    if ((flagL == 0 || flagR == 0) && Choose!=0)//说明有一边是没有白色区域的
    {
        Seed.X = UpInflectionC->X, Seed.Y = row - 1;
#if FORK_DEBUG
        for(int j=0;j<MT9V03X_W-1;j++)//画出100行那条线
        {
            lcd_drawpoint(j, ROW_FINDWHIDE_THRE, PURPLE);
        }
#endif
        SeedGrowFindUpInflection(Choose, Seed, ROW_FINDWHIDE_THRE, UpInflectionC);
    }
    else
    {
        UpInflectionC->Y = row - 1;
    }
}

/********************************************************************************************
 ** 函数功能: 识别三岔
 ** 参    数:
 **           int *LeftLine：左线
 **           int *RightLine:右线
 **           Point *InflectionL:左边拐点
 **           Point *InflectionR:右边拐点
 ** 返 回 值:  0：没有识别到环岛
 **           1：正入三岔
 ** 作    者: LJF
 ** 注    意：1 . 目前仅仅是正入三岔的时候的函数，因为三岔前面都会有个弯道所以会出现车身斜的情况，此时的左右拐点并不一定都存在
 **           2.这个是进三岔的函数，出三岔时候应该重写一个，并在进入三岔后再开启出三岔的判断
 *********************************************************************************************/
uint8 ForkIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR)
{
#if FORK_DEBUG
    lcd_showint32(0, 0, LostNum_RightLine, 3);
    lcd_showint32(0, 1, LostNum_LeftLine, 3);
    lcd_showint32(TFT_X_MAX-50, 0, DownInflectionL.X, 3);
    lcd_showint32(TFT_X_MAX-50, 1, DownInflectionL.Y, 3);
    lcd_showint32(TFT_X_MAX-50, 2, DownInflectionR.X, 3);
    lcd_showint32(TFT_X_MAX-50, 3, DownInflectionR.Y, 3);
#endif
    Point UpInflectionC,LeftUpPoint,LeftDownPoint;//上拐点，左边拐点，左边上顶点，补折线
    //当左右拐点存在,并且两个拐点要在图像下半部分
    if(DownInflectionL.X!=0 && DownInflectionR.X!=0 && DownInflectionL.X<120 && DownInflectionR.X>30)
    {
        //取消这个左右拐点行数的判断，增加运算速率
        if(abs((DownInflectionL.Y-DownInflectionR.Y))<40)//左右两个拐点的行数小于30，才进行判断
        {
            GetForkUpInflection(DownInflectionL, DownInflectionR, &UpInflectionC);//去搜索上拐点
            if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
            {
                FillingLine('R',DownInflectionR,UpInflectionC);//三岔成立了就在返回之前补线
                //补一条折线
                ForkFindSpecialPoint(UpInflectionC.Y,DownInflectionL.X,&LeftUpPoint);
                if(LeftUpPoint.Y!=0)
                {
                    LeftDownPoint.Y=UpInflectionC.Y,LeftDownPoint.X=DownInflectionL.X;
                    FillingLine('L',LeftDownPoint,LeftUpPoint);
                    FillingLine('R',UpInflectionC,LeftUpPoint);
                    UpInflectionC.Y=LeftUpPoint.Y;//为了使得补的这条折线的偏差被使用到
                }
//                Bias=DifferentBias(bias_startline,bias_endline,CentreLine);//无特殊处理时的偏差计算
                //判断Bias如何求
                if(UpInflectionC.Y<bias_endline)//starline<endline<Up.y，则正常循迹
                {
                    Bias=DifferentBias(bias_startline,bias_endline,CentreLine);
                }
                else if(UpInflectionC.Y<bias_startline && bias_endline<UpInflectionC.Y)//starline<UP.y<endline,则按照起始行到上拐点
                {
                    Bias=DifferentBias(bias_startline,UpInflectionC.Y,CentreLine);
                }
                else if(bias_startline<UpInflectionC.Y)//UP.y<starline<endline
                {
                    Bias=DifferentBias(DownInflectionR.Y,UpInflectionC.Y,CentreLine);
                }
                gpio_toggle(LED_BLUE);
                return 1;//三个拐点存在三岔成立：正入三岔
            }
        }
        else
            return 0;
    }
    else if((DownInflectionL.X==0 && DownInflectionR.X==0) || (BinaryImage[MT9V03X_H-5][5]==IMAGE_WHITE && BinaryImage[MT9V03X_H-5][MT9V03X_W-5]==IMAGE_WHITE))//如果左右下拐点不存在并且下面一段出现就丢线的话的话,我们就去看存不存在正上的拐点
    {

        Point ImageDownPointL,ImageDownPointR;//以画面的左下角和右下角作为左右补线的点
        //给上拐点的列一个预测的点，而不是写死为屏幕中间
        if(LeftLine[MT9V03X_H-1]!=0)//如果最下面的一行没有丢线
            ImageDownPointL.X=LeftLine[MT9V03X_H-1]+10,ImageDownPointL.Y=MT9V03X_H-1;
        else ImageDownPointL.X=0,ImageDownPointL.Y=MT9V03X_H-1;
        if(RightLine[MT9V03X_H-1]!=MT9V03X_W-1)
            ImageDownPointR.X=RightLine[MT9V03X_H-1]-10,ImageDownPointR.Y=MT9V03X_H-1;
        else ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.Y=MT9V03X_H-1;
        //找寻上拐点
        GetForkUpInflection(ImageDownPointL, ImageDownPointR, &UpInflectionC);
        if(UpInflectionC.Y!=0 && UpInflectionC.Y>40)//直接访问Y即可，加快速度，因为X默认就会赋值了
        {
            FillingLine('R',ImageDownPointR,UpInflectionC);//三岔成立了就在返回之前补线
            ForkFindSpecialPoint(UpInflectionC.Y,2,&LeftUpPoint);
            if(LeftUpPoint.Y!=0)
            {
                LeftDownPoint.Y=UpInflectionC.Y,LeftDownPoint.X=2;
                FillingLine('L',LeftDownPoint,LeftUpPoint);
                FillingLine('R',UpInflectionC,LeftUpPoint);
                UpInflectionC.Y=LeftUpPoint.Y;//为了使得补的这条折线的偏差被使用到
            }
//            Bias=DifferentBias(bias_startline,bias_endline,CentreLine);//无特殊处理时的偏差计算
            //判断Bias如何求
            if(UpInflectionC.Y<bias_endline)//starline<endline<Up.y，则正常循迹
            {
                Bias=DifferentBias(bias_startline,bias_endline,CentreLine);
            }
            else if(UpInflectionC.Y<bias_startline && bias_endline<UpInflectionC.Y)//starline<UP.y<endline,则按照起始行到上拐点
            {
                Bias=DifferentBias(bias_startline,UpInflectionC.Y,CentreLine);
            }
            else if(bias_startline<UpInflectionC.Y)//UP.y<starline<endline
            {
                Bias=DifferentBias(ImageDownPointR.Y,UpInflectionC.Y,CentreLine);
            }
            gpio_toggle(LED_GREEN);
            return 1;//三岔正入丢失左右拐点那一帧
        }
    }
    //右边丢线超过60，左拐点存在，并且左拐点不能在上半平屏防止误判
    else if(LostNum_RightLine>55 && DownInflectionL.X>0 && DownInflectionL.X<90 && DownInflectionL.Y>40)
    {
        Point ImageDownPointR;//以左拐点对称的点去补线和找拐点
        //给自己设定的右拐点去找上拐点
//        ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.YDownInflectionL.Y=DownInflectionL.Y;
        //运用单边循迹法的思想给拐点，赛道宽度，左斜找上拐点右边的多往右一点
        ImageDownPointR.X=DownInflectionL.X+(145-(119-DownInflectionL.Y)*1.1);ImageDownPointR.Y=DownInflectionL.Y;
        GetForkUpInflection(DownInflectionL, ImageDownPointR, &UpInflectionC);
        if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
        {
            //左斜的时候，其实右拐点会在右下屏幕的最下角，所以用于补线的时候以右下角为补线点
            ImageDownPointR.X=MT9V03X_W-1;ImageDownPointR.Y=MT9V03X_H-1;
            FillingLine('R',ImageDownPointR,UpInflectionC);//三岔成立了就在返回之前补线
//            Bias=DifferentBias(bias_startline,bias_endline,CentreLine);//无特殊处理时的偏差计算
            //判断Bias如何求
            if(UpInflectionC.Y<bias_endline)//starline<endline<Up.y，则正常循迹
            {
                Bias=DifferentBias(bias_startline,bias_endline,CentreLine);
            }
            else if(UpInflectionC.Y<bias_startline && bias_endline<UpInflectionC.Y)//starline<UP.y<endline,则按照起始行到上拐点
            {
                Bias=DifferentBias(bias_startline,UpInflectionC.Y,CentreLine);
            }
            else if(bias_startline<UpInflectionC.Y)//UP.y<starline<endline
            {
                Bias=DifferentBias(ImageDownPointR.Y,UpInflectionC.Y,CentreLine);
            }
            gpio_toggle(LED_RED);
            return 1;//三岔左斜入三岔
        }
    }
    //左边丢线超过60,右拐点存在,并且右拐点不能在上半平屏防止误判
    else if(LostNum_LeftLine>55 && DownInflectionR.X>60 && DownInflectionR.Y>40)
    {
        Point ImageDownPointL;//以左拐点对称的点去补线和找拐点
        //与拐点行数做一个比例关系，越靠近底部了拐点宽度越大.左斜多往左一点
//        ImageDownPointL.X=DownInflectionR.X-(145-(119-DownInflectionL.Y)*1.1);ImageDownPointL.Y=DownInflectionR.Y;
        ImageDownPointL.X=DownInflectionR.X-FORK_INFLECTION_WIDTH;ImageDownPointL.Y=DownInflectionR.Y;
        GetForkUpInflection(ImageDownPointL, DownInflectionR, &UpInflectionC);
        if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
        {
            FillingLine('R',DownInflectionR,UpInflectionC);//三岔成立了就在返回之前补线
            //补一条折线
            ForkFindSpecialPoint(UpInflectionC.Y,5,&LeftUpPoint);
            if(LeftUpPoint.Y!=0)
            {
                LeftDownPoint.Y=UpInflectionC.Y,LeftDownPoint.X=5;
                FillingLine('L',LeftDownPoint,LeftUpPoint);
                FillingLine('R',UpInflectionC,LeftUpPoint);
                UpInflectionC.Y=LeftUpPoint.Y;//为了使得补的这条折线的偏差被使用到
            }
//            Bias=DifferentBias(bias_startline,bias_endline,CentreLine);//无特殊处理时的偏差计算
            //判断Bias如何求
            if(UpInflectionC.Y<bias_endline)//starline<endline<Up.y，则正常循迹
            {
                Bias=DifferentBias(bias_startline,bias_endline,CentreLine);
            }
            else if(UpInflectionC.Y<bias_startline && bias_endline<UpInflectionC.Y)//starline<UP.y<endline,则按照起始行到上拐点
            {
                Bias=DifferentBias(bias_startline,UpInflectionC.Y,CentreLine);
            }
            else if(bias_startline<UpInflectionC.Y)//UP.y<starline<endline
            {
                Bias=DifferentBias(DownInflectionR.Y,UpInflectionC.Y,CentreLine);
            }
            gpio_toggle(LED_WHITE);
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
uint8 ForkFStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 *ForkFlag)
{
    static uint8 StatusChange,num1,num3,numspecial;//三岔识别函数的临时状态变量，用来看状态是否跳转
    uint8 NowFlag=0;//这次的识别结果

    NowFlag=ForkIdentify(LeftLine, RightLine, DownInflectionL, DownInflectionR);
    *ForkFlag=NowFlag;//把识别结果送出去
    //特殊保护
    if(numspecial<200)//防止很久都没有出现进入入口的状态，及时去判断出口
    {
        numspecial++;
    }
    else if(StatusChange<1)//判断状态有没有度过入口状态，若没有则强制跳过
    {
        StatusChange=2;
    }
    //状态机开始部分
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
            if(num1<100)  //给足够长的时间让车走到三岔运行中
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
uint8 ForkSStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 *NowFlag)
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
            if(*NowFlag==1)
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
            if(*NowFlag==0)
            {
                StatusChange=2; //过了中间过度态之后跳转至检测出口
                base_speed-=5; //检测出口减速
            }
            break;
        }
        //出口状态
        case 2:
        {
            if(*NowFlag==1)
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

