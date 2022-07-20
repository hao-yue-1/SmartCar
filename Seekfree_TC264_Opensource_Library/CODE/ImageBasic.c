#include "ImageBasic.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "LED.h"
#include "zf_gpio.h"

#define FINE_RIGHT_ANGLE_INFLECTION_DEBUG   1   //遍历图像黑白跳变找直角拐点预编译的宏定义1：开启 0：关闭

//变量定义
uint8 Mid=MT9V03X_W/2;                        //初始化扫线的中点为图像中点
uint8 Lost_Row=0;                             //中线丢失的行坐标(扫线到赛道外)
uint8 LostNum_LeftLine=0,LostNum_RightLine=0; //记录左右边界丢线数

/*
 ** 函数功能: 扫线提取左中右三线的坐标
 ** 参    数: *LeftLine：左线数组
 **           *CentreLine：中线数组
 **           *RightLine：右线数组
 **           path：默认扫线方向
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void GetImagBasic(int *LeftLine, int *CentreLine, int *RightLine ,char path)
{
    uint8 row,cloum;              //行,列
    uint8 flag_l=0,flag_r=0;    //记录是否丢线flag，flag=0：丢线
    uint8 num=0;                //记录中线连续丢失的次数
    LostNum_LeftLine=0;LostNum_RightLine=0; //把丢线的计数变量清零

    //开始扫线(从下往上,从中间往两边),为了扫线的严谨性,我们做BORDER_BIAS的误差处理，即扫线范围会小于图像大小
    for(row=MT9V03X_H-1;row>0;row--) //从下往上，遍历整幅图像
    {
        //在赛道外，优先按path扫线方向寻找赛道
        if(BinaryImage[row][Mid]==IMAGE_BLACK)  //扫线中点是黑色的（中点在赛道外）
        {
            num++;    //中线连续丢失，+1
            if(path=='L')   //默认向左扫线
            {
                //先向左边扫线，寻找右边界点
                for(cloum=Mid;cloum-BORDER_BIAS>0;cloum--)    //向左扫
                {
                    if(BinaryImage[row][cloum]==IMAGE_WHITE && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_WHITE)  //找到白点（从赛道外扫到赛道内：黑-白）
                    {
                        RightLine[row]=cloum;    //记录右边界点（向左找到的是右边界点）
                        flag_r=1;               //flag做无丢线标记
                        break;
                    }
                }
                //根据上面扫线的结果判断丢失的赛道是在左边还是右边从而决定继续向哪边扫线
                if(flag_r==1)   //找到了右边界（丢失的赛道在左边）
                {
                    //继续向左寻找左边界
                    for(;cloum-BORDER_BIAS>0;cloum--)    //继续向左扫
                    {
                        if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_BLACK)    //找到黑点：（从赛道内扫到赛道外：白-黑）
                        {
                            LeftLine[row]=cloum;   //记录左边界点
                            flag_l=1;              //flag做无丢线标记
                            break;
                        }
                    }
                }
                else            //没有找到右边界（丢失的赛道在右边）
                {
                    for(cloum=Mid;cloum+BORDER_BIAS<MT9V03X_W-1;cloum++)    //向右扫
                    {
                        if(BinaryImage[row][cloum]==IMAGE_WHITE && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_WHITE)
                        {
                            LeftLine[row]=cloum;   //记录左边界点（向右找到的是左边界点）
                            flag_l=1;              //flag做无丢线标记
                            break;
                        }
                    }
                    if(flag_l==1)   //找到了左边界（丢失的赛道在右边）
                    {
                        for(;cloum+BORDER_BIAS<MT9V03X_W-1;cloum++) //继续向右扫
                        {
                            if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_BLACK)
                            {
                                RightLine[row]=cloum;   //记录右边界点
                                flag_r=1;               //flag做无丢线标记
                                break;
                            }
                        }
                    }
                }
            }
            else if(path=='R')  //默认向右扫线
            {
                //先向右边扫线，寻找左边界点
                for(cloum=Mid;cloum+BORDER_BIAS<MT9V03X_W-1;cloum++)    //向右扫
                {
                    //判断左边界点
                    if(BinaryImage[row][cloum]==IMAGE_WHITE && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_WHITE)  //找到白点（从赛道外扫到赛道内：黑-白）
                    {
                        LeftLine[row]=cloum;    //记录左边界点（向右找到的是左边界点）
                        flag_l=1;               //flag做无丢线标记
                        break;
                    }
                }
                //根据上面扫线的结果判断丢失的赛道是在左边还是右边从而决定继续向哪边扫线
                if(flag_l==1)   //找到了左边界（丢失的赛道在右边）
                {
                    //继续向右寻找右边界
                    for(;cloum+BORDER_BIAS<MT9V03X_W-1;cloum++) //继续向右扫
                    {
                        //判断右边界点
                        if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_BLACK)    //找到黑点（从赛道内扫到赛道外：白-黑）
                        {
                            RightLine[row]=cloum;   //记录左边界点
                            flag_r=1;               //flag做无丢线标记
                            break;
                        }
                    }
                }
                else            //没有找到左边界（丢失的赛道在左边）
                {
                    for(cloum=Mid;cloum-BORDER_BIAS>0;cloum--)    //向左扫
                    {
                        //判断右边界点
                        if(BinaryImage[row][cloum]==IMAGE_WHITE && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_WHITE)
                        {
                            RightLine[row]=cloum;   //记录右边界点（向左找到的是右边界点）
                            flag_r=1;               //flag做无丢线标记
                            break;
                        }
                    }
                    if(flag_r==1)   //找到了右边界（丢失的赛道在左边）
                    {
                        //继续向左寻找左边界
                        for(;cloum-BORDER_BIAS>0;cloum--)   //继续向左扫
                        {
                            //判断左边界点
                            if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_BLACK)  //判断右边界点
                            {
                                LeftLine[row]=cloum;    //记录左边界点
                                flag_l=1;               //flag做无丢线标记
                                break;
                            }
                        }
                    }
                }
            }
        }
        //在赛道中，正常扫线
        else
        {
            num=0;  //打断中线连续丢失，=0
            //左边扫线
            for(cloum=Mid;cloum-BORDER_BIAS>0;cloum--)              //向左扫
            {
                if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_BLACK)  //判断左边界点（BORDER_BIAS防偶然因素）
                {
                    LeftLine[row]=cloum;    //记录左边界点
                    flag_l=1;               //flag做无丢线标记
                    break;
                }
            }
            //右边扫线
            for(cloum=Mid;cloum+BORDER_BIAS<MT9V03X_W-1;cloum++)    //向右扫
            {
                if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_BLACK)  //判断右边界点（BORDER_BIAS防偶然因素）
                {
                    RightLine[row]=cloum;   //记录右边界点
                    flag_r=1;               //flag做无丢线标记
                    break;
                }
            }
        }
        //1.29晚上重新写的数据处理
        if(flag_l==0)   //左边界丢线
        {
            LeftLine[row]=0;            //左边界点等于图像的左边界
            LostNum_LeftLine++;         //左丢线数+1
        }
        if(flag_r==0)   //右边界丢线
        {
            RightLine[row]=MT9V03X_W-1; //右边界点等于图像的右边界
            LostNum_RightLine++;        //右丢线数+1
        }
        CentreLine[row]=(LeftLine[row]+RightLine[row])/2;   //计算中线点
        //为下一次扫线做准备
        Mid=CentreLine[row];    //以上一次的中线值为下一次扫线的中间点
        flag_l=0;               //左边界丢线flag置0
        flag_r=0;               //右边界丢线flag置0

//        //LCD绘制图像
//        lcd_drawpoint(LeftLine[row],row,GREEN);
//        lcd_drawpoint(CentreLine[row],row,RED);
//        lcd_drawpoint(RightLine[row],row,BLUE);
//        systick_delay_ms(STM0,50);
    }
}

/*
 ** 函数功能: 根据左右边界线来得到下拐点（十字、三岔、环岛的判断会用上）
 ** 参    数: int starline:     起始行
 **           int endline:      结束行
 **           int *LeftLine：     左线数组
 **           int *RightLine：   右线数组
 **           Point *InflectionL: 左边拐点
 **           Point *InflectionR: 右边拐点
 ** 返 回 值: 无
 ** 说    明: - 用指针带入进来函数，最后得到的点可以两点确定直线进行补线
 **           - 由于图像原点在左上角，所以起始行是大于结束行，左右线数组从下往上遍历
 ** 作    者: LJF
 */
void GetDownInflection(int startline,int endline,int *LeftLine,int *RightLine,Point *InflectionL,Point *InflectionR)
{
    int i=0;
    InflectionL->X=0;InflectionL->Y=0;InflectionR->X=0;InflectionR->Y=0;//左右拐点置零

    for(i=startline;i>endline;i--)
    {
        //遍历左线，求出先变小大后变小的点，多加三个点的判断是为了误判，左边丢线为0
        /*注意：这里如果判断条件是和相邻的1,3的值对比的话，会区间太小从而如果有相等的列数存在的话，会影响判断，所以需要改大比较的区间*/
        //加了个判断InflectionL->Y==0是为了从下往上遍历，找到了之后就不再继续往上找了，这样遍历时候的截距图片就不用刚刚好了
        //2022年5月27日：加多一层条件，即：比>=相邻的点，随后再取筛选它是不是直线：大于比较远的点
        if(InflectionL->Y==0 && LeftLine[i]>=LeftLine[i-1] && LeftLine[i]>=LeftLine[i-3] && LeftLine[i]>=LeftLine[i+1] && LeftLine[i]>=LeftLine[i+3])
        {
            if(LeftLine[i]>LeftLine[i-5] && LeftLine[i]>LeftLine[i-7] && LeftLine[i]>LeftLine[i+5] && LeftLine[i]>LeftLine[i+7])
            {
                InflectionL->X=LeftLine[i];//存入拐点的（x,y）坐标
                InflectionL->Y=i;
                /*debug*/
//                lcd_showint32(0,6,InflectionL->X,3);
//                systick_delay_ms(STM0, 1000);
            }
        }

        //遍历右线，求出列数最小的点就是右边的拐点，右边线丢线为MT9V03X_W-1
        //加了个判断InflectionR->Y==0是为了从下往上遍历，找到了之后就不再继续往上找了，这样遍历时候的截距图片就不用刚刚好了
        if(InflectionR->Y==0 && RightLine[i]<=RightLine[i-1] && RightLine[i]<=RightLine[i-3] && RightLine[i]<=RightLine[i+1] && RightLine[i]<=RightLine[i+3])
        {
            if(RightLine[i]<RightLine[i-5] && RightLine[i]<RightLine[i-7] && RightLine[i]<RightLine[i+5] && RightLine[i]<RightLine[i+7])
            {
                InflectionR->X=RightLine[i];//存入拐点的（x,y）坐标
                InflectionR->Y=i;
               /*打印被判断为拐点的列坐标，用于调试*/
//                lcd_showint32(TFT_X_MAX-50,6,InflectionR->X,3);
//                systick_delay_ms(STM0, 1000);
            }
        }

        /*打印被判断为拐点的列坐标，用于调试*/
//        lcd_drawpoint(RightLine[i],i,RED);
//        lcd_showint32(0,0,LeftLine[i],3);
//        systick_delay_ms(STM0, 800);
    }
}
#define UPINFLECTION_DOWM_MIN_THRESHOLD  15  //上拐点列坐标与下面行数的差值最小阈值
#define UPINFLECTION_UP_MAX_THRESHOLD  5  //上拐点列坐标与上面行数的差值最大阈值
#define UPINFLECTION_COMPARE_INTERVAL 3 //上拐点两点之间比较间隔

/************************************************************************
 ** 函数功能: 根据左右边界得到上拐点
 ** 参    数: char Choose：选择遍历左线还是右线
 **           int startline：起始行
 **           int endline：结束行
 **           Point *UpInflection：上拐点
 ** 返 回 值: 无
 ** 说    明: 起始行要小于结束行，从上往下遍历左右线
 ** 作    者: LJF
 ***********************************************************************/
void GetUpInflection(char Choose,int startline,int endline,Point *UpInflection)
{
    int row=0;
    switch(Choose)
    {
        case 'L':
        {
            //从上往下遍历
            for (row = startline; row < endline; row++)
            {
                //下三行的列坐标-这行列坐标大于阈值，不用ABS是为了速度更快
                if (LeftLine[row] - LeftLine[row+UPINFLECTION_COMPARE_INTERVAL] >= UPINFLECTION_DOWM_MIN_THRESHOLD
                 && LeftLine[row - UPINFLECTION_COMPARE_INTERVAL] - LeftLine[row + UPINFLECTION_COMPARE_INTERVAL] >= UPINFLECTION_DOWM_MIN_THRESHOLD
                 && LeftLine[row-1]-LeftLine[row]<=UPINFLECTION_UP_MAX_THRESHOLD)
                {
                    UpInflection->X = LeftLine[row]; UpInflection->Y = row;
                    /**************debug***********/
//                    lcd_drawpoint(UpInflection->X,UpInflection->Y,PURPLE);
//                    lcd_showint32(TFT_X_MAX-50,0,UpInflection->X,3);
//                    lcd_showint32(TFT_X_MAX-50,1,UpInflection->Y,3);
//                    systick_delay_ms(STM0, 800);
                    /*****************************/
                    break;
                }
                /**************debug***********/
//                lcd_showint32(0,0,LeftLine[row],3);
//                lcd_showint32(0,1,LeftLine[row+UPINFLECTION_COMPARE_INTERVAL],3);
//                systick_delay_ms(STM0, 800);
                /*****************************/
            }
            break;
        }
        case 'R':
        {
            for(row = startline; row < endline; row++)
            {
                //这行列坐标-下三行的列坐标大于阈值，不用ABS是为了速度更快
                if (RightLine[row+UPINFLECTION_COMPARE_INTERVAL] - RightLine[row] >= UPINFLECTION_DOWM_MIN_THRESHOLD
                 && RightLine[row + UPINFLECTION_COMPARE_INTERVAL] - RightLine[row - UPINFLECTION_COMPARE_INTERVAL] >= UPINFLECTION_DOWM_MIN_THRESHOLD
                 && RightLine[row]-RightLine[row-1]<=UPINFLECTION_UP_MAX_THRESHOLD)
                {
                    UpInflection->X=RightLine[row];UpInflection->Y=row;
                   /**************debug***********/
//                   lcd_drawpoint(UpInflection->X,UpInflection->Y,PURPLE);
//                   lcd_showint32(TFT_X_MAX-50,0,UpInflection->X,3);
//                   lcd_showint32(TFT_X_MAX-50,1,UpInflection->Y,3);
//                   systick_delay_ms(STM0, 800);
                   /*****************************/
                    break;
                }
            }
            break;
        }
        default:break;
    }
}
/************************************************************************
 ** 函数功能: 根据图像黑白跳变寻找上直角拐点
 ** 参    数: char Choose：选择是左上还是右上
 **           Point DowmInflection：基准点
 **           Point *UpInflection：找到的上拐点
 **           int RowThr:遍历图像行的阈值（找跳变点到哪里停下）
 **           int CloumnThr:遍历图像列的阈值
 ** 返 回 值: 无
 ** 说    明: 起始行要小于结束行，从上往下遍历左右线
 ** 作    者: LJF
 ***********************************************************************/
void GetRightangleUPInflection(char Choose,Point DowmInflection,Point *UpInflection,int ROWTHR,int CLOUMNTHR)
{
    int row=0,cloumn=0;//起始行,列
    UpInflection->X=0;UpInflection->Y=0;//左上拐点置零
    //从下往上找白跳黑
    for(row=DowmInflection.Y;row>ROWTHR;row--)
    {
#if FINE_RIGHT_ANGLE_INFLECTION_DEBUG
        lcd_drawpoint(DowmInflection.X, row, PURPLE);
#endif
        if(BinaryImage[row][DowmInflection.X]==IMAGE_WHITE&&BinaryImage[row-1][DowmInflection.X]==IMAGE_BLACK)
        {
            row=row-3;//多往上面跳点
            switch(Choose)
            {
               case 'L':
               {
                   //左往右找到黑跳白
                   for(cloumn=DowmInflection.X;cloumn<CLOUMNTHR;cloumn++)
                   {
#if FINE_RIGHT_ANGLE_INFLECTION_DEBUG
                       lcd_drawpoint(cloumn, row, PURPLE);
#endif
                       if(BinaryImage[row][cloumn]==IMAGE_BLACK&&BinaryImage[row][cloumn+1]==IMAGE_WHITE)
                       {
                           UpInflection->X=cloumn;UpInflection->Y=row;
                           break;
                       }
                   }
                   break;
               }
               case 'R':
               {
                   //右往左找到黑跳白
                   for(cloumn=DowmInflection.X;cloumn>CLOUMNTHR;cloumn--)
                   {
                       if(BinaryImage[row][cloumn]==IMAGE_BLACK&&BinaryImage[row][cloumn-1]==IMAGE_WHITE)
                       {
                           UpInflection->X=cloumn;UpInflection->Y=row;
                           break;
                       }
                   }
                   break;
               }
               default:break;
            }
            break;//跳出行循环,没必要继续行循环下去
        }
    }
}
/*---------------------------------------------------------------
 【函    数】Bin_Image_Filter
 【功    能】过滤噪点
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void Bin_Image_Filter(void)
{
    for (int nr=1; nr < MT9V03X_H-1; nr++)
    {
        for (int nc=1; nc < MT9V03X_W-1; nc++)
        {
            if ((BinaryImage[nr][nc] == IMAGE_BLACK)
                    &&(BinaryImage[nr-1][nc]+BinaryImage[nr+1][nc]+BinaryImage[nr][nc+1]+BinaryImage[nr][nc-1]>510))
            {
                BinaryImage[nr][nc] = IMAGE_WHITE;
            }
            else if ((BinaryImage[nr][nc] == IMAGE_WHITE)
                    && (BinaryImage[nr-1][nc]+BinaryImage[nr+1][nc]+BinaryImage[nr][nc+1]+BinaryImage[nr][nc-1]<510))
            {
                BinaryImage[nr][nc] = IMAGE_BLACK;
            }
        }
    }
}

/************************************************************************
 ** 函数功能: 测量赛道宽度函数
 ** 参    数: int startline:要测量的起始行
 **           int endline:要测量的结束行
 ** 返 回 值: 无
 ** 说    明: 用于赛前测试使用，赛道宽度可以给拐弯的时候单边巡线
 ** 作    者: LJF
 ***********************************************************************/
void MeasureWidth(int startline,int endline)
{
    int row=0,width=0;
    for(row=startline;row>endline;row--)
    {
        width=RightLine[row]-LeftLine[row];
        printf("%d\r\n",width);//测到赛道宽度与行的函数关系为y=135-(119-nowline)*1.1
    }
}
