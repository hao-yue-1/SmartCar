#include "ImageBasic.h"

//变量定义
int Mid=MT9V03X_W/2;                        //初始化扫线的中点为图像中点
int Lost_CentreLine=0;                      //中线丢失的行坐标(扫线到赛道外)
int LostNum_LeftLine=0,LostNum_RightLine=0; //记录左右边界丢线数

/*
 ** 函数功能: 扫线提取左中右三线的坐标(坐标在数组中的位置代表其行坐标,坐标在数组中的位置代表其列坐标)
 ** 参    数: int *LeftLine：     左线数组
 **           int *CentreLine： 中线数组
 **           int *RightLine：   右线数组
 ** 返 回 值: 无
 ** 作    者: LJF
 */

void GetImagBasic(int *LeftLine, int *CentreLine, int *RightLine)
{
    int row,cloum;              //行,列
    uint8 flag_l=0,flag_r=0;    //记录是否丢线flag，flag=0：丢线
    LostNum_LeftLine=0;LostNum_RightLine=0;//把丢线的计数变量清零
    //开始扫线(从下往上,从中间往两边),为了扫线的严谨性,我们做BORDER_BIAS的误差处理，即扫线范围会小于图像大小
    for(row=MT9V03X_H;row>0;row--) //图像的原点在左上角
    {
        //下面这个if是为了解决扫线一开始就在赛道外的问题，帮助重新找回赛道
        if(BinaryImage[row][Mid]==IMAGE_BLACK)  //扫线中点是黑色的（中点在赛道外）
        {
            //先向左边扫线，寻找右边界点
            for(cloum=Mid;(cloum-BORDER_BIAS)>0;cloum--)    //向左边扫线
            {
                if(BinaryImage[row][cloum]==IMAGE_WHITE && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_WHITE)  //判断右边界点（从赛道外扫到赛道内应该是黑变白的过程）
                {
                    RightLine[row]=cloum;    //记录右边界点（向左找到的是右边界点）
                    flag_r=1;               //flag做无丢线标记
                    break;
                }
            }
            //根据上面扫线的结果判断丢失的赛道是在左边还是右边从而决定继续向哪边扫线
            if(flag_r==1)   //向左扫线找到了右边界没有丢线（丢失的赛道在左边）
            {
                for(;(cloum-BORDER_BIAS)>0;cloum--)    //继续向左边扫线找到真正的左边界
                {
                    if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_BLACK)    //判断左边界点（正常情况由白变黑）
                    {
                        LeftLine[row]=cloum;   //记录左边界点
                        flag_l=1;              //flag做无丢线标记
                        break;
                    }
                }
            }
            //向左扫线没有找到右边界点，那么向右扫线寻找左边界点
            else
            {
                for(cloum=Mid;(cloum+BORDER_BIAS)<MT9V03X_W;cloum++)    //向右边扫线
                {
                    if(BinaryImage[row][cloum]==IMAGE_WHITE && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_WHITE)  //判断左边界点
                    {
                        LeftLine[row]=cloum;   //记录左边界点（向右找到的是左边界点）
                        flag_l=1;              //flag做无丢线标记
                        break;
                    }
                }
                if(flag_l==1)   //向右扫线找到了左边界没有丢线（丢失的赛道在右边）
                {
                    for(;(cloum+BORDER_BIAS)<MT9V03X_W;cloum++)    //继续向右边扫线，寻找右边界点
                    {
                        if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_BLACK)  //判断右边界点
                        {
                            RightLine[row]=cloum;   //记录右边界点
                            flag_r=1;               //flag做无丢线标记
                            break;
                        }
                    }
                }
            }
        }
        //在赛道中上，正常扫线
        else
        {
            //左边扫线
            for(cloum=Mid;(cloum-BORDER_BIAS)>0;cloum--)
            {
                if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_BLACK)  //判断左边界点（BORDER_BIAS防偶然因素）
                {
                    LeftLine[row]=cloum;    //记录左边界点
                    flag_l=1;               //flag做无丢线标记
                    break;
                }
            }
            //右边扫线
            for(cloum=Mid;(cloum+BORDER_BIAS)<MT9V03X_W;cloum++)
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
            RightLine[row]=MT9V03X_W-1;   //右边界点等于图像的右边界
            LostNum_RightLine++;        //右丢线数+1
        }
        CentreLine[row]=(LeftLine[row]+RightLine[row])/2;   //记录中线点

//        //防止扫线到赛道外，这个方案不行
//        if(BinaryImage[row][CentreLine[row]]==IMAGE_BLACK && BinaryImage[row+BORDER_BIAS][CentreLine[row]]==IMAGE_BLACK)    //row行的中线是黑，扫到了赛道外
//        {
//            Lost_CentreLine=row;    //记录中线点丢失的行坐标
//            if(row>20)              //对前20行不做处理
//                break;              //若已经在20行后发现扫描到了赛道外,直接break跳出该图的扫线处理
//        }

        Mid=CentreLine[row];    //以上一次的中线值为下一次扫线的中间点，若上一次的中线值刚好在边缘上，下一次的扫线会出现中线全跑到中线的情况
        flag_l=0;               //左边界丢线flag置0
        flag_r=0;               //右边界丢线flag置0
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
        if(InflectionL->Y==0 && LeftLine[i]>LeftLine[i-3] && LeftLine[i]>LeftLine[i-5] && LeftLine[i]>LeftLine[i+3] && LeftLine[i]>LeftLine[i+5])
        {
            InflectionL->X=LeftLine[i];//存入拐点的（x,y）坐标
            InflectionL->Y=i;
            /*打印被判断为拐点的列坐标，用于调试*/
//            lcd_showint32(0,6,InflectionL->X,3);
//            systick_delay_ms(STM0, 1000);
        }
        //遍历右线，求出列数最小的点就是右边的拐点，右边线丢线为MT9V03X_W
        //加了个判断InflectionR->Y==0是为了从下往上遍历，找到了之后就不再继续往上找了，这样遍历时候的截距图片就不用刚刚好了
        if(InflectionR->Y==0 && RightLine[i]<RightLine[i-3] && RightLine[i]<RightLine[i-5] && RightLine[i]<RightLine[i+3] && RightLine[i]<RightLine[i+5])
        {
            InflectionR->X=RightLine[i];//存入拐点的（x,y）坐标
            InflectionR->Y=i;
            /*打印被判断为拐点的列坐标，用于调试*/
//            lcd_showint32(TFT_X_MAX-50,6,InflectionR->X,3);
//            systick_delay_ms(STM0, 1000);
        }
        /*打印被判断为拐点的列坐标，用于调试*/
//        lcd_drawpoint(RightLine[i],i,RED);
//        lcd_showint32(0,0,RightLine[i],3);
//        systick_delay_ms(STM0, 800);
    }
}

/*
 ** 函数功能: 根据左右边界线和下拐点来得到上拐点
 ** 参    数: *LeftLine：  左线数组
 **           *RightLine：右线数组
 **           *DownInflectionL: 左下拐点
 **           *DownInflectionR: 右下拐点
 **           *UpInflectionL：    左上拐点
 **           *UpInflectionR：    右上拐点
 ** 返 回 值: 无
 ** 说    明: 在一些特殊元素识别或补线中需要用到上拐点，但不是所有元素都要用到的，所以将上拐点分开单独作为一个函数来实现，方便在不同的
 **           元素识别中进行调用
 ** 作    者: WBN
 */
void GetUpInflection(int *LeftLine,int *RightLine,Point *DownInflectionL,Point *DownInflectionR,Point *UpInflectionL,Point *UpInflectionR)
{
    /*
     ** 简单粗暴的方法：
     ** 左边的上拐点在右边界线，右边的上拐点在左边界线；
     ** 当下拐点出现时，对应的上拐点是一定存在的，只是是否有在摄像头获取的图像中出现的问题；且由于赛道宽度一定，即在最后的图像中，上拐点会
     ** 出现在对应下拐点往上X行的位置，我们只需要做一个是否数组越界的判断
     * */

    //左边的上拐点在右边界线
    if(DownInflectionL->Y+INFLECTION_WIDTH<=MT9V03X_H)
    {
        UpInflectionL->X=RightLine[DownInflectionL->Y+INFLECTION_WIDTH];
        UpInflectionL->Y=DownInflectionL->Y+INFLECTION_WIDTH;
    }
    //右边的上拐点在左边界线
    if(DownInflectionR->Y+INFLECTION_WIDTH<=MT9V03X_H)
    {
        UpInflectionR->X=LeftLine[DownInflectionR->Y+INFLECTION_WIDTH];
        UpInflectionR->Y=DownInflectionR->Y+INFLECTION_WIDTH;
    }


    /*
         ** 另一种方法：
         ** 类似寻找下拐点的方法，寻找一个数组值突变的点
         * */
//    int row;    //行
//    //遍历左线，从左拐点开始扫
//    for(row=DownInflectionL->Y;row<MT9V03X_H;row++)  //行扫：从下往上扫
//    {
//        if()    //左边的上拐点在右线上
//        {
//            UpInflectionL->X=RightLine[row];
//            UpInflectionL->Y=row;
//        }
//    }
//    //遍历右线，从右拐点开始扫
//    for(row=DownInflectionR->Y;row<MT9V03X_H;row++)  //行扫：从下往上扫
//    {
//
//        if()    //右边的上拐点在左线上
//        {
//            UpInflectionR->X=LeftLine[row];
//            UpInflectionR->Y=row;
//        }
//    }
}
