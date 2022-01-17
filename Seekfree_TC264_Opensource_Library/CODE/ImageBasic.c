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
    int row,cloum;   //行,列
    //开始扫线(从下往上,从中间往两边),为了扫线的严谨性,我们做BORDER_BIAS的误差处理，即扫线范围会小于图像大小
    for(row=0;(row-BORDER_BIAS)<MT9V03X_H;row++) //图像的原点在左下角
    {
        //左边扫线
        for(cloum=Mid;(cloum-BORDER_BIAS)>0;cloum--)
        {
            if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum-BORDER_BIAS]==IMAGE_BLACK)  //判断左边界点（BORDER_BIAS防偶然因素）
            {
                LeftLine[row]=cloum;    //记录左边界点
                break;
            }
        }
        //右边扫线
        for(cloum=Mid;(cloum+BORDER_BIAS)<MT9V03X_W;cloum++)
        {
            if(BinaryImage[row][cloum]==IMAGE_BLACK && BinaryImage[row][cloum+BORDER_BIAS]==IMAGE_BLACK)  //判断右边界点（BORDER_BIAS防偶然因素）
            {
                RightLine[row]=cloum;   //记录右边界点
                break;
            }
        }
        //数据处理
        CentreLine[row]=(LeftLine[row]+RightLine[row])/2;   //记录中线点
        if(LeftLine[row]==0)     //左边界丢线
            LostNum_LeftLine++;  //左丢线数+1
        if(RightLine[row]==0)    //右边界丢线
        {
            RightLine[row]=MT9V03X_W;//把右边丢线的点直接拉到最右边的值方便找拐点的时候排序
            LostNum_RightLine++; //右丢线数+1
        }
        //防止扫线到赛道外
        if(BinaryImage[row][CentreLine[row]]==IMAGE_BLACK && BinaryImage[row+BORDER_BIAS][CentreLine[row]]==IMAGE_BLACK)    //row行的中线是黑，扫到了赛道外
        {
            Lost_CentreLine=row;    //记录中线点丢失的行坐标
            if(row>20)              //对前20行不做处理
                break;              //若已经在20行后发现扫描到了赛道外,直接break跳出该图的扫线处理
        }

        /*
         ** 参考的代码中都是有专门的数组单独记录每一行是否有丢线情况 这里先不做这样的记录
         ** 默认丢线情况下LeftLine[]==0
         ** 不等于0的情况都视为没有丢线
         ** */


        Mid=CentreLine[row];      //以上一次的中线值为下一次扫线的中间点
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
 ** 说    明: 用指针带入进来函数，最后得到的点可以两点确定直线进行补线
 ** 作    者: LJF
 */
void GetInflection(int startline,int endline,int *LeftLine,int *RightLine,Point *InflectionL,Point *InflectionR)
{
    int i,tempL=0,tempR=MT9V03X_W;

    for(i=startline;i<endline;i++)
    {
        //遍历左线，求出列数最大的点就是左边的拐点，左线丢线为0
        if(LeftLine[i]>tempL)
        {
            InflectionL->X=LeftLine[i];//存入拐点的（x,y）坐标
            InflectionL->Y=i;
            tempL=LeftLine[i];//暂存，其实也可以不用暂存，直接InflectionL->X即可，但是怕其他要改，这步先放在这里后续优化即可
        }
        //遍历右线，求出列数最小的点就是右边的拐点，右边线丢线为MT9V03X_W
        if(RightLine[i]<tempR)
        {
            InflectionR->X=RightLine[i];//存入拐点的（x,y）坐标
            InflectionR->Y=i;
            tempR=LeftLine[i];
        }
    }
}
