#include "ImageBasic.h"

//变量定义
int mid=MT9V03X_W/2;    //初始化扫线的中点为图像中点

/*
 ** 函数功能: 扫线提取左中右三线
 ** 参    数: uint8 *LeftLine：    左线数组
 **           uint8* CentreLine：中线数组
 **           uint8 *RightLine：  右线数组
 ** 返 回 值: 无
 ** 作    者: LJF
 */

void GetImagBasic(int *LeftLine, int *CentreLine, int *RightLine)
{
    int row,cloum;   //行,列
    //开始扫线（从下往上，从中间往两边）
    for(row=0;row<MT9V03X_H;row++) //图像的原点在左下角
    {
        //左边扫线
        for(cloum=mid;(cloum-BORDER_BIAS)>0;cloum--)
        {
            if(BinaryImage[row][cloum]==0&&BinaryImage[row][cloum-BORDER_BIAS]==0)  //判断左边界点（BORDER_BIAS防偶然因素）
            {
                LeftLine[row]=cloum;    //记录左边界点
                break;
            }
        }
        //右边扫线
        for(cloum=mid;(cloum+BORDER_BIAS)<MT9V03X_W;cloum++)
        {
            if(BinaryImage[row][cloum]==0&&BinaryImage[row][cloum+BORDER_BIAS]==0)  //判断右边界点（BORDER_BIAS防偶然因素）
            {
                RightLine[row]=cloum;   //记录右边界点
                break;
            }
        }
        //数据处理
        CentreLine[row]=(LeftLine[row]+RightLine[row])/2;   //记录中线
        mid=CentreLine[row];      //以上一次的中线值为下一次扫线的中间点
    }
}

