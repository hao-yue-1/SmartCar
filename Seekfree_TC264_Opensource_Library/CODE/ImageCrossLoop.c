/*
 * ImageCrossLoop.c
 *
 *  Created on: 2022年5月25日
 *      Author: yue
 */

#include "ImageSpecial.h"
#include "PID.h"

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
            systick_delay_ms(STM0,300);
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
            base_speed=140;     //降速出环
            Bias=-10;
            systick_delay_ms(STM0,300);
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
