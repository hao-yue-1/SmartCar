/*
 * FuzzyPID.c
 *
 *  Created on: 2022年1月21日
 *  Author: 30516
 *  Effect: 用于模糊PID
 */

#include "FuzzyPID.h"

/********************************************************************************************
 ** 函数功能: 求出偏差的隶属值与隶属度
 ** 参    数: float E: 偏差/偏差论域宏定义（把他们限制在-3~3之间）
 **           float Membership[2]: 用于存储传进来参数两个隶属度的数组，M[0]是比较小的模糊子集的隶属度
 **           int Index[2]: 索引数组，用于索引规则表，跟Membership同理M[0]是比较小的模糊子集的索引值
 ** 返 回 值: 无
 ** 作    者: LJF
 *********************************************************************************************/
void ClacMembership(float E,float Membership[2],int Index[2])
{
    /*三角隶属度函数*/
    if(E>=NB && E<=NM)
    {
        Index[0]=0;//NB
        Index[1]=1;//NM
        Membership[0]=NM-E;//隶属于NB的隶属度(NM-E)/(NM-NB)，因为分母都是1
        Membership[1]=E-NB;//隶属于NM的隶属度(E-NB)/(NM-NB)
    }
    else if(E>=NM && E<=NS)
    {
        Index[0]=1;
        Index[1]=2;
        Membership[0]=NS-E;//隶属于NM的隶属度
        Membership[1]=E-NM;//隶属于NS的隶属度
    }
    else if(E>=NS && E<=ZO)
    {
        Index[0]=2;
        Index[1]=3;
        Membership[0]=ZO-E;//隶属于NS的隶属度
        Membership[1]=E-NS;//隶属于ZO的隶属度
    }
    else if(E>=ZO && E<=PS)
    {
        Index[0]=3;
        Index[1]=4;
        Membership[0]=PS-E;//隶属于ZO的隶属度
        Membership[1]=E-ZO;//隶属于PS的隶属度
    }
    else if(E>=PS && E<=PM)
    {
        Index[0]=4;
        Index[1]=5;
        Membership[0]=PM-E;//隶属于PS的隶属度
        Membership[1]=E-PS;//隶属于PM的隶属度
    }
    else if(E>=PM && E<=PB)
    {
        Index[0]=5;
        Index[1]=6;
        Membership[0]=PB-E;//隶属于PS的隶属度
        Membership[1]=E-PM;//隶属于PM的隶属度
    }
}

/********************************************************************************************
 ** 函数功能: 根据模糊规则表以及重心法求出输出
 ** 参    数: int IndexE[2]: 偏差的索引数组
 **           float MSE[2]: 偏差对于两个隶属值的隶属度数组
 **           int IndexEC[2]: 偏差变化率的索引数组
 **           float MSEC[2]: 偏差变化率对于两个隶属值的隶属度数组
 **           int type: 1：KP 2：KI 3：KD
 ** 返 回 值: K的增量K=K0+detaK*比例系数
 ** 作    者: LJF
 *********************************************************************************************/
int  SolutionFuzzy(int IndexE[2],float MSE[2],int IndexEC[2],float MSEC[2],int type)
{
    int temp=0;
    switch(type)
    {
        case 1:
            //重心法求解
            temp=KPFuzzyRule[IndexE[0]][IndexEC[0]]*MSE[0]*MSEC[0]+
                 KPFuzzyRule[IndexE[0]][IndexEC[1]]*MSE[0]*MSEC[1]+
                 KPFuzzyRule[IndexE[1]][IndexEC[0]]*MSE[1]*MSEC[0]+
                 KPFuzzyRule[IndexE[1]][IndexEC[1]]*MSE[1]*MSEC[1];
            break;
        case 2:
            temp=KIFuzzyRule[IndexE[0]][IndexEC[0]]*MSE[0]*MSEC[0]+
                 KIFuzzyRule[IndexE[0]][IndexEC[1]]*MSE[0]*MSEC[1]+
                 KIFuzzyRule[IndexE[1]][IndexEC[0]]*MSE[1]*MSEC[0]+
                 KIFuzzyRule[IndexE[1]][IndexEC[1]]*MSE[1]*MSEC[1];
            break;
        case 3:
            temp=KDFuzzyRule[IndexE[0]][IndexEC[0]]*MSE[0]*MSEC[0]+
                 KDFuzzyRule[IndexE[0]][IndexEC[1]]*MSE[0]*MSEC[1]+
                 KDFuzzyRule[IndexE[1]][IndexEC[0]]*MSE[1]*MSEC[0]+
                 KDFuzzyRule[IndexE[1]][IndexEC[1]]*MSE[1]*MSEC[1];
            break;
        default:
            break;
    }
    return temp;
}

