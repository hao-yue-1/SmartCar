/*
 * Filter.c
 *
 *  Created on: 2022年3月18日
 *      Author: yue
 */

#include "Filter.h"


/*
 *******************************************************************************************
 ** 函数功能: 一阶滞后滤波算法
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 ** 作    者: WBN
 ********************************************************************************************
 */
float FirstOrderLagFilter(float value)
{
    static float last_value;  //上一次滤波结果

    value=FIRST_LAG_P*value+(1-FIRST_LAG_P)*last_value; //一阶滞后滤波
    last_value=value;   //保存此次滤波结果为上一次滤波结果

    return value;
}


