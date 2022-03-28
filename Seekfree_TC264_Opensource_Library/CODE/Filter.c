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

/*
 *******************************************************************************************
 ** 函数功能: 二阶滞后滤波算法
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 ** 作    者: WBN
 ********************************************************************************************
 */
int16 SecondOrderLagFilter_L(int16 value)
{
    static int16 last_value,last_2_value;

    value=0.2*value+0.4*last_value+0.4*last_2_value;     //二阶滞后滤波

    last_2_value=last_value;
    last_value=value;

    return value;
}

/*
 *******************************************************************************************
 ** 函数功能: 二阶滞后滤波算法
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 ** 作    者: WBN
 ********************************************************************************************
 */
int16 SecondOrderLagFilter_R(int16 value)
{
    static int16 last_value,last_2_value;

    value=0.2*value+0.4*last_value+0.4*last_2_value;     //二阶滞后滤波

    last_2_value=last_value;
    last_value=value;

    return value;
}
