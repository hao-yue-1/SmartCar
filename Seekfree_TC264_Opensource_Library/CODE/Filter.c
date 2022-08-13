/*
 * Filter.c
 *
 *  Created on: 2022年3月18日
 *      Author: yue
 */

#include "Filter.h"

/*
 *******************************************************************************************
 ** 函数功能: 一阶滞后滤波算法，Bias滤波器
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
 ** 函数功能: 一阶滞后滤波算法，Slope滤波器
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 ** 作    者: WBN
 ********************************************************************************************
 */
float FirstOrderLagFilter_Slope(float value)
{
    static float last_value;  //上一次滤波结果

    value=FIRST_LAG_P*value+(1-FIRST_LAG_P)*last_value; //一阶滞后滤波
    last_value=value;   //保存此次滤波结果为上一次滤波结果

    return value;
}

/*
 *******************************************************************************************
 ** 函数功能: 二阶滞后滤波算法，左编码器滤波器
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
 ** 函数功能: 二阶滞后滤波算法，右编码器滤波器
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

/**
 * @brief           一阶卡尔曼滤波初始化
 * @param[out]      state : 滤波结构数据指针
 * @param[in]       q & r
 */
void kalman1_init(kalman1_filter_t* state, float q, float r)
{
    state->x = 0;
    state->p = 0.0f;
    state->A = 1.0f;
    state->H = 1.0f;
    state->q = q;
    state->r = r;
}

/**
 * @brief           一阶卡尔曼滤波
 * @param[out]      state : 滤波结构数据指针
 * @param[in]       z_measure : 原始数据
 */
int16 kalman1_filter(kalman1_filter_t* state, float z_measure)
{
    /* Predict */
    // 时间更新(预测): X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
    state->x = state->A * state->x;
    // 更新先验协方差: P(k|k-1) = A(k,k-1)*A(k,k-1)^T*P(k-1|k-1)+Q(k)
    state->p = state->A * state->A * state->p + state->q;

    /* Measurement */
    // 计算卡尔曼增益: K(k) = P(k|k-1)*H(k)^T/(P(k|k-1)*H(k)*H(k)^T + R(k))
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    // 测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    // 更新后验协方差: P(k|k) =（I-K(k)*H(k))*P(k|k-1)
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}
