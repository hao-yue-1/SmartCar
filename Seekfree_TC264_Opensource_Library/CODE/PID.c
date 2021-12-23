/*
 * PID.c
 *
 *  Created on: 2021年12月10日
 *      Author: yue
 */
#include "PID.h"

/*
 *******************************************************************************************
 ** 函数功能: 根据偏差来求舵机PWM
 ** 参    数: float SlopeBias:    最小回归方程得出来的偏差
 **           SteerPID K:         进行PID的舵机PID参数
 ** 返 回 值: 给舵机的PWM
 ** 作    者: LJF
 ** 注    意：偏差是传进来还是传的是中线进来再根据那个函数求偏差？
 **           返回出去的PWM可能很小看看是在这里对PWM进行缩放还是把它归进去参数?
 **           Bias的正负是这里处理还是传进来之前，这个问题跟第一个问题有关联？
 ********************************************************************************************
 */
int Steer_Position_PID(float SlopeBias,SteerPID K)//舵机位置式PID控制，采用分段式PID控制
{
    static float LastSlopeBias;
    int PWM;
    PWM=K.P*SlopeBias+K.D*(SlopeBias-LastSlopeBias);
    LastSlopeBias=SlopeBias;
    return PWM;
}

