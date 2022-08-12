/*
 * PID.c
 *
 *  Created on: 2021年12月10日
 *      Author: yue
 */

#include "PID.h"
#include "Filter.h" //滤波算法
#include "Steer.h"  //舵机
#include "Motor.h"  //电机

SteerPID SteerK;            //舵机PID参数
MotorPID MotorK_L,MotorK_R; //电机PID参数

/********************************************************************************************
 ** 函数功能: 两个PID参数的赋值初始化
 ** 参    数: SteerPID *SteerK
 **           MotorPID *MotorK
 ** 返 回 值: 无
 ** 作    者: LJF
 *********************************************************************************************/
void PID_init(SteerPID *SteerK,MotorPID *MotorK_L,MotorPID *MotorK_R)
{
    SteerK->P=15.25;SteerK->I=0;SteerK->D=30;           //初始化舵机的PID参数   //校赛参数19.25 5   //华南赛参数14.25 30
    MotorK_L->P=180;MotorK_L->I=0.45;MotorK_L->D=0;     //初始化电机的PID参数   //校赛参数80 0.5    //华南赛参数180 0.45
    MotorK_R->P=180;MotorK_R->I=0.45;MotorK_R->D=0;     //初始化电机的PID参数   //校赛参数80 0.5    //华南赛参数180 0.45
}

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
uint32 Steer_Position_PID(float SlopeBias,SteerPID K)//舵机位置式PID控制，采用分段式PID控制
{
    static float LastSlopeBias;
    int PWM;
    PWM=(int)(K.P*SlopeBias+K.D*(SlopeBias-LastSlopeBias));
    LastSlopeBias=FirstOrderLagFilter(SlopeBias);   //一阶低通滤波
    return STEER_MID+PWM;//假设斜率的范围为[-5,5]，而舵机打角PWM的范围为[850,680]，减去中值之后就能映射到[-85,85]，于此对应，所以返回值应该负号再加中值，KP先猜测为17
}

/*
 *******************************************************************************************
 ** 函数功能: 电机增量式速度PI控制器，根据编码器的值来不断矫正电机的速度   //左电机
 ** 参    数: left_encoder：左电机编码器的值
 **           left_target ：左电机目标速度
 **           K           : 电机PID参数
 ** 返 回 值: 给电机的PWM
 ** 作    者: WBN
 ** 注    意：使用增量式PI控制的优点：
 **           1.算式中没有累加的过程，控制量只与近两次采样值有关，不容易产生大的误差
 **           2.输出的是增量，即变化量，可以有更好的容错
 ********************************************************************************************
 */
int Speed_PI_Left(int16 left_encoder,int16 left_target,MotorPID K)
{
    static int Bias,Last_Bias,PWM,Last_2_Bias;

    Bias=left_target-left_encoder;              //求出当前偏差
    PWM+=(int)(K.P*(Bias-Last_Bias)+K.I*Bias+K.D*(Bias-2*Last_Bias+Last_2_Bias));  //增量式PID，并把结果直接叠加在上一次的PWM上

    Last_2_Bias=Last_Bias;    //保存上一次偏差
    Last_Bias=Bias;           //保存这一次偏差

    //PID输出限幅，防止由于电机和PID工作不同步导致电机超调烧毁的问题
    //注意：对PID输出限幅后会导致响应变慢
//    if(PWM>MOTOR_PWM_MAX)
//    {
//        PWM=MOTOR_PWM_MAX;
//    }
//    else if(PWM<-MOTOR_PWM_MAX)
//    {
//        PWM=-MOTOR_PWM_MAX;
//    }

    return PWM;         //返回可以直接赋值给电机的PWM
}

/*
 *******************************************************************************************
 ** 函数功能: 电机增量式速度PI控制器，根据编码器的值来不断矫正电机的速度   //右电机
 ** 参    数: left_encoder：左电机编码器的值
 **           left_target ：左电机目标速度
 **           K           : 电机PID参数
 ** 返 回 值: 给电机的PWM
 ** 作    者: WBN
 ** 注    意：使用增量式PI控制的优点：
 **           1.算式中没有累加的过程，控制量只与近两次采样值有关，不容易产生大的误差
 **           2.输出的是增量，即变化量，可以有更好的容错
 ********************************************************************************************
 */
int Speed_PI_Right(int16 right_encoder,int16 right_target,MotorPID K)
{
    static int Bias,Last_Bias,PWM,Last_2_Bias;

    Bias=right_target-right_encoder;              //求出当前偏差
    PWM+=(int)(K.P*(Bias-Last_Bias)+K.I*Bias+K.D*(Bias-2*Last_Bias+Last_2_Bias));  //增量式PID，并把结果直接叠加在上一次的PWM上

    Last_2_Bias=Last_Bias;    //保存上一次偏差
    Last_Bias=Bias;           //保存这一次偏差

    //PID输出限幅，防止由于电机和PID工作不同步导致电机超调烧毁的问题
    //注意：对PID输出限幅后会导致响应变慢
//    if(PWM>MOTOR_PWM_MAX)
//    {
//        PWM=MOTOR_PWM_MAX;
//    }
//    else if(PWM<-MOTOR_PWM_MAX)
//    {
//        PWM=-MOTOR_PWM_MAX;
//    }

    return PWM;         //返回可以直接赋值给电机的PWM
}
