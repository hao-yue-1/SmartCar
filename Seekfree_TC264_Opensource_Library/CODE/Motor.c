/*
 * Motor.c
 *
 *  Created on: 2021年12月19日
 *      Author: yue
 */

#include "Motor.h"
#include "Filter.h"         //滤波
#include <stdio.h>
#include "PID.h"            //PID
#include "protocol.h"

int16 speed_l,speed_r;  //电机左右速度目标值的全局变量

/*
*********************************************************************************************************
** 函 数 名: MotorSetPWM
** 功能说明: 控制左右电机的转速与正反转
** 形    参: left_speed：赋值给左电机的转速
**           right_speed：赋值给电机的转速
** 返 回 值: 无
*********************************************************************************************************
*/
void MotorSetPWM(int pwm_l,int pwm_r)
{
    //限幅处理
    if(pwm_l>MOTOR_PWM_MAX)
        pwm_l=MOTOR_PWM_MAX;
    else if(pwm_l<-MOTOR_PWM_MAX)
        pwm_l=-MOTOR_PWM_MAX;
    if(pwm_r>MOTOR_PWM_MAX)
        pwm_r=MOTOR_PWM_MAX;
    else if(pwm_r<-MOTOR_PWM_MAX)
        pwm_r=-MOTOR_PWM_MAX;
    //判断电机的正反转并进行速度赋值
    //下面是逐飞驱动板，使用一个IO口控制正反转，一个IO口输出PWM的控制方式
//    if(pwm_l>=0)   //左电机正转
//    {
//        gpio_set(P02_6,0);
//        pwm_duty(LEFT_MOTOR_PIN1,pwm_l);
//    }
//    else                //左电机反转
//    {
//        gpio_set(P02_6,1);
//        pwm_duty(LEFT_MOTOR_PIN1,-pwm_l);
//    }
//    if(pwm_r>=0)  //右电机正转
//    {
//        gpio_set(P02_7,0);
//        pwm_duty(RIGHT_MOTOR_PIN1,pwm_r);
//    }
//    else                //右电机反转
//    {
//        gpio_set(P02_7,1);
//        pwm_duty(RIGHT_MOTOR_PIN1,-pwm_r);
//    }

    //下面是自制驱动板，使用两个PWM信号驱动一个电机的驱动方法
    //左电机
    if(pwm_l>=0)   //正转
    {
        pwm_duty(LEFT_MOTOR_PIN1,pwm_l);
        pwm_duty(LEFT_MOTOR_PIN2,0);
    }
    else                //反转
    {
        pwm_duty(LEFT_MOTOR_PIN1,0);
        pwm_duty(LEFT_MOTOR_PIN2,-pwm_l);
    }
    //右电机
    if(pwm_r>=0)  //正转
    {
        pwm_duty(RIGHT_MOTOR_PIN1,0);
        pwm_duty(RIGHT_MOTOR_PIN2,pwm_r);
    }
    else                //反转
    {
        pwm_duty(RIGHT_MOTOR_PIN1,-pwm_r);
        pwm_duty(RIGHT_MOTOR_PIN2,0);
    }
}

/*
*********************************************************************************************************
** 函 数 名: MotorEncoder
** 功能说明: 读取左右电机的编码器的值
** 形    参: left_encoder：左电机编码器的值
**           right_encoder：右电机编码器的值
** 返 回 值: 无
** 注    意：由于编码器相对放置，向前运动时对右编码器取反处理
*********************************************************************************************************
*/
void MotorEncoder(int16* left_encoder,int16* right_encoder)
{
    //左编码器
    *left_encoder = gpt12_get(LEFT_ENCODER);
    gpt12_clear(LEFT_ENCODER);
    //右编码器
    *right_encoder = -gpt12_get(RIGHT_ENCODER);
    gpt12_clear(RIGHT_ENCODER);
}

/*
*********************************************************************************************************
** 函 数 名: MotorCtrl
** 功能说明: 使用增量式PI控制器控制左右电机转速
** 形    参: speed_l：左电机速度
**           speed_r：右电机速度
** 返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl(int16 speed_l,int16 speed_r)
{
    int16 encoder_l=0,encoder_r=0;   //左右电机编码器值
    int pwm_l=0,pwm_r=0;             //左右电机PWM

    MotorEncoder(&encoder_l,&encoder_r);              //获取左右电机编码器
    encoder_l=SecondOrderLagFilter_L(encoder_l);      //二阶低通滤波
    encoder_r=SecondOrderLagFilter_R(encoder_r);
    pwm_l=Speed_PI_Left(encoder_l,speed_l,MotorK);    //左右电机PID
    pwm_r=Speed_PI_Right(encoder_r,speed_r,MotorK);
    MotorSetPWM(pwm_l,pwm_r);                         //电机PWM赋值

    //野火上位机调试
//    int data_l=encoder_l,data_r=encoder_r;     //野火上位机只支持int型数据，这里必须做强制转换
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &data_l, 1);      //发送左编码器
//    set_computer_value(SEND_FACT_CMD, CURVES_CH2, &data_r, 1);      //发送右编码器
}

/*
*********************************************************************************************************
** 函 数 名: MotorSetTarget
** 功能说明: 设置左右电机速度的目标值
** 形    参: target_l：左电机速度目标值
**           target_r：右电机速度目标值
** 返 回 值: 无
*********************************************************************************************************
*/
void MotorSetTarget(int16 target_l,int16 target_r)
{
    speed_l=target_l;
    speed_r=target_r;
}
