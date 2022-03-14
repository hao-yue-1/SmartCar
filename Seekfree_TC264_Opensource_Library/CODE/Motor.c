/*
 * Motor.c
 *
 *  Created on: 2021年12月19日
 *      Author: yue
 */

#include "Motor.h"

/*
*********************************************************************************************************
** 函 数 名: MotorCtrl
** 功能说明: 控制左右电机的转速与正反转
** 形    参: left_speed：赋值给左电机的转速
**           right_speed：赋值给电机的转速
** 返 回 值: 无
*********************************************************************************************************
*/
void MotorCtrl(int left_speed,int right_speed)
{
    //限幅处理
    if(left_speed>GTM_ATOM0_PWM_DUTY_MAX)
        left_speed=GTM_ATOM0_PWM_DUTY_MAX;
    else if(left_speed<-GTM_ATOM0_PWM_DUTY_MAX)
        left_speed=-GTM_ATOM0_PWM_DUTY_MAX;
    if(right_speed>GTM_ATOM0_PWM_DUTY_MAX)
        right_speed=GTM_ATOM0_PWM_DUTY_MAX;
    else if(right_speed<-GTM_ATOM0_PWM_DUTY_MAX)
        right_speed=-GTM_ATOM0_PWM_DUTY_MAX;
    //判断电机的正反转并进行速度赋值
    if(left_speed>=0)   //左电机正转
    {
        gpio_set(P02_6,1);
        pwm_duty(LEFT_MOTOR_PIN1,left_speed);
    }
    else                //左电机反转
    {
        gpio_set(P02_6,0);
        pwm_duty(LEFT_MOTOR_PIN1,-left_speed);
    }
    if(right_speed>=0)  //右电机正转
    {
        gpio_set(P02_7,0);
        pwm_duty(RIGHT_MOTOR_PIN1,right_speed);
    }
    else                //右电机反转
    {
        gpio_set(P02_7,1);
        pwm_duty(RIGHT_MOTOR_PIN1,-right_speed);
    }

    //下面注释的是使用两个PWM信号驱动一个电机的驱动方法
//    //左电机
//    if(left_speed>=0)   //正转
//    {
//        pwm_duty(LEFT_MOTOR_PIN1,0);
//        pwm_duty(LEFT_MOTOR_PIN2,left_speed);
//    }
//    else                //反转
//    {
//        pwm_duty(LEFT_MOTOR_PIN1,-left_speed);
//        pwm_duty(LEFT_MOTOR_PIN2,0);
//    }
//    //右电机
//    if(right_speed>=0)  //正转
//    {
//        pwm_duty(RIGHT_MOTOR_PIN1,right_speed);
//        pwm_duty(RIGHT_MOTOR_PIN2,0);
//    }
//    else                //反转
//    {
//        pwm_duty(RIGHT_MOTOR_PIN1,0);
//        pwm_duty(RIGHT_MOTOR_PIN2,-right_speed);
//    }
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

