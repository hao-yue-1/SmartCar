/*
 * Motor.c
 *
 *  Created on: 2021年12月19日
 *      Author: yue
 */

#include "Motor.h"
#include "Filter.h"         //滤波
#include <stdio.h>
#include "BluetoothSend.h"  //蓝牙调参
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
    if(pwm_l>GTM_ATOM0_PWM_DUTY_MAX)
        pwm_l=GTM_ATOM0_PWM_DUTY_MAX;
    else if(pwm_l<-GTM_ATOM0_PWM_DUTY_MAX)
        pwm_l=-GTM_ATOM0_PWM_DUTY_MAX;
    if(pwm_r>GTM_ATOM0_PWM_DUTY_MAX)
        pwm_r=GTM_ATOM0_PWM_DUTY_MAX;
    else if(pwm_r<-GTM_ATOM0_PWM_DUTY_MAX)
        pwm_r=-GTM_ATOM0_PWM_DUTY_MAX;
    //判断电机的正反转并进行速度赋值
    if(pwm_l>=0)   //左电机正转
    {
        gpio_set(P02_6,0);
        pwm_duty(LEFT_MOTOR_PIN1,pwm_l);
    }
    else                //左电机反转
    {
        gpio_set(P02_6,1);
        pwm_duty(LEFT_MOTOR_PIN1,-pwm_l);
    }
    if(pwm_r>=0)  //右电机正转
    {
        gpio_set(P02_7,0);
        pwm_duty(RIGHT_MOTOR_PIN1,pwm_r);
    }
    else                //右电机反转
    {
        gpio_set(P02_7,1);
        pwm_duty(RIGHT_MOTOR_PIN1,-pwm_r);
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
//    *left_encoder=FirstOrderLagFilter(*left_encoder);   //滤波
    gpt12_clear(LEFT_ENCODER);

    //右编码器
    *right_encoder = -gpt12_get(RIGHT_ENCODER);
//    *right_encoder=FirstOrderLagFilter(*right_encoder); //滤波
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
    pwm_l=Speed_PI_Left(encoder_l,speed_l,MotorK);    //左右电机PID
    pwm_r=Speed_PI_Right(encoder_r,speed_r,MotorK);
    MotorSetPWM(pwm_l,pwm_r);                         //电机PWM赋值

//    BluetoothSendToApp(encoder_l,encoder_r);          //蓝牙调参
    int32 encoder=encoder_l;
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &encoder, 1);   //野火上位机给通道1发送实际值
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
