/*
 * Motor.h
 *
 *  Created on: 2021年12月19日
 *      Author: yue
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_gtm_pwm.h"
#include "zf_gpt12.h"
#include "zf_gpio.h"

//电机PWM限幅宏定义
#define MOTOR_PWM_MAX   8300

//定义控制电机的PWM引脚
#define LEFT_MOTOR_PIN1     ATOM0_CH4_P02_4
#define LEFT_MOTOR_PIN2     ATOM0_CH6_P02_6
#define RIGHT_MOTOR_PIN1    ATOM0_CH5_P02_5
#define RIGHT_MOTOR_PIN2    ATOM0_CH7_P02_7
//定义控制编码器的定时器
#define LEFT_ENCODER    GPT12_T2
#define RIGHT_ENCODER   GPT12_T6

extern int16 speed_l,speed_r;       //左右电机速度目标值的全局变量

void MotorSetPWM(int pwm_l,int pwm_r);                          //控制左右电机的转速与正反转
void MotorEncoder(int16* left_encoder,int16* right_encoder);    //读取左右电机编码器的值
void MotorCtrl(int16 speed_l,int16 speed_r);                    //使用PI控制器控制电机速度
void MotorSetTarget(int16 target_l,int16 target_r);             //设置左右电机速度的目标值

#endif /* CODE_MOTOR_H_ */
