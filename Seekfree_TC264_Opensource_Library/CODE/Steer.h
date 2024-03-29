/*
 * Steer.h
 *
 *  Created on: 2021年12月9日
 *      Author: yue
 */

#include "headfile.h"       //逐飞的封装库

#ifndef CODE_STEER_H_
#define CODE_STEER_H_

#define STEER_PIN   ATOM1_CH1_P33_9     //输出舵机PWM引脚
#define STEER_MID   755                 //舵机居中值
#define STEER_LEFT  835                 //舵机向左打死值
#define STEER_RIGHT 675                 //舵机向右打死值

void SteerCtrl(uint32 pwm);     //控制舵机

#endif /* CODE_STEER_H_ */
