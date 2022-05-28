/*
 * PID.h
 *
 *  Created on: 2021年12月10日
 *      Author: yue
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "common.h" //数据类型声明

//舵机PID控制的结构体，便于分段控制PID，图像识别到特殊路段之后PID的参数就发送变化
typedef struct SteerPID
{
    float P;
    float I;
    float D;
}SteerPID;
//电机PID控制的结构体
typedef struct MotorPID
{
    float P;
    float I;
    float D;
}MotorPID;

extern SteerPID SteerK;     //舵机的全局变量
extern MotorPID MotorK_L,MotorK_R;     //电机的全局变量
extern int16 base_speed;    //基础速度的全局变量
extern uint32 SteerPWM;     //舵机PWM

//这里还要写个PID初始化的函数调参在这里进行
void PID_init(SteerPID *SteerK,MotorPID *MotorK_L,MotorPID *MotorK_R);    //两个PID参数的赋值初始化
uint32 Steer_Position_PID(float SlopeBias,SteerPID K);                    //舵机位置式PID控制，采用分段式PID控制
int Speed_PI_Left(int16 left_encoder,int16 left_target,MotorPID K);       //电机增量式PI控制 //左电机
int Speed_PI_Right(int16 right_encoder,int16 right_target,MotorPID K);    //电机增量式PI控制  //右电机

#endif /* CODE_PID_H_ */
