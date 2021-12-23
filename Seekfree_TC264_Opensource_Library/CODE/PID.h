/*
 * PID.h
 *
 *  Created on: 2021年12月10日
 *      Author: yue
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

//舵机PID控制的结构体，便于分段控制PID，图像识别到特殊路段之后PID的参数就发送变化
typedef struct SteerPID
{
    int P;
    int D;
}SteerPID;

//这里还要写个PID初始化的函数调参在这里进行
int Steer_Position_PID(float SlopeBias,SteerPID K);//舵机位置式PID控制，采用分段式PID控制

#endif /* CODE_PID_H_ */
