/*
 * Steer.c
 *
 *  Created on: 2021年12月9日
 *      Author: yue
 */

#include "Steer.h"

/*
*********************************************************************************************************
** 函 数 名: SteerCtrl
** 功能说明: 赋值控制舵机的PWM，并做一些相应的限幅处理
** 形    参: pwm:控制舵机的pwm信号
** 返 回 值: 无
*********************************************************************************************************
*/
void SteerCtrl(uint16 pwm)
{
    //进行限幅操作,防止左右打死
    if(pwm>STEER_LEFT)          pwm=STEER_LEFT;
    else if(pwm<STEER_RIGHT)    pwm=STEER_RIGHT;
    //对PWM信号进行赋值
    pwm_duty(STEER_PIN,pwm);  //调用逐飞封装的PWM占空比设置函数
}
