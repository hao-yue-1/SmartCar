/*
 * Key.c
 *
 *  Created on: 2022年4月12日
 *      Author: yue
 */

#include "Key.h"
#include "zf_stm_systick.h"
#include "zf_gpio.h"
#include "ImageProcess.h"
#include "PID.h"            //修改PID参数
#include "SEEKFREE_18TFT.h" //LCD显示

/*
 ** 函数功能: 按键扫描处理
 ** 参    数: 无
 ** 返 回 值: 对应按下按键的值，0=无按键按下，其他对应查看宏定义
 ** 作    者: WBN
 */
uint8 KeyScan(void)
{
    static uint8 key_up=1;     //按键松开标志
    if(key_up&&(KEY_S1==1||KEY_S2==1||KEY_S3==1||KEY_S4==1||KEY_S5==1))
    {
        systick_delay_ms(STM0,10);
        key_up=0;
        if(KEY_S1==1)       return KEY_S1_PRES;
        else if(KEY_S2==1)  return KEY_S2_PRES;
        else if(KEY_S3==1)  return KEY_S3_PRES;
        else if(KEY_S4==1)  return KEY_S4_PRES;
        else if(KEY_S5==1)  return KEY_S5_PRES;
    }
    else if(KEY_S1==0||KEY_S2==0||KEY_S3==0||KEY_S4==0||KEY_S5==0)
        key_up=1;
    return 0;   //无按键按下
}

void KeyDisplay(uint8 flag)
{
    lcd_clear(WHITE);
    switch(flag)
    {
        case 0:
        {
            lcd_showstr(0, 0, "base_speed:");
            lcd_showint16(0, 1, base_speed);
            break;
        }
        case 1:
        {
            lcd_showstr(0, 0, "diff_speed_kp:");
            lcd_showfloat(0, 1, diff_speed_kp, 2, 3);
            break;
        }
        case 2:
        {
            lcd_showstr(0, 0, "Steer.P:");
            lcd_showfloat(0, 1, SteerK.P, 2, 3);
            break;
        }
        case 3:
        {
            lcd_showstr(0, 0, "Steer.D:");
            lcd_showfloat(0, 1, SteerK.D, 2, 3);
            break;
        }
        case 4:
        {
            lcd_showstr(0, 0, "Motor.P:");
            lcd_showfloat(0, 1, MotorK.P, 2, 3);
            break;
        }
        case 5:
        {
            lcd_showstr(0, 0, "Motor.I:");
            lcd_showfloat(0, 1, MotorK.I, 2, 3);
            break;
        }
    }
}

void ParameterDisplay(void)
{
    lcd_showint16(0, 0, base_speed);
    lcd_showfloat(0, 1, diff_speed_kp, 2, 3);
    lcd_showfloat(0, 2, SteerK.P, 2, 3);
    lcd_showfloat(0, 3, SteerK.D, 2, 3);
    lcd_showfloat(0, 4, MotorK.P, 2, 3);
    lcd_showfloat(0, 5, MotorK.I, 2, 3);
}

/*
 ** 函数功能: 按键调参
 ** 参    数: 无
 ** 返 回 值: 是否退出调参：1:退出调参  0:继续调参
 ** 作    者: WBN
 */
uint8 KeyParameter(void)
{
    static uint8 key_num;   //调整的参数
    switch(KeyScan())
    {
        case KEY_S1_PRES:   //S1：选择下一个参数
        {
            if(key_num>0)
            {
                key_num--;  //选择下一个参数
            }
            KeyDisplay(key_num);
            break;
        }
        case KEY_S2_PRES:   //S2：选择上一个参数
        {
            if(key_num<5)
            {
                key_num++;  //选择上一个参数
            }
            KeyDisplay(key_num);
            break;
        }
        case KEY_S3_PRES:   //S3：目前参数调大
        {
            KeyDisplay(key_num);
            switch(key_num)
            {
                case 0: //基础速度
                {
                    base_speed+=1;
                    lcd_showint16(0, 1, base_speed);
                    break;
                }
                case 1: //差速
                {
                    diff_speed_kp+=0.1;
                    lcd_showfloat(0, 1, diff_speed_kp, 2, 3);
                    break;
                }
                case 2: //舵机P
                {
                    SteerK.P+=0.1;
                    lcd_showfloat(0, 1, SteerK.P, 2, 3);
                    break;
                }
                case 3: //舵机D
                {
                    SteerK.D+=0.1;
                    lcd_showfloat(0, 1, SteerK.D, 2, 3);
                    break;
                }
                case 4: //电机P
                {
                    MotorK.P+=0.1;
                    lcd_showfloat(0, 1, MotorK.P, 2, 3);
                    break;
                }
                case 5: //电机I
                {
                    MotorK.I+=0.1;
                    lcd_showfloat(0, 1, MotorK.I, 2, 3);
                    break;
                }
                default:
                {
                    lcd_showstr(0, 4, "ERROR!!!");
                    break;
                }
            }
            break;
        }
        case KEY_S4_PRES:   //S4：目前参数调小
        {
            KeyDisplay(key_num);
            switch(key_num)
            {
                case 0: //基础速度
                {
                    base_speed-=1;
                    lcd_showint16(0, 1, base_speed);
                    break;
                }
                case 1: //差速
                {
                    diff_speed_kp-=0.1;
                    lcd_showfloat(0, 1, diff_speed_kp, 2, 3);
                    break;
                }
                case 2: //舵机P
                {
                    SteerK.P-=0.1;
                    lcd_showfloat(0, 1, SteerK.P, 2, 3);
                    break;
                }
                case 3: //舵机D
                {
                    SteerK.D-=0.1;
                    lcd_showfloat(0, 1, SteerK.D, 2, 3);
                    break;
                }
                case 4: //电机P
                {
                    MotorK.P-=0.1;
                    lcd_showfloat(0, 1, MotorK.P, 2, 3);
                    break;
                }
                case 5: //电机I
                {
                    MotorK.I-=0.1;
                    lcd_showfloat(0, 1, MotorK.I, 2, 3);
                    break;
                }
                default:
                {
                    lcd_showstr(0, 4, "ERROR!!!");
                    break;
                }
            }
            break;
        }
        case KEY_S5_PRES:   //S5：参数赋值，退出调参
        {
            lcd_clear(WHITE);
            lcd_showint16(0, 0, base_speed);
            lcd_showfloat(0, 1, diff_speed_kp, 2, 3);
            lcd_showfloat(0, 2, SteerK.P, 2, 3);
            lcd_showfloat(0, 3, SteerK.D, 2, 3);
            lcd_showfloat(0, 4, MotorK.P, 2, 3);
            lcd_showfloat(0, 5, MotorK.I, 2, 3);
            systick_delay_ms(STM0,4000);
            return 1;
        }
        default:break;
    }
    systick_delay_ms(STM0,100);
    return 0;
}



