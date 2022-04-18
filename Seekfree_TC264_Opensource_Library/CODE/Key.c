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

/*
 ** 函数功能: 按键调参
 ** 参    数: 无
 ** 返 回 值: 是否退出调参：1:退出调参  0:继续调参
 ** 作    者: WBN
 */
uint8 KeyParameter(void)
{
    static uint8 key_num;   //调整的参数
    static float key_steer_p=SteerK.P,key_steer_d=SteerK.D,key_motor_p=MotorK.P,key_motor_i=MotorK.I,key_diff_speed_kp=diff_speed_kp;    //初始化调节的PID参数
    static int16 key_base_speed=base_speed; //初始化调节的基础速度
    switch(KeyScan())
    {
        case KEY_S1_PRES:   //S1：选择下一个参数
        {
            key_num++;  //选择下一个参数
            break;
        }
        case KEY_S2_PRES:   //S2：选择上一个参数
        {
            key_num--;  //选择上一个参数
            break;
        }
        case KEY_S3_PRES:   //S3：目前参数调大
        {
            switch(key_num)
            {
                case 0: //基础速度
                {
                    key_base_speed+=1;
                    lcd_showstr(0, 0, "Base Speed:")
                    lcd_showint16(0, 1, key_base_speed);
                }
                case 1: //差速
                {
                    key_diff_speed_kp+=0.1;
                    lcd_showstr(0, 0, "Different Speed:")
                    lcd_showfloat(0, 1, key_diff_speed_kp, 2, 3);
                }
                case 2: //舵机P
                {
                    key_steer_p+=0.1;
                    lcd_showstr(0, 0, "Steer.P:")
                    lcd_showfloat(0, 1, key_steer_p, 2, 3);
                }
                case 3: //舵机D
                {
                    key_steer_d+=0.1;
                    lcd_showstr(0, 0, "Steer.D:")
                    lcd_showfloat(0, 1, key_steer_d, 2, 3);
                }
                case 4: //电机P
                {
                    key_motor_p+=0.1;
                    lcd_showstr(0, 0, "Motor.P:")
                    lcd_showfloat(0, 1, key_motor_p, 2, 3);
                }
                case 5: //电机I
                {
                    key_motor_i+=0.1;
                    lcd_showstr(0, 0, "Motor.I:")
                    lcd_showfloat(0, 1, key_motor_i, 2, 3);
                }

            }
            break;
        }
        case KEY_S4_PRES:   //S4：目前参数调小
        {
            switch(key_num)
            {
                case 0: //基础速度
                {
                    key_base_speed-=1;
                    lcd_showstr(0, 0, "Base Speed:")
                    lcd_showint16(0, 1, key_base_speed);
                }
                case 1: //差速
                {
                    key_diff_speed_kp-=0.1;
                    lcd_showstr(0, 0, "Different Speed:")
                    lcd_showfloat(0, 1, key_diff_speed_kp, 2, 3);
                }
                case 2: //舵机P
                {
                    key_steer_p-=0.1;
                    lcd_showstr(0, 0, "Steer.P:")
                    lcd_showfloat(0, 1, key_steer_p, 2, 3);
                }
                case 3: //舵机D
                {
                    key_steer_d-=0.1;
                    lcd_showstr(0, 0, "Steer.D:")
                    lcd_showfloat(0, 1, key_steer_d, 2, 3);
                }
                case 4: //电机P
                {
                    key_motor_p-=0.1;
                    lcd_showstr(0, 0, "Motor.P:")
                    lcd_showfloat(0, 1, key_motor_p, 2, 3);
                }
                case 5: //电机I
                {
                    key_motor_i-=0.1;
                    lcd_showstr(0, 0, "Motor.I:")
                    lcd_showfloat(0, 1, key_motor_i, 2, 3);
                }

            }
            break;
        }
        case KEY_S5_PRES:   //S5：参数赋值，退出调参
        {
            base_speed=key_base_speed;
            diff_speed_kp=key_diff_speed_kp;
            SteerK.P=key_steer_p;
            SteerK.D=key_steer_d;
            MotorK.P=key_motor_p;
            MotorK.I=key_motor_i;
            return 1;
        }
    }
    systick_delay_ms(STM0,100);
    return 0;
}



