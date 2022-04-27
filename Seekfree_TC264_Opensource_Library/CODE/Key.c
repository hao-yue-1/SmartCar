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
 ** 函数功能: 参数显示，显示对应的参数
 ** 参    数: 对应参数的flag
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyParameterDisplay(uint8 flag)
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
        case 6:
        {
            lcd_showstr(0, 0, "Motor.D:");
            lcd_showfloat(0, 1, MotorK.D, 2, 3);
            break;
        }
    }
}

/*
 ** 函数功能: 参数显示，显示所有参数
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void ParameterDisplay(void)
{
    lcd_clear(WHITE);
    lcd_showint16(0, 0, base_speed);
    lcd_showfloat(0, 1, diff_speed_kp, 2, 3);
    lcd_showfloat(0, 2, SteerK.P, 2, 3);
    lcd_showfloat(0, 3, SteerK.D, 2, 3);
    lcd_showfloat(0, 4, MotorK.P, 2, 3);
    lcd_showfloat(0, 5, MotorK.I, 2, 3);
    lcd_showfloat(0, 6, MotorK.D, 2, 3);
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
            KeyParameterDisplay(key_num);
            break;
        }
        case KEY_S2_PRES:   //S2：选择上一个参数
        {
            if(key_num<6)
            {
                key_num++;  //选择上一个参数
            }
            KeyParameterDisplay(key_num);
            break;
        }
        case KEY_S3_PRES:   //S3：目前参数调大
        {
            KeyParameterDisplay(key_num);
            switch(key_num)
            {
                case 0: //基础速度
                {
                    base_speed+=5;
                    lcd_showint16(0, 1, base_speed);
                    break;
                }
                case 1: //差速
                {
                    diff_speed_kp+=0.05;
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
                    SteerK.D+=1;
                    lcd_showfloat(0, 1, SteerK.D, 2, 3);
                    break;
                }
                case 4: //电机P
                {
                    MotorK.P+=1;
                    lcd_showfloat(0, 1, MotorK.P, 2, 3);
                    break;
                }
                case 5: //电机I
                {
                    MotorK.I+=0.1;
                    lcd_showfloat(0, 1, MotorK.I, 2, 3);
                    break;
                }
                case 6:
                {
                    MotorK.D+=0.1;
                    lcd_showfloat(0, 1, MotorK.D, 2, 3);
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
            KeyParameterDisplay(key_num);
            switch(key_num)
            {
                case 0: //基础速度
                {
                    base_speed-=5;
                    lcd_showint16(0, 1, base_speed);
                    break;
                }
                case 1: //差速
                {
                    diff_speed_kp-=0.05;
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
                    SteerK.D-=1;
                    lcd_showfloat(0, 1, SteerK.D, 2, 3);
                    break;
                }
                case 4: //电机P
                {
                    MotorK.P-=1;
                    lcd_showfloat(0, 1, MotorK.P, 2, 3);
                    break;
                }
                case 5: //电机I
                {
                    MotorK.I-=0.1;
                    lcd_showfloat(0, 1, MotorK.I, 2, 3);
                    break;
                }
                case 6:
                {
                    MotorK.D-=0.1;
                    lcd_showfloat(0, 1, MotorK.D, 2, 3);
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
            ParameterDisplay();
            systick_delay_ms(STM0,3000);
            return 1;
        }
        default:break;
    }
    systick_delay_ms(STM0,100);
    return 0;
}

/*
 ** 函数功能: 速度显示，显示对应状态的速度
 ** 参    数: 对应速度的flag
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyBaseSpeedDisplay(uint8 flag)
{
    lcd_clear(WHITE);
    switch(flag)
    {
        case 0:
        {
            lcd_showstr(0, 0, "Left_CircleIsland:");
            lcd_showint16(0, 1, base_speed);
            break;
        }
        case 1:
        {
            lcd_showstr(0, 0, "First_CrossLoop:");
            lcd_showint16(0, 1, speed_case_1);
            break;
        }
        case 2:
        {
            lcd_showstr(0, 0, "Right_CircleIsland:");
            lcd_showint16(0, 1, speed_case_2);
            break;
        }
        case 3:
        {
            lcd_showstr(0, 0, "Left_Garage:");
            lcd_showint16(0, 1, speed_case_3);
            break;
        }
        case 4:
        {
            lcd_showstr(0, 0, "First_Fork:");
            lcd_showint16(0, 1, speed_case_4);
            break;
        }
        case 5:
        {
            lcd_showstr(0, 0, "Second_CrossLoop:");
            lcd_showint16(0, 1, speed_case_5);
            break;
        }
        case 6:
        {
            lcd_showstr(0, 0, "Second_Fork:");
            lcd_showint16(0, 1, speed_case_6);
            break;
        }
        case 7:
        {
            lcd_showstr(0, 0, "Right_Garage:");
            lcd_showint16(0, 1, speed_case_7);
            break;
        }
    }
}

/*
 ** 函数功能: 速度显示，显示所有状态的速度
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void BaseSpeedDisplay(void)
{
    lcd_clear(WHITE);
    lcd_showint16(0, 0, base_speed);
    lcd_showint16(0, 1, speed_case_1);
    lcd_showint16(0, 2, speed_case_2);
    lcd_showint16(0, 3, speed_case_3);
    lcd_showint16(0, 4, speed_case_4);
    lcd_showint16(0, 5, speed_case_5);
    lcd_showint16(0, 6, speed_case_6);
    lcd_showint16(0, 7, speed_case_7);
}

/*
 ** 函数功能: 按键调整各个状态的速度
 ** 参    数: 无
 ** 返 回 值: 是否退出调参：1:退出调参  0:继续调参
 ** 作    者: WBN
 */
uint8 KeyBaseSpeed(void)
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
            KeyBaseSpeedDisplay(key_num);
            break;
        }
        case KEY_S2_PRES:   //S2：选择上一个参数
        {
            if(key_num<7)
            {
                key_num++;  //选择上一个参数
            }
            KeyBaseSpeedDisplay(key_num);
            break;
        }
        case KEY_S3_PRES:   //S3：目前参数调大
        {
            KeyBaseSpeedDisplay(key_num);
            switch(key_num)
            {
                case 0: //开局速度，状态0，左环岛
                {
                    base_speed+=5;
                    lcd_showint16(0, 1, base_speed);
                    break;
                }
                case 1: //状态1，
                {
                    speed_case_1+=5;
                    lcd_showint16(0, 1, speed_case_1);
                    break;
                }
                case 2: //状态2，
                {
                    speed_case_2+=5;
                    lcd_showint16(0, 1, speed_case_2);
                    break;
                }
                case 3: //状态3，
                {
                    speed_case_3+=5;
                    lcd_showint16(0, 1, speed_case_3);
                    break;
                }
                case 4: //状态4，
                {
                    speed_case_4+=5;
                    lcd_showint16(0, 1, speed_case_4);
                    break;
                }
                case 5: //状态5，
                {
                    speed_case_5+=5;
                    lcd_showint16(0, 1, speed_case_5);
                    break;
                }
                case 6: //状态6，
                {
                    speed_case_6+=5;
                    lcd_showint16(0, 1, speed_case_6);
                    break;
                }
                case 7: //状态7，
                {
                    speed_case_7+=5;
                    lcd_showint16(0, 1, speed_case_7);
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
            KeyBaseSpeedDisplay(key_num);
            switch(key_num)
            {
                case 0: //开局速度，状态0，左环岛
                {
                    base_speed-=5;
                    lcd_showint16(0, 1, base_speed);
                    break;
                }
                case 1: //状态1，
                {
                    speed_case_1-=5;
                    lcd_showint16(0, 1, speed_case_1);
                    break;
                }
                case 2: //状态2，
                {
                    speed_case_2-=5;
                    lcd_showint16(0, 1, speed_case_2);
                    break;
                }
                case 3: //状态3，
                {
                    speed_case_3-=5;
                    lcd_showint16(0, 1, speed_case_3);
                    break;
                }
                case 4: //状态4，
                {
                    speed_case_4-=5;
                    lcd_showint16(0, 1, speed_case_4);
                    break;
                }
                case 5: //状态5，
                {
                    speed_case_5-=5;
                    lcd_showint16(0, 1, speed_case_5);
                    break;
                }
                case 6: //状态6，
                {
                    speed_case_6-=5;
                    lcd_showint16(0, 1, speed_case_6);
                    break;
                }
                case 7: //状态7，
                {
                    speed_case_7-=5;
                    lcd_showint16(0, 1, speed_case_7);
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
            BaseSpeedDisplay();
            systick_delay_ms(STM0,1000);
            return 1;
        }
        default:break;
    }
    systick_delay_ms(STM0,100);
    return 0;
}

/*
 ** 函数功能: 按键选择不同的发车方案
 ** 参    数: 无
 ** 返 回 值: 是否退出调参：1:退出选择直接发车  2：退出  0:继续选择
 ** 作    者: WBN
 ** 注    意：比赛发车用，主要是几套固定的参数
 */
uint8 KeyPlan(void)
{
    static uint8 key_num;   //调整的参数
    switch(KeyScan())
    {
        case KEY_S1_PRES:   //S1：方案一，左边元素减速方案
        {

            return 1;
        }
        case KEY_S2_PRES:   //S2：方案二，全程元素减速方案
        {

            return 1;
        }
        case KEY_S3_PRES:   //S3：方案三，左边元素提速方案
        {
            
            return 1;
        }
        case KEY_S4_PRES:   //S4：方案四，退出进行分段微调
        {
            return 2;
        }
        case KEY_S5_PRES:   //S5：默认方案，直接发车
        {

            return 1;
        }
        default:break;
    }
    systick_delay_ms(STM0,100);
    return 0;
}

