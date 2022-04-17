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
    switch(KeyScan())
    {
        case KEY_S1_PRES:   //S1
        {
            gpio_toggle(LED_WHITE);
            break;
        }
        case KEY_S2_PRES:   //S2
        {
            gpio_toggle(LED_GREEN);
            break;
        }
        case KEY_S3_PRES:   //S3
        {
            gpio_toggle(LED_BLUE);
            break;
        }
        case KEY_S4_PRES:   //S4
        {
            gpio_toggle(LED_RED);
            break;
        }
        case KEY_S5_PRES:   //S5：退出调参
        {
            return 1;
        }
    }
    systick_delay_ms(STM0,100);
    return 0;
}



