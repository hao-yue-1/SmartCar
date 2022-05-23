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
 ** 函数功能: 初始化按键对应IO口
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyInit(void)
{
    gpio_init(P33_10, GPI, 0, PULLDOWN);    //初始化按键
    gpio_init(P33_11, GPI, 0, PULLDOWN);
    gpio_init(P33_12, GPI, 0, PULLDOWN);
    gpio_init(P33_13, GPI, 0, PULLDOWN);
    gpio_init(P32_4, GPI, 0, PULLDOWN);
}

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



