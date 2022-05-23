/*
 * LED.c
 *
 *  Created on: 2022年5月23日
 *      Author: yue
 */

#include "LED.h"
#include "zf_gpio.h"

/*
 ** 函数功能: 初始化LED对应IO口，包括了核心板和主控板上的LED
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void LEDInit(void)
{
    gpio_init(P20_8, GPO, 1, PUSHPULL);     //初始化核心板的LED
    gpio_init(P20_9, GPO, 1, PUSHPULL);
    gpio_init(P21_4, GPO, 1, PUSHPULL);
    gpio_init(P21_5, GPO, 1, PUSHPULL);
    gpio_init(P23_1, GPO, 1, PUSHPULL);     //初始化主控板的LED
    gpio_init(P22_1, GPO, 1, PUSHPULL);
    gpio_init(P22_2, GPO, 1, PUSHPULL);
    gpio_init(P22_3, GPO, 1, PUSHPULL);
    gpio_init(P21_2, GPO, 1, PUSHPULL);
}


