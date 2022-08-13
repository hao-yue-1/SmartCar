/*
 * Key.h
 *
 *  Created on: 2022年4月12日
 *      Author: yue
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "common.h"
#include "zf_gpio.h"
#include "ImageBasic.h"

#define KEY_S1      gpio_get(P33_10)
#define KEY_S2      gpio_get(P33_11)
#define KEY_S3      gpio_get(P33_12)
#define KEY_S4      gpio_get(P33_13)
#define KEY_S5      gpio_get(P32_4)

#define KEY_UP      1   //上
#define KEY_DOWN    2   //下
#define KEY_LEFT    3   //左
#define KEY_RIGHT   4   //右
#define KEY_ENTER   5   //确认

extern uint8 key_num_1,key_num_2;

void KeyInit(void);
uint8 KeyScan(void);
void KeyPID(void);
void KeyProcess(void);
void KeyImage(void);
void ImageParameterHandle(uint8 key_num_1,uint8 key_num_2,Point InflectionL,Point InflectionR);
void KeySpeed(void);
void KeySteer(void);

#endif /* CODE_KEY_H_ */
