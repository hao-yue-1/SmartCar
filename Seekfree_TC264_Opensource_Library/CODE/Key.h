/*
 * Key.h
 *
 *  Created on: 2022Äê4ÔÂ12ÈÕ
 *      Author: yue
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "common.h"
#include "zf_gpio.h"

#define KEY_S1      gpio_get(P33_10)
#define KEY_S2      gpio_get(P33_11)
#define KEY_S3      gpio_get(P33_12)
#define KEY_S4      gpio_get(P33_13)
#define KEY_S5      gpio_get(P32_4)

#define KEY_S1_PRES     1
#define KEY_S2_PRES     2
#define KEY_S3_PRES     3
#define KEY_S4_PRES     4
#define KEY_S5_PRES     5

uint8 KeyScan(void);
uint8 KeyParameter(void);
void ParameterDisplay(void);

#endif /* CODE_KEY_H_ */
