/*
 * LED.h
 *
 *  Created on: 2022年5月23日
 *      Author: yue
 */

#ifndef CODE_LED_H_
#define CODE_LED_H_

//主控板LED颜色定义
#define LED_YELLOW   P23_1  //黄色
#define LED_WHITE    P22_1  //白色
#define LED_GREEN    P22_2  //绿色
#define LED_BLUE     P22_3  //蓝色
#define LED_RED      P21_2  //红色

void LEDInit(void);

#endif /* CODE_LED_H_ */
