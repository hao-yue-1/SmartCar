/*
 * ImageProcess.h
 *
 * Created on: 2022年3月29日
 * Author: 30516
 * Effect: Image element processing logic
 */

#ifndef CODE_IMAGEPROCESS_H_
#define CODE_IMAGEPROCESS_H_

#include "ImageBasic.h"     //扫线、搜寻左右下拐点
#include "ImageTack.h"      //计算偏差、补线
#include "ImageSpecial.h"   //元素识别

extern uint8 CrossRoads_flag;        //十字标志变量
extern uint8 Fork_flag;              //三岔识别的标志变量
extern uint8 CircleIsland_flag;      //环岛标志变量
extern uint8 speed_case_1,speed_case_2,speed_case_3,speed_case_4,speed_case_5,speed_case_6,speed_case_7;

//主控板LED颜色定义
#define LED_WHITE    P23_1
#define LED_GREEN    P22_1
#define LED_BLUE     P22_2
#define LED_RED      P22_3
#define LED_YELLOW   P21_2

void Stop(void);
void ImageProcess(void);//图像处理逻辑函数

#endif /* CODE_IMAGEPROCESS_H_ */
