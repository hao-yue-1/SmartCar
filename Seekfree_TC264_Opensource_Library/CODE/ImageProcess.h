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
extern uint8 CrossLoop_flag;         //十字回环标志变量
extern uint8 CircleIsland_flag;      //环岛识别标志变量
extern uint8 speed_case_1,speed_case_2,speed_case_3,speed_case_4,speed_case_5,speed_case_6,speed_case_7;
extern uint8 bias_startline,bias_endline;        //动态前瞻

void ImageProcess(void);//图像处理逻辑函数
void Stop(void);        //停车
void LcdDrawPoint(Point Inflection,uint16 color);
void LcdDrawPoint_V2(uint8 row,uint8 column,uint16 color);
void LcdDrawRow(uint8 row,uint16 color);
void LcdDrawColumn(uint8 column,uint16 color);

#endif /* CODE_IMAGEPROCESS_H_ */
