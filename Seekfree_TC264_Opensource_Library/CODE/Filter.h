/*
 * Filter.h
 *
 *  Created on: 2022年3月18日
 *      Author: yue
 */

#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#include "common.h"

#define FIRST_LAG_P 0.8     //一阶滞后滤波系数，越小滞后效果越强


float FirstOrderLagFilter(float value); //一阶滞后滤波算法
int16 SecondOrderLagFilter_L(int16 value);
int16 SecondOrderLagFilter_R(int16 value);


#endif /* CODE_FILTER_H_ */
