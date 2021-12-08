/*
 **  ImageBasic.h
 **  作用：用来存放提取图像基本信息的模块，类似于扫线什么的
 **  Created on: 2021年12月7日
 **  Author: 30516
 */

#ifndef CODE_IMAGEBASIC_H_
#define CODE_IMAGEBASIC_H_

#include "Binarization.h"//二值化之后的图像数组
#include "SEEKFREE_MT9V03X.h"//为了要uint8这种定义,二值化算法中的某些数学计算,摄像头图像的全局变量

//宏定义
#define BORDER_BIAS 3


void GetImagBasic(int *LeftLine, int *CentreLine, int *RightLine);  //扫线提取左中右三线

#endif /* CODE_IMAGEBASIC_H_ */
