/*
 * ImageTack.h
 *  Created on: 2021年12月22日
 *  Author: 30516
 *  Effect: 用于存放从图像中提取循迹元素的函数
 */

#ifndef CODE_IMAGETACK_H_
#define CODE_IMAGETACK_H_

#include "Binarization.h"       //二值化之后的图像数组
#include "SEEKFREE_MT9V03X.h"   //为了要uint8这种定义,二值化算法中的某些数学计算,摄像头图像的全局变量
#include "ImageBasic.h"         //获取图像基本处理之后的数据

extern float Bias;  //偏差

float Regression_Slope(int startline,int endline,int *CentreLine);//求中线数组点拟合出来的回归方程
void FillingLine(int *LeftLine, int *CentreLine, int *RightLine, Point StarPoint,Point EndPoint);//根据两点进行补线
float DifferentBias(int startline,int endline,int *CentreLine);


#endif /* CODE_IMAGETACK_H_ */
