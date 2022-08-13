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
extern float Slope; //斜率

float Regression_Slope(int startline,int endline,int *CentreLine);//求中线数组点拟合出来的回归方程
void FillingLine(char Choose, Point StarPoint,Point EndPoint);//根据两点进行补线
float DifferentBias(int startline,int endline,int *CentreLine);
float DifferentBias_Circle(uint8 startline,uint8 endline,int *CentreLine);
float DifferentBias_Garage(int startline,int endline,int *CentreLine);//车库专属循迹偏差
void FillinLine_V2(char Choose,int startline,int endline,Point Point1,Point Point2);//比第一版本升级了可以根据已经有点线段去推测出未知的线段补线
void Unilaterally_Plan_CenterLine(char ManualorAuto ,char LorR,int startline,int endline);//判断赛道是否单边丢线使得中线信息失真，根据另外一边未丢线的边消除失真
uint8 SlopeUntie_X(Point point_1,Point point_2,uint8 y);

#endif /* CODE_IMAGETACK_H_ */
