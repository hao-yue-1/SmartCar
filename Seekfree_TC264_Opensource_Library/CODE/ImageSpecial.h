/*
 * ImageSpecial.h
 *
 *  Created on: 2022年1月17日
 *      Author: yue
 */

#ifndef CODE_IMAGESPECIAL_H_
#define CODE_IMAGESPECIAL_H_

#include "Binarization.h"       //二值化之后的图像数组
#include "SEEKFREE_MT9V03X.h"   //为了要uint8这种定义,二值化算法中的某些数学计算,摄像头图像的全局变量
#include "ImageBasic.h"         //获取图像基本处理之后的数据
#include "ImageTack.h"

//起跑线识别
#define G_LINEBIAS  0.8     //判定车库另一边的直道斜率
#define G_ZEBRA_NUM 14      //斑马线标志数量阈值 //若共有n条黑线，则G_ZEBRA_NUM=2*n，现赛道有9条黑线，取偏小值
//环岛判定误差
#define C_BIAS      2       //消除小毛刺的影响，补线更加丝滑
#define C_LOSTLINE  35      //触发环岛入口判断的丢线数         //越大条件越严谨
#define C_LINEBIAS  0.8     //判定环岛入口另一边的直道斜率     //越小条件越严谨
#define C_LOSTNUM   50      //判定环岛出口的丢线数
//Sobel算子检测
#define FastABS(x) (x > 0 ? x : x * -1.0f)
#define BinaryImage(i, j)    BinaryImage[i][j]

/*起跑线相关函数*/
uint8 GarageIdentify(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);      //起跑线识别
int64 SobelTest();      //Sobel算子检测起跑线
/*环岛相关函数*/
uint8 CircleIslandBegin(int *LeftLine,int *RightLine);      //识别环岛入口
uint8 CircleIslandEnd(); //识别环岛出口
uint8 CircleIslandIdentify(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);   //识别环岛
/*三岔相关函数*/
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC);//搜寻出三岔上拐点
uint8 ForkIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//三岔识别
/*十字相关函数*/
void GetCrossRoadsUpInflection(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR);//搜寻出十字的上拐点
uint8 CrossRoadsIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//十字识别函数

#endif /* CODE_IMAGESPECIAL_H_ */
