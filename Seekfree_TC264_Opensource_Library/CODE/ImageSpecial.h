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
#define C_LOSTLINE  35      //触发环岛入口判断的丢线数         //越大条件越严谨
#define C_LOSTNUM   50      //判定环岛出口的丢线数
//Sobel算子检测
#define FastABS(x) (x > 0 ? x : x * -1.0f)
#define BinaryImage(i, j)    BinaryImage[i][j]
#define ZebraTresholeL 2300  //索贝尔测试的阈值
#define ZebraTresholeR 2000  //索贝尔测试车库在右边的阈值
#define SobelLTestStop 2     //索贝尔左边关闭的阈值
//十字回环出口
#define L_LOSTNUM   60

extern uint8 SobelLCount;             //左边索贝尔
extern uint8 SobelRCount;             //右边索贝尔

/*起跑线相关函数*/
int64 SobelTest();      //Sobel算子检测起跑线
uint8 GarageIdentify(char Direction,Point InflectionL,Point InflectionR);      //起跑线识别
/*环岛相关函数*/
uint8 CircleIslandIdentify_L(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);   //左环岛状态机
uint8 CircleIslandIdentify_R(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);   //右环岛状态机
uint8 DealGarageLSpecial();     //左车库特殊情况处理，左斜情况
/*三岔相关函数*/
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC);//搜寻出三岔上拐点
uint8 ForkIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//三岔识别
/*十字相关函数*/
void GetCrossRoadsUpInflection(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR);//搜寻出十字的上拐点
uint8 CrossRoadsIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//十字识别函数
uint8 CrossLoopEnd_F();
uint8 CrossLoopEnd_S();
#endif /* CODE_IMAGESPECIAL_H_ */
