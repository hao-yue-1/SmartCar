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
#define G_HIGH      10      //斑马线距离拐点的高度
#define G_ZEBRA_NUM 10      //斑马线标志数量阈值 //若共有n条黑线，则G_ZEBRA_NUM=2*n
//环岛判定误差
#define C_BIAS      2       //消除小毛刺的影响，补线更加丝滑
#define C_LOSTLINE  35      //触发环岛入口判断的丢线数         //越大条件越严谨
#define C_LINEBIAS  0.8     //判定环岛入口另一边的直道斜率     //越小条件越严谨
#define C_LOSTNUM   40      //判定环岛出口的丢线数
#define C_NUM_1     7       //环岛状态防止过多帧数阈值
#define C_NUM_2     6       //环岛状态防止复杂地形连续误判帧数阈值

/*起跑线相关函数*/
uint8 GarageIdentify(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);      //起跑线识别
/*环岛相关函数*/
uint8 CircleIslandBegin(int *LeftLine,int *RightLine);      //识别环岛入口
uint8 CircleIslandEnd(); //识别环岛出口
uint8 CircleIslandIdentify(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);   //识别环岛
/*十字回环相关函数*/
uint8 CrossLoopEnd(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);
/*三岔相关函数*/
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC);//搜寻出三岔上拐点
uint8 ForkIdentify(int startline,int endline,int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//三岔识别
/*十字相关函数*/
void GetCrossRoadsUpInflection(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR);//搜寻出十字的上拐点
uint8 CrossRoadsIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//十字识别函数

#endif /* CODE_IMAGESPECIAL_H_ */
