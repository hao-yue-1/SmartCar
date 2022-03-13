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
#define S_BLACK_WIDTH  3    //斑马线黑线宽度阈值 //用于判断该线是否为斑马线黑线
#define S_BLACK_NUM    8    //斑马线黑线数量阈值 //用于判断该行是否为斑马线
#define S_BLACK_TIMES  3    //斑马线数量阈值     //用于判断该路段是否为斑马线
//环岛判定误差
#define C_BIAS  2           //用于消除小毛刺的影响，补线更加丝滑

/*起跑线相关函数*/
uint8 StartLineFlag(int *LeftLine,int *RightLine);      //起跑线识别

/*环岛相关函数*/
uint8 CircleIslandBegin(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);
uint8 CircleIslandEnd(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);
uint8 CrossLoopEnd(int *LeftLine,int *RightLine,Point InflectionL,Point InflectionR);
/*三岔相关函数*/
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC);//搜寻出三岔上拐点
uint8 ForkIdentify(int startline,int endline,int *LeftLine,int *RightLine,Point *InflectionL,Point *InflectionR,Point *InflectionC);//三岔识别
/*十字相关函数*/
void GetCrossRoadsUpInflection(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR);//搜寻出十字的上拐点

#endif /* CODE_IMAGESPECIAL_H_ */
