/*
 **  ImageBasic.h
 **  作用：用来存放提取图像基本信息的模块，类似于扫线什么的
 **  Created on: 2021年12月7日
 **  Author: 30516
 */

#ifndef CODE_IMAGEBASIC_H_
#define CODE_IMAGEBASIC_H_

#include "Binarization.h"       //二值化之后的图像数组
#include "SEEKFREE_MT9V03X.h"   //为了要uint8这种定义,二值化算法中的某些数学计算,摄像头图像的全局变量
#include "SEEKFREE_18TFT.h"     //使用LCD调试
#include "zf_stm_systick.h"     //使用延时函数

//宏定义
#define BORDER_BIAS 1   //扫线误差
#define INFLECTION_WIDTH  10    //拐点赛道宽度
#define UPINFLECTION_THRESHOLD  50  //上拐点列坐标的差值阈值
//全局变量
extern int Lost_Row;                    //中线丢失的行坐标(扫线到赛道外)
extern int LostNum_LeftLine,LostNum_RightLine; //记录左右边界丢线数
extern int LeftLine[MT9V03X_H], CentreLine[MT9V03X_H], RightLine[MT9V03X_H];   //扫线处理左中右三线
//结构体
typedef struct Point
{
    int X;
    int Y;
}Point; //点坐标的结构体

void GetImagBasic(int *LeftLine, int *CentreLine, int *RightLine);  //扫线提取左中右三线
void GetDownInflection(int startline,int endline,int *LeftLine,int *RightLine,Point *InflectionL,Point *InflectionR);//根据左右边界线来得到下拐点（十字、三岔、环岛）
void GetUpInflection(char Choose,int startline,int endline,Point *UpInflection);//根据遍历左右线得到行与行之间的列坐标的差值，大于设定的阈值则判断为是上拐点
void Bin_Image_Filter(void);
void EdgeTrack(void);   //八领域扫线

#endif /* CODE_IMAGEBASIC_H_ */
