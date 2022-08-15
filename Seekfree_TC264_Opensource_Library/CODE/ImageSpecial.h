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
#include "common.h"

//起跑线识别
#define G_LINEBIAS  0.8     //判定车库另一边的直道斜率
#define G_ZEBRA_NUM 14      //斑马线标志数量阈值 //若共有n条黑线，则G_ZEBRA_NUM=2*n，现赛道有9条黑线，取偏小值
//环岛判定误差
#define C_LOSTLINE  35      //触发环岛入口判断的丢线数         //越大条件越严谨
#define C_LOSTNUM   50      //判定环岛出口的丢线数
//十字回环出口
#define L_LOSTNUM   60

/*起跑线相关函数*/
void OutGarage(void);       //出库
int64 SobelTest(uint8 starline,uint8 endline,uint8 starcloumn,uint8 endcloumn);      //Sobel算子检测起跑线
uint8 ZebraIndentify(uint8 start_line,uint8 end_line,uint8* black_width);//黑白跳变寻找斑马线
void SeedGrowFindValley_Garage(char Choose,Point Seed,int endline,Point *UpInflectionC,char TRANSVERSE_THR);//车库的种子生长找谷底
void SeedGrowFindPeak_Garage(char Choose,Point Seed,int endline,Point *PeakInflection,char TRANSVERSE_THR);//车库种子生长找到山顶，跟上面函数相反
uint8 GarageLIdentify(char Choose,Point InflectionL,Point InflectionR);             //左车库识别
uint8 GarageLStatusIdentify(char Choose,Point InflectionL,Point InflectionR,uint8* GarageLFlag);  //左车库的状态机转移
uint8 LINGarageEntrance(Point InflectionL,Point InflectionR);//左车库入库识别补线函数
uint8 LINGarageStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag);//左侧车库入库状态机
uint8 RNINGarageIdentify(Point InflectionL,Point InflectionR);             //右车库不入库识别函数
uint8 RNINGarageSpecial(Point InflectionL,Point InflectionR);//右斜补线函数
uint8 RNINGarageStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag);//车库右边不入库状态机
uint8 RINGarageEntrance(Point InflectionL,Point InflectionR);//右车库入口状态识别函数
uint8 RINGarageStatusIdentify(Point InflectionL,Point InflectionR,uint8* GarageLFlag);//右车库入库状态机
/*环岛相关函数*/
uint8 CircleIslandBegin_L(void);
uint8 CircleIslandOverBegin_L(int *LeftLine);
uint8 CircleIslandEnd_L(void);
uint8 CircleIslandExit_L(Point InflectionL);
uint8 CircleIslandMid_L(void);
uint8 CircleIslandIdentify_L(int *LeftLine,Point InflectionL);   //左环岛状态机
uint8 CircleIslandBegin_R(void);
uint8 CircleIslandOverBegin_R(int *RightLine);
uint8 CircleIslandEnd_R(void);
uint8 CircleIslandExit_R(Point InflectionR);
uint8 CircleIslandMid_R(void);
uint8 CircleIslandIdentify_R(int *RightLine,Point InflectionR);   //右环岛状态机
/*十字回环相关函数*/
uint8 CrossLoopBegin_L(Point InflectionL,uint8 status);
uint8 CrossLoopEnd_L(uint8 status);
uint8 CrossLoopIdentify_L(Point InflectionL);    //左十字回环状态机
uint8 CrossLoopBegin_R(Point InflectionR,uint8 status);
uint8 CrossLoopEnd_R(uint8 status);
uint8 CrossLoopIdentify_R(Point InflectionR);    //右十字回环状态机
/*三岔相关函数*/
void ForkFindSpecialPoint(int row,int cloumn,Point *LeftUpPoint);//三岔特殊处理，防止Bias因为扫线变右使得Bias变小
void SeedGrowFindUpInflection(char Choose,Point Seed,int endline,Point *UpInflectionC);//三岔种子生长生长至谷底
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC);     //搜寻出三岔上拐点
uint8 ForkIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);   //三岔识别
uint8 ForkTurnRIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR);//三岔往右边走的识别补线处理函数
uint8 ForkFStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 *NowFlag);           //第一遍三岔状态跳转判断函数
uint8 ForkSStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 *NowFlag);           //第二遍三岔状态跳转判断函数
/*十字相关函数*/
void GetCrossRoadsUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionL,Point *UpInflectionR);//十字路口找寻上拐点函数
uint8 CrossRoadsIdentify(Point DownInflectionL,Point DownInflectionR);      //十字路口识别函数
uint8 CrossRoadsStatusIdentify(Point DownInflectionL,Point DownInflectionR);//十字状态机
/*入库相关函数*/
uint8 ZebraCrossingSearch(uint8 start_line,uint8 end_line);
void GarageInBegin(void);
uint8 GarageInEnd(void);
uint8 GarageInIdentify(void);
#endif /* CODE_IMAGESPECIAL_H_ */
