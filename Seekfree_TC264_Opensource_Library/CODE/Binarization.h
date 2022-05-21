#ifndef CODE_BINARIZATION_H_
#define CODE_BINARIZATION_H_

#include "SEEKFREE_MT9V03X.h"//为了要uint8这种定义,二值化算法中的某些数学计算,摄像头图像的全局变量

//************宏定义***************
#define IMAGE_BLACK 0//二值化的黑点
#define IMAGE_WHITE 255//二值化的白点，用在二值化图像以及扫线那里
//*********************************

//************全局变量**************
extern uint8 BinaryImage[MT9V03X_H][MT9V03X_W];
//**********************************


uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height);     //计算二值化阈值: 大津法 3ms
uint8 GuDiThreshold(uint16 width, uint16 height);     //计算二值化阈值：谷底最小值 2ms
uint8 OneDimensionalThreshold(uint16 width, uint16 height); //计算二值化阈值 4ms

void ImageBinary(void);                                             //根据阈值二值化图像并修改原点坐标

#endif /* CODE_BINARIZATION_H_ */
