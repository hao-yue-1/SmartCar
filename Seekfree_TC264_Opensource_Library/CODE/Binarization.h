#ifndef CODE_BINARIZATION_H_
#define CODE_BINARIZATION_H_

#include "SEEKFREE_MT9V03X.h"//为了要uint8这种定义,二值化算法中的某些数学计算,摄像头图像的全局变量

//************宏定义***************
#define IMAGE_BLACK 0//二值化的黑点
#define IMAGE_WHITE 255//二值化的白点，用在二值化图像以及扫线那里
#define IMAGE_COMPRESS_W    80//压缩之后的图像宽度
#define IMAGE_COMPRESS_H    60//压缩之后的图像高度
#define IMAGECOMPRESS 0//是否开启图像压缩的二值化，1：是 0：否 压缩图像帧率记得调成230
//*********************************

//************全局变量**************
extern uint8 CompressImage[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W];//存储压缩之后的灰度图
#if IMAGECOMPRESS
extern uint8 BinaryImage[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W];
#else
extern uint8 BinaryImage[MT9V03X_H][MT9V03X_W];
#endif
//**********************************


uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height);     //计算二值化阈值: 大津法 3ms
uint8 GuDiThreshold(uint16 width, uint16 height);     //计算二值化阈值：谷底最小值 2ms
uint8 OneDimensionalThreshold(uint16 width, uint16 height); //计算二值化阈值 4ms
void adaptiveThreshold(uint8 *img_data, uint8 *output_data, int width, int height, int block, uint8 clip_value);//自适应阈值二值化
void Get_Compress_Image(void); //压缩原始灰度图像再进行二值化，速度是之前的一半之前二值化一张图5ms，现在1.5ms
void ImageBinary(void); //根据阈值二值化图像

#endif /* CODE_BINARIZATION_H_ */
