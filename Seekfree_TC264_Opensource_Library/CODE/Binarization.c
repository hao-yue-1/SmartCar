#include "Binarization.h"
#include "headfile.h"
#include <stdlib.h>
#include "zf_assert.h"

#define IMAGECOMPRESS 0//是否开启图像压缩的二值化，1：是 0：否 压缩图像帧率记得调成230

uint8 CompressImage[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W]={0};//存储压缩之后的灰度图
#if IMAGECOMPRESS
uint8 BinaryImage[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W]={0};//压缩图像之后的二值化图像
#else
uint8 BinaryImage[MT9V03X_H][MT9V03X_W]={0};//二值化图像
#endif

/*
 *  @brief  大津法二值化0.8ms程序（实际测试4ms在TC264中）
 *  @date:   2018-10
 *  @since      v1.2
 *  *image ：图像地址
 *  width:  图像宽
 *  height：图像高
 *  @author     Z小旋
 */
uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height)
{
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};//每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height;   //总像素点
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针

    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
          //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;   //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];  //总平均灰度
    }

    float maxVariance=0.0;  //最大类间方差
    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8)i;
        }
    }

    return threshold;
}
/***************************************************************************
 ** 函数功能: 谷底最小值二值化算法
 ** 参    数: uint16 width : 图像宽度
 **           uint16 height: 图像高度
 ** 返 回 值: 二值化阈值
 ** 作    者: 师兄
 ** 注    意：运用了摄像头采集的灰度图像的全局变量
 *************************************************************************/
uint8 GuDiThreshold(uint16 width, uint16 height)     //计算二值化阈值：谷底最小值
{
    uint16 graynum[256] = {0};
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            uint8 pix = mt9v03x_image[i][j];
            graynum[pix]++;
        }
    }
    uint8 Max[2] = {0};
    uint8 index[2] = {0};
    for (int i = 0; i < 2; ++i)
    {
        for (uint16 j = 0; j < 256; ++j)
        {
            if (i == 0)
            {
                if (graynum[j] > Max[i])
                {
                    Max[i] = graynum[j];
                    index[i] = j;
                }
            }
            else
            {
                if (graynum[j] > Max[i] && graynum[j] != Max[0])
                {
                    Max[i] = graynum[j];
                    index[i] = j;
                }
            }
        }
    }
    if (index[0] > index[1])
    {
        uint8 temp = 0;
        temp = index[0];
        index[0] = index[1];
        index[1] = temp;
    }

    uint8 Min = 255, index_Min = 0;
    for (uint8 i = index[0]; i < index[1]; ++i)
    {
        if (graynum[i] < Min)
        {
            Min = graynum[i];
            index_Min = i;
        }
    }

    return index_Min;
}

/*
 *******************************************************************************************
 ** 函数功能: 类似一维Means的二值化阈值计算
 ** 参    数: width：图像宽度
 **           height：图像高度
 ** 返 回 值: 二值化阈值
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8 OneDimensionalThreshold(uint16 width, uint16 height)
{
    int row,cloum;
    int G1,G2;
    int g1,g2;
    uint8 threshold=160,threshold_last=0;   //阈值与上一次阈值，初始化为不同的值，第一个阈值是认为随机设定的

    while(abs(threshold-threshold_last)>10)   //只有当连续两次计算的阈值相等时才会跳出while
    {
        //初始化数据
        G1=0;G2=0;
        g1=0;g2=0;
        //进行G1和G2的分类
        for(row=0;row<height;row++)
        {
            for(cloum=0;cloum<width;cloum++)
            {
                if(mt9v03x_image[row][cloum]>threshold)
                {
                    G1+=mt9v03x_image[row][cloum];
                    g1++;
                }
                else
                {
                    G2+=mt9v03x_image[row][cloum];
                    g2++;
                }
            }
        }
        //进行新阈值的计算
        threshold_last=threshold;       //保存上一次的阈值
        threshold=((G1/g1)+(G2/g2))/2;  //阈值=（G1平均值+G2平均值）/ 2
    }
    return threshold;
}

//根据场地条件调用大津法或谷底最小值得到二值化阈值然后根据灰度图得到黑白图像
void ImageBinary()
{
#if IMAGECOMPRESS
    //压缩图像的二值化
    uint8 Image_Threshold = otsuThreshold(CompressImage[0],IMAGE_COMPRESS_W,IMAGE_COMPRESS_H);//使用大津法得到二值化阈值
    for (int i = 0; i < IMAGE_COMPRESS_H; ++i)
    {
        for (int j = 0; j < IMAGE_COMPRESS_W; ++j)
        {
            if (CompressImage[i][j] <= Image_Threshold)//进行二值化之前只是得到阈值
                Binary2Image[i][j] = IMAGE_BLACK;//0是黑色  //图像原点不变
            else
                Binary2Image[i][j] = IMAGE_WHITE;//1是白色  //图像原点不变
        }
    }
#else
//    systick_start(STM1);
//    uint8 Image_Threshold = GuDiThreshold(MT9V03X_W,MT9V03X_H);//使用谷底最小值得到二值化阈值
    uint8 Image_Threshold = otsuThreshold(mt9v03x_image[0],MT9V03X_W,MT9V03X_H);//使用大津法得到二值化阈值
//    uint8 Image_Threshold = OneDimensionalThreshold(MT9V03X_W,MT9V03X_H);//使用一维means法得到二值化阈值
//    lcd_showint32(60, 0, systick_getval_us(STM1), 5);
//    lcd_showuint8(8, 0, Image_Threshold);
    for (int i = 0; i < MT9V03X_H; ++i)
    {
        for (int j = 0; j < MT9V03X_W; ++j)
        {
            if (mt9v03x_image[i][j] <= Image_Threshold)//进行二值化之前只是得到阈值
                BinaryImage[i][j] = IMAGE_BLACK;//0是黑色  //图像原点不变
            else
                BinaryImage[i][j] = IMAGE_WHITE;//1是白色  //图像原点不变
        }
    }
#endif
}

/********************************************************************************************
 ** 函数功能: 自适应阈值二值化图像
 ** 参    数: uint8* img_data：灰度图像
 **           uint8* output_data：二值化图像
 **           int width：图像宽度
 **           int height：图像高度
 **           int block：分割局部阈值的方块大小例如7*7
 **           uint8 clip_value: 局部阈值减去的经验值一般为（2~5）
 ** 返 回 值: 无
 ** 作    者: 上海交大16届智能车智能视觉组SJTUAuTop
 **           https://zhuanlan.zhihu.com/p/391051197
 ** 注    意：adaptiveThreshold(mt9v03x_image[0],BinaryImage[0],MT9V03X_W,MT9V03X_H,5,1);//但是没d用跟大津法一样
 *********************************************************************************************/
void adaptiveThreshold(uint8 *img_data, uint8 *output_data, int width, int height, int block, uint8 clip_value)
{
//  assert(block % 2 == 1); // block必须为奇数
  int half_block = block / 2;
  for(int y=half_block; y<height-half_block; y++)
  {
    for(int x=half_block; x<width-half_block; x++)
    {
      // 计算局部阈值
      int thres = 0;
      for(int dy=-half_block; dy<=half_block; dy++)
      {
        for(int dx=-half_block; dx<=half_block; dx++)
        {
          thres += img_data[(x+dx)+(y+dy)*width];
        }
      }
      thres = thres / (block * block) - clip_value;
      // 进行二值化
      output_data[x+y*width] = img_data[x+y*width]>thres ? 255 : 0;
    }
  }
}

/*************************************************************************
 *  函数名称：void Get_Use_Image (void)
 *  功能说明：把摄像头采集到原始图像，缩放到赛道识别所需大小
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2022年6月4日
 *  备    注： IMAGE_COMPRESS_H:为图像压缩之后的高度120/2=60
 *             IMAGE_COMPRESS_W:为图像压缩之后的高度160/2=80
 *             帧率230的条件下压缩完之后二值化搜线找拐点采集图像的时间为2.3~2.5ms，未压缩：8.0ms，帧率400的时候为5ms
 *             顺便在压缩的时候进行了个均值滤波，对灰度图滤波这个像素点和旁边那个滤去的求均值
 *             看起来跟原来没太大差别，因为环境不恶劣，可以删去，耗时加了0.2ms
 *************************************************************************/
void Get_Compress_Image(void)
{
    short i = 0, j = 0, row = 0, line = 0;

    for (i = 0; i < MT9V03X_H; i += 2)          //神眼高 120 / 2  = 60，
    {
        for (j = 0; j <= MT9V03X_W; j += 2)     //神眼宽188 / 2  = 94，
        {
            CompressImage[row][line]=mt9v03x_image[i][j];
//            CompressImage[row][line] = (mt9v03x_image[i][j]+mt9v03x_image[i][j-1])/2;
            line++;
        }
        line = 0;
        row++;
    }
}
