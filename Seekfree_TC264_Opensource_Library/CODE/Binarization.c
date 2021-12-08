#include "Binarization.h"

uint8 BinaryImage[MT9V03X_H][MT9V03X_W]={0};

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

//根据场地条件调用大津法或谷底最小值得到二值化阈值然后根据灰度图得到黑白图像
void ImageBinary()
{
    uint8 Image_Threshold = otsuThreshold(mt9v03x_image[0],MT9V03X_W,MT9V03X_H);
    //数组中第一维这样处理是为了将图像原点变为左下角（原图像数组为左上角）
    for (int i = 0; i < MT9V03X_H; ++i)
    {
        for (int j = 0; j < MT9V03X_W; ++j)
        {
            if (mt9v03x_image[i][j] <= Image_Threshold)//进行二值化之前只是得到阈值
                BinaryImage[MT9V03X_H - 1 - i][j] = 0;//0是黑色
            else
                BinaryImage[MT9V03X_H - 1 - i][j] = 255;//1是白色
        }
    }
}

