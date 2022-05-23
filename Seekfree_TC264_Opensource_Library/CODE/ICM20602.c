/*
 * ICM20602.c
 *
 *  Created on: 2022年5月19日
 *      Author: yue
 */

#include "ICM20602.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_ICM20602.h"
#include "zf_ccu6_pit.h"
#include "Filter.h"

/*
 ** 函数功能: 获取ICM20602Z轴角速度
 ** 参    数: 无
 ** 返 回 值: Z轴角速度
 ** 作    者: WBN
 ** 注    意：对逐飞封装的修改，直接只从ICM对应寄存器中读取Z轴角速度的值，节省不必要的浪费
 */
int16 GetICM20602Gyro_Z(void)
{
    uint8 dat[2];
    int16 gyro_z;   //Z轴角速度

    simiic_read_regs(ICM20602_DEV_ADDR, ICM20602_GYRO_ZOUT_H, dat, 2, SIMIIC);
    gyro_z = (int16)(((uint16)dat[0]<<8 | dat[1]));

    return gyro_z;
}

/*
 ** 函数功能: 对ICM20602Z轴进行角度积分
 ** 参    数: flag：是否清零积分   1：清零积分    0：继续上次的积分
 ** 返 回 值: Z轴积分的角度
 ** 作    者: WBN
 */
float GetICM20602Angle_Z(uint8 flag)
{
    static float my_angle_z;   //Z轴角度

    if(flag==1) //清零之前的积分
    {
        my_angle_z=0;
        return 0;
    }

    int16 my_gyro_z=GetICM20602Gyro_Z();                //获取Z轴角速度
    my_gyro_z=kalman1_filter(&kalman_gyro, my_gyro_z);  //滤波
    my_angle_z+=0.00012480f*my_gyro_z;                  //积分

    return my_angle_z;
}

/*
 ** 函数功能: 从当前位置开启对ICM20602进行目标积分
 ** 参    数: target_angle：目标角度
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void StartIntegralAngle_Z(float target_angle)
{
    icm_target_angle_z=target_angle;        //设置目标角度
    icm_angle_z_flag=0;                     //积分目标flag=0
    GetICM20602Angle_Z(1);                  //积分清零
    pit_enable_interrupt(CCU6_1, PIT_CH0);  //开启中断
}

/*
 ** 函数功能: 对ICM20602进行姿态解算得出欧拉角
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void GetICM20602Eulerian(void)
{
    get_icm20602_accdata();  //获取加速度计的值
    get_icm20602_gyro();   //获取陀螺仪的值
    IMU_quaterToEulerianAngles(); //解算欧拉角
}
