//
// Created by yue on 2022/5/11.
//

#include "attitude.h"
#include "SEEKFREE_ICM20602.h"
#include <math.h>
#include "zf_stm_systick.h"

#define delta_T      0.005f  //周期 5ms计算一次
#define M_PI 3.1425f         //圆周率

quaterInfo_t Q_info = {1, 0, 0, 0};  //四元数
eulerianAngles_t eulerAngle;      //欧拉角

float I_ex, I_ey, I_ez;  //误差积分
float param_Kp = 50.0;   //加速度计的收敛速率比例增益 50
float param_Ki = 0.20;   //陀螺仪收敛速率的积分增益 0.2
float values[6];        //六轴数据

GyroOffset gyro_offset; //陀螺仪零漂校准值

/*
 ** 函数功能: 陀螺仪零漂初始化，通过采集一定数据求均值计算陀螺仪零点偏移值
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void GyroOffsetInit(void)
{
    gyro_offset.x = 0;
    gyro_offset.y = 0;
    gyro_offset.z = 0;

    for(uint8 i=0;i<100;i++)        //采集100次
    {
        get_icm20602_gyro();        //获取陀螺仪角速度
        gyro_offset.x += icm_gyro_x;
        gyro_offset.y += icm_gyro_y;
        gyro_offset.z += icm_gyro_z;
        systick_delay_ms(STM0,5);   //采样周期
    }

    gyro_offset.x /= 100;
    gyro_offset.y /= 100;
    gyro_offset.z /= 100;
}

//一阶低通滤波系数
#define new_weight           0.35f
#define old_weight           0.65f

/*
 ** 函数功能: 将accel一阶低通滤波，将gyro减去零漂并转换单位
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void IMUGetValues(float *values)
{
    //一阶低通滤波
    static double lastaccel[3]= {0,0,0};
    int i;
    values[0] = ((float)icm_acc_x) * new_weight + lastaccel[0] * old_weight;
    values[1] = ((float)icm_acc_y) * new_weight + lastaccel[1] * old_weight;
    values[2] = ((float)icm_acc_z) * new_weight + lastaccel[2] * old_weight;
    for(i=0; i<3; i++)
    {
        lastaccel[i] = values[i];
    }
    //单位转换，减去零漂
    values[3] = ((float)icm_gyro_x-gyro_offset.x) * M_PI / 180 / 16.4f;
    values[4] = ((float)icm_gyro_y-gyro_offset.y) * M_PI / 180 / 16.4f;
    values[5] = ((float)icm_gyro_z-gyro_offset.z) * M_PI / 180 / 16.4f;
}

//归一化算法
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//姿态解算融合，是Crazepony和核心算法。使用的是互补滤波算法，没有使用Kalman滤波算法
static void IMU_AHRSupdate_noMagnetic(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * delta_T;//采样周期一半
    float vx, vy, vz;           //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;           //四元数计算值与加速度计测量值的误差
    //四元数
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    //四元数相乘，方便后面的计算
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    //对加速度数据进行归一化，得到单位加速度
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //载体坐标系下重力在三个轴上的分量
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //g^b和a^b做向量叉乘，得到陀螺仪的校正补偿向量e的系数
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    /*
    用叉乘误差来做PI修正陀螺零偏，
    通过调节 param_Kp，param_Ki 两个参数，
    可以控制加速度计修正陀螺仪积分姿态的速度。
    */

    //误差累加
    I_ex += delta_T * ex;
    I_ey += delta_T * ey;
    I_ez += delta_T * ez;
    //使用PI控制器消除向量积误差（陀螺仪漂移误差）
    gx = gx+ param_Kp*ex + param_Ki*I_ex;
    gy = gy+ param_Kp*ey + param_Ki*I_ey;
    gz = gz+ param_Kp*ez + param_Ki*I_ez;
    /*数据修正完成，下面是四元数微分方程*/
    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙格库塔求解四元数微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;
    //保存上一次四元数的值
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  //归一化系数
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

//把四元数转换成欧拉角
void IMU_quaterToEulerianAngles(void)
{
    IMUGetValues(values);   //基础数据处理
    IMU_AHRSupdate_noMagnetic(values[3], values[4], values[5], values[0], values[1], values[2]);    //融合滤波解算四元数
    //四元数
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    //换算欧拉角
    eulerAngle.pitch = asin(-2*q1*q3 + 2*q0*q2) * 180/M_PI;                        // pitch
    eulerAngle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180/M_PI; // roll
    eulerAngle.yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180/M_PI;  // yaw

    /*可以不用作姿态限度的限制*/
//    if(eulerAngle.roll>90 || eulerAngle.roll<-90)
//    {
//        if(eulerAngle.pitch > 0)
//        {
//            eulerAngle.pitch = 180-eulerAngle.pitch;
//        }
//        if(eulerAngle.pitch < 0)
//        {
//            eulerAngle.pitch = -(180+eulerAngle.pitch);
//        }
//    }
//    if(eulerAngle.yaw > 180)
//    {
//        eulerAngle.yaw -=360;
//    }
//    else if(eulerAngle.yaw <-180)
//    {
//        eulerAngle.yaw +=360;
//    }
}
