//
// Created by yue on 2022/5/11.
//

#ifndef ICM20602_ATTITUDE_H
#define ICM20602_ATTITUDE_H

typedef struct{ //四元数
    float q0;
    float q1;
    float q2;
    float q3;
}quaterInfo_t;

typedef struct{ //欧拉角
    float pitch;
    float roll;
    float yaw;
}eulerianAngles_t;

extern float values[10];
extern eulerianAngles_t eulerAngle;

void IMU_getValues(float * values);
void IMU_quaterToEulerianAngles(void);

#endif //ICM20602_ATTITUDE_H
