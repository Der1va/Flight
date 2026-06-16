#ifndef __COMMON_IMU_H
#define __COMMON_IMU_H
#include "common_debug.h"
#include "common_config.h"
#include "math.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

extern float RtA;
extern float Gyro_G;
extern float Gyro_Gr;

void Common_IMU_GetEulerAngle(IMU_Data *gyroAccel,
                              Euler_Angle *eulerAngle,
                              float dt);
float Common_IMU_GetNormAccZ(void);

#endif
