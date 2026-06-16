#ifndef APPLICATION_FLIGHT_H
#define APPLICATION_FLIGHT_H

#include "Int_MPU6050.h"
#include "common_debug.h"
#include "Common_filter.h"
#include "math.h"
#include "Common_IMU.h"

/**
    * @brief 根据MPU6050传感器的数据 计算出欧拉角
**/
void App_Calculate_Euler_Angles(void);


#endif // APPLICATION_FLIGHT_H