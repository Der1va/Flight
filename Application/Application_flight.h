#ifndef APPLICATION_FLIGHT_H
#define APPLICATION_FLIGHT_H

#include "Int_MPU6050.h"
#include "common_debug.h"
#include "Common_filter.h"
#include "math.h"
#include "Common_IMU.h"
#include "common_pid.h"
#include "Int_Motor.h"
#include "Int_VL53L1X.h"
#include "Freertos.h"
#include "task.h"

/**
    * @brief 初始化飞行控制相关模块
**/
void App_Flight_Start(void);

/**
    * @brief 根据MPU6050传感器的数据 计算出欧拉角
**/
void App_Calculate_Euler_Angles(void);

/**
    * @brief 飞行控制的PID计算
    * 这里使用了串级PID，外环控制角度，内环控制角速度
    * 外环PID的输出作为内环PID的目标值
**/
void App_flight_pid_process(void);

/**
    * @brief 飞行控制的输出计算
     根据PID的输出计算出电机的PWM值
      这里需要根据具体的飞控设计来计算，可能需要考虑电机的特性、飞控的布局等因素
       例如，可以将PID的输出映射到一个合适的PWM范围内，并且根据飞控的布局调整每个电机的PWM值
**/
void App_flight_control_motor(void);

/**
    * @brief 定高控制的PID计算
**/
void App_flight_fix_height_pid_process(void);

void App_flight_fix_height_reset(uint16_t current_height);

#endif // APPLICATION_FLIGHT_H