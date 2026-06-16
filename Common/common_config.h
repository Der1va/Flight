#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_

#include "main.h"

//遥控状态
typedef enum
{
    REMOTE_STATE_DISCONNECTED = 0,
    REMOTE_STATE_CONNECTED,
}Remote_State;

//飞行状态
typedef enum
{
    FLIGHT_STATE_IDLE = 0,
    FLIGHT_STATE_NORMAL,
    FLIGHT_STATE_STOPPED,
    FLIGHT_STATE_ERROR
}Flight_State;

//油门解锁状态
typedef enum
{
    FREE = 0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK
}Thr_State;

typedef struct
{
    int16_t thr;
    int16_t yaw;//偏航角
    int16_t pitch;//俯仰角
    int16_t roll;//翻滚角
    uint8_t shut_down;
    uint8_t fix_height;
}Remote_Data;

//陀螺仪数据
typedef struct
{
    int16_t gyro_x;//往右飞为正
    int16_t gyro_y;//往前飞为正
    int16_t gyro_z;//逆时针为正
}Gyro_Data;

typedef struct
{
    int16_t accel_x;//往前为正
    int16_t accel_y;//往左为正
    int16_t accel_z;//往上为正
}Accel_Data;

typedef struct
{
    Gyro_Data gyro_data;
    Accel_Data accel_data;
}IMU_Data;

//解算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
}Euler_Angle;


#endif // COMMON_CONFIG_H_
