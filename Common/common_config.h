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
#endif // COMMON_CONFIG_H_
