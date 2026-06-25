#ifndef __APPLICATION_RECEIVE_H__
#define __APPLICATION_RECEIVE_H__

#include "Int_SI24R1.h"
#include "Int_VL53L1X.h"
#include "Int_MPU6050.h"
#include "portmacro.h"
#include "projdefs.h"
#include "common_config.h"
#include "common_debug.h"
#include <stdint.h>
#include <string.h>
#include "Application_flight.h"

#define FRAME_HEADER_CHECK1 'c'
#define FRAME_HEADER_CHECK2 'z'
#define FRAME_HEADER_CHECK3 't'

#define MAX_RETRY_COUNT 20

/**
    * @brief 处理接收到的数据
    * @return 接收数据的状态，0表示成功，1表示失败
**/
uint8_t App_Receive_data(void);

/** 
    * @brief 处理连接状态和飞行状态的LED显示

**/
void App_process_connect_state(uint8_t result);

/**
    * @brief 处理飞行状态

**/
void App_process_flight_state(void);

#endif /* __APPLICATION_RECEIVE_H__ */
