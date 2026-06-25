#ifndef APPLICATION_TRANSMIT_DATA_H
#define APPLICATION_TRANSMIT_DATA_H

#include "Int_SI24R1.h"
#include "Application_process_data.h"
#include "FreeRTOS.h"
#include "task.h"
    
#define FRAME_HEADER_CHECK1 'c'
#define FRAME_HEADER_CHECK2 'z'
#define FRAME_HEADER_CHECK3 't'



typedef struct
{
    uint16_t voltage_mv;
    uint16_t height_mm;
    uint16_t target_mm;
    uint8_t flight_state;
    uint8_t valid;
} Flight_Telemetry;

extern Flight_Telemetry flight_telem;

/** 
    * @brief 发送数据
    * @param args 任务参数
    * @retval None
**/
void App_transmit_data(void);

#endif // APPLICATION_TRANSMIT_DATA_H