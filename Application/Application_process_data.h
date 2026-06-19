#ifndef APPLICATION_PROCESS_DATA_H
#define APPLICATION_PROCESS_DATA_H

#include "Int_KEY.h"
#include "Int_Joystick.h"
#include "common_tool.h"

typedef struct
{
    int16_t thr;//油门
    int16_t yaw;//偏航
    int16_t pitch;//俯仰
    int16_t roll;//翻滚
    uint8_t shut_down;//关机
    uint8_t fix_height;//定高
}Remote_Data;

/**
 * @brief 处理按键数据
 * 
 */
void APP_Process_KEY_Data(void);

/**
 * @brief 处理摇杆数据
 * 
 */
void APP_Process_Joystick_Data(void);

#endif // APPLICATION_PROCESS_DATA_H