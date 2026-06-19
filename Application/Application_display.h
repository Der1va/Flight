#ifndef APPLICATION_DISPLAY_H
#define APPLICATION_DISPLAY_H

#include "Inf_OLED.h"
#include "Int_SI24R1.h"
#include "Application_process_data.h"

/**
 * @brief 初始化屏幕
 * 
 */
void APP_display_Init(void);

/**
 * @brief 显示屏幕
 * 
 */
void APP_display_Show(void);

#endif 