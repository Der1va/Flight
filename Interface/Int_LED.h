#ifndef INT_LED_H
#define INT_LED_H

#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
}LED_Struct;

/**
 * @brief 打开LED
 * 
 * @param led 
 */
void Int_LED_turn_on(LED_Struct *led);


/**
 * @brief 关闭LED
 * 
 * @param led 
 */
void Int_LED_turn_off(LED_Struct *led);

/**
 * @brief 翻转LED
 * 
 * @param led 
 */
void Int_LED_toggle(LED_Struct *led);

#endif