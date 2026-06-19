#ifndef INT_JOYSTICK_H
#define INT_JOYSTICK_H

#include "adc.h"

typedef struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
} Joystick_TypeDef;

/**
    * @brief 初始化摇杆
    * @return None
**/
void Int_Joystick_Init(void);

/**
    * @brief 读取摇杆数据
    * @param joystick: 指向Joystick_TypeDef结构体的指针，用于存储读取到的摇杆数据
    * @return None
**/
void Int_Joystick_Read(Joystick_TypeDef *joystick);

#endif // INT_JOYSTICK_H
