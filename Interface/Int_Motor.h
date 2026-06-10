#ifndef Motor_H
#define Motor_H

#include "tim.h"
#include <stdint.h>
#include "common_debug.h"

typedef struct
{
    TIM_HandleTypeDef *tim;
    uint16_t channel;
    uint16_t speed;
}Motor_Struct;

/**
    * @brief 设置电机速度
    * @param motor: 指向Motor_Struct结构体的指针，包含定时器句柄、通道和速度信息。
    * @note 速度应在0到1000的范围内。超过1000的值将被拒绝。
**/
void Int_Motor_set_speed(Motor_Struct *motor);

/**
    * @brief 启动电机
    * @param motor: 指向Motor_Struct结构体的指针，包含定时器句柄、通道和速度信息。
**/
void Int_Motor_start(Motor_Struct *motor);

#endif /* Motor_H */