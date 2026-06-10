#include "Int_Motor.h"
#include "stm32f1xx_hal_tim.h"

void Int_Motor_set_speed(Motor_Struct *motor)
{
    if(motor->speed > 1000)
    {
        Debug_Printf("Motor speed out of range");
        return;
    }
    
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, motor->speed);
}

void Int_Motor_start(Motor_Struct *motor)
{
    HAL_TIM_PWM_Start(motor->tim, motor->channel);
}