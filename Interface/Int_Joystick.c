#include "Int_Joystick.h"

volatile uint16_t adc_buffer[4];

void Int_Joystick_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 4);
}

void Int_Joystick_Read(Joystick_TypeDef *joystick)
{
    joystick->thr = adc_buffer[0];
    joystick->yaw = adc_buffer[1];
    joystick->pitch = adc_buffer[2];
    joystick->roll = adc_buffer[3];
}