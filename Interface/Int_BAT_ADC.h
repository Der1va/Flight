#ifndef INT_BAT_ADC_H
#define INT_BAT_ADC_H

#include "adc.h"

/** 
    * @brief 初始化电池电压ADC
*/
void Int_BAT_ADC_Init(void);
/**
    * @brief 读取电池电压
    * @return 电压值，单位为伏特
*/
float Int_BAT_ADC_Read(void);

uint16_t Int_BAT_ADC_Read_mV(void);

#endif /* INT_BAT_ADC_H */