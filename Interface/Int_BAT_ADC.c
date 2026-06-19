#include "Int_BAT_ADC.h"
#include "stm32f1xx_hal_adc.h"

void Int_BAT_ADC_Init(void)
{
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_RESET); //使能电池电压检测
    HAL_ADC_Start(&hadc1);
}

float Int_BAT_ADC_Read(void)
{
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    float voltage = (adc_value / 4095.0f) * 3.3f * 2; // 计算电压值，乘以2是因为分压电阻的关系
    return voltage;
}

uint16_t Int_BAT_ADC_Read_mV(void)
{
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    // 电池电压 = ADC / 4095 * 3.3V * 2，单位换成 mV
    return (uint16_t)((adc_value * 3300UL * 2UL) / 4095UL);
}
