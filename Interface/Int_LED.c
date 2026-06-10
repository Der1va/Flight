#include "Int_LED.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"

void Int_LED_turn_on(LED_Struct *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

void Int_LED_turn_off(LED_Struct *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

void Int_LED_toggle(LED_Struct *led)
{
    HAL_GPIO_TogglePin(led->port, led->pin);
}