#include "Application.h"

void power_Task(void *args);

#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY 4
TaskHandle_t POWER_Handler;

void App_FreeRTOS_Start(void)
{
    xTaskCreate(power_Task, "power_Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &POWER_Handler);
    vTaskStartScheduler();
}

void power_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, 10000);
        Int_IP5305T_Start();
    }
}

