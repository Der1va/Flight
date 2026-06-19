#include "Application.h"

///电源管理任务
void power_Task(void *args);
#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY 4
TaskHandle_t POWER_Handler;
#define POWER_TASK_PERIOD 10000

//通讯任务
void comm_Task(void *args);
#define COMM_TASK_STACK_SIZE 256
#define COMM_TASK_PRIORITY 3
TaskHandle_t COMM_Handler;
//任务周期
#define COMM_TASK_PERIOD 10

//按键任务
void key_Task(void *args);
#define KEY_TASK_STACK_SIZE 128
#define KEY_TASK_PRIORITY 2
TaskHandle_t KEY_Handler;
#define KEY_TASK_PERIOD 20

//摇杆任务
void joystick_Task(void *args);
#define JOYSTICK_TASK_STACK_SIZE 128
#define JOYSTICK_TASK_PRIORITY 2
TaskHandle_t JOYSTICK_Handler;
#define JOYSTICK_TASK_PERIOD 10

//屏幕任务
void oled_Task(void *args);
#define OLED_TASK_STACK_SIZE 128
#define OLED_TASK_PRIORITY 1
TaskHandle_t OLED_Handler;
#define OLED_TASK_PERIOD 50

void App_FreeRTOS_Start(void)
{
    xTaskCreate(power_Task, "power_Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &POWER_Handler);
    xTaskCreate(comm_Task, "comm_Task", COMM_TASK_STACK_SIZE, NULL, COMM_TASK_PRIORITY, &COMM_Handler);
    xTaskCreate(key_Task, "key_Task", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, &KEY_Handler);
    xTaskCreate(joystick_Task, "joystick_Task", JOYSTICK_TASK_STACK_SIZE, NULL, JOYSTICK_TASK_PRIORITY, &JOYSTICK_Handler);
    xTaskCreate(oled_Task, "oled_Task", OLED_TASK_STACK_SIZE, NULL, OLED_TASK_PRIORITY, &OLED_Handler);
    vTaskStartScheduler();
}

void power_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, POWER_TASK_PERIOD);
        Int_IP5305T_Start();
    }
}


void comm_Task(void *args)
{
    while (1)
    {
        //将打包的数据发送到飞机
        App_transmit_data();

        vTaskDelay(COMM_TASK_PERIOD);
        //通讯任务的代码
    }
}

void key_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        APP_Process_KEY_Data();
        vTaskDelayUntil(&xLastWakeTime, KEY_TASK_PERIOD);
    }
}

void joystick_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Int_Joystick_Init();
    while (1)
    {
        APP_Process_Joystick_Data();
        vTaskDelayUntil(&xLastWakeTime, JOYSTICK_TASK_PERIOD);
    }
}

void oled_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    APP_display_Init();
    while (1)
    {
        APP_display_Show();
        vTaskDelayUntil(&xLastWakeTime, OLED_TASK_PERIOD);
    }
}
