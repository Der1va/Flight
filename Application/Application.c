#include "Application.h"
#include "stm32f1xx_hal_tim.h"


Motor_Struct Left_Motor_top = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 100};
Motor_Struct Left_Motor_bottom = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 100};
Motor_Struct Right_Motor_top = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 100};
Motor_Struct Right_Motor_bottom = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 100};

LED_Struct LED_1 = {.port = LED1_GPIO_Port, .pin = LED1_Pin};
LED_Struct LED_2 = {.port = LED2_GPIO_Port, .pin = LED2_Pin};
LED_Struct LED_3 = {.port = LED3_GPIO_Port, .pin = LED3_Pin};
LED_Struct LED_4 = {.port = LED4_GPIO_Port, .pin = LED4_Pin};

// 电源管理任务
void Power_Task(void *args);
#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY 4
#define POWER_KEY_PERIOD 10000
TaskHandle_t POWER_Handler;


// 飞行任务
void Flight_Task(void *args);
#define FLIGHT_TASK_STACK_SIZE 128
#define FLIGHT_TASK_PRIORITY 3
#define FLIGHT_TASK_PERIOD 6
TaskHandle_t FLIGHT_Handler;

//LED任务
void LED_Task(void *args);
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 1
#define LED_TASK_PERIOD 100
TaskHandle_t LED_Handler;


//表示当前遥控状态
Remote_State remote_state = REMOTE_STATE_DISCONNECTED;

//表示当前飞行状态
Flight_State flight_state = FLIGHT_STATE_NORMAL;

void App_FreeRTOS_Start(void)
{
    // 创建电源管理任务
    xTaskCreate(Power_Task, "Power_Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &POWER_Handler);
    // 创建飞行任务
    xTaskCreate(Flight_Task, "Flight_Task", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, &FLIGHT_Handler);
    // 创建LED任务
    xTaskCreate(LED_Task, "LED_Task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_Handler);
    
    vTaskStartScheduler();
}


void Power_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, POWER_KEY_PERIOD);
        Int_IP5305T_Start();
    }
}


void Flight_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        /* Int_Motor_start(&Left_Motor_top);
        Int_Motor_start(&Left_Motor_bottom);
        Int_Motor_start(&Right_Motor_top);
        Int_Motor_start(&Right_Motor_bottom); */

        vTaskDelayUntil(&xLastWakeTime, FLIGHT_TASK_PERIOD);
    }
}


void LED_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t count = 0;
    while (1)
    {
        count++;
        // 前两个灯表示遥控器连接状态
        // 判断当前的连接状态
        if(remote_state == REMOTE_STATE_CONNECTED)
        {
            Int_LED_turn_on(&LED_1);
            Int_LED_turn_on(&LED_2);
        }
        else if(remote_state == REMOTE_STATE_DISCONNECTED)
        {
            Int_LED_turn_off(&LED_1);
            Int_LED_turn_off(&LED_2);
        }

        // 后两个灯表示飞行状态
        if(flight_state == FLIGHT_STATE_IDLE)
        {
            //灯慢闪烁
            if(count % 5 == 0)
            {
                Int_LED_toggle(&LED_3);
                Int_LED_toggle(&LED_4);
            }
        }
        else if(flight_state == FLIGHT_STATE_NORMAL)
        {
            //灯快闪烁
            if(count % 2 == 0)
            {
                Int_LED_toggle(&LED_3);
                Int_LED_toggle(&LED_4);
            }
        }
        else if(flight_state == FLIGHT_STATE_STOPPED)
        {
            Int_LED_turn_on(&LED_3);
            Int_LED_turn_on(&LED_4);
        }
        else if(flight_state == FLIGHT_STATE_ERROR)
        {
            Int_LED_turn_off(&LED_3);
            Int_LED_turn_off(&LED_4);
        }

        if(count == 10)
        {
            count = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, LED_TASK_PERIOD);
    }
}