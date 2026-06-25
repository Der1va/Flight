#include "Application.h"
#include <stdint.h>

extern volatile uint16_t telem_height_mm;

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
static SemaphoreHandle_t PowerOff_Semaphore;

// 飞行任务
void Flight_Task(void *args);
#define FLIGHT_TASK_STACK_SIZE 512
#define FLIGHT_TASK_PRIORITY 3
#define FLIGHT_TASK_PERIOD 6
TaskHandle_t FLIGHT_Handler;

//LED任务
void LED_Task(void *args);
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 1
#define LED_TASK_PERIOD 100
TaskHandle_t LED_Handler;


//通讯任务
void comm_Task(void *args);

#define COMM_TASK_STACK_SIZE 512
#define COMM_TASK_PRIORITY 4
TaskHandle_t COMM_Handler;
//任务周期
#define COMM_TASK_PERIOD 10

//表示当前遥控状态
Remote_State remote_state = REMOTE_STATE_DISCONNECTED;

//表示当前飞行状态
Flight_State flight_state = FLIGHT_STATE_IDLE;

//表示接收的数据
Remote_Data remote_data = {.thr = 0, .yaw = 500, .pitch = 500, .roll = 500, .shut_down = 0, .fix_height = 0};

//定高高度
uint16_t fix_height = 0;
int16_t fix_height_base_thr = 0;
uint8_t back_buff[TX_PLOAD_WIDTH] = {0};

void App_FreeRTOS_Start(void)
{
    PowerOff_Semaphore = xSemaphoreCreateBinary();
    if(PowerOff_Semaphore == NULL)
    {
        Error_Handler();
    }
    // 创建电源管理任务
    xTaskCreate(Power_Task, "Power_Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &POWER_Handler);
    // 创建飞行任务
    xTaskCreate(Flight_Task, "Flight_Task", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, &FLIGHT_Handler);
    // 创建LED任务
    xTaskCreate(LED_Task, "LED_Task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_Handler);
    // 创建通信任务
    xTaskCreate(comm_Task, "comm_Task", COMM_TASK_STACK_SIZE, NULL, COMM_TASK_PRIORITY, &COMM_Handler);
    vTaskStartScheduler();
}


void Power_Task(void *args)
{
    while (1)
    {
        if(xSemaphoreTake(PowerOff_Semaphore, POWER_KEY_PERIOD) == pdTRUE)
        {
            Int_IP5305T_Stop();
        }
    }
}


void Flight_Task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    App_Flight_Start();
    uint8_t count = 0;
    while (1)
    {
        App_Calculate_Euler_Angles();

        App_flight_pid_process();

        if(flight_state == FLIGHT_STATE_STOPPED)
        {
            count++;
            if(count >= 4)
            {
                App_flight_fix_height_pid_process();
                count = 0;
            }
        }
        
        App_flight_control_motor();

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

void comm_Task(void *args)
{
    Int_BAT_ADC_Init();
    while (1)
    {
        uint8_t result = App_Receive_data();
        App_process_connect_state(result);

        if(result == 0 && remote_data.shut_down == 1)
        {
            remote_data.shut_down = 0;
            xSemaphoreGive(PowerOff_Semaphore);
        }

        App_process_flight_state();

        uint16_t voltage_mv = Int_BAT_ADC_Read_mV();

        for(uint8_t i = 0; i < TX_PLOAD_WIDTH; i++)
        {
            back_buff[i] = 0;
        }

        uint16_t height_mm = telem_height_mm;
        uint32_t sum = 0;

        back_buff[0] = 'B';
        back_buff[1] = 'H';

        back_buff[2] = (voltage_mv >> 8) & 0xFF;
        back_buff[3] = voltage_mv & 0xFF;

        back_buff[4] = (height_mm >> 8) & 0xFF;
        back_buff[5] = height_mm & 0xFF;

        back_buff[6] = (fix_height >> 8) & 0xFF;
        back_buff[7] = fix_height & 0xFF;

        back_buff[8] = (uint8_t)flight_state;

        /* 9~12 保留，填 0 */
        back_buff[9] = 0;
        back_buff[10] = 0;
        back_buff[11] = 0;
        back_buff[12] = 0;

        for(uint8_t i = 0; i < 13; i++)
        {
            sum += back_buff[i];
        }

        back_buff[13] = (sum >> 24) & 0xFF;
        back_buff[14] = (sum >> 16) & 0xFF;
        back_buff[15] = (sum >> 8) & 0xFF;
        back_buff[16] = sum & 0xFF;

        vTaskDelay(COMM_TASK_PERIOD);
    }
}
