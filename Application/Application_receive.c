#include "Application_receive.h"

extern Remote_Data remote_data;
extern Remote_State remote_state;
extern Flight_State flight_state;
Thr_State thr_state = FREE;
//MAX状态的进入时间
uint32_t max_state_enter_time = 0;
//MIN状态的进入时间
uint32_t min_state_enter_time = 0;

uint8_t rx_buff[TX_PLOAD_WIDTH] = {0};
uint8_t retry_count = 0;

uint8_t App_Receive_data(void)
{
    memset(rx_buff, 0, TX_PLOAD_WIDTH);
    Int_SI24R1_RxPacket(rx_buff);
    if(strlen((char*)rx_buff) == 0)
    {
        return 1;
    }
    if(rx_buff[0] != FRAME_HEADER_CHECK1 || rx_buff[1] != FRAME_HEADER_CHECK2 || rx_buff[2] != FRAME_HEADER_CHECK3)
    {
        return 1;
    }
    uint32_t sum = 0;
    uint32_t sum_check = 0;
    sum_check = (rx_buff[13] << 24) | (rx_buff[14] << 16) | (rx_buff[15] << 8) | rx_buff[16];
    for(uint8_t i = 0; i < 13; i++)
    {
        sum += rx_buff[i];
    }
    if(sum != sum_check)
    {
        return 1;
    }
    remote_data.thr = (rx_buff[3] << 8) | rx_buff[4];
    remote_data.yaw = (rx_buff[5] << 8) | rx_buff[6];
    remote_data.pitch = (rx_buff[7] << 8) | rx_buff[8];
    remote_data.roll = (rx_buff[9] << 8) | rx_buff[10];
    remote_data.shut_down = rx_buff[11];
    remote_data.fix_height = rx_buff[12];

    //Debug_Printf(":%d,%d,%d,%d,%d,%d\n",remote_data.thr, remote_data.yaw,
        //remote_data.pitch, remote_data.roll, remote_data.shut_down, remote_data.fix_height);
    
    return 0;
}

void App_process_connect_state(uint8_t result)
{
    if(result == 0)
    {
        // 接收成功，表示遥控器连接正常
        remote_state = REMOTE_STATE_CONNECTED;
        retry_count = 0;
    }
    else
    {
        retry_count++;
        if(retry_count >= MAX_RETRY_COUNT)
        {
            retry_count = 0;
            remote_state = REMOTE_STATE_DISCONNECTED;
        }
        // 接收失败，表示遥控器连接异常
    }
}

static uint8_t App_process_unlock(void)
{
    switch(thr_state)
    {
        case FREE:
            if(remote_data.thr >= 900)
            {
                thr_state = MAX;
                max_state_enter_time = xTaskGetTickCount();
            }
            break;
        case MAX:
            if(remote_data.thr < 900)
            {
                if(xTaskGetTickCount() - max_state_enter_time >= 1000)
                {
                    thr_state = LEAVE_MAX;
                }
                else
                {
                    thr_state = FREE;
                }
            }
            break;
        case LEAVE_MAX:
            if(remote_data.thr <= 100)
            {
                thr_state = MIN;
                min_state_enter_time = xTaskGetTickCount();
            }
            break;
        case MIN:
            if(xTaskGetTickCount() - min_state_enter_time < 1000)
            {
                if(remote_data.thr > 100)
                {
                    thr_state = FREE;
                }
            }
            else
            {
                thr_state = UNLOCK;
            }
            break;
        case UNLOCK:
            // 已经解锁，无需处理
            break;
        default:
            break;
    }
    if(thr_state == UNLOCK)
    {
        return 0; // 解锁失败
    }
    return 1;
}

void App_process_flight_state(void)
{
    switch (flight_state)
    {
        case FLIGHT_STATE_IDLE:
            // 飞行器处于空闲状态，等待起飞指令
            if(App_process_unlock() == 0)
            {
                flight_state = FLIGHT_STATE_NORMAL;
                thr_state = FREE; // 重置油门状态，准备下一次解锁
            }
            break;
        case FLIGHT_STATE_NORMAL:
            // 飞行器处于正常飞行状态，执行飞行控制逻辑
            if(remote_data.fix_height == 1)
            {
                // 执行定高逻辑
                flight_state = FLIGHT_STATE_STOPPED;
                remote_data.fix_height = 0;
            }
            if(remote_state == REMOTE_STATE_DISCONNECTED)
            {
                // 遥控器断开连接，执行紧急降落逻辑
                flight_state = FLIGHT_STATE_ERROR;
            }
            break;
        case FLIGHT_STATE_STOPPED:
            // 飞行器处于停止状态，执行降落或悬停逻辑
            if(remote_data.fix_height == 1)
            {
                // 取消定高，恢复正常飞行
                flight_state = FLIGHT_STATE_NORMAL;
                remote_data.fix_height = 0;
            }
            if(remote_state == REMOTE_STATE_DISCONNECTED)
            {
                // 遥控器断开连接，执行紧急降落逻辑
                flight_state = FLIGHT_STATE_ERROR;
            }
            break;
        case FLIGHT_STATE_ERROR:
            // 飞行器处于错误状态，执行紧急处理逻辑
            vTaskDelay(1);
            // 这里可以添加一些错误处理逻辑，比如重置飞行器状态、发送报警信息等
            flight_state = FLIGHT_STATE_IDLE; // 重置为初始状态，等待下一次起飞指令

            break;
        default:
            break;
    }
}
