#include "Application_transmit_data.h"
#include "Int_SI24R1.h"
#include "common_debug.h"


Flight_Telemetry flight_telem = {0};
extern Remote_Data remote_data;
uint8_t remote_data_buff[TX_PLOAD_WIDTH] = {0};
uint8_t post_buff[TX_PLOAD_WIDTH] = {0};

static uint16_t App_ParseUint16(uint8_t high, uint8_t low)
{
    return ((uint16_t)high << 8) | low;
}

static void App_ParseTelemetry(uint8_t *buf)
{
    uint32_t sum = 0;
    uint32_t sum_check = 0;

    if(buf[0] != 'B' || buf[1] != 'H')
    {
        flight_telem.valid = 0;
        return;
    }

    for(uint8_t i = 0; i < 13; i++)
    {
        sum += buf[i];
    }

    sum_check = ((uint32_t)buf[13] << 24) |
                ((uint32_t)buf[14] << 16) |
                ((uint32_t)buf[15] << 8)  |
                buf[16];

    if(sum != sum_check)
    {
        flight_telem.valid = 0;
        return;
    }

    flight_telem.voltage_mv = App_ParseUint16(buf[2], buf[3]);
    flight_telem.height_mm = App_ParseUint16(buf[4], buf[5]);
    flight_telem.target_mm = App_ParseUint16(buf[6], buf[7]);
    flight_telem.flight_state = buf[8];
    flight_telem.valid = 1;
}

static void App_PrintTelemetryToUpper(void)
{
    static uint8_t print_div = 0;

    if(flight_telem.valid == 0)
    {
        return;
    }

    /*
     * comm_Task 是 10ms 一次。
     * 这里 5 次打印一次，大约 50ms，也就是 20Hz。
     * 避免 USART1 阻塞打印太频繁影响遥控发送。
     */
    print_div++;
    if(print_div < 5)
    {
        return;
    }
    print_div = 0;

    printf(":%u,%u,%u,%u\n",
       flight_telem.height_mm,
       flight_telem.target_mm,
       flight_telem.voltage_mv,
       flight_telem.flight_state);
}

void App_transmit_data(void)
{

    uint32_t sum = 0;
    
    remote_data_buff[0] = FRAME_HEADER_CHECK1;
    remote_data_buff[1] = FRAME_HEADER_CHECK2;
    remote_data_buff[2] = FRAME_HEADER_CHECK3;

    remote_data_buff[3] = (remote_data.thr >> 8) & 0xFF;
    remote_data_buff[4] = remote_data.thr & 0xFF;

    remote_data_buff[5] = (remote_data.yaw >> 8) & 0xFF;
    remote_data_buff[6] = remote_data.yaw & 0xFF;

    remote_data_buff[7] = (remote_data.pitch >> 8) & 0xFF;
    remote_data_buff[8] = remote_data.pitch & 0xFF;

    remote_data_buff[9] = (remote_data.roll >> 8) & 0xFF;
    remote_data_buff[10] = remote_data.roll & 0xFF;

    taskENTER_CRITICAL();
    remote_data_buff[11] = remote_data.shut_down;
    remote_data.shut_down = 0;
    remote_data_buff[12] = remote_data.fix_height;
    remote_data.fix_height = 0;
    taskEXIT_CRITICAL();

    for(uint8_t i = 0; i < 13; i++)
    {
        sum += remote_data_buff[i];
    }

    remote_data_buff[13] = (sum >> 24) & 0xFF;
    remote_data_buff[14] = (sum >> 16) & 0xFF;
    remote_data_buff[15] = (sum >> 8) & 0xFF;
    remote_data_buff[16] = sum & 0xFF;

    Int_SI24R1_TX_Mode();
    uint8_t result = Int_SI24R1_TxPacket(remote_data_buff);
    Int_SI24R1_RX_Mode();
    if(result == 0)
    {
        if(result == 0)
        {
            uint16_t wait_count = 20;

            while(Int_SI24R1_RxPacket(post_buff) == 1 && wait_count--)
            {
                vTaskDelay(1);
            }

            if(wait_count > 0)
            {
                App_ParseTelemetry(post_buff);
                App_PrintTelemetryToUpper();
            }
            else
            {
                flight_telem.valid = 0;
            }
        }
        else
        {
            flight_telem.valid = 0;
        }
    }
}