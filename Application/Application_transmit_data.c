#include "Application_transmit_data.h"
#include "Int_SI24R1.h"
#include "common_debug.h"

extern Remote_Data remote_data;
uint8_t remote_data_buff[TX_PLOAD_WIDTH] = {0};
uint8_t post_buff[TX_PLOAD_WIDTH] = {0};

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
        while(Int_SI24R1_RxPacket(post_buff) == 1)
        {
        }
    }
}