#include "Application_display.h"

extern Remote_Data remote_data;
extern uint8_t post_buff[TX_PLOAD_WIDTH];

#define LINE1_BEGIN 46
#define LINE2_BEGIN 5
#define LINE3_BEGIN 5
#define BAR1_BEGIN 35
#define BAR2_BEGIN 47
#define Y0 0
#define Y1 14
#define Y2 26

void App_diasplat_Show_bar(uint16_t x, uint16_t y, uint8_t count)
{
    if(count < 13)
    {
        OLED_Show_CH(x, y, 12 + count, 12, 1);
    }
}

void APP_display_Init(void)
{
    OLED_Init();
}

static void App_FormatVoltageString(uint16_t mv, uint8_t *str)
{
    uint16_t cv = (mv + 5) / 10;   // mV -> 0.01V

    str[0] = (cv / 100) + '0';
    str[1] = '.';
    str[2] = ((cv / 10) % 10) + '0';
    str[3] = (cv % 10) + '0';
    str[4] = 'V';
    str[5] = '\0';
}

void APP_display_Show(void)
{
    uint8_t count = 0;
    for(uint8_t i = 0; i < 3; i++)
    {
        OLED_Show_CH(LINE1_BEGIN + 12 *i , Y0, i, 12, 1);
    }
    uint8_t voltage_str[8] = "0.00V";

    if(post_buff[0] == 'B' && post_buff[1] == 'V')
    {
        uint16_t voltage_mv = ((uint16_t)post_buff[2] << 8) | post_buff[3];
        App_FormatVoltageString(voltage_mv, voltage_str);
    }

    OLED_ShowString(LINE2_BEGIN, Y1, "V:", 12, 1);
    OLED_ShowString(LINE2_BEGIN + 12, Y1, voltage_str, 12, 1);

    OLED_ShowString(LINE2_BEGIN, Y2, (const uint8_t *)"THR:", 12, 1);
    if(remote_data.thr > 500)
    {
        count = (remote_data.thr - 500) / 41;
        App_diasplat_Show_bar(BAR1_BEGIN, Y2, 12);
        App_diasplat_Show_bar(BAR2_BEGIN, Y2, count);
    }
    else
    {
        count = remote_data.thr / 41;
        App_diasplat_Show_bar(BAR1_BEGIN, Y2, count);
        App_diasplat_Show_bar(BAR2_BEGIN, Y2, 0);
    }
    
    OLED_Refresh_Gram();
}
