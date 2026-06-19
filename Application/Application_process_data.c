#include "Application.h"

Joystick_TypeDef joystick = {0};
Remote_Data remote_data = {0};

int16_t thr_offset = 0;
int16_t yaw_offset = 0;
int16_t pit_offset = 0;
int16_t roll_offset = 0;

int16_t key_pit_offset = 0;
int16_t key_roll_offset = 0;

void APP_Joystick_Calibration(void)
{
    // 零偏校准的逻辑就是减去零偏的值
    // 首先清空掉按键的微调值
    key_pit_offset = 0;
    key_roll_offset = 0;
    // 多次读取求平均值
    int16_t thr_sum = 0;
    int16_t yaw_sum = 0;
    int16_t pit_sum = 0;
    int16_t rol_sum = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        Int_Joystick_Read(&joystick);

        //处理范围和极性
        joystick.thr = 1000 - (joystick.thr * 1000 / 4095);
        joystick.yaw = 1000 - (joystick.yaw * 1000 / 4095);
        joystick.pitch = 1000 - (joystick.pitch * 1000 / 4095);
        joystick.roll = 1000 - (joystick.roll * 1000 / 4095);

        
        thr_sum += joystick.thr - 0;
        yaw_sum += joystick.yaw - 500;
        pit_sum += joystick.pitch - 500;
        rol_sum += joystick.roll - 500;
        vTaskDelay(10);
    }

    // 零偏校准的偏移值没有累加的效果  会造成两次校准退回的情况
    thr_offset = thr_sum / 10;
    yaw_offset = yaw_sum / 10;
    pit_offset = pit_sum / 10;
    roll_offset = rol_sum / 10;
}

void APP_Process_KEY_Data(void)
{
    Key_TypeDef key = Int_KEY_get();
    key_pit_offset = 0;
    key_roll_offset = 0;
    if(key == KEY_UP)
    {
        key_pit_offset += 10;
    }
    else if(key == KEY_DOWN)
    {
        key_pit_offset -= 10;
    }
    else if(key == KEY_LEFT)
    {
        key_roll_offset -= 10;
    }
    else if(key == KEY_RIGHT)
    {
        key_roll_offset += 10;
    }
    else if(key == KEY_LEFT_X)
    {
        remote_data.shut_down = 1;
    }
    else if(key == KEY_LEFT_X_LONG)
    {
    }
    else if(key == KEY_RIGHT_X)
    {
        remote_data.fix_height = 1;
    }
    else if(key == KEY_RIGHT_X_LONG)
    {
        //校准摇杆
        APP_Joystick_Calibration();
    }
}

void APP_Process_Joystick_Data(void)
{
    Int_Joystick_Read(&joystick);

    //处理范围和极性
    joystick.thr = 1000 - (joystick.thr * 1000 / 4095);
    joystick.yaw = 1000 - (joystick.yaw * 1000 / 4095);
    joystick.pitch = 1000 - (joystick.pitch * 1000 / 4095);
    joystick.roll = 1000 - (joystick.roll * 1000 / 4095);

    //去除偏移
    joystick.thr -= thr_offset;
    joystick.yaw -= yaw_offset;
    joystick.pitch -= pit_offset;
    joystick.roll -= roll_offset;

    joystick.pitch += key_pit_offset;
    joystick.roll += key_roll_offset;

    joystick.thr = Com_limit_int16(joystick.thr, 0, 1000);
    joystick.yaw = Com_limit_int16(joystick.yaw, 0, 1000);
    joystick.pitch = Com_limit_int16(joystick.pitch, 0, 1000);
    joystick.roll = Com_limit_int16(joystick.roll, 0, 1000);

    //Debug_Printf(":%d,%d,%d,%d\n", joystick.thr, joystick.yaw, joystick.pitch, joystick.roll);

    remote_data.thr = joystick.thr;
    remote_data.yaw = joystick.yaw;
    remote_data.pitch = joystick.pitch;
    remote_data.roll = joystick.roll;
}
