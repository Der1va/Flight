#include "Application_flight.h"

IMU_Data imu_data = {0};
Euler_Angle euler_angles = {0};
Gyro_Data last_gyro = {0};
float gyro_z_sum = 0;

extern Remote_Data remote_data;
extern Flight_State flight_state;
extern uint16_t fix_height;
extern TaskHandle_t COMM_Handler;

//俯仰角PID
PID_Controller pitch_pid = {.Kp = -7.5f, .Ki = 0.0f, .Kd = 0.0f};
PID_Controller gyro_y_pid = {.Kp = 3.0f, .Ki = 0.0f, .Kd = 0.5f};

//横滚角PID
PID_Controller roll_pid = {.Kp = -7.5f, .Ki = 0.0f, .Kd = 0.0f};
PID_Controller gyro_x_pid = {.Kp = 3.0f, .Ki = 0.0f, .Kd = 0.5f};

//偏航角PID
PID_Controller yaw_pid = {.Kp = -1.5f, .Ki = 0.0f, .Kd = 0.0f};
PID_Controller gyro_z_pid = {.Kp = -3.5f, .Ki = 0.0f, .Kd = 0.0f};

//定高PID
PID_Controller fix_height_pid = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f};

Motor_Struct Left_Motor_top = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
Motor_Struct Left_Motor_bottom = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
Motor_Struct Right_Motor_top = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
Motor_Struct Right_Motor_bottom = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

void App_Flight_Start(void)
{
    Int_MPU6050_Init();

    Int_Motor_start(&Left_Motor_top);
    Int_Motor_start(&Left_Motor_bottom);
    Int_Motor_start(&Right_Motor_top);
    Int_Motor_start(&Right_Motor_bottom);

    Int_VL53L1X_Init();
}

void App_Calculate_Euler_Angles(void)
{
    Int_MPU6050_Read_IMU(&imu_data);

    imu_data.gyro_data.gyro_x = Common_Filter_LowPass(imu_data.gyro_data.gyro_x, last_gyro.gyro_x);
    imu_data.gyro_data.gyro_y = Common_Filter_LowPass(imu_data.gyro_data.gyro_y, last_gyro.gyro_y);
    imu_data.gyro_data.gyro_z = Common_Filter_LowPass(imu_data.gyro_data.gyro_z, last_gyro.gyro_z);
    last_gyro.gyro_x = imu_data.gyro_data.gyro_x;
    last_gyro.gyro_y = imu_data.gyro_data.gyro_y;
    last_gyro.gyro_z = imu_data.gyro_data.gyro_z;

    imu_data.accel_data.accel_x = Common_Filter_KalmanFilter(&kfs[0], imu_data.accel_data.accel_x);
    imu_data.accel_data.accel_y = Common_Filter_KalmanFilter(&kfs[1], imu_data.accel_data.accel_y);
    imu_data.accel_data.accel_z = Common_Filter_KalmanFilter(&kfs[2], imu_data.accel_data.accel_z);

    //Debug_Printf(":%d,%d,%d\n", imu_data.gyro_data.gyro_x, imu_data.gyro_data.gyro_y, imu_data.gyro_data.gyro_z);
    //Debug_Printf(":%d,%d,%d\n", imu_data.accel_data.accel_x, imu_data.accel_data.accel_y, imu_data.accel_data.accel_z);

    //互补结算
    /* euler_angles.pitch = atan2f(imu_data.accel_data.accel_x, imu_data.accel_data.accel_z) * 180 / M_PI;
    euler_angles.roll = atan2f(imu_data.accel_data.accel_y, imu_data.accel_data.accel_z) * 180 / M_PI;
    gyro_z_sum += (imu_data.gyro_data.gyro_z * 2000 / 32768.0) * 0.006f;
    euler_angles.yaw = gyro_z_sum; */

    //四元数解算
    Common_IMU_GetEulerAngle(&imu_data, &euler_angles, 0.006f);

    //Debug_Printf(":%.2f,%.2f,%.2f\n",euler_angles.pitch, euler_angles.roll, euler_angles.yaw);
};

void App_flight_pid_process(void)
{
    //俯仰角
    pitch_pid.measurement = euler_angles.pitch;
    pitch_pid.target = (remote_data.pitch - 500) / 100.0f;
    gyro_y_pid.measurement = imu_data.gyro_data.gyro_y * 2000 / 32768.0f;
    Common_PID_Calculate_chain(&pitch_pid, &gyro_y_pid);
    
    //横滚角
    int16_t roll_error = remote_data.roll - 500;
    if(roll_error > -5 && roll_error < 5)
    {
        roll_error = 0;
    }
    roll_pid.measurement = euler_angles.roll;
    roll_pid.target = roll_error / 100.0f;
    gyro_x_pid.measurement = imu_data.gyro_data.gyro_x * 2000 / 32768.0f;
    Common_PID_Calculate_chain(&roll_pid, &gyro_x_pid);
    
    //偏航角
    /* yaw_pid.measurement = euler_angles.yaw;
    yaw_pid.target = (remote_data.yaw - 500) / 100.0f;
    gyro_z_pid.measurement = imu_data.gyro_data.gyro_z * 2000 / 32768.0f;
    Common_PID_Calculate_chain(&yaw_pid, &gyro_z_pid); */
    float yaw_angle = euler_angles.yaw;
    float gyro_z = imu_data.gyro_data.gyro_z * 2000 / 32768.0f;
    // 偏航角死区：实际偏航角在 -1~1 度内，认为已经对准目标 0 度
    if(yaw_angle > -1.0f && yaw_angle < 1.0f)
    {
        yaw_angle = 0.0f;
    }
    // Z轴角速度死区：小角速度认为没有自转，减少抖动
    if(gyro_z > -1.0f && gyro_z < 1.0f)
    {
        gyro_z = 0.0f;
    }
    // 偏航角外环：目标角固定为 0 度
    yaw_pid.measurement = yaw_angle;
    yaw_pid.target = 0.0f;
    // 偏航角速度内环
    gyro_z_pid.measurement = gyro_z;
    Common_PID_Calculate_chain(&yaw_pid, &gyro_z_pid);

    //Debug_Printf(":%.2f,%.2f\n", gyro_y_pid.error, gyro_y_pid.output);
}

void App_flight_control_motor(void)
{
    //判断当前飞机的飞行状态
    switch (flight_state)
    {    
        case FLIGHT_STATE_IDLE:
            Left_Motor_top.speed = 0;
            Left_Motor_bottom.speed = 0;
            Right_Motor_top.speed = 0;
            Right_Motor_bottom.speed = 0;
            break;
        case FLIGHT_STATE_NORMAL:
            /* Left_Motor_top.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output;
            Left_Motor_bottom.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output;
            Right_Motor_top.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output;
            Right_Motor_bottom.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output; */
            
            Left_Motor_top.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Common_Limit(gyro_z_pid.output, -250, 250);
            Left_Motor_bottom.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Common_Limit(gyro_z_pid.output, -250, 250);
            Right_Motor_top.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Common_Limit(gyro_z_pid.output, -250, 250);
            Right_Motor_bottom.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Common_Limit(gyro_z_pid.output, -250, 250);
            break;
        case FLIGHT_STATE_STOPPED:
            Left_Motor_top.speed = remote_data.thr + gyro_y_pid.output - gyro_x_pid.output + Common_Limit(gyro_z_pid.output, -250, 250) + fix_height_pid.output;
            Left_Motor_bottom.speed = remote_data.thr - gyro_y_pid.output - gyro_x_pid.output - Common_Limit(gyro_z_pid.output, -250, 250) + fix_height_pid.output;
            Right_Motor_top.speed = remote_data.thr + gyro_y_pid.output + gyro_x_pid.output - Common_Limit(gyro_z_pid.output, -250, 250) + fix_height_pid.output;
            Right_Motor_bottom.speed = remote_data.thr - gyro_y_pid.output + gyro_x_pid.output + Common_Limit(gyro_z_pid.output, -250, 250) + fix_height_pid.output;
            break;
        case FLIGHT_STATE_ERROR:
            Left_Motor_top.speed -= 2;
            Left_Motor_bottom.speed -= 2;
            Right_Motor_top.speed -= 2;
            Right_Motor_bottom.speed -= 2;
            if(Left_Motor_top.speed <= 0 && Left_Motor_bottom.speed <= 0 && Right_Motor_top.speed <= 0 && Right_Motor_bottom.speed <= 0)
            {
                xTaskNotifyGive(COMM_Handler);
            }
            break;
        default:
            break;
    }

    Left_Motor_top.speed = Common_Limit(Left_Motor_top.speed, 0, 800);
    Left_Motor_bottom.speed = Common_Limit(Left_Motor_bottom.speed, 0, 800);
    Right_Motor_top.speed = Common_Limit(Right_Motor_top.speed, 0, 800);
    Right_Motor_bottom.speed = Common_Limit(Right_Motor_bottom.speed, 0, 800);


    if(remote_data.thr <= 50)
    {
        Left_Motor_top.speed = 0;
        Left_Motor_bottom.speed = 0;
        Right_Motor_top.speed = 0;
        Right_Motor_bottom.speed = 0;
    }

    Int_Motor_set_speed(&Left_Motor_top);
    Int_Motor_set_speed(&Left_Motor_bottom);
    Int_Motor_set_speed(&Right_Motor_top);
    Int_Motor_set_speed(&Right_Motor_bottom);
}

void App_flight_fix_height_pid_process(void)
{
    fix_height_pid.measurement = Int_VL53L1X_Read_Distance();
    fix_height_pid.target = fix_height;
    Common_PID_Calculate(&fix_height_pid);
}

Left_Motor_top.speed = remote_data.thr + pitch_output - roll_output + yaw_output;
            Left_Motor_bottom.speed = remote_data.thr - pitch_output - roll_output - yaw_output;
            Right_Motor_top.speed = remote_data.thr + pitch_output + roll_output - yaw_output;
            Right_Motor_bottom.speed = remote_data.thr - pitch_output + roll_output + yaw_output;

Left_Motor_top.speed = remote_data.thr + pitch_output - roll_output + yaw_output + fix_height_pid.output;
            Left_Motor_bottom.speed = remote_data.thr - pitch_output - roll_output - yaw_output + fix_height_pid.output;
            Right_Motor_top.speed = remote_data.thr + pitch_output + roll_output - yaw_output + fix_height_pid.output;
            Right_Motor_bottom.speed = remote_data.thr - pitch_output + roll_output + yaw_output + fix_height_pid.output;