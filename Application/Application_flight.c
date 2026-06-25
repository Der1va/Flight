#include "Application_flight.h"

/* 高度环每4个飞行周期执行一次：4 × 6 ms = 24 ms */
#define HEIGHT_LOOP_DT_S          0.024f
/* 越小越平滑，但延迟越大 */
#define HEIGHT_FILTER_ALPHA       0.25f
#define HEIGHT_SPEED_FILTER_ALPHA 0.20f
/* 垂直速度阻尼，单位约为 PWM/(mm/s) */
#define HEIGHT_SPEED_DAMPING      0.10f
/* 高度环最多修改正负60 PWM */
#define HEIGHT_OUTPUT_LIMIT       60.0f

#define YAW_TEST_ANGLE_DEG        8.0f

static float filtered_height_mm = 0.0f;
static float last_height_mm = 0.0f;
static float filtered_speed_mm_s = 0.0f;

volatile uint16_t telem_height_mm = 0;

IMU_Data imu_data = {0};
Euler_Angle euler_angles = {0};
Gyro_Data last_gyro = {0};
float gyro_z_sum = 0;

extern Remote_Data remote_data;
extern Flight_State flight_state;
extern uint16_t fix_height;
extern int16_t fix_height_base_thr;
extern TaskHandle_t COMM_Handler;

//俯仰角PID
PID_Controller pitch_pid = {.Kp = -7.0f, .Ki = 0.0f, .Kd = 0.0f};
PID_Controller gyro_y_pid = {.Kp = 2.7f, .Ki = 0.0f, .Kd = 0.35f};

//横滚角PID
PID_Controller roll_pid = {.Kp = -7.0f, .Ki = 0.0f, .Kd = 0.0f};
PID_Controller gyro_x_pid = {.Kp = 2.7f, .Ki = 0.0f, .Kd = 0.35f};

//偏航角PID
PID_Controller yaw_pid = {.Kp = -1.5f, .Ki = 0.0f, .Kd = 0.0f};
PID_Controller gyro_z_pid = {.Kp = -3.5f, .Ki = 0.0f, .Kd = 0.0f};

//定高PID
PID_Controller fix_height_pid = {.Kp = -0.2f, .Ki = 0.0f, .Kd = 0.0f};

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
    int16_t pitch_input = remote_data.pitch - 500;
    if(pitch_input > 10)
    {
        pitch_input -= 10;
    }
    else if(pitch_input < -10)
    {
        pitch_input += 10;
    }
    else
    {
        pitch_input = 0;
    }
    pitch_pid.measurement = euler_angles.pitch;
    pitch_pid.target = pitch_input / 75.0f;
    gyro_y_pid.measurement = imu_data.gyro_data.gyro_y * 2000 / 32768.0f;
    Common_PID_Calculate_chain(&pitch_pid, &gyro_y_pid);
    
    //横滚角
    int16_t roll_input  = remote_data.roll - 500;
    if(roll_input > 10)
    {
        roll_input -= 10;
    }
    else if(roll_input < -10)
    {
        roll_input += 10;
    }
    else
    {
        roll_input = 0;
    }
    roll_pid.measurement = euler_angles.roll;
    roll_pid.target = roll_input / 75.0f;
    gyro_x_pid.measurement = imu_data.gyro_data.gyro_x * 2000 / 32768.0f;
    Common_PID_Calculate_chain(&roll_pid, &gyro_x_pid);
    
    //偏航角
    // 偏航控制：摇杆控制转动速度，松杆后保持新的方向
    static float yaw_target = 0.0f;

    int16_t yaw_input = 500 - remote_data.yaw;
    if(yaw_input > 10)
    {
        yaw_input -= 10;
    }
    else if(yaw_input < -10)
    {
        yaw_input += 10;
    }
    else
    {
        yaw_input = 0;
    }

    // 未起飞时，让目标角跟随当前角度，避免解锁瞬间突然转动
    if(flight_state != FLIGHT_STATE_NORMAL &&
    flight_state != FLIGHT_STATE_STOPPED)
    {
        yaw_target = euler_angles.yaw;
    }
    else
    {
        // 最大偏航速度约90度/秒
        float yaw_rate_target = yaw_input * 90.0f / 490.0f;

        // 每6ms累计一次目标角
        yaw_target += yaw_rate_target * PID_TIME;
    }

    yaw_pid.measurement = euler_angles.yaw;
    yaw_pid.target = yaw_target;
    gyro_z_pid.measurement = imu_data.gyro_data.gyro_z * 2000.0f / 32768.0f;
    Common_PID_Calculate_chain(&yaw_pid, &gyro_z_pid);
    
    /* float yaw_angle = euler_angles.yaw;
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
    Common_PID_Calculate_chain(&yaw_pid, &gyro_z_pid); */

    //Debug_Printf(":%.2f,%.2f\n", gyro_y_pid.error, gyro_y_pid.output);
}

void App_flight_control_motor(void)
{
    int16_t pitch_output = Common_Limit((int16_t)gyro_y_pid.output, -200, 200);
    int16_t roll_output  = Common_Limit((int16_t)gyro_x_pid.output, -200, 200);
    int16_t yaw_output   = Common_Limit((int16_t)gyro_z_pid.output, -100, 100);
    int16_t height_output = Common_Limit((int16_t)fix_height_pid.output, -80, 80);
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
            
            Left_Motor_top.speed = remote_data.thr + pitch_output - roll_output + yaw_output;
            Left_Motor_bottom.speed = remote_data.thr - pitch_output - roll_output - yaw_output;
            Right_Motor_top.speed = remote_data.thr + pitch_output + roll_output - yaw_output;
            Right_Motor_bottom.speed = remote_data.thr - pitch_output + roll_output + yaw_output;
            break;
        case FLIGHT_STATE_STOPPED:
            Left_Motor_top.speed = fix_height_base_thr + pitch_output - roll_output + yaw_output + height_output;
            Left_Motor_bottom.speed = fix_height_base_thr - pitch_output - roll_output - yaw_output + height_output;
            Right_Motor_top.speed = fix_height_base_thr + pitch_output + roll_output - yaw_output + height_output;
            Right_Motor_bottom.speed = fix_height_base_thr - pitch_output + roll_output + yaw_output + height_output;
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

    Left_Motor_top.speed = Common_Limit(Left_Motor_top.speed, 0, 700);
    Left_Motor_bottom.speed = Common_Limit(Left_Motor_bottom.speed, 0, 700);
    Right_Motor_top.speed = Common_Limit(Right_Motor_top.speed, 0, 700);
    Right_Motor_bottom.speed = Common_Limit(Right_Motor_bottom.speed, 0, 700);


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
    float raw_height_mm;
    float raw_speed_mm_s;

    raw_height_mm = (float)Int_VL53L1X_Read_Distance();

    /* 一阶低通滤波高度 */
    filtered_height_mm += HEIGHT_FILTER_ALPHA *
                          (raw_height_mm - filtered_height_mm);

    /* 高度变化率就是垂直速度 */
    raw_speed_mm_s =
        (filtered_height_mm - last_height_mm) / HEIGHT_LOOP_DT_S;

    last_height_mm = filtered_height_mm;

    /* 再对垂直速度低通滤波，避免差分放大测距噪声 */
    filtered_speed_mm_s += HEIGHT_SPEED_FILTER_ALPHA *
                           (raw_speed_mm_s - filtered_speed_mm_s);

    /* 高度位置P环 */
    fix_height_pid.measurement = filtered_height_mm;
    fix_height_pid.target = (float)fix_height;
    Common_PID_Calculate(&fix_height_pid);

    /*
     * 垂直速度阻尼：
     * 上升时速度为正，减小电机输出；
     * 下降时速度为负，增加电机输出。
     */
    fix_height_pid.output -=
        HEIGHT_SPEED_DAMPING * filtered_speed_mm_s;

    /* 防止高度环猛烈改变四路电机 */
    if(fix_height_pid.output > HEIGHT_OUTPUT_LIMIT)
    {
        fix_height_pid.output = HEIGHT_OUTPUT_LIMIT;
    }
    else if(fix_height_pid.output < -HEIGHT_OUTPUT_LIMIT)
    {
        fix_height_pid.output = -HEIGHT_OUTPUT_LIMIT;
    }

    telem_height_mm = (uint16_t)filtered_height_mm;
}

void App_flight_fix_height_reset(uint16_t current_height)
{
    filtered_height_mm = (float)current_height;
    last_height_mm = (float)current_height;
    filtered_speed_mm_s = 0.0f;

    fix_height_pid.measurement = (float)current_height;
    fix_height_pid.target = (float)current_height;
    fix_height_pid.error = 0.0f;
    fix_height_pid.integral = 0.0f;
    fix_height_pid.last_error = 0.0f;
    fix_height_pid.output = 0.0f;

    telem_height_mm = current_height;
}
