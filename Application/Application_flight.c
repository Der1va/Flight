#include "Application_flight.h"

IMU_Data imu_data = {0};
Euler_Angle euler_angles = {0};
Gyro_Data last_gyro = {0};
float gyro_z_sum = 0;

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

    Debug_Printf(":%.2f,%.2f,%.2f\n",euler_angles.pitch, euler_angles.roll, euler_angles.yaw);
};
