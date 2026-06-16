#include "Int_MPU6050.h"

int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;
int16_t accel_x_offset = 0;
int16_t accel_y_offset = 0;
int16_t accel_z_offset = 0;

void Int_MPU6050_Write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void Int_MPU6050_Read(uint8_t reg, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

static void Int_MPU6050_Read_Gyro(Gyro_Data *gyro_data)
{
    uint8_t high_byte, low_byte;

    Int_MPU6050_Read(MPU_GYRO_XOUTH_REG, &high_byte);
    Int_MPU6050_Read(MPU_GYRO_XOUTL_REG, &low_byte);
    gyro_data->gyro_x = ((high_byte << 8) | low_byte) - gyro_x_offset;
    
    Int_MPU6050_Read(MPU_GYRO_YOUTH_REG, &high_byte);
    Int_MPU6050_Read(MPU_GYRO_YOUTL_REG, &low_byte);
    gyro_data->gyro_y = ((high_byte << 8) | low_byte) - gyro_y_offset;
    
    Int_MPU6050_Read(MPU_GYRO_ZOUTH_REG, &high_byte);
    Int_MPU6050_Read(MPU_GYRO_ZOUTL_REG, &low_byte);
    gyro_data->gyro_z = ((high_byte << 8) | low_byte) - gyro_z_offset;
}

static void Int_MPU6050_Read_Accel(Accel_Data *accel_data)
{
    uint8_t high_byte, low_byte;

    Int_MPU6050_Read(MPU_ACCEL_XOUTH_REG, &high_byte);
    Int_MPU6050_Read(MPU_ACCEL_XOUTL_REG, &low_byte);
    accel_data->accel_x = ((high_byte << 8) | low_byte) - accel_x_offset;
    
    Int_MPU6050_Read(MPU_ACCEL_YOUTH_REG, &high_byte);
    Int_MPU6050_Read(MPU_ACCEL_YOUTL_REG, &low_byte);
    accel_data->accel_y = ((high_byte << 8) | low_byte) - accel_y_offset;
    
    Int_MPU6050_Read(MPU_ACCEL_ZOUTH_REG, &high_byte);
    Int_MPU6050_Read(MPU_ACCEL_ZOUTL_REG, &low_byte);
    accel_data->accel_z = ((high_byte << 8) | low_byte) - accel_z_offset;
}

//零偏校准
void Int_MPU6050_Calculate_offset(void)
{
    //等待无人机停稳
    Accel_Data curr_accel_data;
    Accel_Data prev_accel_data;
    uint8_t stable_count = 0;
    Int_MPU6050_Read_Accel(&prev_accel_data);
    while (stable_count < 100)
    {
        Int_MPU6050_Read_Accel(&curr_accel_data);
        if (abs(curr_accel_data.accel_x - prev_accel_data.accel_x) < STABLE_THRESHOLD &&
            abs(curr_accel_data.accel_y - prev_accel_data.accel_y) < STABLE_THRESHOLD &&
            abs(curr_accel_data.accel_z - prev_accel_data.accel_z) < STABLE_THRESHOLD)
        {
            stable_count++;
        }
        else
        {
            stable_count = 0; // 如果不稳定，重置计数器
        }
        prev_accel_data = curr_accel_data;
        vTaskDelay(6); // 延时以避免过快采样
    }

    //开始零偏校准
    IMU_Data imu_data;
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;
    int32_t accel_x_sum = 0;
    int32_t accel_y_sum = 0;
    int32_t accel_z_sum = 0;

    for (uint8_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        Int_MPU6050_Read_IMU(&imu_data);
        gyro_x_sum += (imu_data.gyro_data.gyro_x - 0);
        gyro_y_sum += (imu_data.gyro_data.gyro_y - 0);
        gyro_z_sum += (imu_data.gyro_data.gyro_z - 0);
        accel_x_sum += (imu_data.accel_data.accel_x - 0);
        accel_y_sum += (imu_data.accel_data.accel_y - 0);
        accel_z_sum += (imu_data.accel_data.accel_z - 16384);
        vTaskDelay(6); // 延时以确保采样间隔
    }

    // 计算平均值作为零偏
    gyro_x_offset = gyro_x_sum / CALIBRATION_SAMPLES;
    gyro_y_offset = gyro_y_sum / CALIBRATION_SAMPLES;
    gyro_z_offset = gyro_z_sum / CALIBRATION_SAMPLES;
    accel_x_offset = accel_x_sum / CALIBRATION_SAMPLES;
    accel_y_offset = accel_y_sum / CALIBRATION_SAMPLES;
    accel_z_offset = accel_z_sum / CALIBRATION_SAMPLES;
}


void Int_MPU6050_Init(void)
{
    uint8_t data = 0;
    Int_MPU6050_Write(0x6B, 0x80); // 复位MPU60501
    while(data != 0x40) // 等待复位完成
    {
        Int_MPU6050_Read(0x6B, &data);
    }
    Int_MPU6050_Write(MPU_PWR_MGMT1_REG, 0x00); // 退出休眠模式，进入正常工作状态
    Int_MPU6050_Write(MPU_GYRO_CFG_REG, 3 << 3); // 陀螺仪量程设置为±2000°/s
    Int_MPU6050_Write(MPU_ACCEL_CFG_REG, 0x00); // 加速度计量程设置为±2g
    Int_MPU6050_Write(MPU_INT_EN_REG, 0x00); // 禁用中断
    Int_MPU6050_Write(MPU_USER_CTRL_REG, 0x00); // 禁用FIFO 
    Int_MPU6050_Write(MPU_SAMPLE_RATE_REG, 0x01); // 采样率设置为500Hz
    Int_MPU6050_Write(MPU_CFG_REG, 0x01); // 低通滤波器设置为184Hz
    Int_MPU6050_Write(MPU_PWR_MGMT1_REG, 0x01); // 设置时钟源为陀螺仪X轴，提升陀螺仪性能
    Int_MPU6050_Write(MPU_PWR_MGMT2_REG, 0x00); // 禁用电源管理2，保持陀螺仪和加速度计持续工作 

    Int_MPU6050_Calculate_offset(); // 进行零偏校准
}

void Int_MPU6050_Read_IMU(IMU_Data *imu_data)
{
    Int_MPU6050_Read_Gyro(&imu_data->gyro_data);
    Int_MPU6050_Read_Accel(&imu_data->accel_data);
}