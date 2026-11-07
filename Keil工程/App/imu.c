#include "stm32f10x.h"
#include "imu.h"
#include "app_config.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "Delay.h"
#include "bsp.h"

#define MPU6050_ADDR_W      0xD0
#define MPU6050_ADDR_R      0xD1
#define GYRO_LSB_PER_DPS    16.4f       /* +-2000 dps */

static float g_gyro_bias_raw = 0.0f;
static float g_gyro_z_dps = 0.0f;
static float g_yaw_deg = 0.0f;

static int16_t Imu_ReadGyroZRaw(void)
{
    uint8_t h;
    uint8_t l;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR_W);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(MPU6050_GYRO_ZOUT_H);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR_R);
    MyI2C_ReceiveAck();
    h = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    l = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return (int16_t)(((uint16_t)h << 8) | (uint16_t)l);
}

void Imu_Init(void)
{
    MyI2C_Init();

    /* Wake up, clock from X gyro PLL */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);

    /* 1 kHz / (1 + 4) = 200 Hz sample rate, 42 Hz DLPF */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);

    /* +-2000 dps gyro, +-2 g accel (accel not used) */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);

    Delay_ms(50);

    /* Debug aid: WHO_AM_I must read 0x68 (104). A floating AD0, swapped
     * SDA/SCL or a missing module shows up as 0 or 255 here, which would
     * also make the yaw stay at exactly 0 forever. */
    BSP_UartSendString("\r\nMPU6050 WHO_AM_I=");
    BSP_UartSendInt((int32_t)MPU6050_GetID());
    BSP_UartSendString(" (expect 104)\r\n");
}

void Imu_Calibrate(void)
{
    int32_t sum = 0;
    uint16_t i;

    for (i = 0; i < 400; i++)
    {
        sum += Imu_ReadGyroZRaw();
        Delay_ms(5);
    }

    g_gyro_bias_raw = (float)sum / 400.0f;
    g_gyro_z_dps = 0.0f;
    g_yaw_deg = 0.0f;

    /* Debug aid: a healthy MPU6050 gives a small bias (tens to a few
     * hundred counts). 0, -1 or 32767 means the reads are not real. */
    BSP_UartSendString("gyro bias=");
    BSP_UartSendInt((int32_t)g_gyro_bias_raw);
    BSP_UartSendString("\r\n");
}

void Imu_Update(float dt)
{
    int16_t raw = Imu_ReadGyroZRaw();
    float dps = ((float)raw - g_gyro_bias_raw) / GYRO_LSB_PER_DPS * GYRO_Z_SIGN;

    /* light first-order filter, ~3 ms time constant at 200 Hz */
    g_gyro_z_dps = 0.70f * g_gyro_z_dps + 0.30f * dps;
    g_yaw_deg += g_gyro_z_dps * dt;
}

float Imu_GetYaw(void)
{
    return g_yaw_deg;
}

float Imu_GetGyroZ(void)
{
    return g_gyro_z_dps;
}

void Imu_SetYaw(float yaw)
{
    g_yaw_deg = yaw;
}

void Imu_ResetYaw(void)
{
    g_yaw_deg = 0.0f;
    g_gyro_z_dps = 0.0f;
}
