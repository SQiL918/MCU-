#include "stm32f10x.h"
#include "debug.h"
#include "app_config.h"
#include "bsp.h"
#include "control.h"
#include "path.h"
#include "imu.h"

void Debug_Init(void)
{
    BSP_UartSendString("\r\nBOOT 16th team STM32F103\r\n");
}

void Debug_Update(void)
{
    static uint32_t last_ms = 0;
    uint32_t now = BSP_GetTick5ms();
    uint16_t raw_adc;
    uint32_t adc_mv;
    uint32_t bat_mv;

    if ((now - last_ms) < 20)   /* 100 ms */
    {
        return;
    }
    last_ms = now;

    raw_adc = BSP_AdcReadBattery();
    adc_mv = (uint32_t)raw_adc * 3300UL / 4095UL;
    bat_mv = adc_mv * 1000UL / 180UL;

    BSP_UartSendString("t=");
    BSP_UartSendInt((int32_t)(now * 5));
    BSP_UartSendString(" run=");
    BSP_UartSendInt(Path_IsRunning());
    BSP_UartSendString(" mode=");
    BSP_UartSendInt(Path_GetMode());
    BSP_UartSendString(" seg=");
    BSP_UartSendInt(Path_GetSegment());
    BSP_UartSendString(" d=");
    BSP_UartSendInt((int32_t)Control_GetDistance());
    BSP_UartSendString(" yaw=");
    BSP_UartSendInt((int32_t)Imu_GetYaw());
    BSP_UartSendString(" g=");
    BSP_UartSendInt(Control_GetGray());
    BSP_UartSendString(" L=");
    BSP_UartSendInt((int32_t)Control_GetLeftSpeed());
    BSP_UartSendString(" R=");
    BSP_UartSendInt((int32_t)Control_GetRightSpeed());
    BSP_UartSendString(" bat=");
    BSP_UartSendInt((int32_t)(bat_mv / 1000UL));
    BSP_UartSendString(".");
    BSP_UartSendInt((int32_t)((bat_mv % 1000UL) / 100UL));
    BSP_UartSendString("V\r\n");
}
