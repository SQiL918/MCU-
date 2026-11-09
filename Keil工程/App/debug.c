#include "stm32f10x.h"
#include "debug.h"
#include "app_config.h"
#include "bsp.h"
#include "control.h"
#include "path.h"
#include "imu.h"
#include "gray_uart.h"

void Debug_Init(void)
{
    uint8_t rst = BSP_ResetCause();

    BSP_UartSendString("\r\nBOOT 16th team STM32F103 ");
    BSP_UartSendString(FW_TAG);
    BSP_UartSendString("\r\nRST flags=");
    BSP_UartSendInt((int32_t)rst);
    if (rst & 0x01u) { BSP_UartSendString(" POR"); }
    if (rst & 0x02u) { BSP_UartSendString(" PIN"); }
    if (rst & 0x04u) { BSP_UartSendString(" SW"); }
    if (rst & 0x08u) { BSP_UartSendString(" IWDG"); }
    if (rst & 0x10u) { BSP_UartSendString(" WWDG"); }
    if (rst & 0x20u) { BSP_UartSendString(" LPWR"); }
    BSP_UartSendString("\r\n");
    if (rst & 0x08u)
    {
        /* The watchdog only fires when the main loop stopped feeding it:
         * a hang, a hard fault, or a supply brown-out. */
        BSP_UartSendString("WARNING: previous run was reset by the watchdog\r\n");
    }
    BSP_UartSendString("GRAY uart 115200, push mode 0x02 (5B + 8 x 12 bit)\r\n");
}

void Debug_Update(void)
{
    static uint32_t last_ms = 0;
    uint32_t now = BSP_GetTick5ms();
    uint16_t raw_adc;
    uint32_t adc_mv;
    uint32_t bat_mv;
    uint8_t i;

    if ((now - last_ms) < 20)   /* 100 ms */
    {
        return;
    }
    last_ms = now;

    raw_adc = BSP_AdcReadBattery();
    adc_mv = (uint32_t)raw_adc * 3300UL / 4095UL;
    bat_mv = adc_mv * VBAT_ADC_MULT;

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
    BSP_UartSendString(" gz=");
    BSP_UartSendInt((int32_t)(Imu_GetGyroZ() * 10.0f));   /* 0.1 dps */
    BSP_UartSendString(" g=");
    BSP_UartSendInt(Control_GetGray());
    BSP_UartSendString(" gl=");
    BSP_UartSendInt(Gray_IsOnline());
    BSP_UartSendString(" gf=");
    BSP_UartSendInt((int32_t)Gray_GetFrames());
    BSP_UartSendString(" gx=");
    BSP_UartSendInt((int32_t)Gray_GetErrors());
    BSP_UartSendString(" e=");
    BSP_UartSendInt((int32_t)(Path_GetLineError() * 100.0f));   /* 0.01 */
    BSP_UartSendString(" yr=");
    BSP_UartSendInt((int32_t)Path_GetYawRef());
    BSP_UartSendString(" gc=");
    BSP_UartSendInt((int32_t)Path_GetGrayCorr());
    BSP_UartSendString(" lt=");
    BSP_UartSendInt((int32_t)Path_GetLineTheta());
    BSP_UartSendString(" lr=");
    BSP_UartSendInt((int32_t)Path_GetLineRate());
    BSP_UartSendString(" lm=");
    BSP_UartSendInt(Path_GetArcMode());
    BSP_UartSendString(" Rr=");
    BSP_UartSendInt((int32_t)Path_GetArcRadius());
    BSP_UartSendString(" L=");
    BSP_UartSendInt((int32_t)Control_GetLeftSpeed());
    BSP_UartSendString(" R=");
    BSP_UartSendInt((int32_t)Control_GetRightSpeed());
    BSP_UartSendString(" bat=");
    BSP_UartSendInt((int32_t)(bat_mv / 1000UL));
    BSP_UartSendString(".");
    BSP_UartSendInt((int32_t)((bat_mv % 1000UL) / 100UL));
    BSP_UartSendString("V");

    /* Raw 8-channel ADC values: white floor should be a few hundred counts
     * on every channel, black line a few thousand.  This is what to look at
     * when a channel or a sensor is faulty. */
    BSP_UartSendString(" a=");
    for (i = 0; i < GRAY_CHANNELS; i++)
    {
        if (i > 0u)
        {
            BSP_UartSendChar(',');
        }
        BSP_UartSendInt((int32_t)Gray_GetRaw(i));
    }

    BSP_UartSendString("\r\n");

    /* One line per arc, so a run can be diagnosed from the log afterwards. */
    {
        ArcReport rep;
        if (Path_TakeArcReport(&rep))
        {
            BSP_UartSendString("ARC d=");
            BSP_UartSendInt(rep.dist_mm);
            BSP_UartSendString(" yaw=");
            BSP_UartSendInt(rep.yaw_deg);
            BSP_UartSendString(" Rr=");
            BSP_UartSendInt(rep.radius_mm);
            BSP_UartSendString(" lm=");
            BSP_UartSendInt(rep.mode);
            BSP_UartSendString(" why=");
            BSP_UartSendInt(rep.why);
            BSP_UartSendString(" e=");
            BSP_UartSendInt(rep.e_min);
            BSP_UartSendString("..");
            BSP_UartSendInt(rep.e_max);
            BSP_UartSendString(" gc=");
            BSP_UartSendInt(rep.gc_min);
            BSP_UartSendString("..");
            BSP_UartSendInt(rep.gc_max);
            BSP_UartSendString(" g=");
            BSP_UartSendInt(rep.g_min);
            BSP_UartSendString("..");
            BSP_UartSendInt(rep.g_max);
            BSP_UartSendString(" lost=");
            BSP_UartSendInt(rep.lost);
            BSP_UartSendString("\r\n");
        }
    }
}
