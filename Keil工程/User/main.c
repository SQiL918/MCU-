#include "stm32f10x.h"
#include "bsp.h"
#include "imu.h"
#include "control.h"
#include "path.h"
#include "ui.h"
#include "debug.h"
#include "gray_uart.h"

int main(void)
{
    BSP_Init();

    /* Watchdog armed right away: it also covers a hang during the power-on
     * calibration.  The long, expected waits feed it explicitly
     * (Imu_Init / Imu_Calibrate), the main loop feeds it every pass. */
    BSP_WdgInit(IWDG_TIMEOUT_MS);

    BSP_Stage(BSP_STAGE_INIT);

    Control_Init();
    Path_Init();
    Gray_Init();

    BSP_UartSendString("\r\nIMU calibration, keep the car still...\r\n");
    Imu_Init();
    Imu_Calibrate();
    BSP_UartSendString("IMU calibration done\r\n");

    UI_Init();
    Debug_Init();

    BSP_TickStart();

    while (1)
    {
        BSP_Stage(BSP_STAGE_UI);
        UI_Update();
        BSP_Stage(BSP_STAGE_DEBUG);
        Debug_Update();
        BSP_Stage(BSP_STAGE_GRAY);
        Gray_Poll();
        BSP_WdgFeed();
    }
}
