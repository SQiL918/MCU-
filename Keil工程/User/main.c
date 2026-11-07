#include "stm32f10x.h"
#include "bsp.h"
#include "imu.h"
#include "control.h"
#include "path.h"
#include "ui.h"
#include "debug.h"

int main(void)
{
    BSP_Init();
    Control_Init();
    Path_Init();

    BSP_UartSendString("\r\nIMU calibration, keep the car still...\r\n");
    Imu_Init();
    Imu_Calibrate();
    BSP_UartSendString("IMU calibration done\r\n");

    UI_Init();
    Debug_Init();

    BSP_TickStart();

    while (1)
    {
        UI_Update();
        Debug_Update();
    }
}
