#include "stm32f10x.h"
#include "control.h"
#include "app_config.h"
#include "bsp.h"
#include "imu.h"
#include "path.h"
#include "ui.h"
#include "gray_uart.h"

static float g_distance_mm = 0.0f;
static float g_speed_left = 0.0f;
static float g_speed_right = 0.0f;
static float g_speed_left_f = 0.0f;
static float g_speed_right_f = 0.0f;
static float g_integ_left = 0.0f;
static float g_integ_right = 0.0f;
static uint8_t g_gray_bits = 0;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float speed_pi(float target, float actual, float *integ)
{
    float err;
    float duty;

    if (target < 5.0f)
    {
        *integ = 0.0f;
        return 0.0f;
    }

    err = target - actual;

    *integ += SPEED_KI * err * CONTROL_DT;
    *integ = clampf(*integ, -SPEED_I_LIMIT, SPEED_I_LIMIT);

    duty = SPEED_KFF * target + SPEED_KP * err + *integ;

    if (duty > 0.0f)
    {
        duty += SPEED_DEADBAND;
    }

    return clampf(duty, 0.0f, (float)PWM_MAX);
}

void Control_Init(void)
{
    g_distance_mm = 0.0f;
    g_speed_left = 0.0f;
    g_speed_right = 0.0f;
    g_speed_left_f = 0.0f;
    g_speed_right_f = 0.0f;
    g_integ_left = 0.0f;
    g_integ_right = 0.0f;
    g_gray_bits = 0;

    BSP_EncoderReset();
    BSP_MotorBrake();
}

void Control_Reset(void)
{
    g_distance_mm = 0.0f;
    g_speed_left = 0.0f;
    g_speed_right = 0.0f;
    g_speed_left_f = 0.0f;
    g_speed_right_f = 0.0f;
    g_integ_left = 0.0f;
    g_integ_right = 0.0f;
    g_gray_bits = 0;

    BSP_EncoderReset();
    BSP_MotorBrake();
}

void Control_Step(void)
{
    int16_t d_left;
    int16_t d_right;
    float dist_delta;
    PathInput in;
    PathOutput out;

    BSP_Stage(BSP_STAGE_CONTROL);
    BSP_EncoderRead(&d_left, &d_right);

    /* Drain and decode the grayscale UART first, so the path state machine
     * below always works on the newest 8-channel picture. */
    Gray_Task();
    g_gray_bits = Gray_GetBits();

    dist_delta = ((float)d_left + (float)d_right) * 0.5f / COUNTS_PER_MM;
    g_distance_mm += dist_delta;

    g_speed_left  = (float)d_left  * CONTROL_HZ / COUNTS_PER_MM;
    g_speed_right = (float)d_right * CONTROL_HZ / COUNTS_PER_MM;

    g_speed_left_f  = 0.60f * g_speed_left_f  + 0.40f * g_speed_left;
    g_speed_right_f = 0.60f * g_speed_right_f + 0.40f * g_speed_right;

    BSP_Stage(BSP_STAGE_IMU);
    Imu_Update(CONTROL_DT);

    BSP_Stage(BSP_STAGE_PATH);
    in.distance_mm = g_distance_mm;
    in.yaw_deg = Imu_GetYaw();
    in.gyro_z_dps = Imu_GetGyroZ();
    in.gray_bits = g_gray_bits;
    in.dt = CONTROL_DT;
    in.base_speed_mmps = (float)UI_GetSpeedSet();

    Path_Update(&in, &out);

    BSP_Stage(BSP_STAGE_OUTPUT);
    if (out.event != PATH_EVENT_NONE)
    {
        UI_Announce(out.event);
    }

    if (out.brake)
    {
        BSP_MotorBrake();
        g_integ_left = 0.0f;
        g_integ_right = 0.0f;
    }
    else
    {
        float duty_left = speed_pi(out.left_mmps, g_speed_left_f, &g_integ_left);
        float duty_right = speed_pi(out.right_mmps, g_speed_right_f, &g_integ_right);

        BSP_MotorSet((int16_t)duty_left, (int16_t)duty_right);
    }
}

float Control_GetLeftSpeed(void)
{
    return g_speed_left_f;
}

float Control_GetRightSpeed(void)
{
    return g_speed_right_f;
}

float Control_GetDistance(void)
{
    return g_distance_mm;
}

uint8_t Control_GetGray(void)
{
    return g_gray_bits;
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        BSP_Tick5ms();
        Control_Step();
        UI_Tick5ms();
    }
}
