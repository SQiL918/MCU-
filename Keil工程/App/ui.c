#include "stm32f10x.h"
#include "ui.h"
#include "app_config.h"
#include "bsp.h"
#include "control.h"
#include "path.h"
#include "imu.h"
#include "OLED.h"
#include "OLED_CN.h"

static volatile uint16_t g_speed_set = 600;
static uint8_t g_mode = 1;

static uint32_t g_last_ui_ms = 0;
static uint32_t g_last_key_ms = 0;

static uint8_t g_db_start_state = 0;
static uint8_t g_db_speed_state = 0;
static uint8_t g_db_mode_state = 0;
static uint8_t g_db_start_cnt = 0;
static uint8_t g_db_speed_cnt = 0;
static uint8_t g_db_mode_cnt = 0;

static const uint16_t g_pat_start[] = { 20, 0 };            /* 100 ms */
static const uint16_t g_pat_point[] = { 10, 10, 10, 0 };    /* two 50 ms beeps */
static const uint16_t g_pat_stop[]  = { 100, 0 };           /* 500 ms */
static const uint16_t *g_pat = 0;
static uint8_t  g_pat_index = 0;
static uint16_t g_pat_timer = 0;

static uint8_t key_debounce(uint8_t raw, uint8_t *state, uint8_t *count)
{
    if (raw != *state)
    {
        *state = raw;
        *count = 0;
    }
    else if (*count < 3)
    {
        (*count)++;
    }

    if ((*count == 3) && raw)
    {
        *count = 4;
        return 1;
    }

    return 0;
}

/* 声光提示必须同时出现：蜂鸣器、外接 LED（PB2）、板载 LED（PC13）由同一
 * 函数驱动，保证 UI_Announce / UI_Tick5ms 里三者的电平永远一致。 */
static void ui_indicator(uint8_t on)
{
    BSP_Buzzer(on);
    BSP_Led(on);
    BSP_LedOnboard(on);
}

void UI_Init(void)
{
    OLED_Init();
    OLED_Clear();

    /* Team name: 第十六队, 4 x 16x16 glyphs centred on the first two pages */
    OLED_ShowChinese16(32, 0, CN_DI);
    OLED_ShowChinese16(48, 0, CN_SHI);
    OLED_ShowChinese16(64, 0, CN_LIU);
    OLED_ShowChinese16(80, 0, CN_DUI);

    OLED_ShowString(2, 1, "L:     R:");
    OLED_ShowString(3, 1, "V:     M:Q");
    OLED_ShowString(4, 1, "Y:     B:  . V");

    ui_indicator(0);
}

void UI_Announce(PathEvent event)
{
    if (event == PATH_EVENT_START)
    {
        g_pat = g_pat_start;
    }
    else if (event == PATH_EVENT_POINT)
    {
        g_pat = g_pat_point;
    }
    else if (event == PATH_EVENT_STOP)
    {
        g_pat = g_pat_stop;
    }
    else
    {
        return;
    }

    g_pat_index = 0;
    g_pat_timer = g_pat[0];

    ui_indicator(1);
}

void UI_Tick5ms(void)
{
    uint16_t d;

    if (g_pat == 0)
    {
        return;
    }

    if (g_pat_timer > 0)
    {
        g_pat_timer--;
    }

    if (g_pat_timer > 0)
    {
        return;
    }

    g_pat_index++;
    d = g_pat[g_pat_index];

    if (d == 0)
    {
        g_pat = 0;
        ui_indicator(0);
        return;
    }

    g_pat_timer = d;

    if ((g_pat_index & 1u) == 0)
    {
        ui_indicator(1);
    }
    else
    {
        ui_indicator(0);
    }
}

uint16_t UI_GetSpeedSet(void)
{
    return g_speed_set;
}

static void ui_show_numbers(void)
{
    int32_t left = (int32_t)Control_GetLeftSpeed();
    int32_t right = (int32_t)Control_GetRightSpeed();
    int32_t yaw = (int32_t)Imu_GetYaw();
    uint16_t raw_adc = BSP_AdcReadBattery();
    uint32_t adc_mv = (uint32_t)raw_adc * 3300UL / 4095UL;
    uint32_t bat_mv = adc_mv * VBAT_ADC_MULT;

    if (left < 0)   { left = 0; }
    if (left > 9999) { left = 9999; }
    if (right < 0)  { right = 0; }
    if (right > 9999) { right = 9999; }

    while (yaw > 180)  { yaw -= 360; }
    while (yaw < -180) { yaw += 360; }

    OLED_ShowNum(2, 3, (uint32_t)left, 4);
    OLED_ShowNum(2, 10, (uint32_t)right, 4);

    OLED_ShowNum(3, 3, (uint32_t)g_speed_set, 4);
    OLED_ShowNum(3, 11, (uint32_t)g_mode, 1);

    OLED_ShowSignedNum(4, 3, yaw, 4);

    OLED_ShowNum(4, 10, bat_mv / 1000UL, 2);
    OLED_ShowNum(4, 13, (bat_mv % 1000UL) / 100UL, 1);
}

void UI_Update(void)
{
    uint32_t now = BSP_GetTick5ms();

    /* keys every 20 ms */
    if ((now - g_last_key_ms) >= 4)
    {
        uint8_t pressed;

        g_last_key_ms = now;

        pressed = key_debounce(BSP_KeyStart(), &g_db_start_state, &g_db_start_cnt);
        if (pressed)
        {
            if (Path_IsRunning())
            {
                Path_Stop();
                BSP_MotorBrake();
            }
            else
            {
                Control_Reset();
                Imu_ResetYaw();
                Path_Start(g_mode);
            }
        }

        pressed = key_debounce(BSP_KeySpeed(), &g_db_speed_state, &g_db_speed_cnt);
        if (pressed)
        {
            if (g_speed_set >= SPEED_SET_MAX) { g_speed_set = SPEED_SET_MIN; }
            else                              { g_speed_set += SPEED_SET_STEP; }
        }

        pressed = key_debounce(BSP_KeyMode(), &g_db_mode_state, &g_db_mode_cnt);
        if (pressed && !Path_IsRunning())
        {
            g_mode++;
            if (g_mode > 4) { g_mode = 1; }
            if (g_mode != Path_GetMode())
            {
                /* mode is applied on the next start */
            }
        }
    }

    /* OLED at 5 Hz */
    if ((now - g_last_ui_ms) >= (UI_REFRESH_MS / 5))
    {
        g_last_ui_ms = now;
        ui_show_numbers();
    }
}
