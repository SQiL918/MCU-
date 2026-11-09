#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"
#include "app_config.h"

void BSP_Init(void);
void BSP_TickStart(void);
void BSP_Tick5ms(void);
uint32_t BSP_GetTick5ms(void);

/* robustness / diagnostics ------------------------------------------------ */
/* Independent watchdog: if the main loop ever stops feeding it (stuck in a
 * wait loop, hard fault, ...) the chip resets itself instead of hanging. */
void     BSP_WdgInit(uint16_t timeout_ms);
void     BSP_WdgFeed(void);

/* Reset source of THIS boot, sampled before RCC_ClearFlag():
 * bit0 POR, bit1 PIN, bit2 SW, bit3 IWDG, bit4 WWDG, bit5 low power. */
uint8_t  BSP_ResetCause(void);

/* "Where was the code?" marker, printed by the fault handlers so a freeze
 * can be located without a debugger. */
void     BSP_Stage(uint8_t stage);
uint8_t  BSP_GetStage(void);

#define BSP_STAGE_BOOT      1u
#define BSP_STAGE_INIT      2u
#define BSP_STAGE_UI        3u
#define BSP_STAGE_DEBUG     4u
#define BSP_STAGE_GRAY      5u
/* inside the 200 Hz control interrupt */
#define BSP_STAGE_CONTROL   6u   /* encoder + grayscale drain */
#define BSP_STAGE_IMU       7u   /* MPU6050 software I2C (the long one) */
#define BSP_STAGE_PATH      8u   /* path state machine + PID */
#define BSP_STAGE_OUTPUT    9u   /* motor PWM / brake */

/* LED / buzzer */
void BSP_Led(uint8_t on);
void BSP_Buzzer(uint8_t on);
void BSP_LedOnboard(uint8_t on);

/* keys, active low: return 1 when pressed */
uint8_t BSP_KeyStart(void);
uint8_t BSP_KeySpeed(void);
uint8_t BSP_KeyMode(void);

/* grayscale: the 8-channel module is read over the UART, see gray_uart.h.
 * bit0 = channel 1 ... bit7 = channel 8, 1 = black. */

/* motors: duty in PWM counts, -PWM_MAX..+PWM_MAX */
void BSP_MotorSet(int16_t left, int16_t right);
void BSP_MotorBrake(void);
void BSP_MotorCoast(void);

/* encoders: signed delta since the previous call */
void BSP_EncoderRead(int16_t *left, int16_t *right);
void BSP_EncoderReset(void);

/* battery ADC raw value (0..4095) */
uint16_t BSP_AdcReadBattery(void);

/* debug UART, USART1 115200 8N1 */
void BSP_UartSendChar(char c);
void BSP_UartSendString(const char *s);
void BSP_UartSendInt(int32_t v);

#endif
