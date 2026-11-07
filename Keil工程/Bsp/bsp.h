#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"
#include "app_config.h"

void BSP_Init(void);
void BSP_TickStart(void);
void BSP_Tick5ms(void);
uint32_t BSP_GetTick5ms(void);

/* LED / buzzer */
void BSP_Led(uint8_t on);
void BSP_Buzzer(uint8_t on);
void BSP_LedOnboard(uint8_t on);

/* keys, active low: return 1 when pressed */
uint8_t BSP_KeyStart(void);
uint8_t BSP_KeySpeed(void);
uint8_t BSP_KeyMode(void);

/* grayscale, bit0 = OUT1 ... bit7 = OUT8, 1 = black */
uint8_t BSP_GrayRead(void);

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
