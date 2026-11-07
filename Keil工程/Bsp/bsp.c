#include "bsp.h"
#include "Delay.h"

static volatile uint32_t g_tick_5ms = 0;
static uint16_t g_enc_last_left = 0;
static uint16_t g_enc_last_right = 0;

/* ------------------------------------------------------------------ */
/* low level helpers                                                   */
/* ------------------------------------------------------------------ */
static void GPIO_Config(GPIO_TypeDef *port, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = pins;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

/* ------------------------------------------------------------------ */
/* LED / buzzer                                                        */
/* ------------------------------------------------------------------ */
void BSP_Led(uint8_t on)
{
    GPIO_WriteBit(LED_PORT, LED_PIN, on ? Bit_SET : Bit_RESET);
}

void BSP_LedOnboard(uint8_t on)
{
    /* PC13 LED is active low */
    GPIO_WriteBit(LED_ONBOARD_PORT, LED_ONBOARD_PIN, on ? Bit_RESET : Bit_SET);
}

void BSP_Buzzer(uint8_t on)
{
    /* The buzzer board used on the car is low-level triggered:
     * PB0 = 0 -> sounds, PB0 = 1 -> silent. */
    GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, on ? Bit_RESET : Bit_SET);
}

/* ------------------------------------------------------------------ */
/* keys                                                                */
/* ------------------------------------------------------------------ */
uint8_t BSP_KeyStart(void)
{
    return GPIO_ReadInputDataBit(KEY_START_PORT, KEY_START_PIN) == Bit_RESET;
}

uint8_t BSP_KeySpeed(void)
{
    return GPIO_ReadInputDataBit(KEY_SPEED_PORT, KEY_SPEED_PIN) == Bit_RESET;
}

uint8_t BSP_KeyMode(void)
{
    return GPIO_ReadInputDataBit(KEY_MODE_PORT, KEY_MODE_PIN) == Bit_RESET;
}

/* ------------------------------------------------------------------ */
/* grayscale                                                           */
/* ------------------------------------------------------------------ */
uint8_t BSP_GrayRead(void)
{
    uint8_t bits = 0;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3))  { bits |= 0x01; } /* OUT1 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))  { bits |= 0x02; } /* OUT2 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))  { bits |= 0x04; } /* OUT3 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))  { bits |= 0x08; } /* OUT4 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11)) { bits |= 0x10; } /* OUT5 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)) { bits |= 0x20; } /* OUT6 */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)) { bits |= 0x40; } /* OUT7 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)) { bits |= 0x80; } /* OUT8 */

#if GRAY_BLACK_IS_HIGH
    /* already active high on black */
#else
    bits = (uint8_t)~bits;
#endif

#if GRAY_REVERSE
    {
        uint8_t r = 0;
        uint8_t i;
        for (i = 0; i < 8; i++)
        {
            if (bits & (1 << i)) { r |= (uint8_t)(1 << (7 - i)); }
        }
        bits = r;
    }
#endif

    return bits;
}

/* ------------------------------------------------------------------ */
/* motors                                                              */
/* ------------------------------------------------------------------ */
static void Motor_One(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      int16_t duty, uint8_t invert)
{
    uint8_t forward;

    if (duty > 0)       { forward = 1; }
    else if (duty < 0)  { forward = 0; duty = (int16_t)(-duty); }
    else
    {
        /* duty 0: short brake, so the inner wheel really stops in a pivot turn */
        GPIO_WriteBit(in1_port, in1_pin, Bit_SET);
        GPIO_WriteBit(in2_port, in2_pin, Bit_SET);
        return;
    }

    if (invert) { forward = (uint8_t)!forward; }

    if (forward)
    {
        GPIO_WriteBit(in1_port, in1_pin, Bit_SET);
        GPIO_WriteBit(in2_port, in2_pin, Bit_RESET);
    }
    else
    {
        GPIO_WriteBit(in1_port, in1_pin, Bit_RESET);
        GPIO_WriteBit(in2_port, in2_pin, Bit_SET);
    }
}

void BSP_MotorSet(int16_t left, int16_t right)
{
    if (left > PWM_MAX)   { left = PWM_MAX; }
    if (left < -PWM_MAX)  { left = -PWM_MAX; }
    if (right > PWM_MAX)  { right = PWM_MAX; }
    if (right < -PWM_MAX) { right = -PWM_MAX; }

    Motor_One(MOTOR_LEFT_AIN1_PORT, MOTOR_LEFT_AIN1_PIN,
              MOTOR_LEFT_AIN2_PORT, MOTOR_LEFT_AIN2_PIN,
              left, MOTOR_LEFT_INVERT);

    Motor_One(MOTOR_RIGHT_BIN1_PORT, MOTOR_RIGHT_BIN1_PIN,
              MOTOR_RIGHT_BIN2_PORT, MOTOR_RIGHT_BIN2_PIN,
              right, MOTOR_RIGHT_INVERT);

    TIM_SetCompare3(TIM2, (left  >= 0) ? (uint16_t)left  : (uint16_t)(-left));
    TIM_SetCompare4(TIM2, (right >= 0) ? (uint16_t)right : (uint16_t)(-right));
}

void BSP_MotorBrake(void)
{
    /* TB6612: IN1 = IN2 = 1 -> short brake */
    GPIO_WriteBit(MOTOR_LEFT_AIN1_PORT,  MOTOR_LEFT_AIN1_PIN,  Bit_SET);
    GPIO_WriteBit(MOTOR_LEFT_AIN2_PORT,  MOTOR_LEFT_AIN2_PIN,  Bit_SET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN1_PORT, MOTOR_RIGHT_BIN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN2_PORT, MOTOR_RIGHT_BIN2_PIN, Bit_SET);
    TIM_SetCompare3(TIM2, 0);
    TIM_SetCompare4(TIM2, 0);
}

void BSP_MotorCoast(void)
{
    GPIO_WriteBit(MOTOR_LEFT_AIN1_PORT,  MOTOR_LEFT_AIN1_PIN,  Bit_RESET);
    GPIO_WriteBit(MOTOR_LEFT_AIN2_PORT,  MOTOR_LEFT_AIN2_PIN,  Bit_RESET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN1_PORT, MOTOR_RIGHT_BIN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN2_PORT, MOTOR_RIGHT_BIN2_PIN, Bit_RESET);
    TIM_SetCompare3(TIM2, 0);
    TIM_SetCompare4(TIM2, 0);
}

/* ------------------------------------------------------------------ */
/* encoders                                                            */
/* ------------------------------------------------------------------ */
void BSP_EncoderRead(int16_t *left, int16_t *right)
{
    uint16_t now_left  = (uint16_t)TIM_GetCounter(TIM3);
    uint16_t now_right = (uint16_t)TIM_GetCounter(TIM4);
    int16_t d_left  = (int16_t)(now_left  - g_enc_last_left);
    int16_t d_right = (int16_t)(now_right - g_enc_last_right);

    g_enc_last_left  = now_left;
    g_enc_last_right = now_right;

    *left  = (int16_t)(d_left  * ENCODER_LEFT_SIGN);
    *right = (int16_t)(d_right * ENCODER_RIGHT_SIGN);
}

void BSP_EncoderReset(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    g_enc_last_left = 0;
    g_enc_last_right = 0;
}

/* ------------------------------------------------------------------ */
/* battery ADC                                                         */
/* ------------------------------------------------------------------ */
uint16_t BSP_AdcReadBattery(void)
{
    uint32_t sum = 0;
    uint8_t i;

    /* Average 8 conversions (~45 us, about one 20 kHz PWM period) so the
     * motor current ripple / ground bounce does not show up as a jumping
     * battery voltage on the OLED. */
    for (i = 0; i < 8; i++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
        {
        }
        sum += ADC_GetConversionValue(ADC1);
    }

    return (uint16_t)(sum / 8);
}

/* ------------------------------------------------------------------ */
/* debug UART                                                          */
/* ------------------------------------------------------------------ */
void BSP_UartSendChar(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint16_t)(uint8_t)c);
}

void BSP_UartSendString(const char *s)
{
    while (*s)
    {
        BSP_UartSendChar(*s++);
    }
}

void BSP_UartSendInt(int32_t v)
{
    char buf[12];
    uint8_t i = 0;
    uint8_t neg = 0;
    uint32_t u;

    if (v < 0) { neg = 1; u = (uint32_t)(-v); }
    else       { u = (uint32_t)v; }

    if (u == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (u > 0 && i < sizeof(buf))
        {
            buf[i++] = (char)('0' + (u % 10));
            u /= 10;
        }
    }

    if (neg) { BSP_UartSendChar('-'); }
    while (i > 0)
    {
        BSP_UartSendChar(buf[--i]);
    }
}

/* ------------------------------------------------------------------ */
/* initialisation                                                      */
/* ------------------------------------------------------------------ */
void BSP_Tick5ms(void)
{
    g_tick_5ms++;
}

uint32_t BSP_GetTick5ms(void)
{
    return g_tick_5ms;
}

static void BSP_LedInit(void)
{
    GPIO_Config(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);
    GPIO_Config(LED_ONBOARD_PORT, LED_ONBOARD_PIN, GPIO_Mode_Out_PP);
    BSP_Led(0);
    BSP_LedOnboard(0);
}

static void BSP_BuzzerInit(void)
{
    GPIO_Config(BUZZER_PORT, BUZZER_PIN, GPIO_Mode_Out_PP);
    BSP_Buzzer(0);
}

static void BSP_KeyInit(void)
{
    GPIO_Config(KEY_START_PORT, KEY_START_PIN, GPIO_Mode_IPU);
    GPIO_Config(KEY_SPEED_PORT, KEY_SPEED_PIN, GPIO_Mode_IPU);
    GPIO_Config(KEY_MODE_PORT,  KEY_MODE_PIN,  GPIO_Mode_IPU);
}

static void BSP_GrayInit(void)
{
    /* PB3/PB4/PA15 are JTAG pins; JTAG is disabled in BSP_Init(). */
    GPIO_Config(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_14,
                GPIO_Mode_IPU);
    GPIO_Config(GPIOA, GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15,
                GPIO_Mode_IPU);
}

static void BSP_MotorInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    GPIO_InitTypeDef gpio;

    /* direction pins */
    GPIO_Config(MOTOR_LEFT_AIN1_PORT,
                MOTOR_LEFT_AIN1_PIN | MOTOR_LEFT_AIN2_PIN, GPIO_Mode_Out_PP);
    GPIO_Config(MOTOR_RIGHT_BIN1_PORT,
                MOTOR_RIGHT_BIN1_PIN | MOTOR_RIGHT_BIN2_PIN, GPIO_Mode_Out_PP);

    /* PWM pins PA2 = TIM2_CH3, PA3 = TIM2_CH4 */
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    tb.TIM_Prescaler = 0;
    tb.TIM_Period = PWM_MAX;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC3Init(TIM2, &oc);
    TIM_OC4Init(TIM2, &oc);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    BSP_MotorCoast();
}

static void BSP_EncoderInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_Init(GPIOB, &gpio);

    tb.TIM_Prescaler = 0;
    tb.TIM_Period = 65535;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3, &tb);
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    ic.TIM_Channel = TIM_Channel_1;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 6;
    TIM_ICInit(TIM3, &ic);
    TIM_ICInit(TIM4, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &ic);
    TIM_ICInit(TIM4, &ic);

    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

static void BSP_AdcInit(void)
{
    ADC_InitTypeDef adc;

    GPIO_Config(GPIOB, GPIO_Pin_1, GPIO_Mode_AIN);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    ADC_DeInit(ADC1);

    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET)
    {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET)
    {
    }
}

static void BSP_UartInit(void)
{
    USART_InitTypeDef uart;
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate = 115200;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

static void BSP_TickInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef nvic;

    tb.TIM_Prescaler = 7199;      /* 72 MHz / 7200 = 10 kHz */
    tb.TIM_Period = 49;           /* 10 kHz / 50 = 200 Hz */
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void BSP_TickStart(void)
{
    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);
}

void BSP_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1 |
                           RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Free PB3/PB4/PA15, keep SWD (PA13/PA14) for ST-Link. */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    Delay_Init();

    BSP_LedInit();
    BSP_BuzzerInit();
    BSP_KeyInit();
    BSP_GrayInit();
    BSP_MotorInit();
    BSP_EncoderInit();
    BSP_AdcInit();
    BSP_UartInit();
    BSP_TickInit();
}
