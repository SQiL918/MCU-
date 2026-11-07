#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "stm32f10x.h"

/* =========================================================================
 * 16th Team - STM32F103C8T6 line-following car
 * Hardware: Blue Pill + TB6612FNG + 2x MG513 encoder motors
 *           8-channel CH32V006 grayscale + MPU6050 + SSD1306 OLED
 *
 * Pin map (all pins are allocated, do not change without updating the
 * wiring table in 接线图/接线总表.md):
 *
 *  PA0  KEY_START      input pull-up, active low
 *  PA1  KEY_SPEED      input pull-up, active low
 *  PA2  TIM2_CH3/PWMA  left motor PWM
 *  PA3  TIM2_CH4/PWMB  right motor PWM
 *  PA4  AIN1           left motor direction
 *  PA5  AIN2           left motor direction
 *  PA6  TIM3_CH1       left encoder A
 *  PA7  TIM3_CH2       left encoder B
 *  PA8  GRAY_OUT4      grayscale channel 4
 *  PA9  USART1_TX      debug TX -> CH340 RX
 *  PA10 USART1_RX      debug RX <- CH340 TX
 *  PA11 GRAY_OUT5      grayscale channel 5
 *  PA12 GRAY_OUT6      grayscale channel 6
 *  PA13 SWDIO          keep for ST-Link
 *  PA14 SWCLK          keep for ST-Link
 *  PA15 GRAY_OUT7      grayscale channel 7 (JTAG disabled)
 *  PB0  BUZZER         active buzzer through S8050
 *  PB1  ADC_IN9        battery voltage divider
 *  PB2  LED            external status LED
 *  PB3  GRAY_OUT1      grayscale channel 1 (JTAG disabled)
 *  PB4  GRAY_OUT2      grayscale channel 2 (JTAG disabled)
 *  PB5  GRAY_OUT3      grayscale channel 3
 *  PB6  TIM4_CH1       right encoder A
 *  PB7  TIM4_CH2       right encoder B
 *  PB8  OLED_SCL       software I2C
 *  PB9  OLED_SDA       software I2C
 *  PB10 MPU_SCL        software I2C
 *  PB11 MPU_SDA        software I2C
 *  PB12 BIN1           right motor direction
 *  PB13 BIN2           right motor direction
 *  PB14 GRAY_OUT8      grayscale channel 8
 *  PB15 KEY_MODE       input pull-up, active low
 *  PC13 LED_ONBOARD    active low
 * ========================================================================= */

/* ---------------- pins ---------------- */
#define KEY_START_PORT      GPIOA
#define KEY_START_PIN       GPIO_Pin_0
#define KEY_SPEED_PORT      GPIOA
#define KEY_SPEED_PIN       GPIO_Pin_1
#define KEY_MODE_PORT       GPIOB
#define KEY_MODE_PIN        GPIO_Pin_15

#define LED_PORT            GPIOB
#define LED_PIN             GPIO_Pin_2
#define LED_ONBOARD_PORT    GPIOC
#define LED_ONBOARD_PIN     GPIO_Pin_13

#define BUZZER_PORT         GPIOB
#define BUZZER_PIN          GPIO_Pin_0

#define MOTOR_LEFT_AIN1_PORT GPIOA
#define MOTOR_LEFT_AIN1_PIN  GPIO_Pin_4
#define MOTOR_LEFT_AIN2_PORT GPIOA
#define MOTOR_LEFT_AIN2_PIN  GPIO_Pin_5
#define MOTOR_RIGHT_BIN1_PORT GPIOB
#define MOTOR_RIGHT_BIN1_PIN  GPIO_Pin_12
#define MOTOR_RIGHT_BIN2_PORT GPIOB
#define MOTOR_RIGHT_BIN2_PIN  GPIO_Pin_13

/* ---------------- geometry (mm, degrees) ---------------- */
#define TRACK_AB_MM         1000.0f
#define TRACK_CD_MM         1000.0f
#define TRACK_ARC_MM        1256.637f   /* pi * 400 mm */
#define TRACK_DIAG_MM       1280.625f   /* sqrt(1000^2 + 800^2) */
#define TRACK_ANGLE_DEG     38.6598f    /* atan2(800,1000) */
#define TRACK_RADIUS_MM     400.0f
#define CAR_TRACK_WIDTH_MM  150.0f      /* measure centre-to-centre */
#define TURN_FORWARD_MM     47.0f       /* forward travel during a 38.66 deg pivot */
#define TURN_INNER_RATIO    0.0f        /* 0 = inner wheel stopped, 0.3 = forward arc */

/* ---------------- encoder / wheel ---------------- */
/* MG513: 11 PPR x 4 edges x 30 gear = 1320 counts per wheel revolution.
 * 65 mm wheel -> 204.2 mm circumference -> 6.464 counts/mm.
 * Calibrate by pushing the car 1000 mm and reading the count. */
#define ENCODER_COUNTS_PER_REV  1320.0f
#define WHEEL_DIAMETER_MM       65.0f
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)
#define COUNTS_PER_MM           (ENCODER_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_MM)
/* Bench test 2026-11-07 (D153B, motors plugged left -> Motor A, right -> Motor B):
 * with MOTOR_*_INVERT = 0 the car now drives forward, so do NOT invert the
 * motors. Serial showed L = -1421 while driving forward with LEFT_SIGN = -1,
 * i.e. both encoder channels count positive for forward motion. */
#define ENCODER_LEFT_SIGN       (+1)
#define ENCODER_RIGHT_SIGN      (+1)
#define MOTOR_LEFT_INVERT       0
#define MOTOR_RIGHT_INVERT      0
/* Set to -1.0f if a left (counter-clockwise) turn makes yaw decrease. */
#define GYRO_Z_SIGN             (+1.0f)

/* ---------------- control ---------------- */
#define CONTROL_HZ              200.0f
#define CONTROL_DT              (1.0f / CONTROL_HZ)
#define PWM_MAX                 3599    /* 20 kHz at 72 MHz */
#define PWM_FREQ_HZ             20000
#define MAX_SPEED_MMPS          1100.0f
#define SPEED_MIN_MMPS          0.0f
#define SPEED_RAMP_MMPS2        1800.0f

/* speed loop */
#define SPEED_KP                1.10f
#define SPEED_KI                18.0f
#define SPEED_KFF               (3599.0f / 1100.0f)
#define SPEED_I_LIMIT           450.0f
#define SPEED_DEADBAND          90.0f

/* heading hold */
#define YAW_KP                  8.0f    /* mm/s per degree */
#define YAW_KD                  1.5f    /* mm/s per deg/s */
#define YAW_TURN_LIMIT          220.0f

/* line following */
#define LINE_KP                 260.0f  /* mm/s per normalized error */
#define LINE_KD                 45.0f
#define LINE_TURN_LIMIT         260.0f

/* pivot turn */
#define TURN_KP                 5.0f    /* mm/s per degree */
#define TURN_MIN_MMPS           130.0f
#define TURN_MAX_MMPS           520.0f
#define TURN_TOL_DEG            1.8f
#define TURN_TIMEOUT_MS         2500

/* arc finish */
#define ARC_LINE_LOST_SAMPLES   3
#define ARC_FINISH_MARGIN_MM    60.0f
#define SEGMENT_TIMEOUT_MS      15000

/* ---------------- grayscale ---------------- */
/* The CH32V006 board outputs a high level on BLACK (ADC > 2000).
 * If the ribbon is mounted the other way round, set GRAY_REVERSE to 1. */
#define GRAY_BLACK_IS_HIGH      1
#define GRAY_REVERSE            0

/* ---------------- battery ---------------- */
/* WHEELTEC E326S 3S Li-ion pack, 9.0-12.6 V (12.6 V fully charged).
 * The D153B driver board carries the divider on board (10k / 1k -> 1/11)
 * and exposes it on the Output_IO "ADC" pin, so PB1 sees Vbat / 11.
 * If an external 100k / 22k divider (0.180) is used instead, set
 * VBAT_ADC_MULT to 1000/180 = 5 (or restore the old /180 formula).
 * ADC 3.3 V reference, 12 bit. */
/* Bench check 2026-11-07: pack measured 12.5 V with a multimeter while the
 * OLED showed 13.7 V at MULT = 11, i.e. the whole chain has ~+10 % gain.
 * Either the D153B divider is closer to 1/10 than 1/11, or VDDA is ~3.0 V
 * instead of the assumed 3.3 V. MULT = 10 compensates both (constant gain);
 * re-check with a multimeter before trusting the low-battery thresholds. */
#define VBAT_DIVIDER_RATIO      (1.0f / 11.0f)
#define VBAT_ADC_MULT           10UL
#define VBAT_LOW_MV             10200   /* 3.40 V / cell */
#define VBAT_CRITICAL_MV        9600    /* 3.20 V / cell */

/* ---------------- UI ---------------- */
#define SPEED_SET_MIN           300
#define SPEED_SET_MAX           900
#define SPEED_SET_STEP          150
#define UI_REFRESH_MS           200

#endif
