#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "stm32f10x.h"

/* =========================================================================
 * 16th Team - STM32F103C8T6 line-following car
 * Hardware: Blue Pill + TB6612FNG + 2x MG513 encoder motors
 *           8-channel CH32V006 grayscale (UART link) + MPU6050 + SSD1306 OLED
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
 *  PA8  (free)         was GRAY_OUT4, parallel ribbon no longer used
 *  PA9  USART1_TX      debug TX -> CH340 RX  +  grayscale module RX
 *  PA10 USART1_RX      grayscale module TX (do NOT connect CH340 TX here)
 *  PA11 (free)         was GRAY_OUT5
 *  PA12 (free)         was GRAY_OUT6
 *  PA13 SWDIO          keep for ST-Link
 *  PA14 SWCLK          keep for ST-Link
 *  PA15 (free)         was GRAY_OUT7
 *  PB0  BUZZER         active buzzer through S8050
 *  PB1  ADC_IN9        battery voltage divider
 *  PB2  LED            external status LED
 *  PB3  (free)         was GRAY_OUT1 (JTAG disabled)
 *  PB4  (free)         was GRAY_OUT2 (JTAG disabled)
 *  PB5  (free)         was GRAY_OUT3
 *  PB6  TIM4_CH1       right encoder A
 *  PB7  TIM4_CH2       right encoder B
 *  PB8  OLED_SCL       software I2C
 *  PB9  OLED_SDA       software I2C
 *  PB10 MPU_SCL        software I2C
 *  PB11 MPU_SDA        software I2C
 *  PB12 BIN1           right motor direction
 *  PB13 BIN2           right motor direction
 *  PB14 (free)         was GRAY_OUT8
 *  PB15 KEY_MODE       input pull-up, active low
 *  PC13 LED_ONBOARD    active low
 *
 * Grayscale link (fw >= Q2-GRAY-UART):
 *   The CH32V006 module is read over its UART (115200 8N1, binary "simple
 *   protocol"), not over the 8 parallel OUT lines any more.  USART1 is
 *   shared: PA10 takes the module TX, PA9 feeds both the CH340 debug RX and
 *   the module RX (the module only needs the one push-mode command at boot).
 *   The 8 channel ribbon OUT-1..OUT-8 must be unplugged from the car.
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

/* ---------------- arc (schedule mode, Q3/Q4 and the link-down fallback) ---
 *
 * Still used by SEG_ARC.  Q2 uses the grayscale line follower further down.
 *
 *   turn_ff   = arc_dir * base * CAR_TRACK_WIDTH / (2 * TRACK_RADIUS) * ARC_FF_TRIM
 *   gray_corr = -(ARC_GRAY_TRIM_KP * line_err + ARC_GRAY_TRIM_KD * d(line_err))
 *   yaw_ref   = arc_start + arc_dir * 180 * (dist / arc_len) + gray_corr
 *   turn_corr = ARC_YAW_KP * (yaw_ref - yaw)
 *             - ARC_YAW_KD * (gyro_z - rate_ref)    rate_ref = 180*base/arc_len
 *   L = base - (turn_ff + turn_corr),  R = base + (turn_ff + turn_corr)
 *
 * The grayscale trim only shifts the heading reference by at most
 * ARC_GRAY_CORR_MAX degrees. The MPU6050 closes the fast heading loop, so
 * the two loops cannot fight each other.
 *
 * The damping term is a rate feedback on (gyro_z - rate_ref), not on the
 * raw gyro: in a steady turn the raw gyro reads ~65 deg/s and damping that
 * directly would need a permanent heading offset to compensate. */

/* Fixed feedforward. Tune it from the serial log while running an arc:
 * R_act = d(distance) / d(yaw in rad); ARC_FF_TRIM = R_act / TRACK_RADIUS_MM.
 * > 1 = the car under-steers (actual radius too large), < 1 = it cuts inside. */
#define ARC_FF_TRIM             1.00f

/* Grayscale trim used by the ORIGINAL schedule arc (SEG_ARC, Q3/Q4).  Its
 * output is a heading correction in DEGREES, not wheel speed.
 * line_err is normalized to -1..+1 by line_error() in path.c.
 * With a single sensor on the line the error only takes steps of ~0.14, so
 * the gain has to be large enough that one sensor step is visible. */
#define ARC_GRAY_TRIM_KP        20.0f   /* deg per unit normalized error */
#define ARC_GRAY_TRIM_KD        4.0f    /* deg per unit error change per 5 ms */
#define ARC_GRAY_CORR_MAX       40.0f   /* deg, clamp of gray_corr */
#define ARC_REACQ_DEG           25.0f   /* deg, steer-back while the line is lost */
#define ARC_GRAY_LOST_DECAY     0.97f   /* per 5 ms sample while the line is lost */

/* ---------------- grayscale line following (Q2) --------------------------
 *
 * Q2 no longer flies a dead-reckoned path and trims it with the grayscale:
 * both its straights (SEG_LINE) and its arcs (SEG_ARC_LINE) are driven by the
 * eight-channel array itself.  Same controller for both, only the geometric
 * feedforward differs (zero on a straight, the 400 mm curvature on an arc).
 *
 *   w_line     = -(KP * e + KD * de/dt)           deg/s, clamped +-W_MAX
 *   theta_line += w_line * dt, clamped +-THETA_MAX  deg
 *   yaw_ref     = yaw_tangent + theta_line
 *   rate_ref    = rate_ff + w_line
 *   turn_corr   = ARC_YAW_KP * (yaw_ref - yaw)
 *               - ARC_YAW_KD * (gyro_z - rate_ref)
 *   turn        = turn_ff + turn_corr
 *   wheels      = base -/+ turn
 *
 * e is path.c's normalized line error, +1 = line at the OUT8 end of the
 * array.  yaw_tangent is the direction the segment should have right now
 * (constant on a straight, +-180 deg over TRACK_ARC_MM on an arc);
 * theta_line is an integral of the line error, so a constant curvature or a
 * slightly wrong arc radius is absorbed without a standing lateral offset.
 *
 * Tuning (serial e= and lt= tell you which way to go):
 *   car runs wide / cuts inside  -> raise ARC_LINE_KP
 *   car weaves or shakes         -> lower ARC_LINE_KP, then lower KD
 *   correction saturates at once -> lower ARC_LINE_W_MAX
 *   cannot recover at arc entry  -> raise ARC_LINE_THETA_MAX
 */
#define ARC_LINE_KP             60.0f   /* deg/s per unit line error */
#define ARC_LINE_KD             20.0f   /* deg/s per unit error per second */
#define ARC_LINE_D_FILT         0.20f   /* derivative low-pass, per 5 ms sample */
#define ARC_LINE_W_MAX          110.0f  /* deg/s, clamp of the line rate */
#define ARC_LINE_THETA_MAX      55.0f   /* deg, clamp of the line heading offset */
#define ARC_LINE_REACQ_DEG      40.0f   /* deg, offset while the line is lost */

/* Heading loop (MPU6050) inside the arc. */
#define ARC_YAW_KP              10.0f   /* mm/s per degree */
#define ARC_YAW_KD              0.8f    /* mm/s per deg/s, rate damping */
#define ARC_CORR_LIMIT          200.0f  /* mm/s, clamp of the closed-loop part */

/* Degraded mode when the MPU6050 is missing: no heading reference exists,
 * so fall back to feedforward + a plain grayscale differential. */
#define ARC_LINE_FALLBACK_KP    120.0f  /* mm/s per unit normalized error */

/* pivot turn */
#define TURN_KP                 5.0f    /* mm/s per degree */
#define TURN_MIN_MMPS           130.0f
#define TURN_MAX_MMPS           520.0f
#define TURN_TOL_DEG            1.8f
#define TURN_TIMEOUT_MS         2500

/* arc finish */
#define ARC_LINE_LOST_SAMPLES   3
#define ARC_FINISH_MARGIN_MM    5.0f    /* was 60: 60 mm = 8.6 deg of extra arc */
#define ARC_YAW_DONE_DEG        179.0f  /* gross over-rotation guard; the line
                                         * arc's handover gate below handles
                                         * the under-rotation side */
#define ARC_YAW_SNAP_DEG        3.0f    /* only snap yaw when this close */
#define ARC_YAW_FIX_MAX         20.0f   /* hand the nominal heading to the next
                                         * straight only when this close, so a
                                         * dead IMU can never spin the car */
#define SEGMENT_TIMEOUT_MS      15000

/* Line-arc -> straight handover (Q2).
 *
 * The grayscale arc is ended by its encoder length, and the line follower
 * often gets there with the heading still a few degrees short of 180 deg.
 * Before handing over to the following straight the car now STANDS STILL and
 * pivots to (arc_start + arc_dir*180), so the straight always starts pointing
 * exactly the right way (the straight itself is then MPU heading hold).
 *
 * A residual within ARC_SETTLE_TOL is considered good enough and is snapped
 * by arc_finish() instead of pivoting.  The pivot itself reuses the normal
 * turn gains below (TURN_KP / TURN_TOL_DEG / TURN_TIMEOUT_MS). */
#define ARC_SETTLE_TOL          1.0f    /* deg of residual that triggers the pivot */
#define ARC_SETTLE_DONE_TOL     1.0f    /* deg the pivot has to reach */
#define ARC_SETTLE_BRAKE_MS     150     /* stand still this long before pivoting */

/* What the 180 deg is measured from:
 *   1 = the heading the RUN started with (what the team asked for: "yaw must
 *       be 180 deg away from the initial yaw before the straight starts").
 *       Note this also carries a crooked start placement into the straight.
 *   0 = the heading THIS ARC started with (i.e. parallel to the tape at the
 *       arc entry, which cancels a crooked placement). */
#define ARC_SETTLE_ABS_YAW      1

/* ---------------- grayscale link (CH32V006 module over UART) -------------
 *
 * The module (CTL_M_TRACK_CH32V006) is read over its UART, 115200 8N1,
 * binary "simple protocol" - NOT over the 8 parallel OUT lines any more:
 *
 *   request    A5 CMD ARGS...
 *   response   5A CMD STATUS PAYLOAD...
 *   CMD 0x02   READ_REG     arg: reg          -> reg, value
 *   CMD 0x03   WRITE_REG    args: reg,value   -> reg, value
 *   CMD 0x05   READ_GRAY8   no args           -> counter + 8 x u16 little endian
 *   reg  0x04  gray compare bits (bit0 = channel 1, 1 = ADC > threshold)
 *   reg  0x13/0x14  LED compare threshold, default 2000 (0x07D0)
 *   reg  0x15  uart_push_mode: 0 = off, 1 = push bitmap, 2 = push 8 raw ADC
 *
 * At boot the firmware writes 0x15 = 2 ("A5 03 15 02"); the module then
 * pushes a self-framed frame every 10 ms:
 *
 *   5B g1L g1H g2L g2H ... g8L g8H            (17 bytes, 12 bit values)
 *
 * The frame header makes the link self-synchronising: a lost byte costs one
 * frame, not the rest of the run.  GRAY_BLACK_ADC turns the raw values into
 * the same 8 bit "1 = black" map the old parallel code used.
 */
#define GRAY_UART_BAUD          115200
#define GRAY_BLACK_ADC          2000    /* raw ADC above this = black line */
#define GRAY_LINK_TIMEOUT_MS    300     /* no frame for this long = link down */
#define GRAY_LINK_RETRY_MS      200     /* re-send the push-mode command this often */

/* The module compares the ADC itself at 2000; the raw values are 12 bit.
 * If the ribbon/mounting is mirrored, set GRAY_REVERSE to 1. */
#define GRAY_BLACK_IS_HIGH      1
#define GRAY_REVERSE            0

/* Channels to ignore if one of them always reads black (default: none).
 * With the serial link there is no OUT pin left to be stuck, but a dirty or
 * dead sensor still can be.  Check the a= field on the serial log on a pure
 * white floor: every channel should read a few hundred counts.  A channel
 * that stays near 4095 is faulty -> mask it here, e.g. 0x20u for channel 6. */
#define GRAY_MASK               0x00u

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
/* Bench check 2026-11-08: pack measured 12.4 V with a multimeter while the
 * OLED showed ~15.55 V at MULT = 10 (idle, motors off). Corrected gain is
 * 10 * 12.4 / 15.55 = 7.97, so use 8. The effective divider is ~1/8, not
 * the 1/11 assumed for the D153B "ADC" pin. */
#define VBAT_DIVIDER_RATIO      (1.0f / 11.0f)
#define VBAT_ADC_MULT           8UL
#define VBAT_LOW_MV             10200   /* 3.40 V / cell */
#define VBAT_CRITICAL_MV        9600    /* 3.20 V / cell */

/* ---------------- safety / diagnostics ---------------- */
/* Independent watchdog.  It is started after the power-on calibration and
 * fed by the main loop: if the program ever stops (stuck wait, hard fault,
 * brown-out) the car resets itself and the next boot prints
 * "RST flags=.. IWDG" plus a warning line.  Set to 0 only while hunting a
 * bug with a debugger. */
#define IWDG_TIMEOUT_MS         2000

/* ---------------- UI ---------------- */
/* Printed on the boot line so the flashed build can be identified from the
 * serial console without guessing. */
#define FW_TAG                  "Q2-GRAY-UART r8"
#define SPEED_SET_MIN           300
#define SPEED_SET_MAX           900
#define SPEED_SET_STEP          150
#define UI_REFRESH_MS           200

#endif
