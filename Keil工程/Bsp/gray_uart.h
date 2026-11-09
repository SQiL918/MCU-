#ifndef __GRAY_UART_H
#define __GRAY_UART_H

#include "stm32f10x.h"

/* =========================================================================
 * 8-channel CH32V006 grayscale module, UART link.
 *
 * USART1 is shared with the debug console:
 *   PA10 (USART1_RX) <- module TX
 *   PA9  (USART1_TX) -> CH340 RX (debug) and module RX (push-mode command)
 *
 * The module only receives 4 bytes at boot ("A5 03 15 02"); after that it
 * streams one 17 byte frame every 10 ms, so the RX side carries all the
 * traffic.  Nothing in the ASCII debug stream can be mistaken for the 0xA5
 * request header (ASCII is below 0x80).
 * ========================================================================= */

#define GRAY_CHANNELS           8u

void     Gray_Init(void);       /* enable the RX interrupt, arm the push cmd */
void     Gray_Task(void);       /* drain + parse RX, call from the 5 ms loop */
void     Gray_Poll(void);       /* link watchdog / re-send, call from main() */

uint8_t  Gray_IsOnline(void);   /* 1 = a valid frame arrived recently */
uint8_t  Gray_GetBits(void);    /* bit0 = channel 1 ... bit7 = channel 8, 1 = black */
uint16_t Gray_GetRaw(uint8_t index);
uint16_t Gray_GetFrames(void);  /* valid frames since reset, wraps */
uint8_t  Gray_GetErrors(void);  /* frames rejected by the header/range check */

#endif
