#include "stm32f10x.h"
#include "gray_uart.h"
#include "app_config.h"
#include "bsp.h"

/* =========================================================================
 * CH32V006 8-channel grayscale module, UART "simple protocol" driver.
 *
 * Wire protocol (see README / the module manual):
 *   request    A5 CMD ARGS...
 *   response   5A CMD STATUS PAYLOAD...
 *   push 0x02  5B + 8 x 16 bit little endian raw ADC values (17 bytes)
 *
 * The module pushes one frame every 10 ms once register 0x15 (uart push
 * mode) is set to 0x02.  That register lives in RAM and comes back as 0x00
 * after every module reset, so the watchdog in Gray_Poll() re-sends it when
 * no frame has been seen for GRAY_LINK_TIMEOUT_MS.
 * ========================================================================= */

/* ---------------- protocol constants ---------------- */
#define GRAY_REQ_HEAD           0xA5u
#define GRAY_PUSH_HEAD          0x5Bu

#define GRAY_CMD_READ_REG       0x02u
#define GRAY_CMD_WRITE_REG      0x03u
#define GRAY_REG_PUSH_MODE      0x15u
#define GRAY_PUSH_MODE_GRAY8    0x02u

#define GRAY_FRAME_LEN          17u     /* 1 header + 8 x 2 bytes */

/* ---------------- RX ring buffer ---------------- */
/* 256 bytes = 15 frames, more than one full 5 ms control period can hold,
 * so a late Gray_Task() call can never lose data in normal operation. */
#define GRAY_RING_SIZE          256u
#define GRAY_RING_MASK          (GRAY_RING_SIZE - 1u)

static volatile uint8_t  s_rx[GRAY_RING_SIZE];
static volatile uint8_t  s_rx_head = 0u;
static volatile uint8_t  s_rx_tail = 0u;

/* ---------------- parser state ---------------- */
static uint8_t  s_frame[GRAY_FRAME_LEN];
static uint8_t  s_frame_len = 0u;

/* ---------------- decoded state ---------------- */
static uint16_t s_raw[GRAY_CHANNELS];
static uint8_t  s_bits = 0u;
static uint8_t  s_online = 0u;
static volatile uint16_t s_frames = 0u;
static volatile uint8_t  s_errors = 0u;
static volatile uint32_t s_last_frame_tick = 0u;

static uint32_t s_last_cmd_tick = 0u;
static uint8_t  s_cmd_pending = 1u;
static uint8_t  s_cmd_announced = 0u;
static uint8_t  s_link_announced = 0u;
static uint32_t s_announce_tick = 0u;

/* ------------------------------------------------------------------ */
/* helpers                                                            */
/* ------------------------------------------------------------------ */
static uint8_t gray_build_bits(void)
{
    uint8_t i;
    uint8_t bits = 0u;

    for (i = 0u; i < GRAY_CHANNELS; i++)
    {
        uint8_t black;

#if GRAY_BLACK_IS_HIGH
        black = (s_raw[i] > (uint16_t)GRAY_BLACK_ADC) ? 1u : 0u;
#else
        black = (s_raw[i] <= (uint16_t)GRAY_BLACK_ADC) ? 1u : 0u;
#endif

        if (black)
        {
            bits |= (uint8_t)(1u << i);
        }
    }

#if GRAY_REVERSE
    {
        uint8_t r = 0u;
        for (i = 0u; i < GRAY_CHANNELS; i++)
        {
            if (bits & (uint8_t)(1u << i))
            {
                r |= (uint8_t)(1u << (GRAY_CHANNELS - 1u - i));
            }
        }
        bits = r;
    }
#endif

    return bits;
}

static void gray_accept_frame(void)
{
    uint8_t i;

    /* 12 bit values: the high byte can only be 0x00..0x0F.  A byte that
     * breaks this means we locked onto a 0x5B inside a payload (or a lost
     * byte happened), so drop the frame and let the next header resync. */
    for (i = 0u; i < GRAY_CHANNELS; i++)
    {
        if ((s_frame[2u + 2u * i] & 0xF0u) != 0u)
        {
            if (s_errors < 0xFFu) { s_errors++; }
            return;
        }
    }

    for (i = 0u; i < GRAY_CHANNELS; i++)
    {
        s_raw[i] = (uint16_t)s_frame[1u + 2u * i] |
                   ((uint16_t)s_frame[2u + 2u * i] << 8);
    }

    s_bits = gray_build_bits();
    s_online = 1u;
    s_last_frame_tick = BSP_GetTick5ms();

    if (s_frames < 0xFFFFu)
    {
        s_frames++;
    }
}

static void gray_parse_byte(uint8_t b)
{
    if (s_frame_len == 0u)
    {
        if (b == GRAY_PUSH_HEAD)
        {
            s_frame[0] = b;
            s_frame_len = 1u;
        }
        return;
    }

    s_frame[s_frame_len] = b;
    s_frame_len++;

    if (s_frame_len >= GRAY_FRAME_LEN)
    {
        s_frame_len = 0u;
        gray_accept_frame();
    }
}

static void gray_send_push_cmd(void)
{
    /* A5 03 15 02 -> write register 0x15 = 0x02 (push 8 raw channels). */
    BSP_UartSendChar((char)GRAY_REQ_HEAD);
    BSP_UartSendChar((char)GRAY_CMD_WRITE_REG);
    BSP_UartSendChar((char)GRAY_REG_PUSH_MODE);
    BSP_UartSendChar((char)GRAY_PUSH_MODE_GRAY8);
}

/* ------------------------------------------------------------------ */
/* public API                                                         */
/* ------------------------------------------------------------------ */
void Gray_Init(void)
{
    NVIC_InitTypeDef nvic;

    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_frame_len = 0u;
    s_bits = 0u;
    s_online = 0u;
    s_frames = 0u;
    s_errors = 0u;
    s_cmd_pending = 1u;
    s_cmd_announced = 0u;
    s_link_announced = 0u;
    s_announce_tick = 0u;

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Above the 200 Hz control interrupt (TIM1 = 1): the control ISR spends
     * ~1.5 ms in the bit-banged MPU6050 read, and a whole 17 byte grayscale
     * frame would be lost to RX overrun if this handler had to wait for it.
     * This handler only pushes one byte into the ring buffer. */
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* Force the first push-mode command out on the first Gray_Poll(). */
    s_last_cmd_tick = BSP_GetTick5ms() - (uint32_t)(GRAY_LINK_RETRY_MS / 5);
}

void Gray_Task(void)
{
    uint32_t age;

    while (s_rx_tail != s_rx_head)
    {
        uint8_t b = s_rx[s_rx_tail];
        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & GRAY_RING_MASK);
        gray_parse_byte(b);
    }

    age = (uint32_t)BSP_GetTick5ms() - s_last_frame_tick;
    if (age > (uint32_t)(GRAY_LINK_TIMEOUT_MS / 5))
    {
        s_online = 0u;
    }
}

void Gray_Poll(void)
{
    uint32_t now = BSP_GetTick5ms();
    uint32_t since_cmd = now - s_last_cmd_tick;
    uint32_t since_frame = now - s_last_frame_tick;

    /* One line per (re)connect, so the install check has a clear marker in
     * the log instead of a silent gl=0. */
    if (s_online != 0u)
    {
        if ((s_link_announced == 0u) && ((now - s_announce_tick) > 200u))
        {
            s_link_announced = 1u;
            s_announce_tick = now;
            BSP_UartSendString("\r\nGRAY link ok, g=");
            BSP_UartSendInt((int32_t)s_bits);
            BSP_UartSendString("\r\n");
        }
    }
    else
    {
        s_link_announced = 0u;
    }

    if (since_cmd < (uint32_t)(GRAY_LINK_RETRY_MS / 5))
    {
        return;
    }

    if ((s_cmd_pending == 0u) && (since_frame <= (uint32_t)(GRAY_LINK_TIMEOUT_MS / 5)))
    {
        return;
    }

    if (s_cmd_announced == 0u)
    {
        /* The 4 binary bytes below also show up on the CH340 as garbage;
         * this line explains them. */
        s_cmd_announced = 1u;
        BSP_UartSendString("\r\nGRAY push-mode cmd A5 03 15 02\r\n");
    }

    s_cmd_pending = 0u;
    s_last_cmd_tick = now;
    gray_send_push_cmd();
}

uint8_t Gray_IsOnline(void)
{
    return s_online;
}

uint8_t Gray_GetBits(void)
{
    return s_bits;
}

uint16_t Gray_GetRaw(uint8_t index)
{
    if (index >= GRAY_CHANNELS)
    {
        return 0u;
    }
    return s_raw[index];
}

uint16_t Gray_GetFrames(void)
{
    return s_frames;
}

uint8_t Gray_GetErrors(void)
{
    return s_errors;
}

/* ------------------------------------------------------------------ */
/* USART1 interrupt: only move the byte into the ring buffer          */
/* ------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        uint8_t next = (uint8_t)((s_rx_head + 1u) & GRAY_RING_MASK);

        if (next != s_rx_tail)
        {
            s_rx[s_rx_head] = b;
            s_rx_head = next;
        }
    }
}
