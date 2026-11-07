#include "stm32f10x.h"
#include "Delay.h"

/* DWT cycle counter delay.
 * The CMSIS version shipped with this StdPeriph project does not define the
 * DWT struct, so the three registers are accessed by address. This works with
 * both ARMCC and GCC and, unlike a SysTick based delay, is safe inside ISRs. */
#define DEMCR_ADDR      0xE000EDFCUL
#define DWT_CTRL_ADDR   0xE0001000UL
#define DWT_CYCCNT_ADDR 0xE0001004UL
#define TRCENA_BIT      (1UL << 24)
#define CYCCNTENA_BIT   (1UL << 0)

void Delay_Init(void)
{
    *(volatile uint32_t *)DEMCR_ADDR |= TRCENA_BIT;
    *(volatile uint32_t *)DWT_CYCCNT_ADDR = 0;
    *(volatile uint32_t *)DWT_CTRL_ADDR |= CYCCNTENA_BIT;
}

void Delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (us == 0)
    {
        return;
    }

    start = *(volatile uint32_t *)DWT_CYCCNT_ADDR;
    ticks = us * (SystemCoreClock / 1000000U);

    while ((*(volatile uint32_t *)DWT_CYCCNT_ADDR - start) < ticks)
    {
        /* wait */
    }
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}

void Delay_s(uint32_t s)
{
    while (s--)
    {
        Delay_ms(1000);
    }
}
