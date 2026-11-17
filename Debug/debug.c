/*
 * debug.c - delays and SDI printf.
 *
 * WHY THIS FILE EXISTS IN THIS FORM
 *
 * SDI printf hands bytes to the debug module's mailbox at 0xE00000F4 and then
 * waits for the host to acknowledge by writing zero back. The stock WCH
 * implementation waits forever. On a board running from its own battery there
 * is no host, so the acknowledgement never comes and the program stops dead -
 * typically seven characters into the first printf, before any application
 * code has run. The board looks unprogrammed when in fact it is executing
 * faithfully, parked in a spin loop.
 *
 * Here the wait is bounded. When it expires the host is taken to be absent:
 * the output is dropped, a latch is set, and every later call returns
 * immediately. The cost is one timeout, once, after reset. With a debugger
 * attached nothing changes.
 */
#include <debug.h>

static uint8_t  p_us = 0;
static uint16_t p_ms = 0;

/* Debug module mailbox. DATA0's low byte is the length (max 7); the rest of
 * DATA0 plus all of DATA1 carry the payload. */
#define DEBUG_DATA0_ADDRESS  ((volatile uint32_t*)0xE00000F4)
#define DEBUG_DATA1_ADDRESS  ((volatile uint32_t*)0xE00000F8)

/* Loop iterations, not microseconds. ~20 ms at 48 MHz. Paid once. */
#define SDI_HOST_POLL_LIMIT  100000u

static volatile uint8_t sdi_host_absent = 0;

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Derives the SysTick counts per microsecond and per millisecond
 *          from SystemCoreClock. Call after SystemCoreClockUpdate().
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = SystemCoreClock / 8000000;
    p_ms = (uint16_t)p_us * 1000;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond delay.
 *
 * @param   n - microseconds.
 *
 * @return  none
 */
void Delay_Us(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_us;

    SysTick->CMP = i;
    SysTick->CNT = 0;
    SysTick->CTLR |= (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0));
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond delay.
 *
 * @param   n - milliseconds.
 *
 * @return  none
 */
void Delay_Ms(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_ms;

    SysTick->CMP = i;
    SysTick->CNT = 0;
    SysTick->CTLR |= (1 << 0);

    while((SysTick->SR & (1 << 0)) != (1 << 0));
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      SDI_Printf_Enable
 *
 * @brief   Clears the mailbox and arms SDI output. Safe with no debugger
 *          attached - it does not wait for anything.
 *
 * @return  none
 */
void SDI_Printf_Enable(void)
{
    *(DEBUG_DATA0_ADDRESS) = 0;
    sdi_host_absent = 0;            /* assume a host until proven otherwise */
    Delay_Init();
    Delay_Ms(1);
}

/*********************************************************************
 * @fn      SDI_Host_Present
 *
 * @brief   Non-zero while output is still being accepted.
 *
 * @return  1 if a host appears to be reading, 0 once a write has timed out
 */
uint8_t SDI_Host_Present(void)
{
#if (SDI_PRINT == SDI_PR_OPEN)
    return (uint8_t)((sdi_host_absent == 0u) ? 1u : 0u);
#else
    return 0u;
#endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Backs printf. Bounded wait on the host, then discard.
 *
 * @param   fd   - ignored.
 *          buf  - bytes to send.
 *          size - how many.
 *
 * @return  size, always - the caller is never told to retry
 */
__attribute__((used))
int _write(int fd, char *buf, int size)
{
#if (SDI_PRINT == SDI_PR_OPEN)

    uint32_t guard;
    uint32_t d0, d1;
    int      i = 0;
    int      left = size;
    int      k, n;

    (void)fd;

    if(sdi_host_absent != 0u)
    {
        return size;                    /* no host: drop it, do not stall */
    }

    while(left > 0)
    {
        /* Wait for the host to take the previous chunk - but not forever. */
        guard = 0;
        while(*(DEBUG_DATA0_ADDRESS) != 0u)
        {
            if(++guard >= SDI_HOST_POLL_LIMIT)
            {
                sdi_host_absent = 1u;
                *(DEBUG_DATA0_ADDRESS) = 0;     /* leave the mailbox clean */
                return size;
            }
        }

        n  = (left > 7) ? 7 : left;
        d0 = (uint32_t)n;               /* low byte of DATA0 is the length */
        d1 = 0;

        /* Only the bytes that exist are read, and each is taken as unsigned
         * so a byte above 0x7F cannot sign-extend over its neighbours. */
        for(k = 0; (k < n) && (k < 3); k++)
        {
            d0 |= ((uint32_t)(uint8_t)buf[i + k]) << (8 * (k + 1));
        }
        for(k = 3; k < n; k++)
        {
            d1 |= ((uint32_t)(uint8_t)buf[i + k]) << (8 * (k - 3));
        }

        *(DEBUG_DATA1_ADDRESS) = d1;
        *(DEBUG_DATA0_ADDRESS) = d0;    /* writing DATA0 hands it to the host */

        i    += n;
        left -= n;
    }

#else

    (void)fd;
    (void)buf;

#endif

    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Heap growth, required by newlib. Nothing in this project
 *          allocates, but printf's internals reference it.
 *
 * @return  previous break, or (void*)-1 on failure
 */
__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}
