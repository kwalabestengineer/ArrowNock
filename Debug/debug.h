/*
 * debug.h - delays and printf output.
 *
 * printf goes out over SDI, the single-wire debug interface on PD1. That is
 * the only output path in this project.
 *
 * SDI output is discarded when no debugger is listening. It never blocks.
 * See the note in debug.c.
 */
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <ch32v00X.h>
#include <stdio.h>

/* Compile-time switch. SDI_PR_CLOSE builds every printf away to nothing,
 * which is what a production image should use. The project currently passes
 * -DSDI_PRINT=SDI_PR_OPEN on the command line. */
#define SDI_PR_CLOSE   0
#define SDI_PR_OPEN    1

#ifndef SDI_PRINT
#define SDI_PRINT   SDI_PR_OPEN
#endif

/* Microsecond/millisecond delays, SysTick based.
 * Delay_Init() must run after SystemCoreClockUpdate(). */
void Delay_Init(void);
void Delay_Us(uint32_t n);
void Delay_Ms(uint32_t n);

/* Clears the debug mailbox and arms SDI output. Safe to call with no
 * debugger attached. */
void SDI_Printf_Enable(void);

/* Non-zero while output is still being accepted by a host. Goes to zero the
 * first time a write times out, and stays there until SDI_Printf_Enable()
 * is called again. Useful for skipping work whose only purpose is printing. */
uint8_t SDI_Host_Present(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */
