#ifndef HELIMODULES_HELITIMER_H_
#define HELIMODULES_HELITIMER_H_

// *******************************************************
//
// heliTimer.h
//
// Helicopter project
// Group: A03 Group 10
// Last edited: 30-05-2018 by Thomas M
//
// Purpose: More accurate timer for delays and loop timing using a
// 32-bit down counter and timer 5.
// ************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "limits.h"

#include "inc/hw_memmap.h"  // for TIMER0_BASE etc
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#define TIMER_PERIPH            SYSCTL_PERIPH_WTIMER5
#define TIMER_BASE              WTIMER5_BASE
#define TIMER_INTERAL           TIMER_A
#define TIMER_MODE              TIMER_CFG_A_PERIODIC
#define TIMER_MAX_TICKS         UINT_MAX  // 32 bits for Timer0 A
#define TIMER_OVERSHOOT_TICKS   (TIMER_MAX_TICKS / 2)

// enable the hardware timer and calculate clock parameters
void initTimer(void);


// return the current timer value in tick s.
uint32_t timerGet(void);


// waits for some given milliseconds.
void timerWaitUntil(uint32_t milliseconds);


// returns true after milliseconds have passed from reference
bool timerBeen(uint32_t milliseconds, uint32_t reference);


// waits until a given milliseconds passed some reference timer value.
// useful for keeping time in a loop with many operations.
void timerWaitFrom(uint32_t milliseconds, uint32_t reference);

#endif /* HELIMODULES_HELITIMER_H_ */
