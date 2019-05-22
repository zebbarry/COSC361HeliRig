// *******************************************************
//
// heliTimer.c
//
// Purpose: More accurate timer for delays and loop timing using a
// 32-bit down counter and timer 5.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   22.5.2019
//
// *******************************************************


#include "heliTimer.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"


static uint32_t clockRate;
static uint32_t ticksPerMs;


// enable the hardware timer and calculate clock parameters
void initTimer(void)
{
    clockRate = SysCtlClockGet();
    ticksPerMs = clockRate / 1000;  // 1000 ms = 1 s

    SysCtlPeripheralReset(TIMER_PERIPH);  // reset for good measure

    // timer counts down by default
    // config to reset to max value
    SysCtlPeripheralEnable(TIMER_PERIPH);
    TimerDisable(TIMER_BASE, TIMER_INTERAL);
    TimerConfigure(TIMER_BASE, TIMER_MODE);
    TimerLoadSet(TIMER_BASE, TIMER_INTERAL, TIMER_MAX_TICKS);
    TimerEnable(TIMER_BASE, TIMER_INTERAL);
}


// return the current timer value in ticks.
uint32_t timerGet(void)
{
    return TimerValueGet(TIMER_BASE, TIMER_INTERAL);
}


// waits for some given milliseconds.
void timerWaitUntil(uint32_t milliseconds)
{
    timerWaitFrom(milliseconds, timerGet());
}


// returns true after milliseconds have passed from reference.
bool timerBeen(uint32_t milliseconds, uint32_t reference)
{
    // minus since counts down
    uint32_t target = reference - milliseconds * ticksPerMs;
    uint32_t current = timerGet(); //get time
    uint32_t diff = target - current;  // +ve small number when past target (timer counts down)

    // false until this condition is met
    return diff < TIMER_OVERSHOOT_TICKS;
}


// waits until a given milliseconds passed some reference timer value.
// useful for keeping time in a loop with many operations.
void timerWaitFrom(uint32_t milliseconds, uint32_t reference)
{
    while (true) {

        // block until we pass the target time
        if (timerBeen(milliseconds, reference)) return;
    }
}

