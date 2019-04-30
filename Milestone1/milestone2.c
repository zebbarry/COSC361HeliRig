//*****************************************************************************
//
// milestone2.c - Interrupt driven program which samples with AIN9 and
//              transmits the ADC value through UART0 at 4 Hz, while detecting
//              pin changes on PB0 and PB1 for quadrature decoding.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   29.4.2019
//
//*****************************************************************************
// Based on the 'convert' series from 2016
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "circBufT.h"
#include "buttons4.h"
#include "USBUART.h"
#include "display.h"
#include "yaw.h"
#include "heliADC.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 100
#define SAMPLE_RATE_HZ 1000
#define SLOWTICK_RATE_HZ 4
#define MAX_STR_LEN 16


//*****************************************************************************
// Global variables
//*****************************************************************************
circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
volatile uint8_t slowTick = false;
char statusStr[MAX_STR_LEN + 1];
static uint16_t inADC_max;
static uint8_t displayState = SCALED;


//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3); 
    g_ulSampCnt++;

    //
    static uint8_t tickCount = 0;
    const uint8_t ticksPerSlow = SAMPLE_RATE_HZ / SLOWTICK_RATE_HZ;

    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }

    updateButtons();
}


//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    // Allow time for the oscillator to settle down.
    SysCtlDelay(100);

    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


//********************************************************
// initAltitude - Calibrate height
//********************************************************
void
initAltitude (meanVal)
{
    inADC_max = meanVal - ALT_RANGE;
}

//********************************************************
// calcMean - Calculate mean ADC from buffer.
//********************************************************
uint16_t calcMean(void)
{
    uint16_t i;
    int32_t sum = 0;
    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf (&g_inBuffer);
    // Calculate and display the rounded mean of the buffer contents
    return (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}


int
main(void)
{
    uint16_t meanVal = 0;
    bool init_prog = true;

    initButtons ();
    initClock ();
    initADC ();
    initYaw ();
    initDisplay ();
    initUSB_UART ();
    initCircBuf (&g_inBuffer, BUF_SIZE);

    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    while (1)
    {
        //
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number, as long
        // as the entire buffer has been written into.
        if (g_inBuffer.written)
        {
            meanVal = calcMean();

            // If start of program, calibrate ADC input
            if (init_prog)
            {
                init_prog = false;
                initAltitude(readCircBuf (&g_inBuffer));
            }
        }


        // Check buttons - LEFT = Reset zero position
        //               - UP   = Switch altitude unit
        if (checkButton(LEFT) == PUSHED)
        {
            initAltitude (meanVal);

        }
        if (checkButton(UP) == PUSHED || checkButton(UP) == RELEASED)
        {
            switch (displayState)
            {
            case SCALED: displayState = MEAN;
                break;
            case MEAN: displayState = CLEAR;
                break;
            case CLEAR: displayState = SCALED;
                break;
            }
        }


        // Time to send a message through UART at set lower frequency SLOW_TICK_RATE_HZ
        if (slowTick && !init_prog)
        {
            slowTick = false;

            // Form and send a status message for altitude to the console
            usnprintf (statusStr, sizeof(statusStr), "ADC = %4d \r\n", meanVal); // * usprintf
            UARTSend (statusStr);

            // Form and send a status message for yaw to the console
            int16_t mappedYaw = mapYaw2Deg();
            usnprintf (statusStr, sizeof(statusStr), "YAW = %4d \r\n", mappedYaw); // * usprintf
            UARTSend (statusStr);

            // Update OLED display with ADC and yaw value.
            displayMeanVal (meanVal, inADC_max, displayState);
            displayYaw (mappedYaw);
        }


    }
}

