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
#include "heliPWM.h"
#include "motorControl.h"
#include "stateMachine.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE            100
#define SAMPLE_RATE_HZ      1000
#define SLOWTICK_RATE_HZ    5
#define MAX_STR_LEN         17


//*****************************************************************************
// Global variables
//*****************************************************************************
circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
volatile uint8_t slowTick = false;
char statusStr[MAX_STR_LEN + 1];
static uint16_t inADCMax;
rotor_t mainRotor;
rotor_t tailRotor;
static int16_t desiredAlt = 0;  // Percentage.
static int32_t desiredYaw = 0;  // Raw yaw value.

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

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


//********************************************************
// initAltitude - Calibrate height
//********************************************************
void
initAltitude (uint16_t altRaw)
{
    inADCMax = altRaw - ALT_RANGE;
}


//********************************************************
// handleHMI - Handle output to UART port and display.
//********************************************************
void
handleHMI (uint16_t mappedAlt, int16_t mappedYaw)
{
    // Form and send a status message for altitude to the console
    usnprintf (statusStr, sizeof(statusStr), "ALT: %3d [%3d]\r\n", mappedAlt, desiredAlt); // * usprintf
    UARTSend (statusStr);

    // Form and send a status message for yaw to the console
    int16_t mappedDesiredYaw = mapYaw2Deg (desiredYaw);
    usnprintf (statusStr, sizeof(statusStr), "YAW: %4d [%4d]\r\n", mappedYaw, mappedDesiredYaw); // * usprintf
    UARTSend (statusStr);

    usnprintf (statusStr, sizeof(statusStr), "MAIN %2d TAIL %2d\r\n", mainRotor.duty, tailRotor.duty); // * usprintf
    UARTSend (statusStr);

    char* state;
    if (heliState == LANDED)
    {
        state = "LANDED";
    } else if (heliState == TAKING_OFF)
    {
        state = "TAKING OFF";
    } else if (heliState == FLYING)
    {
        state = "FLYING";
    } else if (heliState == LANDING)
    {
        state = "LANDING";
    } else
    {
        state = "ERROR";
    }
    // Leave enough space for the template, state and null terminator.
    usnprintf (statusStr, sizeof(statusStr), "HELI: %s\r\n\n", state); // * usprintf
    UARTSend (statusStr);

    // Update OLED display with ADC and yaw value.
    displayMeanVal (mappedAlt, desiredAlt);
    displayYaw (mappedYaw, desiredYaw);
    displayPWM (&mainRotor, &tailRotor);
    displayState (heliState);
}


void
controlDuty(void)
{
    if (checkButton(UP) == PUSHED && mainRotor.duty < PWM_DUTY_MAX_PER)
    {
        mainRotor.duty += PWM_DUTY_STEP_PER;
        setPWM (&mainRotor);
    }
    if (checkButton(DOWN) == PUSHED && mainRotor.duty > PWM_DUTY_MIN_PER)
    {
        mainRotor.duty -= PWM_DUTY_STEP_PER;
        setPWM (&mainRotor);
    }
    if (checkButton(RIGHT) == PUSHED && tailRotor.duty < PWM_DUTY_MAX_PER)
    {
        tailRotor.duty += PWM_DUTY_STEP_PER;
        setPWM (&tailRotor);
    }
    if (checkButton(LEFT) == PUSHED && tailRotor.duty > PWM_DUTY_MIN_PER)
    {
        tailRotor.duty -= PWM_DUTY_STEP_PER;
        setPWM (&tailRotor);
    }
}


int
main(void)
{
    uint16_t altRaw = 0;
    int16_t mappedAlt = 0;
    bool init_prog = true;
    int32_t yawError;
    int32_t altError;
    yawErrorInt = 0;
    altErrorInt = 0;
    heliState = LANDED;

    initButtons ();
    initClock ();
    initADC ();
    initYaw ();
    initDisplay ();
    initUSB_UART ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initPWMMain (&mainRotor); // Initialise motors with set freq and duty cycle
    initPWMTail (&tailRotor);

    //
    // Enable interrupts to the processor.
    IntMasterEnable();

    while (1)
    {
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number, as long
        // as the buffer has been written into.
        if (g_inBuffer.written)
        {
            altRaw = calcMean (&g_inBuffer, BUF_SIZE);
            mappedAlt = mapAlt(altRaw, inADCMax);

            // If start of program, calibrate ADC input
            if (init_prog)
            {
                init_prog = false;
                initAltitude(readCircBuf (&g_inBuffer));
            }
        }

        // FSM based on SW1, orientation and altitude.
        switch (heliState)
        {
        case LANDED:
            landed (&mainRotor, &tailRotor);
            break;

        case TAKING_OFF:
            takeOff (&mainRotor, &tailRotor);
            break;

        case FLYING:
            desiredAlt = updateDesiredAlt (desiredAlt);
            desiredYaw = updateDesiredYaw (desiredYaw);
            altError = calcAltError(desiredAlt, mappedAlt);
            yawError = calcYawError(desiredYaw, yaw);
            flight (&mainRotor, &tailRotor, altError, yawError);
            break;

        case LANDING:
            desiredAlt = 0;
            desiredYaw = 0;
            altError = calcAltError(desiredAlt, mappedAlt);
            yawError = calcYawError(desiredYaw, yaw);
            land (&mainRotor, &tailRotor, altError, yawError, mappedAlt);
            break;
        }


        // Time to send a message through UART at set lower frequency SLOW_TICK_RATE_HZ
        if (slowTick && !init_prog)
        {
            slowTick = false;
            int16_t mappedYaw = mapYaw2Deg(yaw);
            handleHMI (mappedAlt, mappedYaw);
        }
    }
}

