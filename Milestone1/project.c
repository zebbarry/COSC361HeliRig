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
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
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
#include "heliHMI.h"
#include "heliTimer.h"
#include "kernel.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE            100
#define SAMPLE_RATE_HZ      1000
#define DISPLAY_RATE        8
#define CONTROLLER_RATE     100
#define ALT_UPDATE_RATE     100


//*****************************************************************************
// Global variables
//*****************************************************************************
circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts

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

    updateButtons();

    /*if (checkButton(RESET) == PUSHED) {
        SysCtlReset();
    }*/
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
// initAltitude - Calibrate height by setting limit.
//********************************************************
uint16_t
initAltitude (uint16_t altRaw)
{
    return altRaw - ALT_RANGE;
}

static void
updateAltTask (heli_t *data)
{
    heli_t *heli = data;
    uint16_t altRaw = 0;
    static uint16_t inADCMax;

    if (g_inBuffer.written)
    {
        altRaw = calcMean (&g_inBuffer, BUF_SIZE);
        heli->mappedAlt = mapAlt(altRaw, inADCMax);

        // If start of program, calibrate ADC input
        if (heli->initProg)
        {
            heli->initProg = false;
            inADCMax = initAltitude(readCircBuf (&g_inBuffer));
        }
    }
}

static void
heliInfoOutputTask (heli_t *data)
{
    heli_t *heli = data;
    if (!heli->initProg)
    {
        heli->mappedYaw = mapYaw2Deg(yaw, false);
        handleHMI (heli);
    }
}

static void
stateMachineTask (heli_t *data)
{
    heli_t *heli = data;

    int32_t yawError;
    int32_t altError;

    // FSM based on SW1, orientation and altitude.
    switch (heli->heliState)
    {
    case LANDED:    // Turn motors off and check for SW change
        heli->desiredAlt = 0;
        heli->heliState = landed (heli->mainRotor, heli->tailRotor);
        break;

    case TAKING_OFF:    // Hover and find yaw ref
        heli->heliState = takeOff (heli->mainRotor, heli->tailRotor);

        if (heli->heliState == FLYING)
        {
            yawRefIntDisable();
        }
        break;

    case FLYING:    // Fly to desired position and check for SW change
        heli->desiredAlt = updateDesiredAlt (heli->desiredAlt);
        heli->desiredYaw = updateDesiredYaw (heli->desiredYaw);
        altError = calcAltError(heli->desiredAlt, heli->mappedAlt);
        yawError = calcYawError(heli->desiredYaw, yaw);
        heli->heliState = flight (heli->mainRotor, heli->tailRotor, altError, yawError);

        if (heli->heliState == LANDING)
        {
            heli->desiredYaw = 0;
        }
        break;

    case LANDING:   // Land Heli and change to LANDED once alt < 1%
        if (heli->desiredAlt - DROP_ALT_STEP > 0) {
            heli->desiredAlt = heli->desiredAlt - DROP_ALT_STEP;
        }
        altError = calcAltError(heli->desiredAlt, heli->mappedAlt);
        yawError = calcYawError(heli->desiredYaw, YAW_DEG(yaw));
        heli->heliState = land (heli->mainRotor, heli->tailRotor, altError, yawError, heli->mappedAlt);

        if (heli->heliState == LANDED)
        {
            yawRefIntEnable();
        }
        break;
    }
}

int
main(void)
{
    yawErrorInt = 0;
    altErrorInt = 0;
    rotor_t mainRotor;
    rotor_t tailRotor;

    initButtons ();
    initClock ();
    initTimer ();
    initADC ();
    initYaw ();
    initDisplay ();
    initUSB_UART ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initPWMMain (&mainRotor); // Initialise motors with set freq and duty cycle
    initPWMTail (&tailRotor);

    // Enable interrupts to the processor.
    IntMasterEnable();
    heli_t heli = {
       .mainRotor = &mainRotor,
       .tailRotor = &tailRotor,
       .initProg = true,
       .mappedAlt = 0,
       .mappedYaw = 0,
       .desiredAlt = 0,
       .desiredYaw = 0,
       .heliState = LANDED
    };

    task_t tasks[] = {
          {.handler = stateMachineTask, .data = &heli, .updateFreq = CONTROLLER_RATE},
          {.handler = updateAltTask, .data = &heli, .updateFreq = ALT_UPDATE_RATE},
          {.handler = heliInfoOutputTask, .data = &heli, .updateFreq = DISPLAY_RATE}
    };

    runTasks(tasks);
}

