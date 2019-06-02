//*****************************************************************************
//
// project.c - Program for controlling a small helicopter through PID control.
//             Helicopter altitude and yaw is measured using interrupts, with
//             ADC sampling for altitude and quadratue encoding for yaw. Motor
//             speeds are controlled using PWM signals output from the board.
//             Helicopter states and orientation are controlled using a switch
//             for take off and landing and buttons to control orientation.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   20.5.2019
//
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
#define BASE_FREQ           250


//*****************************************************************************
// Global variables
//*****************************************************************************
circBuf_t g_inBuffer;               // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
rotor_t mainRotor;
rotor_t tailRotor;

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void
sysTickIntHandler(void)
{
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3); 
    g_ulSampCnt++;

    updateButtons();

    // If reset button has been pressed, call Reset function
    if (checkButton(RESET) == PUSHED) {
        SysCtlReset();
    }
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
    SysTickIntRegister(sysTickIntHandler);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


//********************************************************
// initAltitude - Calibrate height by setting max limit.
//********************************************************
uint16_t
initAltitude (uint16_t altRaw)
{
    return altRaw - ALT_RANGE;
}

//********************************************************
// updateAltTask - Updates altitude value by averaging the samples.
//********************************************************
static void
updateAltTask (heli_t *data)
{
    heli_t *heli = data;
    uint16_t altRaw = 0;
    static uint16_t inADCMax;

    // If values have been written to the buffer, then calculate the average
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

//********************************************************
// heliInfoOutputTask - Outputs varius important information
// about helicopter via UART and OLED display.
//********************************************************
static void
heliInfoOutputTask (heli_t *data)
{
    heli_t *heli = data;
    if (!heli->initProg)
    {
        // Heli mappped yaw definition so that display of value is immune to interrupt changes
        heli->mappedYaw = mapYaw2Deg(yaw, false);
        handleHMI (heli);
    }
}

//********************************************************
// stateMachineTask - Controls helicopter state, using PID
// control to hold altitude and yaw at desired values.
// Lands and takes off helicopter depending on state of switch.
//********************************************************
static void
stateMachineTask (heli_t *data)
{
    heli_t *heli = data;
    int32_t yawError;
    int32_t altError;

    // FSM for different helicopter states.
    // Uses switch inputs, orientation and elevation to control
    // helicopter state.
    switch (heli->heliState)
    {
    // LANDED - Turn motors off, check for upwards SW change to
    //          move to TAKING_OFF
    case LANDED:    // Turn motors off and check for SW change
        heli->desiredAlt = 0;
        heli->heliState = landed (heli->mainRotor, heli->tailRotor);
        break;

    // TAKING_OFF - Turn on motors and rotate until yaw reference
    //              point found then switch to FLYING. Disable yawRefInt
    //              once state changes.
    case TAKING_OFF:    // Hover and find yaw ref
        heli->heliState = takeOff (heli->mainRotor, heli->tailRotor);
        break;

    // FLYING - Control heli height and yaw using PID control,
    //          adjusting desired position based on button inputs.
    //          Change to LANDING when SW move to down.
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

    // LANDING - Reduce desired height by DROP_ALT_STEP at 8Hz,
    //           maintaining desired yaw of 0.
    //           Move to LANDED once altitude is within 2%.
    case LANDING:   // Land Heli and change to LANDED once alt < 1%
        if (heli->desiredAlt - DROP_ALT_STEP > 0) {
            heli->desiredAlt = heli->desiredAlt - DROP_ALT_STEP;
        }
        altError = calcAltError(heli->desiredAlt, heli->mappedAlt);
        yawError = calcYawError(heli->desiredYaw, YAW_DEG(yaw));
        heli->heliState = land (heli->mainRotor, heli->tailRotor, altError, yawError, heli->mappedAlt);
        break;
    }
}

int
main(void)
{
    yawErrorInt = 0;
    altErrorInt = 0;

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

    // Set initial values for helicopter
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

    // Define tasks for the scheduler and their frequencies
    task_t tasks[] = {
          {.handler = stateMachineTask, .data = &heli, .updateFreq = CONTROLLER_RATE},
          {.handler = updateAltTask, .data = &heli, .updateFreq = ALT_UPDATE_RATE},
          {.handler = heliInfoOutputTask, .data = &heli, .updateFreq = DISPLAY_RATE},
          {0}   // Null terminator
    };

    // Run scheduler
    runTasks(tasks, BASE_FREQ);
}

