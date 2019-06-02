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

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE            100
#define SAMPLE_RATE_HZ      1000    // SysTickIntHandler frequency
#define SLOWTICK_RATE_HZ    8       // Slower tick rate used for display and UART


//*****************************************************************************
// Global variables
//*****************************************************************************
circBuf_t g_inBuffer;               // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
volatile uint8_t slowTick = false;  // Flag for set lower tick rate SLOWTICK_RATE_HZ

// Global definitions of motors
rotor_t mainRotor;
rotor_t tailRotor;

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

    static uint8_t tickCount = 0;
    const uint8_t ticksPerSlow = SAMPLE_RATE_HZ / SLOWTICK_RATE_HZ;

    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = true;
    }

    // Poll buttons updating states
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
    SysTickIntRegister(SysTickIntHandler);

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


int
main(void)
{
    uint16_t inADCMax;
    uint16_t altRaw     = 0;
    int16_t  mappedAlt  = 0;
    bool     init_prog  = true;
    int32_t  yawError   = 0;
    int32_t  altError   = 0;
    int16_t  desiredAlt = 0;  // Percentage.
    int32_t  desiredYaw = 0;  // Raw yaw value.
    yawErrorInt = 0;
    altErrorInt = 0;
    heliState   = LANDED;

    initButtons ();
    initClock ();
    initADC ();
    initYaw ();
    initDisplay ();
    initUSB_UART ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initPWMMain (&mainRotor); // Initialise motors with set freq and duty cycle
    initPWMTail (&tailRotor);

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
                inADCMax = initAltitude(readCircBuf (&g_inBuffer));
            }
        }

        // FSM for different helicopter states.
        // Uses switch inputs, orientation and elevation to control
        // helicopter state.
        switch (heliState)
        {
        // LANDED - Turn motors off, check for upwards SW change to
        //          move to TAKING_OFF
        case LANDED:
            desiredAlt = 0;
            landed (&mainRotor, &tailRotor);
            break;

        // TAKING_OFF - Turn on motors and rotate until yaw reference
        //              point found then switch to FLYING. Disable yawRefInt
        //              once state changes.
        case TAKING_OFF:
            takeOff (&mainRotor, &tailRotor);

            if (heliState == FLYING)
            {
                yawRefIntDisable();
            }
            break;

        // FLYING - Control heli height and yaw using PID control,
        //          adjusting desired position based on button inputs.
        //          Change to LANDING when SW move to down.
        case FLYING:    //
            desiredAlt = updateDesiredAlt (desiredAlt);
            desiredYaw = updateDesiredYaw (desiredYaw);
            altError = calcAltError(desiredAlt, mappedAlt);
            yawError = calcYawError(desiredYaw, yaw);
            flight (&mainRotor, &tailRotor, altError, yawError);

            if (heliState == LANDING)
            {
                desiredYaw = 0;
            }
            break;

        // LANDING - Reduce desired height by DROP_ALT_STEP at 8Hz,
        //           maintaining desired yaw of 0.
        //           Move to LANDED once altitude is within 2%.
        case LANDING:
            if (desiredAlt - DROP_ALT_STEP > 0 && slowTick)
            {
                desiredAlt = desiredAlt - DROP_ALT_STEP;
            } else if (desiredAlt - DROP_ALT_STEP < 0)
            {
                desiredAlt = 0;
            }

            altError = calcAltError(desiredAlt, mappedAlt);
            yawError = calcYawError(desiredYaw, YAW_DEG(yaw));
            land (&mainRotor, &tailRotor, altError, yawError, mappedAlt);

            if (heliState == LANDED)
            {
                yawRefIntEnable();
            }
            break;
        }

        // Time to send a message through UART at set lower frequency SLOW_TICK_RATE_HZ
        if (slowTick && !init_prog)
        {
            slowTick = false;
            int16_t mappedYaw = mapYaw2Deg(yaw, false);
            handleHMI (&mainRotor, &tailRotor, mappedAlt, mappedYaw, desiredAlt, desiredYaw);
        }
    }
}

