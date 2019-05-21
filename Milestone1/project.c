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

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE            100
#define SAMPLE_RATE_HZ      1000
#define SLOWTICK_RATE_HZ    5


//*****************************************************************************
// Global variables
//*****************************************************************************
circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
volatile uint8_t slowTick = false;
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
// initAltitude - Calibrate height by setting limit.
//********************************************************
uint16_t
initAltitude (uint16_t altRaw)
{
    return altRaw - ALT_RANGE;
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
    }/*
    if (checkButton(RIGHT) == PUSHED && tailRotor.duty < PWM_DUTY_MAX_PER)
    {
        tailRotor.duty += PWM_DUTY_STEP_PER;
        setPWM (&tailRotor);
    }
    if (checkButton(LEFT) == PUSHED && tailRotor.duty > PWM_DUTY_MIN_PER)
    {
        tailRotor.duty -= PWM_DUTY_STEP_PER;
        setPWM (&tailRotor);
    }*/
}



int
main(void)
{
    uint16_t inADCMax;
    uint16_t altRaw = 0;
    int16_t mappedAlt = 0;
    bool    init_prog = true;
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
    if (debug) {
        motorPower (&mainRotor, true);
        motorPower (&tailRotor, true);
    }

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

        if (debug) {
            controlDuty();
            heliState = 4;

            desiredYaw = updateDesiredYaw (desiredYaw);
            altError = calcAltError(desiredAlt, mappedAlt);
            yawError = calcYawError(desiredYaw, yaw);
            updateMotors (&mainRotor, &tailRotor, altError, yawError);
        }

        // FSM based on SW1, orientation and altitude.
        switch (heliState)
        {
        case LANDED:    // Turn motors off and check for SW change
            landed (&mainRotor, &tailRotor);
            break;

        case TAKING_OFF:    // Hover and find yaw ref.
            takeOff (&mainRotor, &tailRotor);
            break;

        case FLYING:    // Fly to desired position and check for SW change
            desiredAlt = updateDesiredAlt (desiredAlt);
            desiredYaw = updateDesiredYaw (desiredYaw);
            altError = calcAltError(desiredAlt, mappedAlt);
            yawError = calcYawError(desiredYaw, yaw);
            flight (&mainRotor, &tailRotor, altError, yawError);
            break;

        case LANDING:   // Land Heli and change to LANDED once alt < 1%
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
            handleHMI (&mainRotor, &tailRotor, mappedAlt, mappedYaw, desiredAlt, desiredYaw);
        }
    }
}

