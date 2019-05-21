// *******************************************************
//
// yaw.c
//
// Initialisation files and interrupt handlers for quadrature
// decoding of a GPIO input signal for the yaw of a helicopter.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID: 46510499
// Group:   Thu am 22
// Last modified:   29.4.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "yaw.h"
#include "display.h"


//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void)
{
    uint32_t intStatus = GPIOIntStatus(YAW_PORT_BASE, true);

    currentState = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);

    switch (currentState)
    {
    case BOTH_ZERO:
        if (previousState == B_ONE)     // A leads
        {
            dir = CCW;
        } else {                        // B leads, etc.
            dir = CW;
        }
        break;
    case A_ONE:
        if (previousState == BOTH_ZERO)
        {
            dir = CCW;
        } else {
            dir = CW;
        }
        break;
    case BOTH_ONE:
        if (previousState == A_ONE)
        {
            dir = CCW;
        } else {
            dir = CW;
        }
        break;
    case B_ONE:
        if (previousState == BOTH_ONE)
        {
            dir = CCW;
        } else {
            dir = CW;
        }
        break;
    }

    yaw += dir;
    previousState = currentState;

    GPIOIntClear(YAW_PORT_BASE, intStatus);
}

//*****************************************************************************
// The handler for the reference pin change for PC4.
//*****************************************************************************
void
yawRefIntHandler(void)
{
    uint32_t intStatus = GPIOIntStatus(YAW_PORT_BASE_REF, true);

    hitYawRef = true;
    yaw = 0;

    GPIOIntClear(YAW_PORT_BASE_REF, intStatus);
}

//********************************************************
// initYaw - Initialise yaw pins
//********************************************************
void
initYaw (void)
{
    // Enable GPIO Port for yaw.
    SysCtlPeripheralEnable(YAW_SYSCTL_PERIPH);
    SysCtlPeripheralEnable(YAW_SYSCTL_PERIPH_REF);

    // Congifure Pin A, Pin B and Ref Pin for input WPD
    GPIOPadConfigSet(YAW_PIN_A, YAW_PIN_A, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(YAW_PIN_B, YAW_PIN_B, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(YAW_PIN_REF, YAW_PIN_REF, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);

    // Set data direction register as input
    GPIOPinTypeGPIOInput(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);
    GPIOPinTypeGPIOInput(YAW_PORT_BASE_REF, YAW_PIN_REF);

    // Set and register interrupts for pin A and B.
    GPIOIntTypeSet(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B, GPIO_BOTH_EDGES);
    GPIOIntRegister(YAW_PORT_BASE, yawIntHandler);
    GPIOIntEnable(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);

    // Set and register interrupts for Ref Pin
    GPIOIntTypeSet(YAW_PORT_BASE_REF, YAW_PIN_REF, GPIO_RISING_EDGE);
    GPIOIntRegister(YAW_PORT_BASE_REF, yawRefIntHandler);
    GPIOIntEnable(YAW_PORT_BASE_REF, YAW_PIN_REF);

    // Set initial state.
    currentState = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);
    yaw = 0;
    hitYawRef = false;
    dir = STATIC;
}

//********************************************************
// mapYaw2Deg - Maps yaw value from raw input to degrees.
//********************************************************
int16_t
mapYaw2Deg(int16_t yawVal)
{
    int16_t yawDeg = YAW_DEG(yawVal);
    int16_t scaledYaw = yawDeg % DEG_CIRC;
    int16_t mappedYaw = scaledYaw;
    if (abs(scaledYaw) > (DEG_CIRC / 2)) {
        if (scaledYaw > 0) {
            mappedYaw = scaledYaw - 360;
        } else
        {
            mappedYaw = scaledYaw + 360;
        }
    }
    return mappedYaw;
}

