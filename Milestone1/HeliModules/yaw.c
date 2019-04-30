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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "yaw.h"
#include "display.h"
#include "USBUART.h"
#include "utils/ustdlib.h"


//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void)
{
    uint32_t intStatus = GPIOIntStatus(YAW_PORT_BASE, true);
    GPIOIntClear(YAW_PORT_BASE, intStatus);

    currentState = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);

    switch (currentState)
    {
    case BOTH_ZERO:      // BA = 00
        if (previousState == B_ONE)
        {
            dir = CW;
        } else {
            dir = CCW;
        }
        break;
    case A_ONE:          // BA = 01
        if (previousState == BOTH_ZERO)
        {
            dir = CW;
        } else {
            dir = CCW;
        }
        break;
    case BOTH_ONE:       // BA = 11
        if (previousState == A_ONE)
        {
            dir = CW;
        } else {
            dir = CCW;
        }
        break;
    case B_ONE:          // BA = 10
        if (previousState == BOTH_ONE)
        {
            dir = CW;
        } else {
            dir = CCW;
        }
        break;
    }
    yaw += dir;

    previousState = currentState;
}


//********************************************************
// initYaw - Initialise yaw pins
//********************************************************
void
initYaw (void)
{
    // Enable GPIO Port for yaw.
    SysCtlPeripheralEnable(YAW_SYSCTL_PERIPH);

    // Congifure Pin A and Pin B for input WPD
    GPIOPadConfigSet(YAW_PIN_A, YAW_PIN_A, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(YAW_PIN_B, YAW_PIN_B, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);

    // Set data direction register as input
    GPIOPinTypeGPIOInput(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);

    // Set and register interrupts for pin A and B.
    GPIOIntTypeSet(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B, GPIO_BOTH_EDGES); // | GPIO_RISING_EDGE
    GPIOIntRegister(YAW_PORT_BASE, yawIntHandler);
    GPIOIntEnable(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);

    // Set initial state.
    currentState = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);
    yaw = 0;
    dir = STATIC;
}

//********************************************************
// mapYaw2Deg - Maps yaw value from raw input to degrees.
//********************************************************
int16_t
mapYaw2Deg(void)
{
    int16_t yawDeg = (2*(yaw * DEG_CIRC) + YAW_TABS) / 2 / YAW_TABS;
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

