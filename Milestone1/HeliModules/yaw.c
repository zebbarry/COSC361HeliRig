// *******************************************************
//
// yaw.c
//
// Initialisation files and interrupt handlers for quadrature
// decoding of a GPIO input signal for the yaw of a helicopter.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID: 23567059
// Author:  Jack Topliss        ID:
// Group:   Thu am 22
// Last modified:   9.4.2019
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

//*****************************************************************************
// Constants
//*****************************************************************************

#define YAW_DEG     360
#define YAW_TABS    112


//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void)
{
    uint32_t intStatus = GPIOIntStatus(YAW_PORT_BASE, true);
    GPIOIntClear(YAW_PORT_BASE, intStatus);

    uint8_t newStateA = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_A) == YAW_PIN_A;
    uint8_t newStateB = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_B) == YAW_PIN_B;

    if (stateA == 1 && stateB == 1) // Limit yaw to change once every cycle.
    {
        if (newStateA != stateA)    // A leads B
        {
            yaw--;                  // Moving counterclockwise
        }
        else if (newStateB != stateB)   // B leads A
        {
            yaw++;                  // Moving clockwise
        }
    }

    stateA = newStateA;
    stateB = newStateB;
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
    GPIOIntTypeSet(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B, GPIO_FALLING_EDGE  | GPIO_RISING_EDGE);
    GPIOIntRegister(YAW_PORT_BASE, yawIntHandler);
    GPIOIntEnable(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);

    // Set initial state.
    stateA = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_A);
    stateB = GPIOPinRead(YAW_PORT_BASE, YAW_PIN_B);
    yaw = YAW_START;
}

//********************************************************
// mapYaw2Deg - Maps yaw value from raw input to degrees.
//********************************************************
int16_t
mapYaw2Deg(int16_t yaw)
{
    return (2*(yaw * YAW_DEG) + YAW_TABS) / 2 / YAW_TABS;
}

