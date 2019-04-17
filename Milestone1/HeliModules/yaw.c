// *******************************************************
//
// yaw.c
//
// Initialisation files and interrupt handlers for quadrature
// decoding of a GPIO input signal for the yaw of a helicopter.
//
// Author:  Zeb Barry           ID: 79313790
// Author:  Mitchell Hollows    ID:
// Author:  Jack Topliss        ID:
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
#include "buttons4.h"

//*****************************************************************************
// The handler for the pin change interrupts for pin A and B
//*****************************************************************************
void
yawIntHandler(void)
{
    uint32_t intStatusU = GPIOIntStatus(UP_BUT_PORT_BASE, true);
    uint32_t intStatusD = GPIOIntStatus(DOWN_BUT_PORT_BASE, true);
    GPIOIntClear(UP_BUT_PORT_BASE, intStatusU);
    GPIOIntClear(DOWN_BUT_PORT_BASE, intStatusD);

    uint8_t newStateA = GPIOPinRead(UP_BUT_PORT_BASE, UP_BUT_PIN) == UP_BUT_PIN;
    uint8_t newStateB = GPIOPinRead(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN) == DOWN_BUT_PIN;

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
    GPIOIntTypeSet(UP_BUT_PORT_BASE, UP_BUT_PIN, GPIO_FALLING_EDGE  | GPIO_RISING_EDGE);
    GPIOIntTypeSet(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN, GPIO_FALLING_EDGE  | GPIO_RISING_EDGE);

    GPIOIntRegister(YAW_PORT_BASE, yawIntHandler);
    GPIOIntRegister(UP_BUT_PORT_BASE, yawIntHandler);
    GPIOIntRegister(DOWN_BUT_PORT_BASE, yawIntHandler);

    GPIOIntEnable(YAW_PORT_BASE, YAW_PIN_A | YAW_PIN_B);
    GPIOIntEnable(UP_BUT_PORT_BASE, UP_BUT_PIN);
    GPIOIntEnable(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN);

    // Set initial state.
    stateA = GPIOPinRead(UP_BUT_PORT_BASE, UP_BUT_PIN);
    stateB = GPIOPinRead(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN);
    yaw = YAW_START;
}
