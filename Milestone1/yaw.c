// *******************************************************
//
// buttons4.c
//
// Support for a set of FOUR specific buttons on the Tiva/Orbit.
// ENCE361 sample code.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva.
//
// Note that pin PF0 (the pin for the RIGHT pushbutton - SW2 on
//  the Tiva board) needs special treatment - See PhilsNotesOnTiva.rtf.
//
// P.J. Bones UCECE
// Last modified:  7.2.2018
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
    uint8_t newStateA = GPIOPinRead(GPIO_PORTE_BASE, UP_BUT_PIN) == UP_BUT_PIN;
    uint8_t newStateB = GPIOPinRead(GPIO_PORTD_BASE, DOWN_BUT_PIN) == DOWN_BUT_PIN;

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

    stateA = newStateB;
    stateB = newStateB;
}


//********************************************************
// initYaw - Initialise yaw pins
//********************************************************
void
initYaw (void)
{
    // Enable GPIO Port B
    SysCtlPeripheralEnable(YAW_SYSCTL_PERIPH);

    // Congifure Pin A and Pin B for input WPD
    GPIOPadConfigSet(YAW_PIN_A, YAW_PIN_A, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPadConfigSet(YAW_PIN_B, YAW_PIN_B, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);

    // Set data direction register as input
    GPIOPinTypeGPIOInput(YAW_PORT_BASE, YAW_PIN_A);
    GPIOPinTypeGPIOInput(YAW_PORT_BASE, YAW_PIN_B);

    IntRegister(INT_GPIOB, yawIntHandler);
    IntRegister(INT_GPIOE, yawIntHandler);
    IntRegister(INT_GPIOD, yawIntHandler);
    IntEnable(INT_GPIOB);
    IntEnable(INT_GPIOE);
    IntEnable(INT_GPIOD);

    stateA = GPIOPinRead(UP_BUT_PORT_BASE, UP_BUT_PIN);
    stateB = GPIOPinRead(DOWN_BUT_PORT_BASE, DOWN_BUT_PIN);
    yaw = YAW_START;
}
